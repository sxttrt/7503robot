"""Hardware boundary regressions; never initialize ROS or move any actuator."""
import json
import math
import time
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
import pytest
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from robot_navigation.localization import Localization
from robot_navigation.navigation import NavigationServer
from robot_navigation.path_tracker import PathTracker
from robot_navigation.tracking_control import PathController
from mission_manager.adapters.navigation import AsyncAction
from mission_manager.adapters.lift import Lift
from test_path_tracker import cfg, goal
from test_real_navigation import health


def localization_double():
    estimate = dict(pose=(1.,1.,math.pi),velocity=(0.,0.,0.),covariance=np.eye(3)*1e-6)
    n=NS(cfg=cfg(),last_stamp=None,last_valid=0.,valid=False,reason='',
         matcher=NS(last_stamp=None,update=lambda pts,t:estimate),
         publisher=NS(publish=lambda m:None),dynamic_tf=NS(sendTransform=lambda m:None),
         get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)))
    return n


def scan(stamp=10,frame='laser'):
    m=LaserScan();m.header.frame_id=frame;m.header.stamp.sec=stamp
    m.angle_min=-math.pi;m.angle_increment=.02;m.range_min=.1;m.range_max=10.
    m.ranges=[1.]*400
    return m


@pytest.mark.parametrize('bad',[scan(1000),scan(10,'wrong_frame')])
def test_rejected_scan_cannot_poison_timestamp_or_block_next_valid_scan(bad):
    n=localization_double();Localization.on_scan(n,bad)
    assert n.last_stamp is None
    Localization.on_scan(n,scan());assert n.valid


def test_localization_does_not_mutate_input_scan_frame():
    n=localization_double();m=scan();Localization.on_scan(n,m)
    assert n.valid and m.header.frame_id=='laser'


@pytest.mark.parametrize('attr,value',[('angle_increment',0.),('angle_increment',float('nan')),
    ('angle_min',float('inf')),('range_min',-1.),('range_max',.05)])
def test_invalid_scan_geometry_never_reaches_matcher(attr,value):
    n=localization_double();m=scan();setattr(m,attr,value)
    Localization.on_scan(n,m);assert not n.valid


def test_zero_previous_command_is_not_permission_to_change_axis_while_coasting():
    c=PathController(goal(((1,1),(1.5,1))),cfg())
    command=c.step((1.1,1.009,math.pi),(-.05,0,0),10.,False,.05)
    assert command==(0.,0.,0.)


def test_reversing_direction_waits_for_actual_stop():
    c=PathController(goal(((1,1),(1.5,1))),cfg())
    c.previous=(-.08,0.,0.);c.previous_world=(.08,0.)
    command=c.step((1.53,1.,math.pi),(-.08,0,0),10.,False,.05)
    assert command==(0.,0.,0.)


def test_stalled_base_aborts_before_full_navigation_deadline():
    c=PathController(goal(((1,1),(1.5,1))),cfg())
    with pytest.raises(ValueError,match='progress'):
        for i in range(250):c.step((1.,1.,math.pi),(0,0,0),10+i*.05,True,.05)


@pytest.mark.parametrize('method',[NavigationServer.on_base,PathTracker.on_base])
@pytest.mark.parametrize('stamp',[{'sec':10.,'nanosec':0},{'sec':11,'nanosec':-1_000_000_000}])
def test_malformed_health_stamp_does_not_replace_feedback(method,stamp):
    import threading
    n=NS(cfg=cfg(),lock=threading.RLock(),base_stamp=9.,base=None,stop_base=None,
        now=lambda:10.,payload_stamp=-math.inf)
    data=health(10);data['stamp']=stamp;method(n,String(data=json.dumps(data)))
    assert n.base_stamp==9. and n.stop_base is None


def test_post_stop_map_waits_for_real_scan_not_periodic_health_heartbeat():
    cfg_=cfg();now=[10.];scanstamp=[9.9];sleep_calls=[]
    n=NS(cfg=cfg_,abort_event=NS(is_set=lambda:False),now=lambda:now[0],odom_stamp=9.9,
        lidar={'stamp':{'sec':10,'nanosec':0}})
    def inputs():
        n.odom_stamp=scanstamp[0]
        return (((1.,1.,math.pi),(0,0,0)),False,
                NS(metadata=NS(update_time=NS(sec=int(now[0]),nanosec=int(now[0]%1*1e9)))))
    n.inputs=inputs
    def advance(_):
        now[0]+=.1;sleep_calls.append(now[0]);n.lidar['stamp']={'sec':int(now[0]),'nanosec':int(now[0]%1*1e9)}
        if len(sleep_calls)>=7:scanstamp[0]=now[0]
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True),patch(
            'robot_navigation.navigation.time.sleep',side_effect=advance):
        NavigationServer.wait_post_stop_map(n,NS(is_cancel_requested=False),time.monotonic()+1,False)
    assert len(sleep_calls)>=7


def test_lift_known_boolean_but_unsettled_is_not_completion():
    from robot_navigation.core import health_valid
    data=health(10);data.update(lift_is_up=True,lift_settled=False)
    assert not health_valid(data,10.,.6)
    data['lift_settled']=True;assert health_valid(data,10.,.6)


def test_accepted_action_result_transport_failure_is_never_idle():
    reports=[];a=AsyncAction.__new__(AsyncAction)
    a.requests={'r':dict(cancelled=False,handle=None,callback=lambda *args:reports.append(args))}
    def broken():raise RuntimeError('lost result channel')
    h=NS(accepted=True,get_result_async=broken,cancel_goal_async=lambda:None)
    f=Future();f.set_result(h);a.accepted('r',f)
    assert not a.idle() and reports and reports[0][1] is False


def test_failed_result_future_retains_unconfirmed_accepted_action():
    reports=[];a=AsyncAction.__new__(AsyncAction)
    a.requests={'r':dict(cancelled=False,handle=object(),callback=lambda *args:reports.append(args))}
    f=Future();f.set_exception(RuntimeError('transport closed'));a.result('r',f)
    assert not a.idle() and len(reports)==1


@pytest.mark.parametrize('tau',[.08,.25,.40])
def test_inertia_noise_and_duplicate_scan_ticks_converge_without_diagonal_command(tau):
    c=PathController(goal(((1.,1.),(1.5,1.),(1.5,1.5))),cfg())
    pose=[1.,1.,math.pi];velocity=[0.,0.,0.];observed=tuple(pose);stamp=10.
    commands=[]
    for i in range(1600):
        # Radar at 10Hz; control at 20Hz. Stable-frame counting must not count
        # the intermediate control tick as another measured frame.
        if i%2==0:
            stamp=10.+i*.05
            observed=(pose[0]+.0003*math.sin(i),pose[1]+.0003*math.cos(i),pose[2])
        cmd=c.step(observed,tuple(velocity),stamp,math.hypot(*velocity[:2])<.002,.05)
        commands.append(cmd)
        for k in (0,1):velocity[k]+=(cmd[k]-velocity[k])*min(1.,.05/tau)
        wx,wy=-velocity[0],-velocity[1]
        pose[0]+=wx*.05;pose[1]+=wy*.05
        assert abs(cmd[0])<1e-7 or abs(cmd[1])<1e-7
        if c.done:break
    assert c.done and math.dist(pose[:2],(1.5,1.5))<=c.goal.position_tolerance
    assert commands[-1]==(0.,0.,0.)


@pytest.mark.parametrize('kind',['x_nan','x_inf','quaternion_nan','zero_quaternion','rotated'])
def test_invalid_costmap_origin_rejected_before_planner_and_does_not_poison_cache(kind):
    from robot_navigation.core import PublishedCostmapCache
    from test_published_costmap import published
    m=published(10.)
    if kind=='x_nan':m.metadata.origin.position.x=float('nan')
    if kind=='x_inf':m.metadata.origin.position.x=float('inf')
    if kind=='quaternion_nan':m.metadata.origin.orientation.z=float('nan')
    if kind=='zero_quaternion':m.metadata.origin.orientation.w=0.
    if kind=='rotated':m.metadata.origin.orientation.z=.1
    cache=PublishedCostmapCache();assert not cache.accept(m,10.)
    assert cache.accept(published(10.),10.)


def test_cancel_channel_error_preserves_action_until_terminal_result():
    reports=[];a=AsyncAction.__new__(AsyncAction)
    def broken():raise RuntimeError('cancel transport')
    a.requests={'r':dict(cancelled=False,handle=NS(cancel_goal_async=broken),callback=lambda *args:reports.append(args))}
    a.cancel('r');assert not a.idle() and len(reports)==1
    a.finish('r',False,{'reason':'late terminal result'})
    assert a.idle() and len(reports)==1


def test_tracker_shutdown_keeps_waiting_for_measured_stop():
    # Invoke the action worker on a double; no executor/context is created.
    import threading
    h=NS(request=goal(),is_cancel_requested=False,abort=lambda:None)
    n=NS(lock=threading.RLock(),cfg=cfg(),closing=True,now=lambda:10.,active=None,
        busy=True,zero=lambda:None)
    result=NS(success=False,stopped=True)
    def complete():
        until=time.monotonic()+1.
        while n.active is None and time.monotonic()<until:time.sleep(.001)
        time.sleep(.1);n.active['result']=result;n.active['done'].set()
    worker=threading.Thread(target=complete);worker.start()
    with patch('robot_navigation.path_tracker.rclpy.ok',return_value=True):
        actual=PathTracker.execute(n,h)
    worker.join();assert actual.stopped and not n.busy
