"""离线检查；不调用 rclpy.init、不创建节点或启动任何进程。"""
import copy
import math
import threading
import time
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from mission_interfaces.action import TrackPath
from mission_manager.project_paths import project_root
from robot_navigation.core import (load_settings,laser_to_base,WallLocalizer,
    health_valid,pose_values,planner_for,settled,route_distance)
from robot_navigation.navigation import NavigationServer, Tracking

def config():
    cfg=load_settings(project_root()/'src/robot_bringup/config/navigation_robot.yaml')
    cfg['calibration_verified']=True
    return cfg

def pose(x=1.,y=1.,yaw=math.pi):
    p=Pose();p.position.x=x;p.position.y=y
    p.orientation.z=math.sin(yaw/2);p.orientation.w=math.cos(yaw/2)
    return p

def stamped(x=1.,y=1.,yaw=math.pi,stamp=5):
    p=PoseStamped();p.header.frame_id='map';p.header.stamp.sec=stamp;p.pose=pose(x,y,yaw)
    return p

def health(stamp=5):
    return {'stamp':{'sec':stamp,'nanosec':0},'modules':{'base':True,'lift':True},
            'stopped':True,'lift_is_up':False,'lift_state_source':'measured','fault':''}

def grid():
    return NS(header=NS(frame_id='map'),metadata=NS(resolution=.025,size_x=122,size_y=80,
        origin=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.)),
        update_time=NS(sec=5,nanosec=0)),data=[0]*(122*80))

def test_template_not_calibrated():
    assert load_settings(project_root()/'src/robot_bringup/config/navigation_robot.yaml')['calibration_verified'] is False

@pytest.mark.parametrize('name,value', [('laser_pose',[0,0,float('nan'),0]),
    ('initial_pose',[9,9,0]),('loaded_half_size',[.01,.01]),('travel_yaw',.5),
    ('max_linear_speed',1.),('margin',-1.),('feedback_timeout_sec',0),
    ('calibration_verified','true'),('yaw_tolerance',.4)])
def test_bad_parameters(tmp_path,name,value):
    import yaml
    cfg=config();cfg[name]=value
    p=tmp_path/'settings.yaml';p.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError):load_settings(p)

def test_extrinsic_scan_matching():
    cfg=config();xy=(1.5,1.);yaw=.12
    world=np.vstack((np.column_stack((np.zeros(100),np.linspace(.1,1.9,100))),
                     np.column_stack((np.linspace(.1,2.9,100),np.zeros(100)))))
    c,s=math.cos(yaw),math.sin(yaw)
    base=(world-np.array(xy))@np.array([[c,-s],[s,c]])
    ext=[.08,-.03,.15,.2];c,s=math.cos(ext[3]),math.sin(ext[3])
    laser=(base-np.array(ext[:2]))@np.array([[c,-s],[s,c]])
    converted=laser_to_base(laser,ext)
    assert np.max(np.abs(converted-base))<1e-10
    estimate=WallLocalizer(cfg['field_size'],[1.51,1.01,.13]).update(converted,5.)
    assert np.max(np.abs(estimate['pose']-np.array([*xy,yaw])))<1e-6

@pytest.mark.parametrize('change', [{'lift_state_source':'estimated'}, {'lift_is_up':None},
    {'fault':'motor fault'},{'stopped':1},{'modules':{'base':False,'lift':True}},
    {'stamp':{'sec':2,'nanosec':0}}, {'stamp':{'sec':6,'nanosec':0}}])
def test_bad_health(change):
    data=health();data.update(change);assert not health_valid(data,5.,.6)

def test_fresh_measured_health():assert health_valid(health(),5.1,.6)

def test_pose_nonfinite_rejected():
    p=pose();p.position.x=float('nan')
    with pytest.raises(ValueError):pose_values(p)

def test_nonplanar_or_invalid_quaternion():
    p=pose();p.orientation.x=.3
    with pytest.raises(ValueError):pose_values(p)

def test_loaded_footprint_changes_clearance():
    cfg=config();message=grid()
    assert planner_for(cfg,message,False).segment_free((1.,.2),(2.,.2))
    assert not planner_for(cfg,message,True).segment_free((1.,.2),(2.,.2))

def test_unknown_cells_block():
    cfg=config();cfg['allow_unknown_in_field']=False;message=grid();message.data=[255]*len(message.data)
    assert planner_for(cfg,message,False).plan((1.,1.),(2.,1.)) is None

def test_rotation_uses_circle_sweep():
    cfg=config();message=grid()
    assert planner_for(cfg,message,False).segment_free((1.,.18),(1.,.18))
    assert not planner_for(cfg,message,False,True).segment_free((1.,.18),(1.,.18))

def test_plan_axis_segments():
    p=planner_for(config(),grid(),False);start=(1.,.6);goal=(2.,1.3)
    route=p.plan(start,goal);assert route
    for a,b in zip([start]+route,route):assert a[0]==b[0] or a[1]==b[1]

def test_stop_requires_speed_and_heading():
    cfg=config();p=(1.,1.,math.pi)
    assert settled(p,(0,0,0),p,cfg)
    assert not settled(p,(.02,0,0),p,cfg)
    assert not settled((1.,1.,math.pi+.1),(0,0,0),p,cfg)

def test_cross_track_distance():
    route=[(1,1),(2,1),(2,1.5)]
    assert route_distance((1.5,1.01,0),route)==pytest.approx(.01)
    assert route_distance((2,1.2,0),route)==pytest.approx(0)
    assert route_distance((1.1,1.,0),[(1,1)])==pytest.approx(.1)

class FakeNav:
    def __init__(self,mode='success'):
        self.cfg=config();self.cfg['settle_duration_sec']=.2;self.cfg['cancel_timeout_sec']=.01
        self.abort_event=threading.Event();self.fault='';self.odom=((1.,1.,math.pi),(0.,0.,0.))
        self.odom_stamp=5.;self.base={'stopped':True};self.path_pub=NS(publish=lambda p:None)
        self.checks=0;self.mode=mode;self.goal=None;self.callback=None;self.cancelled=False
        self.track=NS(send=self.send,cancel=self.cancel)
    def now(self):return 5.
    def send(self,request,goal,callback):
        self.goal=goal;self.callback=callback
        if self.mode=='success':callback(request,True,{'result':NS(final_pose=stamped(),stopped=True)})
        elif self.mode=='bad_final':callback(request,True,{'result':NS(final_pose=stamped(x=2),stopped=True)})
        else:self.abort_event.set()
    def cancel(self,request):
        self.cancelled=True
        if self.mode!='silent_cancel':self.callback(request,False,{'result':NS(stopped=True),'reason':'cancelled'})
    def check_route(self,*_args):self.checks+=1;self.odom_stamp+=.1

def track_path():
    p=Path();p.header.frame_id='map';p.poses=[stamped(),stamped()];return p

def fake_handle():return NS(is_cancel_requested=False,publish_feedback=lambda f:None)

def test_delegate_and_own_continuous_settle():
    n=FakeNav()
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        NavigationServer.run_track(n,fake_handle(),track_path(),(1.,1.,math.pi),False,time.monotonic()+1)
    assert n.checks>=3 and isinstance(n.goal,TrackPath.Goal)
    assert not n.goal.allow_in_place_rotation and n.goal.path.header.frame_id=='map'
    assert n.goal.position_tolerance==n.cfg['position_tolerance']

def test_ack_final_pose_insufficient():
    n=FakeNav('bad_final')
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True), pytest.raises(ValueError,match='final measured'):
        NavigationServer.run_track(n,fake_handle(),track_path(),(1.,1.,math.pi),False,time.monotonic()+1)

@pytest.mark.parametrize('mode,locked',[('cancel',False),('silent_cancel',True)])
def test_cancel_waits_result_and_locks_if_missing(mode,locked):
    n=FakeNav(mode)
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True),pytest.raises(ValueError,match='cancelled'):
        NavigationServer.run_track(n,fake_handle(),track_path(),(1.,1.,math.pi),False,time.monotonic()+1)
    assert n.cancelled and bool(n.fault)==locked

def test_late_track_accept_is_cancelled_not_idle():
    accepted=Future();terminal=Future();handle=NS(accepted=True,cancel_count=0)
    handle.get_result_async=lambda:terminal
    def cancel():handle.cancel_count+=1
    handle.cancel_goal_async=cancel
    client=NS(server_is_ready=lambda:True,send_goal_async=lambda *a,**k:accepted)
    t=Tracking.__new__(Tracking);t.client=client;t.requests={};t.feedback={}
    finished=[];t.send('r',TrackPath.Goal(),lambda *args:finished.append(args));t.cancel('r')
    assert not t.idle()
    accepted.set_result(handle)
    assert handle.cancel_count==1 and not t.idle()
    result=TrackPath.Result();result.stopped=True;result.success=False
    terminal.set_result(NS(status=GoalStatus.STATUS_CANCELED,result=result))
    assert t.idle() and len(finished)==1 and finished[0][1] is False

def test_uncalibrated_inputs_refused():
    n=NS(cfg=config(),fault='');n.cfg['calibration_verified']=False
    with pytest.raises(ValueError,match='calibration'):NavigationServer.inputs(n)

def test_stale_source_costmap_refused():
    cfg=config();msg=grid();msg.metadata.update_time.sec=1
    n=NS(cfg=cfg,fault='',odom=((1,1,math.pi),(0,0,0)),odom_received=time.monotonic(),
        odom_stamp=5.,lidar={'valid':True},lidar_received=time.monotonic(),
        base=health(),base_received=time.monotonic(),track=NS(available=lambda:True),
        cache=NS(snapshot=lambda:(msg,time.monotonic(),1)),now=lambda:5.)
    with pytest.raises(ValueError,match='source stale'):NavigationServer.inputs(n)

def test_launch_contains_real_nodes_only():
    import importlib.util
    path=project_root()/'src/robot_bringup/launch/robot.launch.py'
    spec=importlib.util.spec_from_file_location('real_launch',path);mod=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    description=mod.generate_launch_description();assert len(description.entities)>=10
    text=path.read_text()
    assert "package='robot_navigation'" in text and "'use_sim_time':False" in text
    assert 'framework_execution' not in text and 'kinematic_driver' not in text


def test_live_collision_cancels_track():
    n=FakeNav('cancel');n.abort_event.clear()
    n.send=lambda request,goal,callback:setattr(n,'callback',callback)
    n.track.send=n.send
    def check(*args):
        n.checks+=1
        if n.checks>1:raise ValueError('live radar route blocked')
    n.check_route=check
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True),pytest.raises(ValueError,match='blocked'):
        NavigationServer.run_track(n,fake_handle(),track_path(),(1.,1.,math.pi),False,time.monotonic()+1)
    assert n.cancelled and not n.fault

def test_loaded_goal_forbids_rotation():
    n=NS(cfg=config(),busy=True,lock=threading.Lock(),get_logger=lambda:NS(error=lambda text:None))
    n.inputs=lambda:(((1.,1.,math.pi),(0,0,0)),True,grid())
    n.wait_handoff=lambda *args:None
    n.wait_payload_map=lambda *a:n.inputs()
    handle=NS(request=NS(pose=stamped(yaw=math.pi+.2)),is_cancel_requested=False,
              abort=NS(),succeed=NS())
    calls=[];handle.abort=lambda:calls.append('abort');handle.succeed=lambda:calls.append('success')
    n.run_track=lambda *a:calls.append('track')
    result=NavigationServer.execute(n,handle)
    assert calls==['abort'] and result.error_code!=0 and not n.busy

def test_start_scan_restores_heading_before_translation():
    cfg=config();calls=[];start=(2.55,1.,math.pi+math.pi/6)
    n=NS(cfg=cfg,busy=True,lock=threading.Lock(),get_logger=lambda:NS(error=lambda text:None))
    n.inputs=lambda:((start,(0,0,0)),False,grid())
    n.wait_handoff=lambda *args:None
    n.wait_payload_map=lambda *a:n.inputs()
    n.path_message=lambda points,yaw:(points,yaw)
    def run(_h,path,target,carry,_deadline,rotating=False):
        calls.append((path,target,carry,rotating))
        n.inputs=lambda:((target,(0,0,0)),False,grid())
    n.run_track=run
    finished=[];handle=NS(request=NS(pose=stamped(x=1.6,y=1.,yaw=math.pi)),
        is_cancel_requested=False,succeed=lambda:finished.append(True),abort=lambda:finished.append(False))
    result=NavigationServer.execute(n,handle)
    assert finished==[True] and result.error_code==0
    assert calls[0][3] is True and calls[1][3] is False
    assert calls[0][1][:2]==start[:2] and calls[0][1][2]==math.pi

def test_payload_map_wait_is_cancellable_without_track_dispatch():
    n=NS(abort_event=threading.Event());n.abort_event.set()
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True),pytest.raises(ValueError,match='cancelled'):
        NavigationServer.wait_payload_map(n,fake_handle(),time.monotonic()+1)

def test_real_policy_and_targets_reject_unmeasured_values():
    from mission_manager.config_loader import load_config
    folder=project_root()/'src/robot_bringup/config'
    with pytest.raises(ValueError,match='尚未填写'):
        load_config(folder/'mission_robot.yaml',folder/'interfaces.yaml',folder/'targets_robot.yaml','robot')

def test_real_policy_automatic_full_flow():
    from mission_manager.config_loader import read_yaml, validate
    folder=project_root()/'src/robot_bringup/config'
    targets=read_yaml(folder/'targets_rack.yaml');targets['simulation_only']=False
    cfg=validate({'mission':read_yaml(folder/'mission_robot.yaml'),
        'interfaces':read_yaml(folder/'interfaces.yaml'),'targets':targets},'robot')
    assert cfg['mission']['confirm_end_qr'] is False
    assert cfg['mission'].get('manual_step',False) is False
    assert all(set(item)=={'qr'} for item in cfg['targets']['racks'].values())
    assert cfg['mission']['scan_route'] is True
    assert cfg['mission']['qr_timeout_advance'] is False
