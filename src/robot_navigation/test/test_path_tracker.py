"""无节点离线测试：闭环积分台架、异常目标、数据断流与停车结果。"""
import math
import threading
import time
from types import SimpleNamespace as NS
import pytest
from geometry_msgs.msg import PoseStamped
from mission_interfaces.action import TrackPath
from mission_manager.project_paths import project_root
from robot_navigation.core import load_settings,wrap
from robot_navigation.tracking_control import validate_goal,PathController,body_velocity
from robot_navigation.path_tracker import PathTracker

def cfg():
    c=load_settings(project_root()/'src/robot_bringup/config/navigation_robot.yaml')
    c['calibration_verified']=True;return c

def goal(points=((1,1),(1.3,1),(1.3,1.4)),yaw=math.pi,rotation=False,carrying=False):
    g=TrackPath.Goal();g.request_id='test';g.path.header.frame_id='map'
    for x,y in points:
        p=PoseStamped();p.header.frame_id='map';p.pose.position.x=float(x);p.pose.position.y=float(y)
        p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);g.path.poses.append(p)
    g.carrying=carrying;g.allow_in_place_rotation=rotation
    for key in ('max_linear_speed','max_angular_speed','position_tolerance','yaw_tolerance',
                'stop_linear_speed','stop_angular_speed','settle_duration_sec','max_duration_sec'):
        setattr(g,key,float(cfg()[key]))
    return g

def simulate(controller,initial,steps=1000):
    pose=list(initial);vel=(0,0,0);commands=[];stamp=10.
    for _ in range(steps):
        command=controller.step(tuple(pose),vel,stamp,
            math.hypot(*vel[:2])<.001 and abs(vel[2])<.001,.05)
        vx,vy,wz=command;c,s=math.cos(pose[2]),math.sin(pose[2])
        wx,wy=c*vx-s*vy,s*vx+c*vy
        commands.append((wx,wy,wz))
        pose[0]+=wx*.05;pose[1]+=wy*.05;pose[2]=wrap(pose[2]+wz*.05)
        vel=command;stamp+=.05
        if controller.done:return pose,commands
    raise AssertionError('controller did not converge')

def test_feedback_loop_tracks_corner_and_stops():
    c=PathController(goal(),cfg());pose,commands=simulate(c,(1,1,math.pi))
    assert math.dist(pose[:2],(1.3,1.4))<=c.goal.position_tolerance
    assert abs(wrap(pose[2]-math.pi))<1e-9
    assert all(abs(x)<1e-8 or abs(y)<1e-8 for x,y,_ in commands)
    assert all(w==0 for _,_,w in commands)
    assert commands[-1]==(0,0,0)

def test_cross_track_drift_is_corrected_without_diagonal_commands():
    c=PathController(goal(((1,1),(1.3,1))),cfg())
    pose,commands=simulate(c,(1,1.009,math.pi))
    assert math.dist(pose[:2],(1.3,1))<=c.goal.position_tolerance
    assert any(abs(y)>.001 for _,y,_ in commands)
    assert all(abs(x)<1e-8 or abs(y)<1e-8 for x,y,_ in commands)

def test_small_heading_noise_does_not_stop_every_control_tick():
    c=PathController(goal(((1,1),(1.3,1))),cfg())
    first=c.step((1,1,math.pi),(0,0,0),1,True,.05)
    second=c.step((1.001,1,math.pi+.002),first,1.1,False,.05)
    assert math.hypot(*second[:2])>0 and second[2]==0

def test_empty_rotation_only_angular_and_wraps_pi():
    target=math.pi+math.pi/6
    c=PathController(goal(((1,1),(1,1)),target,True),cfg())
    pose,commands=simulate(c,(1,1,math.pi))
    assert abs(wrap(pose[2]-target))<=c.goal.yaw_tolerance
    assert all(x==0 and y==0 for x,y,_ in commands)

def test_body_frame_conversion_for_head_minus_x():
    vx,vy=body_velocity(.1,0,math.pi)
    assert vx==pytest.approx(-.1) and vy==pytest.approx(0)

@pytest.mark.parametrize('kind',['diagonal','empty','loaded_rotation','mixed_heading',
    'bad_frame','nonfinite','overspeed','bad_timeout'])
def test_invalid_goals_rejected(kind):
    g=goal()
    if kind=='diagonal':g.path.poses[1].pose.position.y=1.2
    elif kind=='empty':g.path.poses=[]
    elif kind=='loaded_rotation':g.carrying=True;g.allow_in_place_rotation=True
    elif kind=='mixed_heading':g.path.poses[0].pose.orientation.z=0.;g.path.poses[0].pose.orientation.w=1.
    elif kind=='bad_frame':g.path.header.frame_id='odom'
    elif kind=='nonfinite':g.path.poses[0].pose.position.x=float('nan')
    elif kind=='overspeed':g.max_linear_speed=2.
    elif kind=='bad_timeout':g.max_duration_sec=0.
    with pytest.raises(ValueError):validate_goal(g,cfg())

def test_duplicate_frames_cannot_finish_single_point():
    c=PathController(goal(((1,1),)),cfg())
    for _ in range(30):c.step((1,1,math.pi),(0,0,0),5.,True,.05)
    assert not c.done
    c.step((1,1,math.pi),(0,0,0),5.2,True,.05)
    c.step((1,1,math.pi),(0,0,0),5.5,True,.05)
    assert c.done

def test_measured_moving_blocks_success():
    c=PathController(goal(((1,1),)),cfg())
    for i in range(20):c.step((1,1,math.pi),(.02,0,0),5+i*.1,False,.05)
    assert not c.done

def test_heading_drift_aborts_translation():
    c=PathController(goal(),cfg())
    with pytest.raises(ValueError,match='heading'):c.step((1,1,math.pi+.1),(0,0,0),5,True,.05)

class FakeTracker:
    def __init__(self):
        self.cfg=cfg();self.lock=threading.RLock();self.busy=True;self.fault=''
        self.odom=((1.,1.,math.pi),(0.,0.,0.));self.odom_stamp=5.;self.odom_received=time.monotonic()
        self.base={'stamp':{'sec':5,'nanosec':0},'modules':{'base':True,'lift':True},
            'stopped':True,'lift_is_up':False,'lift_state_source':'measured','fault':''}
        self.base_stamp=5.;self.base_received=time.monotonic()
        self.handle=NS(request=goal(),is_cancel_requested=False,publish_feedback=lambda f:None)
        self.active={'handle':self.handle,'controller':PathController(self.handle.request,self.cfg),
            'done':threading.Event(),'started':time.monotonic(),'reason':'','result':None}
        self.last_tick=time.monotonic();self.commands=[];self.command=NS(publish=self.commands.append)
        self.zero=lambda:PathTracker.zero(self)
        self.begin_stop=lambda *args:PathTracker.begin_stop(self,*args)
        self.stopped_after=lambda *args:PathTracker.stopped_after(self,*args)
        self.finish=lambda *args:PathTracker.finish(self,*args)
    def now(self):return self.odom_stamp
    def feedback_inputs(self):raise ValueError('radar odom stale')

def test_feedback_loss_commands_zero_and_waits_measured_stop():
    n=FakeTracker();PathTracker.tick(n)
    assert n.commands and all(m.linear.x==m.linear.y==m.angular.z==0 for m in n.commands)
    assert not n.active['done'].is_set()
    for stamp in (5.1,5.3,5.6):
        n.odom_stamp=stamp;n.odom_received=time.monotonic();n.base_stamp=stamp
        n.base['stamp']={'sec':int(stamp),'nanosec':int(stamp%1*1e9)}
        PathTracker.tick(n)
    result=n.active['result'];assert result.stopped and not result.success
    assert result.final_pose.header.frame_id=='map'

def test_no_feedback_no_false_stop_success():
    n=FakeTracker();PathTracker.tick(n)
    n.active['stop_started']=time.monotonic()-n.cfg['cancel_timeout_sec']-1
    PathTracker.tick(n)
    assert n.active['done'].is_set() and not n.active['result'].stopped and n.fault

def test_idle_tracker_does_not_override_dock():
    n=FakeTracker();n.active=None;PathTracker.tick(n);assert not n.commands

def test_cancel_sends_zero_immediately():
    n=FakeTracker();PathTracker.cancel(n,None)
    assert n.active['reason']=='cancelled' and n.commands

def test_nav_heartbeat_loss_rejected():
    c=cfg();now=time.monotonic()
    n=NS(cfg=c,fault='',closing=False,odom=((1,1,math.pi),(0,0,0)),odom_stamp=5.,
        odom_received=now,base=FakeTracker().base,base_received=now,
        lidar={'valid':True},lidar_received=now,lidar_stamp=5.,nav={'ready':True},nav_stamp=5.,
        nav_received=now-1,now=lambda:5.)
    with pytest.raises(ValueError,match='heartbeat'):PathTracker.feedback_inputs(n)


def test_changing_axis_waits_for_wheel_feedback():
    c=PathController(goal(((1,1),(1.3,1))),cfg())
    c.step((1,1,math.pi),(0,0,0),1,True,.05)
    c.step((1.01,1.009,math.pi),(-.01,0,0),1.1,False,.05)
    command=c.step((1.01,1.009,math.pi),(0,0,0),1.2,False,.05)
    assert command==(0,0,0) and c.phase=='AXIS_STOP'

def test_repeated_health_stamp_does_not_refresh_heartbeat():
    from std_msgs.msg import String
    import json
    n=NS(lock=threading.RLock(),cfg=cfg(),nav={'ready':True},nav_stamp=5.,nav_received=1.,now=lambda:5.1)
    data={'stamp':{'sec':5,'nanosec':0},'ready':True}
    PathTracker.on_health(n,String(data=json.dumps(data)),'nav')
    assert n.nav_received==1.


def test_no_motion_without_active_navigation_monitor():
    n=FakeTracker();n.nav={'active':False}
    n.feedback_inputs=lambda:(n.odom,n.base)
    PathTracker.tick(n)
    assert not n.active['done'].is_set() and not n.active['reason']
    assert n.commands and all(m.linear.x==m.linear.y==m.angular.z==0 for m in n.commands)
    n.active['started']=time.monotonic()-n.cfg['feedback_timeout_sec']-.1
    PathTracker.tick(n)
    assert 'monitor not active' in n.active['reason']
