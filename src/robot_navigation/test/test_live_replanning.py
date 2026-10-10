"""Stopped replanning contract; no ROS initialization or live processes."""
import math
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import patch
import pytest
from nav_msgs.msg import Path
from robot_navigation.navigation import NavigationServer,RouteChanged
from test_real_navigation import FakeNav,config,grid,stamped,fake_handle,track_path

@pytest.mark.parametrize('confirmed',[True,False])
def test_route_change_requires_terminal_measured_stop(confirmed):
    node=FakeNav('cancel');node.abort_event.clear()
    def send(request,goal,callback):node.callback=callback
    node.track.send=send
    if not confirmed:node.mode='silent_cancel'
    def check(*args):
        node.checks+=1
        if node.checks>1:raise RouteChanged('live radar route blocked')
    node.check_route=check
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        with pytest.raises(ValueError) as caught:
            NavigationServer.run_track(node,fake_handle(),track_path(),(1.,1.,math.pi),False,time.monotonic()+1)
    assert node.cancelled
    assert isinstance(caught.value,RouteChanged)==confirmed
    assert bool(node.fault)==(not confirmed)

def test_same_navigation_request_replans_more_than_mission_retry_limit():
    cfg=config();events=[];start=(1.,1.,math.pi);message=grid();outcomes=[]
    node=NS(cfg=cfg,busy=True,lock=threading.Lock(),abort_event=threading.Event(),
        get_logger=lambda:NS(info=lambda text:None,error=lambda text:None),
        inputs=lambda:((start,(0,0,0)),False,message),path_message=lambda points,yaw:points)
    node.wait_payload_map=lambda *args:node.inputs()
    node.wait_handoff=lambda *args:events.append('measured_stop')
    def fresh(*args):events.append('fresh_scan_map');return start,message
    node.wait_post_stop_map=fresh
    calls=[]
    def track(handle,path,target,carrying,deadline):
        calls.append((target,deadline));events.append('track')
        if len(calls)<=3:raise RouteChanged('live radar route blocked')
    node.run_track=track
    handle=NS(request=NS(pose=stamped(x=2.,y=1.)),is_cancel_requested=False,
        succeed=lambda:outcomes.append('succeeded'),abort=lambda:outcomes.append('aborted'))
    result=NavigationServer.execute(node,handle)
    assert result.error_code==0 and outcomes==['succeeded']
    assert len(calls)==4 and len({deadline for target,deadline in calls})==1
    assert all(target==(2.,1.,math.pi) for target,deadline in calls)
    assert events==['measured_stop','track']+['measured_stop','fresh_scan_map','track']*3

def test_completed_segments_not_checked_but_remaining_segment_is():
    node=FakeNav('cancel');node.abort_event.clear();seen=[]
    path=Path();path.header.frame_id='map';path.poses=[stamped(x=1.),stamped(x=1.5),stamped(x=2.)]
    def send(request,goal,callback):
        node.callback=callback;node.track.feedback={'request_id':request,'segment_index':1}
    node.track.send=send
    def check(points,*args):
        seen.append(points)
        if len(seen)>1:raise RouteChanged('live radar route blocked')
    node.check_route=check
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True),pytest.raises(RouteChanged):
        NavigationServer.run_track(node,fake_handle(),path,(2.,1.,math.pi),False,time.monotonic()+1)
    assert seen==[[(1.,1.),(1.5,1.),(2.,1.)],[(1.5,1.),(2.,1.)]]

def test_post_stop_map_wait_rejects_cancellation():
    node=NS(now=lambda:5.,abort_event=threading.Event())
    node.abort_event.set()
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True),pytest.raises(ValueError,match='cancelled'):
        NavigationServer.wait_post_stop_map(node,fake_handle(),time.monotonic()+1,False)

def test_post_stop_map_wait_requires_new_scan_and_its_map():
    cfg=config();message=grid();message.metadata.update_time.sec=5;message.metadata.update_time.nanosec=500000000
    node=NS(now=lambda:5.,abort_event=threading.Event(),odom_stamp=5.1,lidar={'stamp':{'sec':5,'nanosec':100000000}})
    frames=[]
    def inputs():
        frames.append(1)
        if len(frames)==2:node.odom_stamp=5.6
        if len(frames)==3:message.metadata.update_time.nanosec=700000000
        return (((1.,1.,math.pi),(0,0,0)),False,message)
    node.inputs=inputs
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        pose,msg=NavigationServer.wait_post_stop_map(node,fake_handle(),time.monotonic()+1,False)
    assert len(frames)==3 and msg is message

def test_new_obstacle_behind_current_segment_does_not_cancel_future_motion():
    cfg=config();message=grid();message.data[40*122+60]=254 # x=1.50, y=1.00, behind
    node=NS(cfg=cfg,inputs=lambda:(((1.8,1.,math.pi),(0,0,0)),False,message))
    NavigationServer.check_route(node,[(1.,1.),(2.,1.)],False)

def test_projection_does_not_hide_tracker_corridor_departure():
    cfg=config();message=grid()
    node=NS(cfg=cfg,inputs=lambda:(((1.8,1.02,math.pi),(0,0,0)),False,message))
    with pytest.raises(ValueError,match='tracker left planned corridor'):
        NavigationServer.check_route(node,[(1.,1.),(2.,1.)],False)
