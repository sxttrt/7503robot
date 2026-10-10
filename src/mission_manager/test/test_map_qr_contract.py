"""Offline contract integration, without rclpy.init or launched nodes."""
import json
import math
import threading
from types import SimpleNamespace as NS
from unittest.mock import patch
import pytest
import test_framework_endpoints as endpoint_fixture
from test_framework_rack import Backend,config
from mission_manager.framework_execution import FrameworkExecution,scene
from mission_manager.state_machine import Mission
from mission_manager.adapters.qr import QR
from mission_v3 import MissionV3
from frame_geometry import apply,inverse
from nav2_msgs.action import NavigateToPose
from mission_interfaces.action import Dock


def test_map_anchor_initial_identity_and_nonidentity_transform_roundtrip():
    transform=(-2.1,.8,.35)
    for point in ((2.72,1.),(.25,1.05),(.55,.7)):
        assert apply(inverse(transform),apply(transform,point))==pytest.approx(point)
    assert apply(inverse(transform),apply(transform,(1.,2.)))==pytest.approx((1.,2.))


def test_nonidentity_map_goal_converted_before_odometry_controller():
    node=endpoint_fixture.EndpointTests().server();node.busy=True;node.current_payload=None
    transform=(.2,-.1,.01);node.map_from_odom=lambda:transform
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
    goal.pose.pose.position.x=1.4;goal.pose.pose.position.y=.6
    goal.pose.pose.orientation.z=1.;goal.pose.pose.orientation.w=0.
    headings=[];targets=[]
    node.turn=lambda heading:headings.append(heading) or True
    node.move_target=lambda target:targets.append(target) or True
    handle=NS(request=goal,is_cancel_requested=False,succeed=lambda:None)
    result=node.navigate(handle)
    assert result.error_code==0
    assert targets[0]['xy']==pytest.approx(apply(inverse(transform),(1.4,.6)))
    assert abs(math.atan2(math.sin(headings[-1]-(math.pi-.01)),math.cos(headings[-1]-(math.pi-.01))))<1e-9


def test_public_path_points_and_heading_are_in_map_frame():
    node=endpoint_fixture.EndpointTests().server();node.plan_origin=(1.,1.);node.pose=(1.,1.,math.pi)
    transform=(.2,-.1,.1);node.map_from_odom=lambda:transform
    published=[];node.path_pub=NS(publish=published.append)
    node.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:__import__('builtin_interfaces.msg',fromlist=['Time']).Time()))
    with patch.object(MissionV3,'plan_for',return_value=[(2.,1.)]):
        assert node.plan_for({'name':'navigation'})==[(2.,1.)]
    path=published[-1];assert path.header.frame_id=='map'
    for message,point in zip(path.poses,((1.,1.),(2.,1.))):
        assert (message.pose.position.x,message.pose.position.y)==pytest.approx(apply(transform,point))


def test_enter_succeeds_without_private_camera_or_qr_cache():
    node=endpoint_fixture.EndpointTests().server();node.busy=True;node.current_payload=None
    node.turn=lambda heading:True;node.move_target=lambda target:True
    node.select_payload=lambda name:True
    request=Dock.Goal();request.rack_id='B';request.operation='ENTER'
    request.expected_qr=scene.RACKS['B'][5];request.max_duration_sec=60.
    handle=NS(request=request,is_cancel_requested=False,succeed=lambda:None)
    result=node.dock(handle)
    assert result.success and result.ready_to_lift
    assert node.entered_rack=='B'
    assert not hasattr(node,'last_qr')


def test_confirmed_external_qr_advances_state_once_and_does_not_release_motion_gate():
    c=config();assert c['mission']['manual_step'] is False
    assert c['mission']['qr_confirm_frames']==1
    backend=Backend();qr=QR.__new__(QR)
    qr.policy=c['mission'];qr.pending=None;qr.last_frame=None;qr.count=0
    clock=NS(ns=1_000_000_000)
    qr.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=clock.ns)))
    original_start=backend.start
    def start(kind,request,payload,callback):
        original_start(kind,request,payload,callback)
        if kind=='qr':qr.start(request,payload,callback)
    backend.start=start
    mission=Mission(c,backend,clock=lambda:clock.ns/1e9)
    assert mission.arm()[0];assert mission.state=='NAV_START'
    backend.finish(mission);assert mission.state=='WAIT_START'
    def observe(raw,stamp=None):
        clock.ns+=100_000_000;stamp=clock.ns if stamp is None else stamp
        data={'image_stamp':{'sec':stamp//1_000_000_000,'nanosec':stamp%1_000_000_000},
              'status':'ok','detections':[{'raw':raw}]}
        qr.observe(data,clock.ns);mission.tick();return data
    observe('RACKA_XXXX');assert mission.state=='WAIT_START'
    observe('START',1);assert mission.state=='WAIT_START'
    data=observe('START');assert mission.state=='NAV_RACK'
    qr.observe(data,clock.ns);mission.tick();assert mission.state=='NAV_RACK'
    backend.finish(mission);assert mission.state=='WAIT_RACK_QR'
    observe(c['targets']['racks']['A']['qr']);assert mission.state=='DOCK_ENTER'
    backend.finish(mission);assert mission.state=='LIFT_UP'
    backend.finish(mission);assert mission.state=='NAV_END'
    backend.finish(mission);assert mission.state=='LIFT_DOWN'
    backend.finish(mission);assert mission.state=='DOCK_EXIT'
    backend.finish(mission);assert mission.state=='NAV_RACK' and mission.rack=='B'


def test_external_mode_image_heartbeat_does_not_decode_or_publish_qr():
    node=endpoint_fixture.EndpointTests().server();node.use_builtin_qr=False
    node.process_image=lambda message:pytest.fail('external mode decoded image')
    node.on_image(object())
    assert node.camera_seen>0
    assert not hasattr(node,'pending_image')


def test_map_alignment_anchors_radar_field_once_without_slam_dependency():
    from map_alignment import MapAlignment
    from nav_msgs.msg import Odometry
    import time
    odom=Odometry();odom.header.stamp.sec=1
    odom.pose.pose.position.x=2.72;odom.pose.pose.position.y=1.
    published=[]
    node=NS(calibrated=False,odom=odom,odom_seen=time.monotonic(),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000,to_msg=lambda:odom.header.stamp)),
            broadcaster=NS(sendTransform=published.append),get_logger=lambda:NS(info=lambda *a:None))
    MapAlignment.align(node)
    assert node.calibrated and len(published)==1
    result=published[0]
    assert result.header.frame_id=='map' and result.child_frame_id=='odom'
    assert result.transform.translation.x==result.transform.translation.y==0.
    assert result.transform.rotation.w==1.
    MapAlignment.align(node);assert len(published)==1


def test_map_alignment_rejects_stale_or_moving_radar_odom():
    from map_alignment import MapAlignment
    from nav_msgs.msg import Odometry
    import time
    odom=Odometry();odom.header.stamp.sec=1;odom.twist.twist.linear.x=.1
    published=[]
    node=NS(calibrated=False,odom=odom,odom_seen=time.monotonic(),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000,to_msg=lambda:odom.header.stamp)),
            broadcaster=NS(sendTransform=published.append))
    MapAlignment.align(node);assert not published
    odom.twist.twist.linear.x=0.
    node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=3_000_000_000))
    MapAlignment.align(node);assert not published


@pytest.mark.parametrize('initial_missing',[False,True])
def test_map_transition_wait_stops_and_uses_only_fresh_tf(initial_missing):
    from geometry_msgs.msg import TransformStamped
    clock=NS(wall=0.,ros=10.);commands=[];warnings=[]
    def lookup(*args):
        if initial_missing and clock.wall<.1:raise RuntimeError('no TF yet')
        result=TransformStamped()
        result.header.stamp.sec=8 if clock.wall<.1 else 10
        result.header.stamp.nanosec=450_000_000 if clock.wall<.1 else 0
        result.transform.translation.x=99. if clock.wall<.1 else .2
        result.transform.rotation.w=1.
        return result
    node=NS(tf=NS(lookup_transform=lookup),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)),
            pub_vel=lambda:commands.append('stop'),fresh=lambda:True,
            get_logger=lambda:NS(warn=warnings.append),
            spin=lambda dt:setattr(clock,'wall',clock.wall+dt))
    with patch('mission_manager.framework_execution.time.monotonic',lambda:clock.wall), \
         patch('mission_manager.framework_execution.rclpy.ok',lambda:True):
        result=FrameworkExecution.map_from_odom(node)
    assert result==pytest.approx((.2,0.,0.))
    assert commands and warnings and clock.wall>=.1


@pytest.mark.parametrize('sensors_valid',[False,True])
def test_map_transition_wait_is_bounded_and_never_accepts_stale_tf(sensors_valid):
    from geometry_msgs.msg import TransformStamped
    clock=NS(wall=0.);commands=[]
    result=TransformStamped();result.header.stamp.sec=8;result.transform.rotation.w=1.
    node=NS(tf=NS(lookup_transform=lambda *args:result),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)),
            pub_vel=lambda:commands.append('stop'),fresh=lambda:sensors_valid,
            get_logger=lambda:NS(warn=lambda *args:None),
            spin=lambda dt:setattr(clock,'wall',clock.wall+dt))
    with patch('mission_manager.framework_execution.time.monotonic',lambda:clock.wall), \
         patch('mission_manager.framework_execution.rclpy.ok',lambda:True):
        with pytest.raises(ValueError,match='fresh TF wait failed'):
            FrameworkExecution.map_from_odom(node)
    assert commands
    assert clock.wall<=2.1
    if not sensors_valid:assert clock.wall==0.


def test_static_navigation_tf_is_timeless_but_dynamic_tf_remains_checked():
    from geometry_msgs.msg import TransformStamped
    result=TransformStamped();result.transform.rotation.w=1.
    node=NS(tf=NS(lookup_transform=lambda *args:result),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=50_000_000_000)),
            pub_vel=lambda:pytest.fail('valid static TF blocked navigation'))
    with patch('mission_manager.framework_execution.rclpy.ok',lambda:True):
        assert FrameworkExecution.map_from_odom(node)==(0.,0.,0.)


def test_navigation_rejects_old_and_duplicate_odometry():
    from nav_msgs.msg import Odometry
    node=NS(last_odom_stamp=None,odom_seq=0,odom_seen=0.,pose=None,
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)))
    message=Odometry();message.header.frame_id='odom';message.child_frame_id='base_link'
    message.pose.pose.orientation.w=1.;message.header.stamp.sec=9
    MissionV3.on_odom(node,message);assert node.odom_seq==0 and node.pose is None
    message.header.stamp.sec=10
    MissionV3.on_odom(node,message);assert node.odom_seq==1
    seen=node.odom_seen;message.pose.pose.position.x=99.
    MissionV3.on_odom(node,message);assert node.odom_seq==1 and node.pose[0]==0. and node.odom_seen==seen
    message.header.stamp.nanosec=10_000_000;message.child_frame_id='wrong'
    MissionV3.on_odom(node,message);assert node.odom_seq==1


@pytest.mark.parametrize('received,updated,expected',[(9.9,9.9,True),(8.,9.9,False),(9.9,8.,False)])
def test_cached_map_checks_both_receive_and_source_age_without_service_wait(received,updated,expected):
    from nav2_msgs.msg import Costmap
    from geometry_msgs.msg import TransformStamped
    message=Costmap();message.header.frame_id='map'
    message.metadata.resolution=.025;message.metadata.size_x=message.metadata.size_y=2
    message.metadata.origin.orientation.w=1.
    message.metadata.update_time.sec=int(updated)
    message.metadata.update_time.nanosec=int(round((updated-int(updated))*1e9))
    message.data=[0,0,0,0]
    transform=TransformStamped();transform.transform.rotation.w=1.
    node=NS(fresh=lambda:True,costmaps=NS(snapshot=lambda:(message,received,1)),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)),
            tf=NS(lookup_transform=lambda *args:transform),
            request=lambda *a:pytest.fail('blocking service request in control'),
            get_logger=lambda:NS(warn=lambda *a:None))
    with patch('mission_v3.time.monotonic',lambda:10.):
        assert (MissionV3.get_grid(node) is not None)==expected


def test_slam_mapping_cannot_publish_second_navigation_tf():
    from pathlib import Path
    import yaml
    config=yaml.safe_load((Path(scene.__file__).resolve().parent.parent/'config/slam_toolbox.yaml').read_text())
    assert config['slam_toolbox']['ros__parameters']['transform_publish_period']==0.
