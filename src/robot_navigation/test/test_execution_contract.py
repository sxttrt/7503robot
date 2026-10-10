"""Offline cross-module regression: physics topology, actuator gate and handoff.

No ROS context/node or Gazebo server is created. Real classes are invoked on
test doubles; plugin mechanism state is covered by the C++ actuator_core_test.
"""
import ast
import copy
import json
import math
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import pytest
from std_msgs.msg import String
from robot_navigation.core import measured_base_stopped,health_valid
from robot_navigation.navigation import NavigationServer
from robot_navigation.path_tracker import PathTracker
from mission_manager.project_paths import project_root
from test_real_navigation import config

def health(stamp=10.,**changes):
    data=dict(stamp=stamp if isinstance(stamp,dict) else {'sec':int(stamp),'nanosec':int((stamp%1)*1e9)},
              lift_state_source='measured',lift_is_up=False,lift_settled=True,
              modules={'base':True,'lift':True},stopped=True,fault='',
              base_motion_stopped=True,lift_motion_stopped=True)
    data.update(changes);return data

@pytest.mark.parametrize('lift',[None,True,False])
def test_stop_measurement_survives_unknown_payload_or_latched_fault(lift):
    data=health(lift_is_up=lift,lift_settled=False,stopped=False,fault='actuator error',modules={'base':False,'lift':False})
    assert not health_valid(data,10.,.6)
    assert measured_base_stopped(data,10.,.6)

@pytest.mark.parametrize('changes',[{'base_motion_stopped':False},{'base_motion_stopped':1},
    {'lift_state_source':'estimated'},{'stamp':{'sec':9,'nanosec':0}},{'stamp':{'sec':12,'nanosec':0}}])
def test_stop_requires_current_real_evidence(changes):
    assert not measured_base_stopped(health(**changes),10.,.6)

def test_planar_model_has_no_free_z_roll_pitch_dof():
    world=ET.parse(project_root()/'navigation/worlds/rack_world.sdf').getroot().find('world')
    robot=world.find("model[@name='robot']")
    assert robot.attrib['canonical_link']=='base_link'
    chain=[('chassis_x','world','x_slide','prismatic','-1 0 0'),
           ('chassis_y','x_slide','y_slide','prismatic','0 -1 0'),
           ('chassis_yaw','y_slide','base_link','revolute','0 0 1')]
    for name,parent,child,kind,axis in chain:
        j=robot.find(f"joint[@name='{name}']")
        assert j.attrib['type']==kind and j.findtext('parent')==parent and j.findtext('child')==child
        assert j.findtext('axis/xyz')==axis and j.find('axis/xyz').attrib['expressed_in']=='__model__'
    lift=robot.find("joint[@name='lift_joint']")
    assert lift.attrib['type']=='prismatic' and lift.findtext('parent')=='base_link'
    assert len(robot.findall('joint'))==4
    assert not robot.findall("plugin[@name='gz::sim::systems::PosePublisher']")
    for name in ('a','b','c','d'):
        assert world.find(f"model[@name='rack_{name}']").findtext('static')=='true'
    text=(project_root()/'navigation/scripts/kinematic_driver.py').read_text()
    assert 'SetEntityPose' not in text and 'teleport(' not in text

def test_handoff_waits_for_new_idle_feedback_before_motion_dispatch():
    cfg=config();cfg['cancel_timeout_sec']=2.;now=[10.]
    node=NS(cfg=cfg,abort_event=threading.Event(),now=lambda:now[0],
            base=health(),base_stamp=9.,odom_stamp=9.)
    calls=[]
    node.inputs=lambda **kw:(((1.,1.,math.pi),(0.,0.,0.)),False,None)
    def tick(_):
        now[0]+=.1;node.odom_stamp=now[0];node.base_stamp=now[0]
        # Dock's old busy sample can arrive after its successful action result.
        node.base['stopped']=now[0]>=10.3;calls.append(now[0])
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True),patch('robot_navigation.navigation.time.sleep',side_effect=tick):
        NavigationServer.wait_handoff(node,NS(is_cancel_requested=False),time.monotonic()+2)
    assert len(calls)>=7 and now[0]>=10.7

def test_cancel_during_handoff_cannot_dispatch_tracking():
    node=NS(cfg=config(),now=lambda:10.,abort_event=threading.Event())
    node.abort_event.set()
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True),pytest.raises(ValueError,match='cancelled'):
        NavigationServer.wait_handoff(node,NS(is_cancel_requested=False),time.monotonic()+2)

def test_navigation_stop_confirms_base_even_when_lift_not_ready_or_faulted():
    node=NS(cfg=config(),now=lambda:10.,lock=threading.Lock(),abort_event=threading.Event(),
      base=None,stop_base=health(lift_is_up=None,lift_settled=False,stopped=False,fault='lift fault'),
      base_stamp=10.,busy=False,track=NS(idle=lambda:True),fault='prior tracking fault',
      odom=(((1.,1.,math.pi)),(0.,0.,0.)),odom_stamp=10.,odom_received=time.monotonic())
    response=NS(success=False,message='')
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        NavigationServer.stop(node,None,response)
    assert response.success

def test_tracker_stop_uses_measured_base_and_radar_while_cargo_unknown():
    g=NS(stop_linear_speed=.01,stop_angular_speed=.01,settle_duration_sec=.4)
    node=NS(cfg=config(),now=lambda:11.,base=None,stop_base=health(stamp=10.5,lift_is_up=None,lift_settled=False,stopped=False),
            odom=((1.,1.,math.pi),(0.,0.,0.)),odom_stamp=10.5,base_stamp=10.5,odom_received=time.monotonic())
    record=dict(stop_stamp=10.,stable_since=None,stable_frames=0,last_stamp=-math.inf,handle=NS(request=g))
    for stamp in (10.5,10.7,10.99):
        node.now=lambda:stamp+.01
        node.odom_stamp=stamp;node.base_stamp=stamp;node.stop_base=health(stamp=stamp,lift_is_up=None,stopped=False)
        stopped=PathTracker.stopped_after(node,record)
    assert stopped

def driver_double():
    # Load methods without initializing rclpy.Node.
    p=project_root()/'navigation/scripts/kinematic_driver.py';tree=ast.parse(p.read_text())
    cls=next(x for x in tree.body if isinstance(x,ast.ClassDef));cls.bases=[]
    cls.body=[x for x in cls.body if not isinstance(x,ast.FunctionDef) or x.name!='__init__']
    ns=dict(time=time,math=math,json=json,scene=__import__('scene'),
            Twist=lambda:NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.)),
            String=lambda **kw:NS(**kw),Bool=lambda **kw:NS(**kw))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'<driver-double>','exec'),ns)
    node=ns['KinematicDriver']();node.cmd=ns['Twist']();node.last_cmd=0.;node.fault=None
    node.now=lambda:10.;node.actuator=None;node.actuator_stamp=-math.inf;node.actuator_seen=0.
    node.lidar_valid=True;node.lidar_seen=node.lidar_status_seen=time.monotonic();node.allow_scan_rotation=True
    commands=[];packets=[];node.command_pub=NS(publish=commands.append);node.status_pub=NS(publish=packets.append)
    node.carry_pub=NS(publish=lambda m:None);node.get_logger=lambda:NS(error=lambda m:None)
    return node,commands,packets

def plugin_sample():
    return dict(stamp={'sec':10,'nanosec':0},source='gazebo_planar_actuator',selected='rack_b',
       carrying=False,engaged=False,lift=0.,lift_target=0.,lift_settled=True,
       base_motion_stopped=True,lift_motion_stopped=True,ready=True,fault='',
       base_pose=[.25,1.05,0.,math.pi],relative_xy=[0.,0.],payload_center=[.25,1.05,0.])

def test_ros_watchdog_and_stale_plugin_cannot_emit_motion():
    node,commands,packets=driver_double();node.on_actuator(String(data=json.dumps(plugin_sample())))
    assert node.ready();node.cmd.linear.x=.3;node.last_cmd=time.monotonic()-1.
    node.tick();assert commands[-1].linear.x==0.
    node.cmd.linear.x=.3;node.last_cmd=time.monotonic();node.actuator_seen-=1.
    node.tick();assert commands[-1].linear.x==0.
    report=json.loads(packets[-1].data)
    assert not report['actuator_feedback_fresh'] and not report['base_motion_stopped']

def test_repeated_source_stamp_cannot_refresh_plugin_feedback():
    node,_,_=driver_double();msg=String(data=json.dumps(plugin_sample()));node.on_actuator(msg)
    node.actuator_seen=1.;node.on_actuator(msg);assert node.actuator_seen==1.

@pytest.mark.parametrize('value',[float('nan'),float('inf'),-.5])
def test_ros_gate_rejects_nonfinite_or_excessive_speed(value):
    node,commands,_=driver_double();node.cmd.linear.x=value
    node.on_cmd(node.cmd);assert node.fault and commands[-1].linear.x==0.

def test_fault_does_not_falsely_erase_measured_carrying_evidence():
    node,commands,packets=driver_double();sample=plugin_sample();sample.update(carrying=True,engaged=True,lift=.3,lift_target=.3)
    node.on_actuator(String(data=json.dumps(sample)));node.set_fault('bad command');node.publish_status()
    report=json.loads(packets[-1].data)
    assert report['carrying'] and not report['ready'] and report['base_motion_stopped']
