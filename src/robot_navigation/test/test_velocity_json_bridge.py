"""速度 JSON 与断流/异常处理的离线验证，不启动 ROS 节点或串口。"""
import importlib.util
import json
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest
from geometry_msgs.msg import Twist

from mission_manager.project_paths import project_root
from robot_navigation.core import load_settings
from robot_navigation.velocity_json_bridge import VelocityJsonBridge, encode_velocity, zero_json


def command(vx=0.0, vy=0.0, wz=0.0):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
    return msg


@pytest.mark.parametrize('velocity', [(0.1,0.,0.), (-0.1,0.,0.), (0.,0.1,0.),
                                    (0.,-0.1,0.), (0.,0.,0.2), (0.,0.,-0.2), (0.,0.,0.)])
def test_command_keeps_body_axis_signs_si_units_and_exact_schema(velocity):
    vx,vy,wz = velocity
    result = json.loads(encode_velocity(command(*velocity), .15, .2))
    assert result == {'linear': {'x':vx,'y':vy,'z':0.}, 'angular': {'x':0.,'y':0.,'z':wz}}
    assert isinstance(result['linear']['x'],float)


@pytest.mark.parametrize('component', ['linear.x','linear.y','linear.z','angular.x','angular.y','angular.z'])
@pytest.mark.parametrize('bad', [float('nan'),float('inf'),-float('inf')])
def test_nonfinite_input_cannot_produce_nonstandard_json(component,bad):
    msg = command()
    vector, axis = component.split('.')
    setattr(getattr(msg,vector),axis,bad)
    with pytest.raises(ValueError,match='finite'):encode_velocity(msg,.15,.2)


@pytest.mark.parametrize('component', ['linear.z','angular.x','angular.y'])
def test_unsupported_axis_is_rejected_instead_of_silently_discarded(component):
    msg=command();vector,axis=component.split('.')
    setattr(getattr(msg,vector),axis,.01)
    with pytest.raises(ValueError,match='supported'):encode_velocity(msg,.15,.2)


@pytest.mark.parametrize('velocity',[(.151,0.,0.),(0.,-.151,0.),(0.,0.,.201),(.12,.12,0.)])
def test_excessive_velocity_does_not_bypass_tracking_limits(velocity):
    with pytest.raises(ValueError,match='limits'):encode_velocity(command(*velocity),.15,.2)


def bridge():
    packets=[];errors=[]
    node=NS(cfg={'max_linear_speed':.15,'max_angular_speed':.2,'tracking_command_timeout_sec':.3},
        last_received=None,last_error='',closing=False,
        output=NS(publish=lambda msg:packets.append(json.loads(msg.data))),
        get_logger=lambda:NS(error=errors.append))
    node.publish_zero=lambda:VelocityJsonBridge.publish_zero(node)
    return node,packets,errors


def test_valid_source_command_is_forwarded_immediately_without_extra_fields():
    node,packets,errors=bridge()
    with patch('robot_navigation.velocity_json_bridge.time.monotonic',return_value=10.):
        VelocityJsonBridge.on_twist(node,command(-.1,0.,0.))
        VelocityJsonBridge.watchdog(node)
    assert len(packets)==1 and packets[0]['linear']['x']==-.1
    assert node.last_received==10. and not errors


def test_watchdog_never_replays_last_nonzero_velocity():
    node,packets,errors=bridge()
    with patch('robot_navigation.velocity_json_bridge.time.monotonic',return_value=10.):
        VelocityJsonBridge.on_twist(node,command(.1))
    with patch('robot_navigation.velocity_json_bridge.time.monotonic',return_value=10.299):
        VelocityJsonBridge.watchdog(node)
    assert len(packets)==1
    with patch('robot_navigation.velocity_json_bridge.time.monotonic',return_value=10.301):
        for _ in range(3):VelocityJsonBridge.watchdog(node)
    assert len(packets)==4 and all(p==json.loads(zero_json()) for p in packets[1:])


def test_startup_and_shutdown_only_emit_zero_until_a_valid_new_command():
    node,packets,errors=bridge()
    VelocityJsonBridge.watchdog(node)
    node.closing=True
    VelocityJsonBridge.on_twist(node,command(.1))
    VelocityJsonBridge.watchdog(node)
    assert len(packets)==3 and all(p==json.loads(zero_json()) for p in packets)


def test_invalid_command_clears_previous_fresh_command_and_recovers_only_on_new_valid_input():
    node,packets,errors=bridge()
    with patch('robot_navigation.velocity_json_bridge.time.monotonic',return_value=10.):
        VelocityJsonBridge.on_twist(node,command(.1))
        VelocityJsonBridge.on_twist(node,command(float('nan')))
        VelocityJsonBridge.watchdog(node)
        VelocityJsonBridge.on_twist(node,command(0.,-.05))
    assert len(packets)==4 and len(errors)==1
    assert packets[1]==packets[2]==json.loads(zero_json())
    assert packets[-1]['linear']['y']==-.05


def test_config_defaults_preserve_existing_config_files(tmp_path):
    import yaml
    cfg=load_settings(project_root()/'src/robot_navigation/config/navigation_robot.yaml')
    cfg.pop('cmd_vel_json_topic')
    path=tmp_path/'old.yaml';path.write_text(yaml.safe_dump(cfg))
    assert load_settings(path)['cmd_vel_json_topic']=='base/cmd_vel_json'


@pytest.mark.parametrize('name',['cmd_vel_topic','cmd_vel_json_topic'])
@pytest.mark.parametrize('bad',[None,'',False,' '])
def test_invalid_topic_configuration_cannot_start_bridge(name,bad,tmp_path):
    import yaml
    cfg=load_settings(project_root()/'src/robot_navigation/config/navigation_robot.yaml');cfg[name]=bad
    path=tmp_path/'invalid.yaml';path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError,match=name):load_settings(path)


def test_real_and_hybrid_launch_include_same_converter():
    # Construct only launch descriptions, not nodes/executors. ROS launch Node actions
    # expand to the same registered console executable in both environments.
    for filename in ('robot.launch.py','rack_hybrid_backend.launch.py'):
        path=project_root()/'src/robot_bringup/launch'/filename
        spec=importlib.util.spec_from_file_location('velocity_launch_'+filename,path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        description=module.generate_launch_description()
        executables=[getattr(entity,'node_executable',[]) for entity in description.entities]
        values=[executable if isinstance(executable,str) else
                ''.join(getattr(part,'text','') for part in executable) for executable in executables]
        assert values.count('velocity_json_bridge')==1
