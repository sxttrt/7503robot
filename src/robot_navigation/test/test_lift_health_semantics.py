"""跨导航/机构/任务链的离线回归；不启动节点。"""
import json
import sys
from mission_manager.project_paths import project_root
sys.path.insert(0,str(project_root()/'src/mission_manager/test'))
import math
import time
from types import SimpleNamespace as NS
from unittest.mock import patch
import pytest
from std_msgs.msg import String
from robot_navigation.core import health_valid
from robot_navigation.navigation import NavigationServer
from robot_navigation.path_tracker import PathTracker
from mission_manager.hybrid_execution import HybridExecution
from mission_manager.adapters.safety import Safety
from mission_manager.adapters.lift import Lift
from mission_manager.state_machine import Mission
from mission_v3 import MissionV3
from test_costmap_startup import node_double,message,measured_health
from test_real_navigation import stamped

def moving():
    return dict(measured_health(),lift_is_up=None,lift_settled=False,stopped=False)

def nav_node(data):
    n,maps,packets=node_double(message());n.costmap_observed=True;n.base_stamp=4.
    n.get_logger=lambda:NS(warn=lambda text:None)
    n.footprint_pub=NS(publish=lambda msg:pytest.fail('cannot change footprint while lift state is unknown'))
    NavigationServer.on_base(n,String(data=json.dumps(data)))
    return n,maps,packets

def test_lift_moving_packet_is_healthy_but_never_motion_ready():
    data=moving()
    assert health_valid(data,5.,.6,require_lift_settled=False)
    assert not health_valid(data,5.,.6)
    # 已有文档允许的旧发送端 null（没有新可选字段）同样可表示未到位。
    data.pop('lift_settled')
    assert health_valid(data,5.,.6,require_lift_settled=False)
    assert not health_valid(data,5.,.6)

@pytest.mark.parametrize('change',[{'fault':'motor_error'}, {'stamp':{'sec':3,'nanosec':0}},
    {'modules':{'base':True,'lift':False}}, {'lift_state_source':'estimated'},
    {'lift_settled':True}, {'lift_settled':'false'}, {'lift_is_up':0}])
def test_real_fault_stale_or_malformed_packet_still_not_healthy(change):
    data=moving();data.update(change)
    assert not health_valid(data,5.,.6,require_lift_settled=False)

def test_missing_state_is_not_an_explicit_unconfirmed_measurement():
    data=moving();data.pop('lift_is_up')
    assert not health_valid(data,5.,.6,require_lift_settled=False)

def test_navigation_reports_lift_transition_without_fault_or_velocity_permission():
    node,maps,packets=nav_node(moving())
    assert node.base['lift_is_up'] is None and node.payload_stamp==math.inf
    NavigationServer.publish_footprint(node)
    NavigationServer.publish_status(node);status=json.loads(packets[-1].data)
    assert status['healthy'] is True and status['ready'] is False and not status['fault']
    assert 'lift' in status['reason']
    with pytest.raises(ValueError,match='measured'):NavigationServer.inputs(node)
    from rclpy.action import GoalResponse
    assert NavigationServer.accept(node,NS(pose=stamped()))==GoalResponse.REJECT

def hybrid_health(nav_status,lift_settled=False,carrying=False):
    msgs=[];node=NS(initialized=True,busy=not lift_settled,nav_status=nav_status,
        nav_seen=time.monotonic(),camera_seen=time.monotonic(),speed=0.,angular_speed=0.,
        pose=(1.,1.,3.141592653589793),odom_seen=time.monotonic(),status_seen=time.monotonic(),
        status={'ready':True,'lift_settled':lift_settled,'carrying':carrying,'fault':'',
                'actuator_feedback_fresh':True,'base_motion_stopped':True,'lift_motion_stopped':lift_settled},
        get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS(sec=5,nanosec=0))),
        health_pub=NS(publish=lambda p:None),base_feedback_pub=NS(publish=msgs.append),up_pub=NS(publish=lambda msg:None))
    with patch.object(MissionV3,'fresh',lambda *a,**k:True):HybridExecution.health(node)
    return json.loads(msgs[-1].data)

def test_full_feedback_cycle_preserves_health_during_lift_and_recovers_ready():
    node,maps,packets=nav_node(moving())
    NavigationServer.publish_status(node)
    healthy=hybrid_health(json.loads(packets[-1].data))
    assert healthy['modules']['navigation'] and healthy['modules']['lift']
    assert healthy['lift_is_up'] is None and not healthy['lift_settled'] and not healthy['stopped']
    # 该新鲜正常的升降中状态再次输入导航，不能形成 ready=False -> 整机故障的循环。
    healthy['stamp']['nanosec']=100_000_000;node.now=lambda:5.1
    NavigationServer.on_base(node,String(data=json.dumps(healthy)));NavigationServer.publish_status(node)
    assert json.loads(packets[-1].data)['healthy'] and not json.loads(packets[-1].data)['ready']
    done=hybrid_health({'healthy':True,'ready':False},lift_settled=True,carrying=True)
    done['stamp']['nanosec']=200_000_000;node.now=lambda:5.2
    NavigationServer.on_base(node,String(data=json.dumps(done)));NavigationServer.publish_status(node)
    assert json.loads(packets[-1].data)['healthy'] and json.loads(packets[-1].data)['ready']
    assert node.base['lift_is_up'] is True
    # 仍然要等新载货足迹/地图，不因 healthy=true 绕过执行门禁。
    with pytest.raises(ValueError,match='payload change'):NavigationServer.inputs(node)

def test_actual_module_failure_still_propagates_to_framework():
    data=hybrid_health({'healthy':False,'ready':False,'reason':'radar odom stale'})
    assert not data['modules']['navigation'] and 'stale' in data['navigation_diagnostic']
    safety=Safety.__new__(Safety);safety.policy={'health_timeout_sec':2};safety.diagnostic=''
    Safety.on_status(safety,String(data=json.dumps({'ready':False,'fault':'radar odom stale'})))
    assert not safety.healthy() and 'radar odom stale' in safety.diagnostic

def test_map_loss_is_not_hidden_by_legitimate_lift_transition():
    node,maps,packets=nav_node(moving());maps['message']=message(255)
    NavigationServer.publish_status(node)
    assert not json.loads(packets[-1].data)['healthy']

def test_unknown_lift_state_during_startup_cannot_enable_auto_arm():
    node,maps,packets=nav_node(moving());node.costmap_observed=False
    NavigationServer.publish_status(node)
    status=json.loads(packets[-1].data)
    assert not status['healthy'] and not status['ready']

def test_lift_failure_result_is_not_overridden_by_online_feedback():
    completed=[];lift=Lift.__new__(Lift)
    lift.requests={'up':{'up':True,'cancelled':False,'callback':lambda *a:completed.append(a)}}
    Lift.result(lift,'up',NS(result=lambda:NS(success=False,message='physical endpoint not reached')))
    assert completed[0][1] is False and not lift.requests


def test_tracker_cannot_follow_path_when_platform_is_moving():
    # 默认跟踪检查仍严格拒绝机构状态，不能由宽松健康检查放行。
    node,maps,packets=nav_node(moving());node.closing=False
    with pytest.raises(ValueError,match='feedback'):PathTracker.feedback_inputs(node,require_nav=False)

def test_mission_lift_up_does_not_stop_on_healthy_unsettled_feedback():
    from test_qr_timeout_advance import system
    from test_state_machine import advance_to
    mission,backend,clock=system();advance_to(mission,backend,'LIFT_UP')
    data=hybrid_health({'healthy':True,'ready':False})
    safety=Safety.__new__(Safety);safety.policy={'health_timeout_sec':2};safety.diagnostic=''
    Safety.on_status(safety,String(data=json.dumps({'ready':True,'fault':''})))
    backend.is_healthy=safety.healthy();mission.tick()
    assert mission.state=='LIFT_UP' and not backend.stops
    backend.finish();mission.tick();assert mission.state=='NAV_END' and mission.cargo=='UP'
