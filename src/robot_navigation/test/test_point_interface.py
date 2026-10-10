"""新版框架点位接口、异步交接和实机/仿真共同扫描点；不创建 ROS 节点。"""
import math
import threading
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest
from builtin_interfaces.msg import Time
from mission_interfaces.action import NavigateToPoint
from nav2_msgs.action import NavigateToPose
from robot_navigation.point_targets import PointTargets, PointGoalHandle, stamped_target, point_result
from robot_navigation.navigation import NavigationServer


def targets(sim=False):
    return PointTargets({'simulation_only':sim,
        'points':{'A':[1.,1.,math.pi], 'START_SCAN':[2.,1.,math.pi+math.pi/6]},
        'drop_by_rack':{k:[.25,.4+i*.3,math.pi] for i,k in enumerate('ABCD')}})


def context(rack='A'):
    return {'state':'NAV_END','cargo':'UP','rack':rack}


def test_point_id_resolves_without_framework_coordinates():
    assert targets().resolve('A',carrying=False)==(1.,1.,math.pi)


@pytest.mark.parametrize('name',['FINAL_SCAN','unknown','', 'a'])
def test_real_rejects_unconfigured_and_unknown_names(name):
    with pytest.raises(ValueError):targets().resolve(name,carrying=False)


def test_simulation_start_keeps_left_30_degrees():
    p=targets(True).resolve('START_SCAN',carrying=False,simulation=True)
    assert p[2]==math.pi+math.pi/6


def test_real_start_uses_measured_coordinates_with_same_scan_heading():
    assert targets().resolve('START_SCAN',carrying=False)[2]==math.pi+math.pi/6


def test_real_never_accepts_simulation_point_file():
    with pytest.raises(ValueError,match='实机'):targets(True).resolve('A',carrying=False)


@pytest.mark.parametrize('rack',list('ABCD'))
def test_drop_off_uses_confirmed_current_rack_not_call_count(rack):
    table=targets()
    assert table.resolve('DROP_OFF',carrying=True,context=context(rack))==tuple(table.drops[rack])


@pytest.mark.parametrize('ctx',[None,{}, {'state':'NAV_RACK','cargo':'UP','rack':'A'},
    {'state':'NAV_END','cargo':'EMPTY','rack':'A'}])
def test_drop_off_does_not_guess_missing_or_wrong_context(ctx):
    with pytest.raises(ValueError,match='上下文'):targets().resolve('DROP_OFF',carrying=True,context=ctx)


def test_drop_off_empty_and_rack_approach_loaded_rejected():
    with pytest.raises(ValueError):targets().resolve('DROP_OFF',carrying=False,context=context())
    with pytest.raises(ValueError):targets().resolve('A',carrying=True)


@pytest.mark.parametrize('pose',[None,[1.,float('nan'),0.],[1,2], [True,1.,0.]])
def test_invalid_measured_points_cannot_resolve(pose):
    with pytest.raises(ValueError):
        t=PointTargets({'simulation_only':False,'points':{'A':pose}})
        t.resolve('A',carrying=False)


def test_public_result_and_feedback_keep_new_action_schema():
    packets=[];terminal=[]
    h=NS(request=NS(target_id='A'),is_cancel_requested=False,
        publish_feedback=packets.append,succeed=lambda:terminal.append(True))
    facade=PointGoalHandle(h,stamped_target((1.,1.,math.pi),Time(sec=1,nanosec=0)))
    assert isinstance(facade.request,NavigateToPose.Goal)
    feedback=NavigateToPose.Feedback();feedback.distance_remaining=.2
    facade.publish_feedback(feedback);facade.succeed()
    assert isinstance(packets[0],NavigateToPoint.Feedback) and 'A' in packets[0].phase and terminal
    result=NavigateToPose.Result();assert point_result(result).success
    result.error_code=1;result.error_msg='stopped failure';assert not point_result(result).success


def point_node():
    n=NS(cfg={'cancel_timeout_sec':.1,'field_size':[3.02,2.]},lock=threading.Lock(),busy=True,
        abort_event=threading.Event(),point_targets=targets(),mission_context=context(),
        mission_seen=__import__('time').monotonic(),get_parameter=lambda k:NS(value=False),
        get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(sec=1,nanosec=0))))
    n.inputs=lambda **kw:(((1.,1.,math.pi),(0.,0.,0.)),True,None)
    calls=[]
    def execute(facade):
        calls.append(facade.request.pose);facade.succeed();n.busy=False
        return NavigateToPose.Result()
    n.execute=execute
    done=[]
    h=NS(request=NS(target_id='DROP_OFF'),is_cancel_requested=False,
        succeed=lambda:done.append('success'),abort=lambda:done.append('abort'),
        canceled=lambda:done.append('cancel'),publish_feedback=lambda f:None)
    return n,h,calls,done


def test_lift_completion_before_feedback_waits_without_dispatching_velocity():
    n,h,calls,done=point_node();attempts=[]
    def inputs(**kw):
        attempts.append(True)
        if len(attempts)==1:raise ValueError('fresh measured base/lift feedback required')
        return (((1.,1.,math.pi),(0.,0.,0.)),True,None)
    n.inputs=inputs
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        result=NavigationServer.execute_point(n,h)
    assert result.success and len(attempts)==2 and len(calls)==1 and done==['success']


def test_drop_waits_for_mission_context_received_after_action_request():
    n,h,calls,done=point_node();n.mission_context=None
    def update(_seconds):n.mission_context=context('B')
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True),patch(
            'robot_navigation.navigation.time.sleep',side_effect=update):
        result=NavigationServer.execute_point(n,h)
    assert result.success and calls[0].pose.position.y==pytest.approx(.7)


def test_cancel_during_point_handoff_never_starts_pose_navigation():
    n,h,calls,done=point_node();h.is_cancel_requested=True
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        result=NavigationServer.execute_point(n,h)
    assert not result.success and not calls and done==['cancel'] and not n.busy


def test_failed_sensor_handoff_has_bounded_timeout_without_motion():
    n,h,calls,done=point_node();n.cfg['cancel_timeout_sec']=.001
    n.inputs=lambda **kw:(_ for _ in ()).throw(ValueError('fresh measured base/lift feedback required'))
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        result=NavigationServer.execute_point(n,h)
    assert not result.success and not calls and done==['abort'] and not n.busy
