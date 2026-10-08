"""验证实机边界：动作结果、取消、简单在线反馈和升降响应异常。

使用真实 ROS 类型和适配器，只把时钟及异步回报替换为可控测试对象。
不连接硬件，不用成功字符串或固定距离伪造状态机推进。
"""

import json
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from mission_manager.adapters.navigation import AsyncAction, Navigation
from mission_manager.adapters.lift import Lift
from mission_manager.adapters.safety import Safety


class Future:
    """允许分别构造正常响应、传输异常和迟到回调。"""
    def __init__(self, value=None, error=None):
        self.value, self.error = value, error
        self.callbacks = []

    def result(self):
        if self.error:
            raise self.error
        return self.value

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


def controlled_node():
    time = SimpleNamespace(ns=10_000_000_000)
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=time.ns)))
    return node, time


def action_reader():
    action = AsyncAction.__new__(AsyncAction)
    action.node, clock = controlled_node()
    outcomes = []
    action.requests = {'r1': {'cancelled': False, 'handle': None,
                             'callback': lambda *args: outcomes.append(args)}}
    action.pending_results, action.feedback = {}, {}
    action.safety = SimpleNamespace(healthy=lambda: True, stamp_ns=clock.ns,
                                   latest={'stopped': True})
    action.decode = lambda _result: (True, {})
    return action, outcomes, clock


def test_action_result_transport_error_cannot_prove_idle():
    action, outcomes, _clock = action_reader()
    action.result('r1', Future(error=RuntimeError('模拟结果链路断开')))
    assert not action.idle() and len(outcomes) == 1 and outcomes[0][1] is False


def test_cancel_exception_retains_action_until_terminal_result():
    action, outcomes, _clock = action_reader()
    def broken_cancel():
        raise RuntimeError('模拟取消发送失败')
    action.requests['r1']['handle'] = SimpleNamespace(cancel_goal_async=broken_cancel)
    action.cancel('r1')
    assert not action.idle() and len(outcomes) == 1
    action.result('r1', Future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED, result=None)))
    assert action.idle() and len(outcomes) == 1


def test_failed_late_cancel_does_not_erase_accepted_goal():
    action, outcomes, _clock = action_reader()
    action.requests['r1']['cancelled'] = True
    result_future = Future()
    def broken_cancel():
        raise RuntimeError('模拟迟到目标取消失败')
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result_future,
                             cancel_goal_async=broken_cancel)
    action.accepted('r1', Future(handle))
    assert not action.idle() and len(result_future.callbacks) == 1
    assert outcomes[0][1] is False


@pytest.mark.parametrize('status', [None, GoalStatus.STATUS_EXECUTING])
def test_nonterminal_action_response_is_not_completion(status):
    action, outcomes, _clock = action_reader()
    action.result('r1', Future(SimpleNamespace(status=status, result=None)))
    assert not action.idle() and outcomes[0][1] is False



@pytest.mark.parametrize('result', [None, SimpleNamespace(error_code=True), SimpleNamespace(error_code='0')])
def test_navigation_invalid_error_code_is_not_success(result):
    assert Navigation.__new__(Navigation).decode(result)[0] is False


def test_action_success_does_not_wait_for_extra_status():
    action, outcomes, _clock = action_reader()
    action.safety = None
    action.result('r1', Future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=None)))
    assert action.idle() and outcomes[0][1] is True


def test_cancelled_success_cannot_advance():
    action, outcomes, _clock = action_reader()
    action.cancel('r1')
    action.result('r1', Future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=None)))
    assert outcomes[0][1] is False


def test_stop_completed_releases_old_goal_and_late_acceptance_is_cancelled():
    action, outcomes, _clock = action_reader()
    action.cancel('r1')
    action.release_cancelled()
    cancelled = []
    action.accepted('r1', Future(SimpleNamespace(accepted=True, cancel_goal_async=lambda: cancelled.append(True))))
    assert action.idle() and cancelled == [True] and not outcomes


def health_reader(monkeypatch):
    reader = Safety.__new__(Safety)
    reader.policy = {'health_timeout_sec': 2.0}
    reader.latest = reader.received_at = reader.pending = None
    reader.diagnostic = ''
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr('mission_manager.adapters.safety.time.monotonic', lambda: clock.now)
    return reader, clock


def test_simple_health_needs_no_token_stamp_or_sensor_source(monkeypatch):
    reader, clock = health_reader(monkeypatch)
    reader.on_status(SimpleNamespace(data=json.dumps({'ready': True, 'fault': ''})))
    assert reader.healthy()
    clock.now += 2.0
    assert not reader.healthy()


@pytest.mark.parametrize('data', [{'ready': False}, {'ready': True, 'fault': '电机故障'}, {'ready': 'true'}, {}, []])
def test_health_not_ready_faulty_or_malformed_cannot_enable(monkeypatch, data):
    reader, _clock = health_reader(monkeypatch)
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    assert not reader.healthy()


def test_stop_response_completes_without_health_or_action_receipts(monkeypatch):
    reader, _clock = health_reader(monkeypatch)
    outcomes = []
    record = {'request': 'stop1', 'callback': lambda *args: outcomes.append(args)}
    reader.pending = record
    reader.on_stop_response(record, Future(SimpleNamespace(success=True, message='已停止')))
    assert outcomes == [('stop1', True, {'reason': '已停止'})]


def test_old_stop_response_cannot_complete_new_stop(monkeypatch):
    reader, _clock = health_reader(monkeypatch)
    outcomes = []
    old = {'request': 'old', 'callback': lambda *args: outcomes.append(args)}
    reader.pending = {'request': 'new'}
    reader.on_stop_response(old, Future(SimpleNamespace(success=True, message='')))
    assert not outcomes and reader.pending['request'] == 'new'


def lift_reader():
    lift = Lift.__new__(Lift)
    outcomes = []
    lift.requests = {'r1': {'up': True, 'cancelled': False, 'callback': lambda *args: outcomes.append(args)}}
    return lift, outcomes


@pytest.mark.parametrize('success', [True, False, 'true'])
def test_lift_reply_is_final_result_without_second_observation(success):
    lift, outcomes = lift_reader()
    lift.result('r1', Future(SimpleNamespace(success=success, message='执行结果')))
    assert not lift.requests and outcomes[0][1] is (success is True)
    assert outcomes[0][2]['is_up'] is True


def test_lift_transport_exception_reports_failure():
    lift, outcomes = lift_reader()
    lift.result('r1', Future(error=RuntimeError('链路断开')))
    assert outcomes[0][1] is False


def test_cancelled_lift_reply_is_ignored_after_stop():
    lift, outcomes = lift_reader()
    lift.cancel('r1')
    lift.release_cancelled()
    lift.result('r1', Future(SimpleNamespace(success=True, message='旧结果')))
    assert not outcomes and not lift.requests


def test_lift_request_uses_standard_setbool_only():
    from std_srvs.srv import SetBool
    lift, outcomes = lift_reader()
    lift.requests = {}
    requests = []
    lift.client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda message: requests.append(message) or Future())
    lift.start('r1', {'up': True}, lambda *args: outcomes.append(args))
    assert isinstance(requests[0], SetBool.Request) and requests[0].data is True
    lift.start('r2', {'up': False}, lambda *args: outcomes.append(args))
    assert len(requests) == 1 and outcomes[0][1] is False


def test_qr_large_frame_gap_restarts_confirmation():
    from test_adapters import qr_reader, observation
    reader, outcomes = qr_reader()
    reader.observe(observation(10_100_000_000), 10_100_000_000)
    reader.observe(observation(10_200_000_000), 10_200_000_000)
    reader.observe(observation(11_300_000_000), 11_300_000_000)
    assert reader.count == 1 and not outcomes
