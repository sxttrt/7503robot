"""验证实机边界：未确认的动作、反馈乱序、源时间戳和升降响应异常。

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
def test_nonterminal_action_response_blocks_stop_confirmation(status):
    action, outcomes, _clock = action_reader()
    action.result('r1', Future(SimpleNamespace(status=status, result=None)))
    assert not action.idle() and outcomes[0][1] is False


def test_success_requires_post_result_stopped_observation():
    action, outcomes, clock = action_reader()
    action.result('r1', Future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=None)))
    assert action.idle() and not outcomes
    action.poll()
    assert not outcomes
    action.safety.stamp_ns = clock.ns + 1
    action.safety.latest['stopped'] = False
    action.poll()
    assert not outcomes
    action.safety.latest['stopped'] = True
    action.poll()
    assert len(outcomes) == 1 and outcomes[0][1] is True


def test_cancelled_pending_success_cannot_advance():
    action, outcomes, _clock = action_reader()
    action.result('r1', Future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=None)))
    action.cancel('r1')
    action.safety.stamp_ns += 1
    action.poll()
    assert not action.pending_results and not outcomes


@pytest.mark.parametrize('result', [None, SimpleNamespace(error_code=True),
                                   SimpleNamespace(error_code='0')])
def test_navigation_missing_or_malformed_error_code_is_not_success(result):
    assert Navigation.__new__(Navigation).decode(result)[0] is False


def health_reader(monkeypatch):
    reader = Safety.__new__(Safety)
    reader.node, clock = controlled_node()
    reader.policy = {'health_timeout_sec': 2.0, 'qr_future_tolerance_sec': 0.1,
                     'allow_estimated_lift': False}
    reader.latest = reader.received_at = reader.stamp_ns = reader.pending = None
    reader.diagnostic = ''
    monotonic = SimpleNamespace(now=100.0)
    monkeypatch.setattr('mission_manager.adapters.safety.time.monotonic', lambda: monotonic.now)
    data = {'schema_version': 2, 'stamp': {'sec': 10, 'nanosec': 0},
            'modules': {name: True for name in ('navigation', 'qr', 'docking', 'lift', 'base')},
            'stopped': True, 'lift_is_up': False, 'lift_state_source': 'measured',
            'lift_safety_token': 'token0', 'fault': ''}
    return reader, data, clock, monotonic


def test_health_source_age_is_not_extended_by_recent_arrival(monkeypatch):
    reader, data, clock, monotonic = health_reader(monkeypatch)
    clock.ns += 1_900_000_000
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    assert reader.healthy()
    clock.ns += 200_000_000
    monotonic.now += 0.2
    assert not reader.healthy()


@pytest.mark.parametrize('key,value', [('lift_is_up', 'false'), ('schema_version', True),
                                     ('lift_state_source', 'unknown'), ('fault', None)])
def test_malformed_health_cannot_arm(monkeypatch, key, value):
    reader, data, _clock, _monotonic = health_reader(monkeypatch)
    data[key] = value
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    assert not reader.healthy()


def test_replayed_health_cannot_refresh_receive_watchdog(monkeypatch):
    reader, data, _clock, monotonic = health_reader(monkeypatch)
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    monotonic.now += 2.1
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    assert not reader.healthy()


def test_stop_can_confirm_stillness_during_module_fault(monkeypatch):
    reader, data, clock, _monotonic = health_reader(monkeypatch)
    data['fault'] = '模拟导航故障'
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    outcomes = []
    reader.pending = {'ack': True, 'begin_ns': clock.ns - 1, 'request': 'stop1',
                      'callback': lambda *args: outcomes.append(args)}
    assert not reader.healthy()
    reader.poll(actions_idle=True)
    assert outcomes and outcomes[0][1] is True


def test_lift_state_source_must_match_policy(monkeypatch):
    reader, data, _clock, _monotonic = health_reader(monkeypatch)
    data['lift_state_source'] = 'estimated'
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    assert not reader.lift_matches(False)
    reader.policy['allow_estimated_lift'] = True
    assert reader.lift_matches(False)


def lift_reader():
    lift = Lift.__new__(Lift)
    lift.node, clock = controlled_node()
    lift.policy = {'allow_estimated_lift': False}
    outcomes = []
    lift.requests = {'r1': {'up': True, 'cancelled': False, 'begin_ns': clock.ns - 1,
                           'safety_token': 'token0',
                           'callback': lambda *args: outcomes.append(args)}}
    lift.pending_results = {}
    lift.safety = SimpleNamespace(healthy=lambda: True, stamp_ns=clock.ns,
                                 latest={'lift_is_up': True, 'lift_state_source': 'measured',
                                         'lift_safety_token': 'token0',
                                         'stopped': True})
    return lift, outcomes, clock


def test_lift_transport_exception_retains_unresolved_request():
    lift, outcomes, _clock = lift_reader()
    lift.result('r1', Future(error=RuntimeError('模拟服务链路断开')))
    assert not lift.idle() and outcomes[0][1] is False


def test_lift_send_exception_cannot_allow_overlapping_retry():
    lift, outcomes, _clock = lift_reader()
    lift.requests = {}
    def broken_send(_message):
        raise RuntimeError('发送之后链路异常')
    lift.client = SimpleNamespace(service_is_ready=lambda: True, call_async=broken_send)
    lift.start('r1', {'up': True}, lambda *args: outcomes.append(args))
    assert not lift.idle() and outcomes[0][1] is False
    lift.cancel('r1')
    assert not lift.idle()


def test_lift_confirmation_requires_new_status_after_response():
    lift, outcomes, clock = lift_reader()
    lift.result('r1', Future(SimpleNamespace(request_id='r1', success=True, message='完成')))
    lift.poll()
    assert not outcomes
    lift.safety.stamp_ns = clock.ns + 1
    lift.poll()
    assert len(outcomes) == 1 and outcomes[0][1] is True


def test_cancelled_lift_response_cannot_complete_old_task():
    lift, outcomes, _clock = lift_reader()
    lift.cancel('r1')
    lift.result('r1', Future(SimpleNamespace(request_id='r1', success=True, message='迟到完成')))
    lift.poll()
    assert lift.idle() and not outcomes


def test_qr_large_frame_gap_restarts_consecutive_confirmation():
    from test_adapters import qr_reader, observation
    reader, outcomes = qr_reader()
    reader.observe(observation(10_100_000_000), 10_100_000_000)
    reader.observe(observation(10_200_000_000), 10_200_000_000)
    reader.observe(observation(11_300_000_000), 11_300_000_000)
    assert reader.count == 1 and not outcomes


def test_lift_success_waits_until_all_motion_has_stopped():
    lift, outcomes, clock = lift_reader()
    lift.result('r1', Future(SimpleNamespace(request_id='r1', success=True, message='完成')))
    lift.safety.stamp_ns = clock.ns + 1
    lift.safety.latest['stopped'] = False
    lift.poll()
    assert not outcomes and lift.pending_results
    lift.safety.latest['stopped'] = True
    lift.poll()
    assert len(outcomes) == 1 and outcomes[0][1] is True


def test_cargo_observation_remains_valid_during_module_fault(monkeypatch):
    reader, data, clock, _monotonic = health_reader(monkeypatch)
    data['fault'] = '模拟运输故障'
    data['lift_is_up'] = True
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    assert not reader.healthy() and reader.lift_observation() == 'UP'
    clock.ns += 2_100_000_000
    assert reader.lift_observation() is None


def test_disallowed_estimated_cargo_is_not_used_as_observation(monkeypatch):
    reader, data, _clock, _monotonic = health_reader(monkeypatch)
    data['lift_state_source'] = 'estimated'
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    assert reader.lift_observation() is None
    reader.policy['allow_estimated_lift'] = True
    assert reader.lift_observation() == 'EMPTY'


@pytest.mark.parametrize('token', [None, '', '   ', 123])
def test_health_without_valid_lift_token_cannot_enable(monkeypatch, token):
    reader, data, _clock, _monotonic = health_reader(monkeypatch)
    data['lift_safety_token'] = token
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    assert not reader.healthy()


def test_stop_confirmation_waits_for_new_controller_token(monkeypatch):
    reader, data, clock, _monotonic = health_reader(monkeypatch)
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    outcomes = []
    reader.pending = {'ack': True, 'begin_ns': clock.ns - 1, 'old_token': 'token0',
                      'request': 's1', 'callback': lambda *args: outcomes.append(args)}
    reader.poll(True)
    assert not outcomes
    data['stamp']['nanosec'] = 1
    data['lift_safety_token'] = 'token1'
    reader.on_status(SimpleNamespace(data=json.dumps(data)))
    reader.poll(True)
    assert outcomes[0][1] is True


def test_wrong_lift_response_id_remains_unconfirmed():
    lift, outcomes, _clock = lift_reader()
    lift.result('r1', Future(SimpleNamespace(request_id='old', success=True, message='错误响应')))
    assert not lift.idle() and outcomes[0][1] is False


def test_lift_completion_with_changed_token_cannot_advance():
    lift, outcomes, clock = lift_reader()
    lift.result('r1', Future(SimpleNamespace(request_id='r1', success=True, message='完成')))
    lift.safety.stamp_ns = clock.ns + 1
    lift.safety.latest['lift_safety_token'] = 'token1'
    lift.poll()
    assert outcomes[0][1] is False and not lift.pending_results
