"""使用可控等待检查模拟升降真实占用运动计数，不需要启动机器人硬件。"""

import threading
import json
from types import SimpleNamespace

import pytest

from mission_manager.mock_modules import MockModules


@pytest.mark.parametrize('outcome', ['success', 'failure', 'stop', 'shutdown', 'exception'])
def test_mock_lift_is_moving_until_every_exit_path_finishes(monkeypatch, outcome):
    """暂停在升降途中取样，再分别制造成功、失败、中止、退出和异常。"""
    mock = MockModules.__new__(MockModules)
    mock.lock = threading.RLock()
    mock.stop_epoch = mock.motion_count = 0
    mock.is_up = False
    mock.counts = {}
    mock.status = {'state': 'LIFT_UP', 'rack': 'A'}
    mock.mission_active_at = None
    mock.config = {'targets': {'racks': {}}}
    observations = []
    mock.health_pub = SimpleNamespace(publish=lambda message: observations.append(json.loads(message.data)))
    mock.lift_pub = mock.qr_pub = SimpleNamespace(publish=lambda _message: None)
    mock.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=10_000_000_000, to_msg=lambda: SimpleNamespace(sec=10, nanosec=0)))
    mock.scenario = {'lift_duration_sec': 0.25,
                     'failures': {'LIFT_UP': 1} if outcome == 'failure' else {}}
    clock = SimpleNamespace(now=100.0, running=True)
    monkeypatch.setattr('mission_manager.mock_modules.rclpy.ok', lambda: clock.running)
    monkeypatch.setattr('mission_manager.mock_modules.time.monotonic', lambda: clock.now)

    def pause(_seconds):
        assert mock.motion_count == 1 and mock.is_up is False
        mock.publish_observations()
        assert observations[-1] == {'ready': True, 'fault': ''}
        if outcome == 'stop':
            mock.stop_epoch += 1  # 模拟停止回调已中断当前动作
        elif outcome == 'shutdown':
            clock.running = False
        elif outcome == 'exception':
            raise RuntimeError('模拟等待异常')
        clock.now += 0.3

    monkeypatch.setattr('mission_manager.mock_modules.time.sleep', pause)
    if outcome == 'exception':
        with pytest.raises(RuntimeError, match='模拟等待异常'):
            mock.lift(SimpleNamespace(data=True), SimpleNamespace())
    else:
        response = mock.lift(SimpleNamespace(data=True), SimpleNamespace())
        assert response.success is (outcome == 'success')
    assert mock.motion_count == 0 and mock.is_up is (outcome == 'success')
    mock.publish_observations()
    assert observations[-1] == {'ready': True, 'fault': ''}


def test_mock_navigation_rejects_unknown_point():
    """未知点返回失败，不能错误进入模拟运动。"""
    from mission_interfaces.action import NavigateToPoint
    mock = MockModules.__new__(MockModules)
    aborted = []
    handle = SimpleNamespace(request=SimpleNamespace(target_id='UNKNOWN'), abort=lambda: aborted.append(True))
    result = mock.navigate(handle)
    assert isinstance(result, NavigateToPoint.Result)
    assert result.success is False and aborted == [True]


@pytest.mark.parametrize('target,state', [('A', 'NAV_RACK'), ('DROP_OFF', 'NAV_END')])
def test_mock_navigation_dispatches_by_point_name(target, state):
    mock = MockModules.__new__(MockModules)
    mock.scenario = {}
    states, succeeded = [], []
    mock.should_fail = lambda phase: states.append(phase) or False
    mock.wait_action = lambda *_args: 'finished'
    handle = SimpleNamespace(request=SimpleNamespace(target_id=target), succeed=lambda: succeeded.append(True))
    result = mock.navigate(handle)
    assert result.success is True and states == [state] and succeeded == [True]
