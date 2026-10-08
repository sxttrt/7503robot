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
    mock.lift_safety_token = 'token0'
    mock.seen_lift_requests = set()
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
        assert observations[-1]['stopped'] is False
        if outcome == 'stop':
            MockModules.stop(mock, None, SimpleNamespace())
        elif outcome == 'shutdown':
            clock.running = False
        elif outcome == 'exception':
            raise RuntimeError('模拟等待异常')
        clock.now += 0.3

    monkeypatch.setattr('mission_manager.mock_modules.time.sleep', pause)
    if outcome == 'exception':
        with pytest.raises(RuntimeError, match='模拟等待异常'):
            mock.lift(SimpleNamespace(up=True, request_id='r1', safety_token='token0'), SimpleNamespace())
    else:
        response = mock.lift(SimpleNamespace(up=True, request_id='r1', safety_token='token0'), SimpleNamespace())
        assert response.success is (outcome == 'success')
    assert mock.motion_count == 0 and mock.is_up is (outcome == 'success')
    mock.publish_observations()
    assert observations[-1]['stopped'] is True


def token_mock():
    """只构造升降和停止处理器，令牌与真实服务端使用同一套校验。"""
    mock = MockModules.__new__(MockModules)
    mock.lock = threading.RLock()
    mock.stop_epoch = mock.motion_count = 0
    mock.lift_safety_token = 'token0'
    mock.seen_lift_requests = set()
    mock.is_up = False
    mock.counts = {}
    mock.scenario = {'lift_duration_sec': 0.0}
    return mock


def test_old_lift_arriving_after_stop_cannot_move(monkeypatch):
    mock = token_mock()
    old_request = SimpleNamespace(request_id='old', up=True, safety_token=mock.lift_safety_token)
    mock.stop(None, SimpleNamespace())
    monkeypatch.setattr('mission_manager.mock_modules.rclpy.ok', lambda: True)
    response = mock.lift(old_request, SimpleNamespace())
    assert response.success is False and mock.is_up is False and mock.motion_count == 0


def test_new_token_allows_later_legitimate_lift_but_not_duplicate(monkeypatch):
    mock = token_mock()
    mock.stop(None, SimpleNamespace())
    monkeypatch.setattr('mission_manager.mock_modules.rclpy.ok', lambda: True)
    request = SimpleNamespace(request_id='new', up=True, safety_token=mock.lift_safety_token)
    assert mock.lift(request, SimpleNamespace()).success
    assert not mock.lift(request, SimpleNamespace()).success
    assert mock.counts['LIFT_UP'] == 1 and mock.is_up is True


def test_controller_restart_also_rejects_previous_token(monkeypatch):
    mock = token_mock()
    mock.lift_safety_token = 'new_boot_token'
    monkeypatch.setattr('mission_manager.mock_modules.rclpy.ok', lambda: True)
    response = mock.lift(SimpleNamespace(request_id='old', up=True, safety_token='token0'), SimpleNamespace())
    assert not response.success and not mock.is_up
