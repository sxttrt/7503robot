"""验证流程的行为边界：固定顺序、旧反馈、限时、货物状态和真实配置。"""

from copy import deepcopy
from pathlib import Path

import pytest

from mission_manager.config_loader import load_config, read_yaml, validate
from mission_manager.state_machine import FINAL_STATES, Mission


CONFIG = Path(__file__).resolve().parents[2] / 'robot_bringup' / 'config'


class Clock:
    """可推进的单调时钟，使超时验证不必等待真实的三分钟。"""
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class Backend:
    """记录真实调用意图；由测试决定何时返回，便于制造迟到反馈。"""
    def __init__(self):
        self.calls = []
        self.cancelled = []
        self.stops = []
        self.is_healthy = True
        self.is_ready = True

    def ready(self):
        return self.is_ready

    def healthy(self):
        return self.is_healthy

    def poll(self):
        pass

    def start(self, kind, request, payload, callback):
        self.calls.append((kind, request, payload, callback))

    def cancel(self, kind, request):
        self.cancelled.append((kind, request))

    def stop(self, request, callback):
        self.stops.append((request, callback))

    def finish(self, success=True, details=None, call=None):
        kind, request, payload, callback = call or self.calls[-1]
        if details is None:
            details = {'ready_to_lift': True, 'exited': True}
            if kind == 'lift':
                details['is_up'] = payload['up']
        callback(request, success, details)

    def confirm_stop(self, success=True):
        request, callback = self.stops[-1]
        callback(request, success, {'reason': '测试停止响应'})


@pytest.fixture
def system():
    config = load_config(CONFIG / 'mission.yaml', CONFIG / 'interfaces.yaml',
                         CONFIG / 'targets_sim.yaml', 'simulation')
    clock, backend = Clock(), Backend()
    mission = Mission(config, backend, clock)
    return mission, backend, clock


def advance_to(mission, backend, state):
    """只推进正常动作，测试目标状态的失败由调用方单独制造。"""
    if mission.state == 'IDLE':
        assert mission.arm()[0]
    for _ in range(60):
        if mission.state == state:
            return
        backend.finish()
        mission.tick()
    raise AssertionError(f'未到达目标状态：{state}')


def test_four_racks_are_strictly_ordered(system):
    mission, backend, _clock = system
    assert mission.arm()[0]
    for _ in range(60):
        if mission.state == 'STOPPING':
            backend.confirm_stop()
        elif mission.state in FINAL_STATES:
            break
        else:
            backend.finish()
        mission.tick()
    assert mission.state == 'FINISHED'
    assert mission.completed == mission.delivered == ['A', 'B', 'C', 'D']
    assert mission.cargo == 'EMPTY'
    entered = [payload['rack_id'] for kind, _, payload, _ in backend.calls
               if kind == 'docking' and payload['operation'] == 'ENTER']
    assert entered == ['A', 'B', 'C', 'D']


def test_arming_waits_for_readiness(system):
    mission, backend, _clock = system
    backend.is_ready = False
    assert not mission.arm()[0]
    assert mission.state == 'IDLE' and not backend.calls


def test_clock_only_starts_after_start_qr(system):
    mission, backend, clock = system
    mission.arm()
    assert mission.started_at is None
    clock.now += 61
    mission.tick()
    assert mission.state == 'WAIT_START' and mission.started_at is None
    backend.finish()
    mission.tick()
    started = mission.started_at
    # 再发送旧 START 结果不会改变起始计时或任务顺序。
    backend.finish(call=backend.calls[-2])
    mission.tick()
    assert mission.started_at == started and mission.state == 'NAV_RACK'


def test_retry_waits_for_stop_and_discards_old_success(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_RACK')
    old_call = backend.calls[-1]
    backend.finish(False, {'reason': '导航失败'})
    mission.tick()
    assert mission.state == 'STOPPING'
    number = len(backend.calls)
    backend.finish(True, call=old_call)
    mission.tick()
    assert mission.state == 'STOPPING' and len(backend.calls) == number
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'NAV_RACK' and mission.rack == 'A'
    assert mission.active_request != old_call[1]
    assert mission.attempts['NAV_RACK'] == 1


def test_exhausted_retry_never_skips_to_b(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_RACK')
    for _ in range(2):
        backend.finish(False, {'reason': '不可达'})
        mission.tick()
        backend.confirm_stop()
        mission.tick()
    assert mission.state == 'FAULT' and mission.rack == 'A'
    assert mission.completed == []


@pytest.mark.parametrize('state', ['NAV_RACK', 'WAIT_RACK_QR', 'DOCK_ENTER', 'LIFT_UP',
                                  'NAV_END', 'WAIT_END_QR', 'LIFT_DOWN', 'DOCK_EXIT'])
def test_success_at_deadline_cannot_advance(system, state):
    mission, backend, clock = system
    advance_to(mission, backend, state)
    clock.now = mission.deadline
    backend.finish()
    mission.tick()
    assert mission.state == 'STOPPING'
    assert mission.completed == []


def test_qr_identity_does_not_replace_ready_to_lift(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'DOCK_ENTER')
    backend.finish(details={'ready_to_lift': False})
    mission.tick()
    assert mission.state == 'STOPPING'
    assert not any(kind == 'lift' for kind, *_ in backend.calls)


def test_lift_failure_keeps_unknown_cargo(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'LIFT_UP')
    backend.finish(False, {'reason': '没有到位反馈'})
    mission.tick()
    assert mission.cargo == 'UNKNOWN'
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FAULT' and mission.rack == 'A'


def test_loaded_navigation_failure_does_not_lower_or_skip(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_END')
    assert mission.cargo == 'UP'
    backend.finish(False, {'reason': '运输路径被挡住'})
    mission.tick()
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FAULT' and mission.cargo == 'UP'
    assert not any(kind == 'lift' and payload['up'] is False for kind, _, payload, _ in backend.calls)


def test_lowering_and_exit_are_recorded_separately(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'DOCK_EXIT')
    assert mission.delivered == ['A'] and mission.completed == []
    backend.finish(False, {'reason': '无法退出'})
    mission.tick()
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FAULT' and mission.cargo == 'EMPTY'
    assert mission.delivered == ['A'] and mission.completed == []


@pytest.mark.parametrize('state', ['NAV_RACK', 'WAIT_RACK_QR', 'DOCK_ENTER', 'LIFT_UP',
                                  'NAV_END', 'WAIT_END_QR', 'LIFT_DOWN', 'DOCK_EXIT'])
def test_time_up_stops_every_active_phase(system, state):
    mission, backend, clock = system
    advance_to(mission, backend, state)
    previous_calls = len(backend.calls)
    clock.now = mission.started_at + 180
    backend.finish()
    mission.tick()
    assert mission.state == 'STOPPING'
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FINISHED'
    assert len(backend.calls) == previous_calls


def test_time_up_overrides_pending_retry(system):
    mission, backend, clock = system
    advance_to(mission, backend, 'NAV_RACK')
    # 临近比赛结束才进入停止握手，避免把停止本身也拖过五秒时限。
    clock.now = mission.started_at + 179
    backend.finish(False)
    mission.tick()
    clock.now = mission.started_at + 180
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FINISHED' and len(backend.calls) == 2


def test_stop_timeout_never_restarts_motion(system):
    mission, backend, clock = system
    advance_to(mission, backend, 'NAV_RACK')
    backend.finish(False)
    mission.tick()
    clock.now = mission.deadline
    mission.tick()
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FAULT' and len(backend.calls) == 2


def test_health_failure_stops_and_manual_stop_is_terminal(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_RACK')
    backend.is_healthy = False
    mission.tick()
    assert mission.state == 'STOPPING'
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FAULT'
    assert not mission.arm()[0]


def test_operator_stop_preserves_task(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'WAIT_RACK_QR')
    mission.manual_stop()
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'STOPPED' and mission.rack == 'A'


def test_operator_stop_cancels_already_planned_retry(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_RACK')
    backend.finish(False)
    mission.tick()
    assert mission.stop_plan[0] == 'NAV_RACK'
    mission.manual_stop()
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'STOPPED' and len(backend.calls) == 2


def test_real_config_reports_all_unknown_codes():
    with pytest.raises(ValueError) as error:
        load_config(CONFIG / 'mission.yaml', CONFIG / 'interfaces.yaml',
                    CONFIG / 'targets_robot.yaml', 'robot')
    for rack in 'ABCD':
        assert f'racks.{rack}.qr' in str(error.value)
    assert 'destination_pose' in str(error.value)


def test_simulation_config_is_rejected_in_robot_mode(system):
    mission, _backend, _clock = system
    with pytest.raises(ValueError, match='禁止使用模拟'):
        validate(deepcopy(mission.config), 'robot')


@pytest.mark.parametrize('field,value', [('rack_order', ['B', 'A', 'C', 'D']),
                                      ('game_duration_sec', float('nan')),
                                      ('stop_timeout_sec', True),
                                      ('qr_confirm_frames', 0)])
def test_invalid_configuration_is_rejected(system, field, value):
    mission, _backend, _clock = system
    config = deepcopy(mission.config)
    config['mission'][field] = value
    with pytest.raises(ValueError):
        validate(config, 'simulation')
