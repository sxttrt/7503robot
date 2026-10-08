"""验证流程的行为边界：固定顺序、旧反馈、限时、货物状态和真实配置。"""

from copy import deepcopy
from pathlib import Path

import pytest

from mission_manager.config_loader import load_config, read_yaml, validate
from mission_manager.state_machine import FINAL_STATES, Mission


CONFIG = Path(__file__).resolve().parents[2] / 'robot_bringup' / 'config'


@pytest.mark.parametrize('name', ['nav goal', 'nav//goal', '/nav', 'nav/', '2nav', '~nav'])
def test_invalid_ros_interface_name_rejected(name):
    config = load_config(CONFIG / 'mission.yaml', CONFIG / 'interfaces.yaml',
                         CONFIG / 'targets_sim.yaml', 'simulation')
    config['interfaces']['navigation_action'] = name
    with pytest.raises(ValueError, match='合法相对名称'):
        validate(config, 'simulation')


@pytest.mark.parametrize('key', ['stop_timeout_sec', 'health_timeout_sec'])
def test_tick_must_be_shorter_than_safety_deadlines(key):
    config = load_config(CONFIG / 'mission.yaml', CONFIG / 'interfaces.yaml',
                         CONFIG / 'targets_sim.yaml', 'simulation')
    config['mission'][key] = config['mission']['tick_period_sec']
    with pytest.raises(ValueError, match='调度周期'):
        validate(config, 'simulation')


def test_different_action_types_cannot_share_one_endpoint():
    config = load_config(CONFIG / 'mission.yaml', CONFIG / 'interfaces.yaml',
                         CONFIG / 'targets_sim.yaml', 'simulation')
    config['interfaces']['docking_action'] = config['interfaces']['navigation_action']
    with pytest.raises(ValueError, match='名称不能相同'):
        validate(config, 'simulation')


def test_missing_config_reports_explicit_error():
    with pytest.raises(ValueError, match='配置字典'):
        validate({'mission': []}, 'simulation')
    config = load_config(CONFIG / 'mission.yaml', CONFIG / 'interfaces.yaml',
                         CONFIG / 'targets_sim.yaml', 'simulation')
    config['mission'].pop('health_timeout_sec')
    with pytest.raises(ValueError, match='health_timeout_sec'):
        validate(config, 'simulation')


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
        self.is_cargo_consistent = True
        self.is_stationary = True
        self.is_up = False

    def stationary(self):
        return self.is_stationary

    def cargo_observation(self):
        up = self.is_up if self.is_cargo_consistent else not self.is_up
        return 'UP' if up else 'EMPTY'

    def ready(self):
        return self.is_ready

    def healthy(self):
        return self.is_healthy

    def cargo_matches(self, _cargo):
        return self.is_cargo_consistent

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
        if kind == 'lift' and success is True and type(details.get('is_up')) is bool:
            self.is_up = details['is_up']
        callback(request, success, details)

    def confirm_stop(self, success=True):
        request, callback = self.stops[-1]
        callback(request, success, {'reason': '测试停止响应'})


@pytest.fixture
def system():
    config = load_config(CONFIG / 'mission.yaml', CONFIG / 'interfaces.yaml',
                         CONFIG / 'targets_sim.yaml', 'simulation')
    config['mission']['confirm_end_qr'] = True
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






def test_state_observer_failure_still_requests_stop(system):
    mission, backend, _clock = system
    def broken_observer(*_args):
        raise OSError('模拟日志磁盘已满')
    mission.on_transition = broken_observer
    mission.arm()
    assert mission.state == 'STOPPING' and backend.stops and not backend.calls
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FAULT'


def test_backend_poll_exception_never_retries_motion(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_RACK')
    count = len(backend.calls)
    def broken_poll():
        raise RuntimeError('模拟通信回调异常')
    backend.poll = broken_poll
    mission.tick()
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FAULT' and len(backend.calls) == count


@pytest.mark.parametrize('terminal', ['FAULT', 'STOPPED'])
def test_game_expiry_preserves_fault_or_operator_stop(system, terminal):
    mission, backend, clock = system
    advance_to(mission, backend, 'NAV_END')
    clock.now = mission.started_at + 179
    if terminal == 'FAULT':
        backend.finish(False)
        mission.tick()
    else:
        mission.manual_stop()
    clock.now += 1
    backend.confirm_stop()
    mission.tick()
    assert mission.state == terminal


def test_stop_confirmation_cannot_retry_unhealthy_modules(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_RACK')
    backend.finish(False)
    mission.tick()
    count = len(backend.calls)
    backend.is_healthy = False
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FAULT' and len(backend.calls) == count


def test_slow_state_observer_cannot_dispatch_after_game_expiry(system):
    mission, backend, clock = system
    advance_to(mission, backend, 'NAV_RACK')
    count = len(backend.calls)
    def slow_observer(_previous, state, _reason):
        if state == 'WAIT_RACK_QR':
            clock.now = mission.started_at + 180
    mission.on_transition = slow_observer
    backend.finish()
    mission.tick()
    assert mission.state == 'STOPPING' and len(backend.calls) == count
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FINISHED'


def test_slow_state_observer_cannot_dispatch_after_step_deadline(system):
    mission, backend, clock = system
    def slow_observer(_previous, state, _reason):
        if state == 'WAIT_START':
            clock.now += 61
    mission.on_transition = slow_observer
    mission.arm()
    assert mission.state == 'STOPPING' and not backend.calls


def test_truthy_string_result_is_not_success(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_RACK')
    backend.finish('false')
    mission.tick()
    assert mission.state == 'STOPPING'


def test_health_change_during_transition_blocks_next_request(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_RACK')
    count = len(backend.calls)
    def observer(_previous, state, _reason):
        if state == 'WAIT_RACK_QR':
            backend.is_healthy = False
    mission.on_transition = observer
    backend.finish()
    mission.tick()
    assert mission.state == 'STOPPING' and len(backend.calls) == count


@pytest.mark.parametrize('terminal', ['FAULT', 'FINISHED', 'STOPPED'])
def test_explicit_stop_in_terminal_reissues_without_resuming(system, terminal):
    """故障后的第二次人工停止必须真实发出，同时保留任务结束结果。"""
    mission, backend, clock = system
    advance_to(mission, backend, 'NAV_END')
    mission.stopping('原始结束原因', terminal=terminal)
    backend.confirm_stop()
    mission.tick()
    calls = len(backend.calls)
    old_request, old_callback = backend.stops[-1]
    assert mission.manual_stop(reissue=True)[0]
    assert len(backend.stops) == 2 and mission.state == 'STOPPING'
    deadline = mission.deadline
    # 程序退出循环不能不断重发或延长截止时间；旧停止回调也不能完成新握手。
    clock.now += 0.1
    mission.manual_stop()
    old_callback(old_request, True, {})
    mission.tick()
    assert mission.state == 'STOPPING' and mission.deadline == deadline
    assert len(backend.stops) == 2
    backend.confirm_stop()
    mission.tick()
    assert mission.state == terminal and mission.reason == '原始结束原因'
    assert len(backend.calls) == calls and not mission.arm()[0]
    assert not mission.manual_stop()[0] and len(backend.stops) == 2


def test_second_stop_after_unconfirmed_stop_reaches_physical_backend(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_END')
    mission.manual_stop()
    backend.confirm_stop(False)
    mission.tick()
    assert mission.state == 'FAULT'
    original_reason = mission.reason
    # 检查 ROS 人工停止服务自身，不只是调用状态机的方法。
    from types import SimpleNamespace
    from mission_manager.mission_node import MissionNode
    response = MissionNode.stop(SimpleNamespace(mission=mission), None, SimpleNamespace())
    assert response.success and len(backend.stops) == 2
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FAULT' and mission.reason == original_reason


def test_slow_stop_observer_cannot_delay_stop_dispatch(system):
    mission, backend, clock = system
    advance_to(mission, backend, 'NAV_END')
    def slow_observer(_previous, state, _reason):
        if state == 'STOPPING':
            assert len(backend.stops) == 1
            clock.now += 6.0
    mission.on_transition = slow_observer
    mission.manual_stop()
    assert len(backend.stops) == 1
    mission.tick()
    assert mission.state == 'FAULT' and '停止未得到确认' in mission.reason


def test_tick_rechecks_stop_deadline_after_slow_observer(system):
    mission, backend, clock = system
    advance_to(mission, backend, 'NAV_END')
    def slow_observer(_previous, state, _reason):
        if state == 'STOPPING':
            backend.confirm_stop()
            clock.now += 6.0
    mission.on_transition = slow_observer
    backend.is_healthy = False
    mission.tick()
    assert mission.state == 'FAULT' and '停止未得到确认' in mission.reason






def test_shutdown_during_fault_stop_preserves_original_fault(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_END')
    backend.finish(False, {'reason': '运输路径无法规划'})
    mission.tick()
    deadline = mission.deadline
    mission.manual_stop()
    mission.manual_stop(reissue=True)
    assert mission.deadline == deadline and len(backend.stops) == 1
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'FAULT' and mission.reason == '运输路径无法规划'






@pytest.mark.parametrize('key,other', [
    ('qr_topic', 'health_topic'), ('lift_service', 'stop_service'),
    ('health_topic', 'mission_status_topic'), ('lift_state_topic', 'qr_topic'),
])
def test_interface_endpoint_collisions_rejected(system, key, other):
    mission, _backend, _clock = system
    config = deepcopy(mission.config)
    config['interfaces'][key] = config['interfaces'][other]
    with pytest.raises(ValueError, match='名称不能相同'):
        validate(config, 'simulation')


@pytest.mark.parametrize('reserved', ['mission/stop', 'mission/arm'])
def test_physical_stop_cannot_call_mission_control_itself(system, reserved):
    mission, _backend, _clock = system
    config = deepcopy(mission.config)
    config['interfaces']['stop_service'] = reserved
    with pytest.raises(ValueError, match='调用自身'):
        validate(config, 'simulation')


@pytest.mark.parametrize('outcome', ['success', 'failure', 'timeout'])
def test_main_shutdown_retries_unconfirmed_stop_once(system, monkeypatch, outcome):
    """直接执行退出入口；失败后的补发有界，不能在退出循环无限重发。"""
    from types import SimpleNamespace
    import mission_manager.mission_node as module
    mission, backend, clock = system
    advance_to(mission, backend, 'NAV_END')
    mission.manual_stop()
    backend.confirm_stop(False)
    mission.tick()
    node = SimpleNamespace(mission=mission, closing=False, config=mission.config,
        publish_status=lambda: None, destroy_node=lambda: None,
        log_stream=SimpleNamespace(close=lambda: None))
    running = SimpleNamespace(value=True)
    def spin_once(_node, timeout_sec):
        if not node.closing:
            node.closing = True
        else:
            mission.manual_stop()
            if outcome == 'timeout':
                clock.now = mission.deadline
            else:
                backend.confirm_stop(outcome == 'success')
            mission.tick()
    monkeypatch.setattr(module, 'MissionNode', lambda: node)
    monkeypatch.setattr(module.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: running.value)
    monkeypatch.setattr(module.rclpy, 'spin_once', spin_once)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: setattr(running, 'value', False))
    monkeypatch.setattr(module.signal, 'signal', lambda *_args: None)
    module.main()
    assert len(backend.stops) == 2 and mission.state == 'FAULT'
    assert mission.status()['stop']['status'] == {
        'success': 'CONFIRMED', 'failure': 'FAILED', 'timeout': 'TIMED_OUT'}[outcome]


def test_shutdown_does_not_repeat_already_confirmed_stop(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_END')
    mission.manual_stop()
    backend.confirm_stop()
    mission.tick()
    mission.prepare_shutdown()
    assert len(backend.stops) == 1 and mission.state == 'STOPPED'






def test_repeated_stop_reports_new_confirmation_separately_from_old_fault(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_END')
    mission.manual_stop()
    backend.confirm_stop(False)
    mission.tick()
    original_reason = mission.reason
    original_request = mission.status()['stop']['request_id']
    mission.manual_stop(reissue=True)
    assert mission.status()['stop']['status'] == 'PENDING'
    backend.confirm_stop()
    mission.tick()
    status = mission.status()
    assert status['state'] == 'FAULT' and status['reason'] == original_reason
    assert status['stop']['status'] == 'CONFIRMED'
    assert status['stop']['request_id'] != original_request
    assert '本次' in status['stop']['reason']


def test_default_navigation_arrival_directly_starts_lowering():
    config = load_config(CONFIG / 'mission.yaml', CONFIG / 'interfaces.yaml', CONFIG / 'targets_sim.yaml', 'simulation')
    assert config['mission']['confirm_end_qr'] is False
    backend = Backend()
    mission = Mission(config, backend, Clock())
    advance_to(mission, backend, 'NAV_END')
    backend.finish()
    mission.tick()
    assert mission.state == 'LIFT_DOWN'
    assert not any(item['to'] == 'WAIT_END_QR' for item in mission.history)


def test_module_success_is_enough_without_auxiliary_confirmation(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'DOCK_ENTER')
    backend.is_stationary = False
    backend.is_cargo_consistent = False
    backend.finish(details={})
    mission.tick()
    assert mission.state == 'LIFT_UP'
    backend.finish(details={'is_up': True})
    mission.tick()
    assert mission.state == 'NAV_END' and mission.cargo == 'UP'


def test_stop_reply_is_enough_without_auxiliary_confirmation(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'NAV_RACK')
    mission.manual_stop()
    backend.is_stationary = False
    backend.is_cargo_consistent = False
    backend.confirm_stop()
    mission.tick()
    assert mission.state == 'STOPPED' and mission.stop_status == 'CONFIRMED'


def test_exit_success_records_completion_without_extra_flag(system):
    mission, backend, _clock = system
    advance_to(mission, backend, 'DOCK_EXIT')
    backend.finish(details={})
    mission.tick()
    assert mission.completed == ['A'] and mission.rack == 'B'
