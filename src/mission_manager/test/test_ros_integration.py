"""启动真实 ROS 节点验证正常闭环及故障；默认不随纯逻辑测试运行。

执行方式：RUN_ROS_INTEGRATION=1 python3 -m pytest 此文件 -q。
每个测试只关闭自己创建的进程组，既不停止其他机器人节点，也不删除文件。
"""

from pathlib import Path
import os
import re
import signal
import subprocess
import time

import pytest
import yaml


pytestmark = pytest.mark.skipif(os.environ.get('RUN_ROS_INTEGRATION') != '1',
                                reason='真实 ROS 集成测试需显式设置 RUN_ROS_INTEGRATION=1')
CONFIG = Path(__file__).resolve().parents[2] / 'robot_bringup' / 'config'


@pytest.mark.parametrize('scenario,expected', [
    ('normal', 'FINISHED'),
    ('fault_during_delivery', 'FAULT'),
    ('navigation_fail_once', 'FINISHED'),
    ('navigation_always_fails', 'FAULT'),
    ('navigation_timeout', 'FAULT'),
    ('wrong_rack_qr', 'FAULT'),
    ('lift_failure', 'FAULT'),
    ('lift_timeout', 'FAULT'),
    ('delivery_navigation_failure', 'FAULT'),
    ('lowering_failure', 'FAULT'),
    ('exit_failure', 'FAULT'),
    ('health_dropout', 'FAULT'),
    ('stop_failure', 'FAULT'),
    ('delayed_cancel_result', 'FAULT'),
    ('delayed_goal_accept', 'FAULT'),
    ('match_time_up', 'FINISHED'),
    ('operator_stop', 'STOPPED'),
    ('shutdown_during_delivery', 'STOPPED'),
    ('operator_retries_stop_after_fault', 'FAULT'),
    ('shutdown_during_fault_stop', 'FAULT'),
    ('shutdown_after_unconfirmed_stop', 'FAULT'),
])
def test_real_ros_execution(tmp_path, scenario, expected):
    """通过订阅公开任务状态判断结果，不直接调用状态机的成功函数。"""
    import json
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    policy = yaml.safe_load((CONFIG / 'mission.yaml').read_text(encoding='utf-8'))
    policy['timeouts'] = {key: (0.8 if 'QR' in key else 1.5) for key in policy['timeouts']}
    policy['timeouts']['WAIT_START'] = 3.0
    policy['stop_timeout_sec'] = 2.0
    policy['game_duration_sec'] = 60.0
    if scenario == 'delayed_goal_accept':
        policy['timeouts']['NAV_RACK'] = 0.35
    if scenario == 'match_time_up':
        policy['game_duration_sec'] = 1.2
    file = tmp_path / 'mission_test.yaml'
    file.write_text(yaml.safe_dump(policy, allow_unicode=True), encoding='utf-8')
    namespace, domain = 'team2/check', 63
    selected = 'normal' if scenario in ('match_time_up', 'operator_stop', 'shutdown_during_delivery') else scenario
    if scenario == 'operator_retries_stop_after_fault':
        selected = 'stop_fail_once'
    if scenario == 'shutdown_during_fault_stop':
        selected = 'delivery_navigation_failure'
    if scenario == 'shutdown_after_unconfirmed_stop':
        selected = 'stop_fail_once'
    command = ['ros2', 'launch', 'robot_bringup', 'simulation.launch.py',
               f'scenario:={selected}', f'mission_file:={file}',
               f'namespace:={namespace}', f'domain_id:={domain}', 'auto_arm:=false']
    log_file = tmp_path / 'ros_output.log'
    stream = log_file.open('w', encoding='utf-8')
    process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                               start_new_session=True)
    rclpy.init(domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    observer = Node('integration_observer')
    states = []
    health_states = []
    observer.create_subscription(String, f'/{namespace}/mission/status',
                                  lambda message: states.append(json.loads(message.data)), 10)
    observer.create_subscription(String, f'/{namespace}/robot/status',
                                  lambda message: health_states.append(json.loads(message.data)), 10)
    stop_client = observer.create_client(Trigger, f'/{namespace}/mission/stop')
    arm_client = observer.create_client(Trigger, f'/{namespace}/mission/arm')
    requested_arm = False
    requested_stop = False
    repeated_stop_seen = False
    original_fault = None
    repeat_response = None
    try:
        until = time.monotonic() + 40.0
        while time.monotonic() < until:
            rclpy.spin_once(observer, timeout_sec=0.1)
            if process.poll() is not None:
                raise AssertionError('启动进程异常退出：\n' + log_file.read_text(encoding='utf-8'))
            # 观察端完成发现并收到就绪状态后再启用，防止漏掉第一货架的早期事件。
            if (states and health_states and states[-1].get('ready_to_arm') and not requested_arm and
                    arm_client.service_is_ready()):
                arm_client.call_async(Trigger.Request())
                requested_arm = True
            if scenario == 'operator_stop' and states and states[-1]['state'] == 'NAV_END' and not requested_stop:
                if stop_client.service_is_ready():
                    stop_client.call_async(Trigger.Request())
                    requested_stop = True
            if scenario == 'shutdown_during_delivery' and states and states[-1]['state'] == 'NAV_END' and not requested_stop:
                # 仅给本测试启动的主程序发退出信号，模拟执行模块继续提供停止确认。
                match = re.search(r'\[mission_node-\d+\]: process started with pid \[(\d+)\]',
                                  log_file.read_text(encoding='utf-8'))
                assert match, '未找到本测试主程序的进程编号'
                os.kill(int(match.group(1)), signal.SIGTERM)
                requested_stop = True
            if (scenario == 'shutdown_during_fault_stop' and states and
                    states[-1]['state'] == 'STOPPING' and not requested_stop):
                match = re.search(r'\[mission_node-\d+\]: process started with pid \[(\d+)\]',
                                  log_file.read_text(encoding='utf-8'))
                assert match, '未找到本测试主程序的进程编号'
                os.kill(int(match.group(1)), signal.SIGTERM)
                requested_stop = True
            if scenario == 'shutdown_after_unconfirmed_stop' and states:
                if states[-1]['state'] == 'FAULT' and not requested_stop:
                    assert states[-1]['stop']['status'] == 'FAILED'
                    match = re.search(r'\[mission_node-\d+\]: process started with pid \[(\d+)\]',
                                      log_file.read_text(encoding='utf-8'))
                    assert match, '未找到本测试主程序的进程编号'
                    os.kill(int(match.group(1)), signal.SIGTERM)
                    requested_stop = True
                    continue
                if not requested_stop or states[-1]['stop']['status'] != 'CONFIRMED':
                    continue
            if scenario == 'operator_retries_stop_after_fault' and states:
                if states[-1]['state'] == 'FAULT' and not requested_stop:
                    original_fault = states[-1]['reason']
                    repeat_response = stop_client.call_async(Trigger.Request())
                    requested_stop = True
                    continue
                if requested_stop and states[-1]['state'] == 'STOPPING':
                    repeated_stop_seen = True
                if not repeated_stop_seen:
                    continue
            if states and states[-1]['state'] in ('FINISHED', 'FAULT', 'STOPPED'):
                break
        assert states and states[-1]['state'] == expected, (states[-1:] or '无状态')
        final = states[-1]
        # 默认流程只在货架识别 QR，到目标区由导航成功直接进入放下。
        assert not any(item['state'] == 'WAIT_END_QR' for item in states)
        assert health_states and all(set(item) == {'ready', 'fault'} for item in health_states)
        if scenario == 'operator_retries_stop_after_fault':
            assert repeated_stop_seen and repeat_response.done() and repeat_response.result().success
            assert final['reason'] == original_fault
            assert final['stop']['status'] == 'CONFIRMED'
        if scenario == 'shutdown_after_unconfirmed_stop':
            assert requested_stop and final['stop']['status'] == 'CONFIRMED'
        if scenario == 'shutdown_during_fault_stop':
            assert requested_stop and '模拟导航失败' in final['reason']
        if scenario == 'fault_during_delivery':
            assert final['cargo'] == 'UP'
            assert '停止未得到确认' not in final['reason']
            assert not any(state['state'] == 'LIFT_DOWN' for state in states)
        if scenario in ('normal', 'navigation_fail_once'):
            assert final['completed'] == final['delivered'] == ['A', 'B', 'C', 'D']
            racks = []
            for state in states:
                if state['state'] == 'DOCK_ENTER' and (not racks or racks[-1] != state['rack']):
                    racks.append(state['rack'])
            assert racks == ['A', 'B', 'C', 'D']
        else:
            assert final['completed'] == []
        if scenario in ('delivery_navigation_failure', 'operator_stop', 'shutdown_during_delivery'):
            assert final['cargo'] == 'UP'
            assert not any(state['state'] == 'LIFT_DOWN' for state in states)
        if scenario in ('lift_failure', 'lift_timeout', 'lowering_failure'):
            assert final['cargo'] == 'UNKNOWN'
        if scenario == 'exit_failure':
            assert final['delivered'] == ['A'] and final['cargo'] == 'EMPTY'
        # 导航失败次数耗尽不能跳过 A 开始 B。
        if scenario not in ('normal', 'navigation_fail_once'):
            assert not any(state['rack'] == 'B' for state in states)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        stream.close()
        observer.destroy_node()
        rclpy.shutdown()
