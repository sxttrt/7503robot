"""模拟导航、二维码、对准、升降和健康状态；不连接任何真实硬件。

模拟服务端使用真实 ROS 类型，主程序无需为模拟绕过接口。
此文件里的睡眠只模拟服务端动作耗时，多线程执行器保持健康定时器工作。
"""

import json
import threading
import time

from mission_interfaces.action import Dock
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .config_loader import load_config, read_yaml


class MockModules(Node):
    def __init__(self):
        super().__init__('mock_modules')
        for key, default in (('mission_file', ''), ('interfaces_file', ''),
                             ('targets_file', ''), ('scenarios_file', ''), ('scenario', 'normal')):
            self.declare_parameter(key, default)
        self.config = load_config(
            self.get_parameter('mission_file').value,
            self.get_parameter('interfaces_file').value,
            self.get_parameter('targets_file').value, 'simulation',
        )
        scenarios = read_yaml(self.get_parameter('scenarios_file').value)['scenarios']
        selected = self.get_parameter('scenario').value
        if selected not in scenarios:
            raise ValueError(f'未知模拟场景：{selected}；可选项：{list(scenarios)}')
        self.scenario = scenarios[selected] or {}
        self.lock = threading.RLock()
        self.group = ReentrantCallbackGroup()
        self.status = {'state': 'IDLE', 'rack': 'A'}
        self.counts = {}
        self.is_up = False
        self.motion_count = 0
        self.stop_epoch = 0
        self.started_at = time.monotonic()
        self.mission_active_at = None
        names = self.config['interfaces']
        self.qr_pub = self.create_publisher(String, names['qr_topic'], 10)
        self.health_pub = self.create_publisher(String, names['health_topic'], 10)
        self.lift_pub = self.create_publisher(Bool, names['lift_state_topic'], 10)
        self.status_sub = self.create_subscription(String, names['mission_status_topic'], self.on_status, 10,
                                                   callback_group=self.group)
        self.nav_server = ActionServer(self, NavigateToPose, names['navigation_action'],
                                       self.navigate, goal_callback=self.accept_goal,
                                       cancel_callback=self.accept_cancel, callback_group=self.group)
        self.dock_server = ActionServer(self, Dock, names['docking_action'],
                                        self.dock, goal_callback=self.accept_goal,
                                        cancel_callback=self.accept_cancel, callback_group=self.group)
        self.lift_server = self.create_service(SetBool, names['lift_service'], self.lift,
                                               callback_group=self.group)
        self.stop_server = self.create_service(Trigger, names['stop_service'], self.stop,
                                               callback_group=self.group)
        self.timer = self.create_timer(0.1, self.publish_observations, callback_group=self.group)
        self.get_logger().info(f'仅模拟模块已启动，场景={selected}；不发送真实速度或访问硬件')

    def on_status(self, message):
        try:
            data = json.loads(message.data)
            with self.lock:
                self.status = data
                if data.get('state') != 'IDLE' and self.mission_active_at is None:
                    self.mission_active_at = time.monotonic()
        except (ValueError, TypeError):
            return

    def accept_goal(self, _request):
        # 此场景可用于验证超时之后才接受的旧目标，不能因此触发下一任务。
        time.sleep(float(self.scenario.get('accept_delay_sec', 0.0)))
        return GoalResponse.ACCEPT

    @staticmethod
    def accept_cancel(_handle):
        return CancelResponse.ACCEPT

    def count(self, state):
        with self.lock:
            self.counts[state] = self.counts.get(state, 0) + 1
            return self.counts[state]

    def should_fail(self, state):
        return self.count(state) <= int(self.scenario.get('failures', {}).get(state, 0))

    def wait_action(self, handle, state, duration, feedback):
        """持续处理取消和停止；动作结束后才报告实际停止。"""
        with self.lock:
            epoch = self.stop_epoch
            self.motion_count += 1
        begin = time.monotonic()
        delayed_cancel = False
        try:
            while rclpy.ok():
                with self.lock:
                    stopped = epoch != self.stop_epoch
                if handle.is_cancel_requested or stopped:
                    if not delayed_cancel:
                        delayed_cancel = True
                        time.sleep(float(self.scenario.get('cancel_result_delay_sec', 0.0)))
                    return 'cancelled'
                if state not in self.scenario.get('timeout_states', []) and time.monotonic() - begin >= duration:
                    return 'finished'
                handle.publish_feedback(feedback)
                time.sleep(0.05)
            return 'cancelled'
        finally:
            with self.lock:
                self.motion_count -= 1

    def navigate(self, handle):
        pose = handle.request.pose.pose.position
        destination = self.config['targets']['destination_pose']
        state = 'NAV_END' if abs(pose.x - destination[0]) < 1e-6 and abs(pose.y - destination[1]) < 1e-6 else 'NAV_RACK'
        failed = self.should_fail(state)
        outcome = self.wait_action(handle, state, float(self.scenario.get('action_duration_sec', 0.35)),
                                   NavigateToPose.Feedback())
        result = NavigateToPose.Result()
        if outcome == 'cancelled':
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        elif failed:
            result.error_code = 1
            result.error_msg = '模拟导航失败'
            handle.abort()
        else:
            handle.succeed()
        return result

    def dock(self, handle):
        goal = handle.request
        result = Dock.Result()
        if goal.operation not in ('ENTER', 'EXIT') or goal.rack_id not in ('A', 'B', 'C', 'D'):
            result.message = '操作或货架编号无效'
            handle.abort()
            return result
        if goal.expected_qr != self.config['targets']['racks'][goal.rack_id]['qr']:
            result.message = '对准请求中的货架二维码不匹配'
            handle.abort()
            return result
        state = 'DOCK_ENTER' if goal.operation == 'ENTER' else 'DOCK_EXIT'
        failed = self.should_fail(state)
        feedback = Dock.Feedback()
        feedback.phase = '对准进入中' if goal.operation == 'ENTER' else '退出中'
        outcome = self.wait_action(handle, state, float(self.scenario.get('action_duration_sec', 0.35)), feedback)
        if outcome == 'cancelled':
            result.message = '模拟动作已经停止'
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        elif failed:
            result.message = '模拟对准或退出失败'
            handle.abort()
        else:
            result.success = True
            result.ready_to_lift = goal.operation == 'ENTER' and not self.scenario.get('not_ready_to_lift', False)
            result.exited = goal.operation == 'EXIT'
            result.message = '模拟动作完成'
            handle.succeed()
        return result

    def lift(self, request, response):
        state = 'LIFT_UP' if request.data else 'LIFT_DOWN'
        failed = self.should_fail(state)
        with self.lock:
            epoch = self.stop_epoch
        begin = time.monotonic()
        while rclpy.ok():
            with self.lock:
                stopped = epoch != self.stop_epoch
            if stopped:
                response.success, response.message = False, '升降已按停止请求中止，保持当前状态'
                return response
            if state not in self.scenario.get('timeout_states', []) and time.monotonic() - begin >= float(self.scenario.get('lift_duration_sec', 0.25)):
                break
            time.sleep(0.05)
        if failed:
            response.success, response.message = False, '模拟升降失败'
        else:
            with self.lock:
                self.is_up = bool(request.data)
            response.success, response.message = True, '模拟升降实际完成，状态随后发布'
        return response

    def stop(self, _request, response):
        with self.lock:
            self.stop_epoch += 1
        response.success = not self.scenario.get('stop_failure', False)
        response.message = '停止请求已执行；等待新鲜静止状态确认'
        return response

    def publish_observations(self):
        with self.lock:
            state, rack = self.status.get('state'), self.status.get('rack')
            is_up, stopped = self.is_up, self.motion_count == 0
            stop_requested = self.stop_epoch > 0
            lift_requested = self.counts.get('LIFT_UP', 0) > 0
        stamp = self.get_clock().now().to_msg()
        active_elapsed = 0.0 if self.mission_active_at is None else time.monotonic() - self.mission_active_at
        if active_elapsed < float(self.scenario.get('health_dropout_after_sec', 1e9)):
            health = {'schema_version': 1, 'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
                      'modules': {name: True for name in ('navigation', 'qr', 'docking', 'lift', 'base')},
                      'stopped': stopped and not (stop_requested and self.scenario.get('stop_never_confirmed', False)),
                      'lift_is_up': is_up,
                      'lift_state_source': self.scenario.get('lift_state_source', 'measured') if lift_requested else 'measured',
                      'fault': ''}
            self.health_pub.publish(String(data=json.dumps(health, ensure_ascii=False)))
        self.lift_pub.publish(Bool(data=is_up))
        raw = None
        if state == 'WAIT_START' and not self.scenario.get('no_start', False):
            raw = 'START'
        elif state == 'WAIT_RACK_QR' and rack in self.config['targets']['racks']:
            raw = self.config['targets']['racks'][rack]['qr']
        elif state == 'WAIT_END_QR':
            raw = 'END'
        if state in self.scenario.get('qr_wrong_states', []):
            raw = 'RACKA_OTHERTEAM'
        if state in self.scenario.get('qr_empty_states', []):
            raw = None
        if self.scenario.get('qr_stale', False):
            stamp.sec -= 20
        qr = {'image_stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
              'processed_at_ns': self.get_clock().now().nanoseconds, 'frame_id': 'camera_link',
              'status': 'ok', 'detections': [] if raw is None else [{'raw': raw}]}
        self.qr_pub.publish(String(data=json.dumps(qr, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = MockModules()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # 主程序退出时已请求停止；这里同时让模拟动作循环结束。
        if rclpy.ok():
            rclpy.shutdown()
        executor.shutdown(timeout_sec=3.0)
        node.destroy_node()
