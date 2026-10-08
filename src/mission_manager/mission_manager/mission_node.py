"""ROS 入口：读取配置、创建适配器、发布状态和接收操作员控制。"""

import json
from pathlib import Path
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .adapters.docking import Docking
from .adapters.lift import Lift
from .adapters.navigation import Navigation
from .adapters.qr import QR
from .adapters.safety import Safety
from .config_loader import load_config
from .state_machine import FINAL_STATES, Mission


class ROSBackend:
    """状态机唯一的外部执行入口；模拟与实机共用此后端。"""

    def __init__(self, node, config):
        self.node = node
        names, policy = config['interfaces'], config['mission']
        self.safety = Safety(node, names, policy)
        self.navigation = Navigation(node, names['navigation_action'], self.safety)
        self.docking = Docking(node, names['docking_action'], self.safety)
        self.lift = Lift(node, names, policy, self.safety)
        self.qr = QR(node, names['qr_topic'], policy)
        self.modules = {'navigation': self.navigation, 'docking': self.docking,
                        'lift': self.lift, 'qr': self.qr}

    def ready(self):
        # 在线反馈与接口可用即可启用，不要求额外的升降传感器或静止确认。
        return (self.healthy() and self.safety.available() and self.navigation.available()
                and self.docking.available() and self.lift.available())

    def healthy(self):
        return self.safety.healthy()

    def start(self, kind, request, payload, callback):
        if self.node.closing:
            callback(request, False, {'reason': '程序正在退出，禁止发起新动作'})
            return
        self.modules[kind].start(request, payload, callback)

    def cancel(self, kind, request):
        self.modules[kind].cancel(request)

    def stop(self, request, callback):
        def completed(stop_request, success, details):
            if success:
                for module in (self.navigation, self.docking, self.lift):
                    module.release_cancelled()
            callback(stop_request, success, details)
        self.safety.stop(request, completed)

    def poll(self):
        # 通信由 ROS 异步回调驱动，不增加完成后的二次握手。
        pass


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_manager')
        for key, default in (('mode', 'simulation'), ('mission_file', ''),
                             ('interfaces_file', ''), ('targets_file', ''),
                             ('auto_arm', False), ('log_dir', '~/7503robot_ws/runtime_logs')):
            self.declare_parameter(key, default)
        self.config = load_config(
            self.get_parameter('mission_file').value,
            self.get_parameter('interfaces_file').value,
            self.get_parameter('targets_file').value,
            self.get_parameter('mode').value,
        )
        self.auto_arm = self.get_parameter('auto_arm').value
        if self.config['mode'] == 'robot' and self.auto_arm:
            raise ValueError('实机模式必须人工启用，禁止自动启用')
        self.closing = False
        self.publisher = self.create_publisher(String, self.config['interfaces']['mission_status_topic'], 10)
        log_dir = Path(self.get_parameter('log_dir').value).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / (time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns()}_mission.jsonl')
        self.log_stream = self.log_path.open('a', encoding='utf-8', buffering=1)
        self.backend = ROSBackend(self, self.config)
        self.mission = Mission(self.config, self.backend, on_transition=self.on_transition)
        self.arm_service = self.create_service(Trigger, 'mission/arm', self.arm)
        self.stop_service = self.create_service(Trigger, 'mission/stop', self.stop)
        self.timer = self.create_timer(self.config['mission']['tick_period_sec'], self.on_tick)
        self.status_timer = self.create_timer(0.2, self.publish_status)
        self.get_logger().info(f'第二队任务系统已启动，模式={self.config["mode"]}；货架固定 A→B→C→D')
        self.get_logger().info(f'运行日志：{self.log_path}')

    def on_transition(self, previous, state, reason):
        line = {'event': 'transition', 'from': previous, 'to': state, 'reason': reason,
                'stamp_ns': self.get_clock().now().nanoseconds, **self.mission.status()}
        self.log_stream.write(json.dumps(line, ensure_ascii=False) + '\n')
        self.get_logger().info(f'{previous} → {state}；货架={self.mission.rack}；原因={reason or "正常推进"}')
        self.publish_status()

    def publish_status(self):
        data = self.mission.status()
        data['diagnostic'] = self.backend.safety.diagnostic
        data['ready_to_arm'] = self.mission.state == 'IDLE' and self.backend.ready()
        data['progress'] = {'navigation': self.backend.navigation.feedback,
                            'docking': self.backend.docking.feedback}
        data['stamp_ns'] = self.get_clock().now().nanoseconds
        self.publisher.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def arm(self, _request, response):
        response.success, response.message = self.mission.arm()
        return response

    def stop(self, _request, response):
        response.success, response.message = self.mission.manual_stop(reissue=True)
        return response

    def on_tick(self):
        try:
            if self.closing:
                self.mission.manual_stop()
            elif self.auto_arm and self.mission.state == 'IDLE':
                self.mission.arm()
            self.mission.tick()
        except Exception as exc:
            self.mission.fail_safe(f'主循环异常：{exc}')


def main(args=None):
    """退出前尽力完成停止握手；底盘独立看门狗仍由真实控制模块负责。"""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_handlers = {}
    try:
        node = MissionNode()

        def request_shutdown(_signal, _frame):
            # 信号可打断状态切换；这里只置标志，停止在主循环的安全边界执行。
            node.closing = True

        for kind in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[kind] = signal.signal(kind, request_shutdown)
        try:
            while rclpy.ok() and not node.closing:
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as exc:
            node.closing = True
            node.mission.fail_safe(f'ROS 回调或主循环异常：{exc}')
    finally:
        if node is not None:
            node.closing = True
            node.mission.prepare_shutdown()
            until = time.monotonic() + node.config['mission']['stop_timeout_sec'] + 0.5
            while rclpy.ok() and node.mission.state not in FINAL_STATES and time.monotonic() < until:
                try:
                    rclpy.spin_once(node, timeout_sec=0.1)
                except Exception as exc:
                    node.mission.fail_safe(f'退出期间回调异常：{exc}')
            try:
                node.publish_status()
            except Exception:
                # 发布状态失效不影响已经执行的停止握手和后续资源关闭。
                pass
        for kind, handler in previous_handlers.items():
            signal.signal(kind, handler)
        if node is not None:
            try:
                node.log_stream.close()
            finally:
                # 日志关闭失败也必须销毁 ROS 节点；运动停止握手已在前面执行。
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
