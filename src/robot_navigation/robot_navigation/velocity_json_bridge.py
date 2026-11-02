"""把车体 /cmd_vel Twist 转为供 ESP32 通信模块使用的 JSON String。

正常指令逐条转发，不重发缓存的非零速度。输入断流/非法时发布零速；
此节点不操作串口、不发布 cmd_vel、不把发送成功当作实物完成反馈。
"""
import json
import math
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .resources import navigation_config
from .core import load_settings


def zero_json():
    return json.dumps({'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                       'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                      separators=(',', ':'), allow_nan=False)


def encode_velocity(message, max_linear_speed, max_angular_speed):
    """保留原始车体方向和 SI 单位；拒绝无法由本车执行的输入。"""
    linear = (message.linear.x, message.linear.y, message.linear.z)
    angular = (message.angular.x, message.angular.y, message.angular.z)
    if not all(type(v) in (int, float) and math.isfinite(v) for v in linear + angular):
        raise ValueError('velocity must contain six finite numbers')
    if linear[2] != 0 or angular[0] != 0 or angular[1] != 0:
        raise ValueError('only linear.x, linear.y and angular.z are supported')
    if (math.hypot(*linear[:2]) > max_linear_speed + 1e-9 or
            abs(angular[2]) > max_angular_speed + 1e-9):
        raise ValueError('velocity exceeds navigation speed limits')
    return json.dumps({'linear': dict(zip(('x', 'y', 'z'), map(float, linear))),
                       'angular': dict(zip(('x', 'y', 'z'), map(float, angular)))},
                      separators=(',', ':'), allow_nan=False)


class VelocityJsonBridge(Node):
    def __init__(self):
        super().__init__('velocity_json_bridge')
        self.declare_parameter('config_file', str(navigation_config()))
        self.cfg = load_settings(self.get_parameter('config_file').value)
        self.last_received = None
        self.closing = False
        self.last_error = ''
        # 与跟踪器一样使用可靠、volatile、深度1；不向新接收端提供历史命令。
        self.output = self.create_publisher(String, self.cfg['cmd_vel_json_topic'], 1)
        self.input = self.create_subscription(Twist, self.cfg['cmd_vel_topic'], self.on_twist, 1)
        # 即使 Gazebo 暂停、/clock 不更新，断流停车仍以单调墙钟计时。
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.cfg['tracking_frequency'], self.watchdog, clock=self.wall_clock)
        self.get_logger().info(
            f'Velocity JSON: {self.input.topic_name} -> {self.output.topic_name}; '
            f'input timeout={self.cfg["tracking_command_timeout_sec"]:.3f}s; base_link m/s, rad/s')

    def publish_zero(self):
        self.output.publish(String(data=zero_json()))

    def on_twist(self, message):
        if self.closing:
            self.publish_zero()
            return
        try:
            payload = encode_velocity(message, self.cfg['max_linear_speed'], self.cfg['max_angular_speed'])
        except ValueError as exc:
            self.last_received = None
            self.publish_zero()
            reason = str(exc)
            if reason != self.last_error:
                self.get_logger().error('JSON bridge rejected command; sent zero: ' + reason)
            self.last_error = reason
            return
        self.last_error = ''
        self.last_received = time.monotonic()
        self.output.publish(String(data=payload))

    def watchdog(self):
        age = None if self.last_received is None else time.monotonic() - self.last_received
        if self.closing or age is None or not 0 <= age < self.cfg['tracking_command_timeout_sec']:
            self.publish_zero()


def main(args=None):
    quitting = threading.Event()
    previous = {sig: signal.signal(sig, lambda _sig, _frame: quitting.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = VelocityJsonBridge()
        while rclpy.ok() and not quitting.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        try:
            if node:
                node.closing = True
                if rclpy.ok():
                    # 退出期间拒绝排队中的非零命令；给零速短暂的 DDS 发送时间。
                    node.publish_zero()
                    for _ in range(3):
                        if rclpy.ok():
                            rclpy.spin_once(node, timeout_sec=0.05)
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
