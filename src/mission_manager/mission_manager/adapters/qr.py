"""接收已有 QR 节点的带时间戳 JSON，过滤过期、错码和重复帧。"""

import json

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class QR:
    def __init__(self, node, name, policy):
        self.node, self.policy = node, policy
        self.pending = None
        self.last_frame = None
        self.count = 0
        # 尽力接收兼容可靠和尽力发送端；历史缓存不用于触发新任务。
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = node.create_subscription(String, name, self.on_message, qos)

    def start(self, request, payload, callback):
        self.pending = (request, payload['expected_qr'], callback,
                        self.node.get_clock().now().nanoseconds)
        self.last_frame, self.count = None, 0

    def cancel(self, request):
        if self.pending and self.pending[0] == request:
            self.pending = None
            self.count = 0

    def on_message(self, message):
        try:
            data = json.loads(message.data)
            now = self.node.get_clock().now().nanoseconds
            self.observe(data, now)
        except (ValueError, TypeError, KeyError, AttributeError):
            self.count = 0

    def observe(self, data, now_ns):
        """单独保留观察判定，便于不启动摄像头就验证异常数据。"""
        if not self.pending:
            return
        request, expected, callback, begin_ns = self.pending
        stamp = data['image_stamp']
        if type(stamp['sec']) is not int or type(stamp['nanosec']) is not int:
            self.count = 0
            return
        if not 0 <= stamp['nanosec'] < 1_000_000_000:
            self.count = 0
            return
        image_ns = stamp['sec'] * 1_000_000_000 + stamp['nanosec']
        age = (now_ns - image_ns) / 1e9
        if image_ns < begin_ns or image_ns <= 0 or age > self.policy['qr_max_age_sec'] or age < -self.policy['qr_future_tolerance_sec']:
            self.count = 0
            return
        if self.last_frame is not None and image_ns <= self.last_frame:
            return
        if self.last_frame is not None and (image_ns - self.last_frame) / 1e9 > self.policy['qr_max_age_sec']:
            # 图像长时间中断后，连续确认必须从第一帧重新计数。
            self.count = 0
        self.last_frame = image_ns
        detections = data.get('detections')
        if not isinstance(detections, list) or data.get('status') != 'ok':
            self.count = 0
            return
        valid = any(isinstance(item, dict) and item.get('raw') == expected for item in detections)
        self.count = self.count + 1 if valid else 0
        if self.count >= self.policy['qr_confirm_frames']:
            self.pending = None
            callback(request, True, {'raw': expected, 'image_stamp_ns': image_ns})
