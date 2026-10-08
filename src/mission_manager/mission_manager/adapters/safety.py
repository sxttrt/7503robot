"""简单在线状态与停止接口：信任执行模块返回的完成结果。"""

import json
import time

from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class Safety:
    def __init__(self, node, interfaces, policy):
        self.node, self.policy = node, policy
        self.latest = None
        self.received_at = None
        self.last_up = None
        self.last_up_received = None
        self.pending = None
        self.diagnostic = '等待在线反馈'
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = node.create_subscription(String, interfaces['health_topic'], self.on_status, qos)
        self.client = node.create_client(Trigger, interfaces['stop_service'])

    def on_status(self, message):
        # 队友只需定期发布两个字段，不要求源时间戳、令牌或传感器来源。
        try:
            data = json.loads(message.data)
            if type(data.get('ready')) is not bool or not isinstance(data.get('fault', ''), str):
                return
            self.latest = {'ready': data['ready'], 'fault': data.get('fault', '')}
            self.received_at = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            self.diagnostic = '在线反馈格式错误'

    def fresh(self):
        return (self.received_at is not None and
                time.monotonic() - self.received_at < self.policy['health_timeout_sec'])

    def healthy(self):
        if not self.fresh():
            self.diagnostic = '在线反馈未到达或已中断'
            return False
        if not self.latest['ready'] or self.latest['fault']:
            self.diagnostic = self.latest['fault'] or '执行模块尚未就绪'
            return False
        self.diagnostic = '执行模块就绪'
        return True

    def available(self):
        return self.client.service_is_ready()

    def stop(self, request, callback):
        if not self.available():
            callback(request, False, {'reason': '停止服务未就绪'})
            return
        record = {'request': request, 'callback': callback}
        self.pending = record
        try:
            self.client.call_async(Trigger.Request()).add_done_callback(
                lambda future: self.on_stop_response(record, future))
        except Exception as exc:
            self.pending = None
            callback(request, False, {'reason': f'停止请求异常：{exc}'})

    def on_stop_response(self, record, future):
        if self.pending is not record:
            return
        self.pending = None
        try:
            result = future.result()
            success = result.success is True
            details = {'reason': result.message or ('停止完成' if success else '停止失败')}
        except Exception as exc:
            success, details = False, {'reason': f'停止响应异常：{exc}'}
        # 成功响应即表示执行模块已经停止，不再等待第二份状态或旧动作回执。
        record['callback'](record['request'], success, details)
