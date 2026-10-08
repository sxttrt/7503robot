"""健康状态和停止确认；停止响应与实际静止状态分别检查。"""

import json
import time

from std_msgs.msg import String
from std_srvs.srv import Trigger


class Safety:
    def __init__(self, node, interfaces, policy):
        self.node, self.policy = node, policy
        self.latest = None
        self.received_at = None
        self.stamp_ns = None
        self.last_up_received = None
        self.last_up = None
        self.pending = None
        self.diagnostic = '等待模块健康状态'
        self.subscription = node.create_subscription(String, interfaces['health_topic'], self.on_status, 10)
        self.client = node.create_client(Trigger, interfaces['stop_service'])

    def on_status(self, message):
        try:
            data = json.loads(message.data)
            stamp = data['stamp']
            if type(stamp['sec']) is not int or type(stamp['nanosec']) is not int:
                return
            if not 0 <= stamp['nanosec'] < 1_000_000_000:
                return
            stamp_ns = stamp['sec'] * 1_000_000_000 + stamp['nanosec']
            now_ns = self.node.get_clock().now().nanoseconds
            age = (now_ns - stamp_ns) / 1e9
            if stamp_ns <= 0 or age > self.policy['health_timeout_sec'] or age < -self.policy['qr_future_tolerance_sec']:
                return
            if self.stamp_ns is not None and stamp_ns <= self.stamp_ns:
                return
            if not isinstance(data.get('modules'), dict) or type(data.get('stopped')) is not bool:
                return
            self.latest, self.received_at, self.stamp_ns = data, time.monotonic(), stamp_ns
        except (ValueError, TypeError, KeyError, AttributeError):
            self.diagnostic = '健康状态格式错误，等待合法数据'

    def healthy(self):
        if self.received_at is None or time.monotonic() - self.received_at > self.policy['health_timeout_sec']:
            self.diagnostic = '模块健康状态尚未到达或已过期'
            return False
        modules = self.latest['modules']
        if any(modules.get(name) is not True for name in ('navigation', 'qr', 'docking', 'lift', 'base')):
            self.diagnostic = '至少一个必需模块未就绪'
            return False
        if self.latest.get('fault'):
            self.diagnostic = '模块故障：' + str(self.latest['fault'])
            return False
        self.diagnostic = '健康状态正常'
        return True

    def available(self):
        return self.client.service_is_ready()

    def stop(self, request, callback):
        if not self.available():
            callback(request, False, {'reason': '停止服务未就绪'})
            return
        pending = {'request': request, 'callback': callback, 'ack': False,
                   'begin_ns': self.node.get_clock().now().nanoseconds}
        self.pending = pending
        try:
            future = self.client.call_async(Trigger.Request())
            future.add_done_callback(lambda result: self.on_stop_response(pending, result))
        except Exception as exc:
            self.pending = None
            callback(request, False, {'reason': f'停止请求异常：{exc}'})

    def on_stop_response(self, pending, future):
        if self.pending is not pending:
            return
        try:
            result = future.result()
            if not result.success:
                self.pending = None
                pending['callback'](pending['request'], False, {'reason': result.message})
            else:
                pending['ack'] = True
        except Exception as exc:
            self.pending = None
            pending['callback'](pending['request'], False, {'reason': str(exc)})

    def poll(self, actions_idle):
        pending = self.pending
        if not pending or not pending['ack'] or not actions_idle:
            return
        # 必须有请求之后生成的新健康状态，不能用缓存的“已停止”。
        if (self.healthy() and self.stamp_ns >= pending['begin_ns'] and
                self.latest['stopped'] is True):
            self.pending = None
            pending['callback'](pending['request'], True, {'reason': '停止响应和新鲜静止状态均已确认'})
