"""健康状态和停止确认；停止响应与实际静止状态分别检查。"""

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
        self.stamp_ns = None
        self.last_up_received = None
        self.last_up = None
        self.pending = None
        self.diagnostic = '等待模块健康状态'
        # 兼容可靠和尽力发送端，只处理最近状态，避免旧消息队列延迟停止判断。
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = node.create_subscription(String, interfaces['health_topic'], self.on_status, qos)
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
            if (type(data.get('schema_version')) is not int or data['schema_version'] != 1 or
                    type(data.get('lift_is_up')) is not bool or
                    data.get('lift_state_source') not in ('measured', 'estimated') or
                    not isinstance(data.get('fault'), str)):
                return
            if any(type(data['modules'].get(name)) is not bool
                   for name in ('navigation', 'qr', 'docking', 'lift', 'base')):
                return
            self.latest, self.received_at, self.stamp_ns = data, time.monotonic(), stamp_ns
        except (ValueError, TypeError, KeyError, AttributeError):
            self.diagnostic = '健康状态格式错误，等待合法数据'

    def fresh(self):
        """同时检查收包时间和源时间戳，防止延迟状态被额外沿用一个超时周期。"""
        if self.received_at is None or time.monotonic() - self.received_at >= self.policy['health_timeout_sec']:
            self.diagnostic = '模块健康状态尚未到达或已过期'
            return False
        age = (self.node.get_clock().now().nanoseconds - self.stamp_ns) / 1e9
        if age >= self.policy['health_timeout_sec'] or age < -self.policy['qr_future_tolerance_sec']:
            self.diagnostic = '模块健康状态源时间戳已过期或时钟不一致'
            return False
        return True

    def healthy(self):
        if not self.fresh():
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

    def lift_matches(self, up):
        """实际到位值和反馈来源必须同时满足配置要求。"""
        return (self.healthy() and self.latest['lift_is_up'] is up and
                (self.latest['lift_state_source'] == 'measured' or self.policy['allow_estimated_lift']))

    def lift_observation(self):
        """模块报故障时仍读取合法到位值，防止停机后的携货信息继续失真。

        此方法仅反映升降观察，不能用来证明模块健康或允许恢复运动。
        """
        if not self.fresh():
            return None
        if self.latest['lift_state_source'] != 'measured' and not self.policy['allow_estimated_lift']:
            return None
        return 'UP' if self.latest['lift_is_up'] else 'EMPTY'

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
        if (self.fresh() and self.stamp_ns > pending['begin_ns'] and
                self.latest['stopped'] is True):
            self.pending = None
            pending['callback'](pending['request'], True, {'reason': '停止响应和新鲜静止状态均已确认'})
