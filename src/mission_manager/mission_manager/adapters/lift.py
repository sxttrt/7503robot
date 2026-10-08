"""升降服务适配；第一版严格要求服务响应表示动作已经完成。"""

import time

from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class Lift:
    def __init__(self, node, interfaces, policy, safety):
        self.node, self.policy, self.safety = node, policy, safety
        self.client = node.create_client(SetBool, interfaces['lift_service'])
        self.subscription = node.create_subscription(Bool, interfaces['lift_state_topic'], self.on_state, 10)
        self.requests = {}
        self.pending_results = {}

    def on_state(self, message):
        # 普通 Bool 仅作显示参考，不作为当前升降完成的唯一依据。
        self.safety.last_up = bool(message.data)
        self.safety.last_up_received = time.monotonic()

    def available(self):
        return self.client.service_is_ready()

    def idle(self):
        return not self.requests

    def start(self, request, payload, callback):
        if self.requests or self.pending_results:
            callback(request, False, {'reason': '旧升降请求或到位确认尚未结束，禁止重复下发'})
            return
        if not self.available():
            callback(request, False, {'reason': '升降服务未就绪'})
            return
        info = {'up': bool(payload['up']), 'callback': callback, 'cancelled': False,
                'begin_ns': self.node.get_clock().now().nanoseconds}
        self.requests[request] = info
        message = SetBool.Request()
        message.data = info['up']
        try:
            self.client.call_async(message).add_done_callback(lambda result: self.result(request, result))
        except Exception as exc:
            # 请求可能已经到达机构，发送异常也不能视为动作已停止。
            info['reported'] = True
            callback(request, False, {'reason': f'升降请求异常，执行状态未确认：{exc}'})

    def result(self, request, future):
        info = self.requests.get(request)
        if info is None:
            return
        try:
            result = future.result()
            if type(getattr(result, 'success', None)) is not bool:
                raise ValueError('升降响应缺少合法完成字段')
        except Exception as exc:
            # 响应传输异常不能证明机构动作已经终止，仍阻止停止握手提前通过。
            if not info['cancelled'] and not info.get('reported'):
                info['reported'] = True
                info['callback'](request, False, {'reason': f'升降结果异常，执行状态未确认：{exc}'})
            return
        self.requests.pop(request)
        if info['cancelled'] or info.get('reported'):
            return
        try:
            if not result.success:
                info['callback'](request, False, {'reason': result.message or '升降失败'})
            else:
                # 响应和健康状态可能通过不同连接乱序到达，因此等待新鲜状态。
                info['begin_ns'] = self.node.get_clock().now().nanoseconds
                self.pending_results[request] = info
        except Exception as exc:
            info['callback'](request, False, {'reason': f'升降结果异常：{exc}'})

    def poll(self):
        if not self.safety.healthy():
            return
        data = self.safety.latest
        for request, info in list(self.pending_results.items()):
            if self.safety.stamp_ns <= info['begin_ns'] or data.get('lift_is_up') is not info['up']:
                continue
            source = data.get('lift_state_source')
            if source not in ('measured', 'estimated'):
                continue
            self.pending_results.pop(request)
            allowed = source == 'measured' or self.policy['allow_estimated_lift']
            info['callback'](request, allowed, {
                'is_up': info['up'], 'source': source,
                'reason': '到位确认完成' if allowed else '配置禁止使用推定升降状态',
            })

    def cancel(self, request):
        # 服务不能强行取消；停止确认还会等待旧服务回调结束。
        info = self.requests.get(request)
        if info:
            info['cancelled'] = True
        self.pending_results.pop(request, None)
