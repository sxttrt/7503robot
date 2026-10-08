"""升降接口：发送升／降，直接使用队友返回的实际完成结果。"""

import time
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class Lift:
    def __init__(self, node, interfaces, policy, safety):
        self.node, self.safety = node, safety
        self.client = node.create_client(SetBool, interfaces['lift_service'])
        self.subscription = node.create_subscription(Bool, interfaces['lift_state_topic'], self.on_state, 10)
        self.requests = {}

    def on_state(self, message):
        # 可选的状态显示，不参与主流程的二次到位校验。
        self.safety.last_up = bool(message.data)
        self.safety.last_up_received = time.monotonic()

    def available(self):
        return self.client.service_is_ready()

    def start(self, request, payload, callback):
        if self.requests:
            callback(request, False, {'reason': '已有升降请求，禁止同时下发'})
            return
        if not self.available():
            callback(request, False, {'reason': '升降服务未就绪'})
            return
        info = {'up': bool(payload['up']), 'callback': callback, 'cancelled': False}
        self.requests[request] = info
        message = SetBool.Request()
        message.data = info['up']
        try:
            self.client.call_async(message).add_done_callback(lambda future: self.result(request, future))
        except Exception as exc:
            # 由统一停止路径处理可能已到达的请求，避免重复下发。
            info['reported'] = True
            callback(request, False, {'reason': f'升降请求异常：{exc}'})

    def result(self, request, future):
        info = self.requests.get(request)
        if info is None:
            return
        try:
            result = future.result()
            success = result.success is True
            details = {'is_up': info['up'], 'reason': result.message or '升降返回结果'}
        except Exception as exc:
            success, details = False, {'reason': f'升降响应异常：{exc}'}
        self.requests.pop(request, None)
        if not info['cancelled'] and not info.get('reported'):
            info['callback'](request, success, details)

    def cancel(self, request):
        # Service 不能撤回；停止交给统一停止接口，旧响应不推进任务。
        if request in self.requests:
            self.requests[request]['cancelled'] = True

    def release_cancelled(self):
        # 执行模块确认停止后允许新任务；迟到的旧响应仍会被忽略。
        self.requests = {key: value for key, value in self.requests.items() if not value['cancelled']}
