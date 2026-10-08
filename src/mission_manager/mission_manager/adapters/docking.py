"""对准模块接口；几何计算和实际底盘移动由队友服务端实现。"""

from mission_interfaces.action import Dock

from .navigation import AsyncAction


class Docking(AsyncAction):
    def __init__(self, node, name, safety=None):
        super().__init__(node, Dock, name, safety)

    def start(self, request, payload, callback):
        goal = Dock.Goal()
        goal.request_id = request
        goal.operation = payload['operation']
        goal.rack_id = payload['rack_id']
        goal.expected_qr = payload['expected_qr']
        goal.max_duration_sec = float(payload['max_duration_sec'])
        self.send(request, goal, callback)

    def decode(self, result):
        return bool(result.success), {
            'ready_to_lift': bool(result.ready_to_lift),
            'exited': bool(result.exited),
            'reason': result.message or result.error_code or '对准模块未说明原因',
        }
