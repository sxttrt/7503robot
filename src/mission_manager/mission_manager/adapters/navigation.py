"""导航动作适配；同时提供对准动作可复用的异步取消处理。"""

import math

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class AsyncAction:
    """跟踪接受、结果及取消；超时后才接受的旧目标也必须取消。"""

    def __init__(self, node, action_type, name):
        self.node = node
        self.client = ActionClient(node, action_type, name)
        self.requests = {}
        self.feedback = {}

    def available(self):
        return self.client.server_is_ready()

    def idle(self):
        # 停止确认还需要所有旧动作都得到最终结果或被拒绝。
        return not self.requests

    def send(self, request, goal, callback):
        if not self.available():
            callback(request, False, {'reason': '动作服务端未就绪'})
            return
        record = {'cancelled': False, 'handle': None, 'callback': callback}
        self.requests[request] = record
        try:
            future = self.client.send_goal_async(
                goal, feedback_callback=lambda message: self.on_feedback(request, message.feedback))
            future.add_done_callback(lambda result: self.accepted(request, result))
        except Exception as exc:
            self.finish(request, False, {'reason': f'发送动作失败：{exc}'})

    def accepted(self, request, future):
        record = self.requests.get(request)
        if record is None:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                self.finish(request, False, {'reason': '动作目标被拒绝'})
                return
            record['handle'] = handle
            handle.get_result_async().add_done_callback(lambda result: self.result(request, result))
            if record['cancelled']:
                handle.cancel_goal_async()
        except Exception as exc:
            self.finish(request, False, {'reason': f'动作接受或取消异常：{exc}'})

    def on_feedback(self, request, feedback):
        """进度只用于观察，不把距离很小或某个阶段名当作任务完成。"""
        if request not in self.requests:
            return
        self.feedback = {'request_id': request}
        if hasattr(feedback, 'phase'):
            self.feedback['phase'] = feedback.phase
        if hasattr(feedback, 'distance_remaining'):
            self.feedback['distance_remaining_m'] = float(feedback.distance_remaining)

    def result(self, request, future):
        try:
            response = future.result()
            success, details = self.decode(response.result)
            success = success and response.status == GoalStatus.STATUS_SUCCEEDED
            if not success:
                details.setdefault('reason', f'动作结束状态：{response.status}')
            self.finish(request, success, details)
        except Exception as exc:
            self.finish(request, False, {'reason': f'读取动作结果失败：{exc}'})

    def finish(self, request, success, details):
        record = self.requests.pop(request, None)
        if record:
            record['callback'](request, success, details)

    def cancel(self, request):
        record = self.requests.get(request)
        if record:
            record['cancelled'] = True
            if record['handle'] is not None:
                record['handle'].cancel_goal_async()


class Navigation(AsyncAction):
    """输入地图位姿，只有动作最终成功才报告到达。"""

    def __init__(self, node, name):
        super().__init__(node, NavigateToPose, name)

    def start(self, request, payload, callback):
        goal = NavigateToPose.Goal()
        x, y, yaw = payload['pose']
        goal.pose.header.frame_id = payload['frame_id']
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.send(request, goal, callback)

    def decode(self, result):
        code = int(getattr(result, 'error_code', 0))
        return code == 0, {'reason': getattr(result, 'error_msg', '') or f'导航错误码：{code}'}
