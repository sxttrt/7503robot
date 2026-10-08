"""导航动作适配；同时提供对准动作可复用的异步取消处理。"""

import math

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class AsyncAction:
    """跟踪接受、结果及取消；超时后才接受的旧目标也必须取消。"""

    def __init__(self, node, action_type, name, safety=None):
        self.node = node
        self.client = ActionClient(node, action_type, name)
        self.requests = {}
        self.safety = safety
        self.feedback = {}

    def available(self):
        return self.client.server_is_ready()

    def idle(self):
        return not self.requests

    def release_cancelled(self):
        # 停止模块已报告完成，无需再等待旧动作结果；回调仍按旧编号过滤。
        self.requests = {key: value for key, value in self.requests.items() if not value['cancelled']}

    def send(self, request, goal, callback):
        if self.requests:
            callback(request, False, {'reason': '已有动作请求，禁止同时下发新目标'})
            return
        if not self.available():
            callback(request, False, {'reason': '动作服务端未就绪'})
            return
        record = {'cancelled': False, 'handle': None, 'callback': callback}
        self.requests[request] = record
        self.feedback = {}
        try:
            future = self.client.send_goal_async(
                goal, feedback_callback=lambda message: self.on_feedback(request, message.feedback))
            future.add_done_callback(lambda result: self.accepted(request, result))
        except Exception as exc:
            self.unconfirmed_failure(request, f'发送动作异常，执行状态未确认：{exc}')

    def unconfirmed_failure(self, request, reason):
        """通信异常不能证明服务端已经停止；保留记录以阻止重试重叠。"""
        record = self.requests.get(request)
        if record and not record.get('reported'):
            record['reported'] = True
            record['callback'](request, False, {'reason': reason})

    def accepted(self, request, future):
        record = self.requests.get(request)
        if record is None:
            # 停止成功后才接受的旧目标仍要取消，不能变成一项新任务。
            try:
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            except Exception:
                pass
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
            self.unconfirmed_failure(request, f'动作接受或取消异常，执行状态未确认：{exc}')

    def on_feedback(self, request, feedback):
        """进度只用于观察，不把距离很小或某个阶段名当作任务完成。"""
        if request not in self.requests or self.requests[request]['cancelled']:
            return
        self.feedback = {'request_id': request}
        if hasattr(feedback, 'phase'):
            self.feedback['phase'] = feedback.phase
        if hasattr(feedback, 'distance_remaining'):
            self.feedback['distance_remaining_m'] = float(feedback.distance_remaining)

    def result(self, request, future):
        record = self.requests.get(request)
        if record is None:
            return
        try:
            response = future.result()
        except Exception as exc:
            self.unconfirmed_failure(request, f'读取动作结果异常，执行状态未确认：{exc}')
            return
        status = getattr(response, 'status', None)
        if status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                          GoalStatus.STATUS_CANCELED):
            self.unconfirmed_failure(request, f'动作返回非终止状态：{status}')
            return
        try:
            success, details = self.decode(response.result)
            success = success and response.status == GoalStatus.STATUS_SUCCEEDED
            if record['cancelled'] or record.get('reported'):
                success = False
            if not success:
                details['reason'] = details.get('reason') or f'动作结束状态：{response.status}'
                self.finish(request, False, details)
            else:
                # 信任模块的完成语义：到达并停止后返回成功，直接推进下一步。
                self.finish(request, True, details)
        except Exception as exc:
            # 已确认 ROS 动作终止，但结果字段无效，不能报告成功。
            self.finish(request, False, {'reason': f'读取动作结果失败：{exc}'})


    def finish(self, request, success, details):
        record = self.requests.pop(request, None)
        if record and not record.get('reported'):
            record['callback'](request, success, details)

    def cancel(self, request):
        record = self.requests.get(request)
        if record:
            record['cancelled'] = True
            if record['handle'] is not None:
                try:
                    record['handle'].cancel_goal_async()
                except Exception as exc:
                    self.unconfirmed_failure(request, f'取消请求异常，执行状态未确认：{exc}')


class Navigation(AsyncAction):
    """输入地图位姿，只有动作最终成功才报告到达。"""

    def __init__(self, node, name, safety=None):
        super().__init__(node, NavigateToPose, name, safety)

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
        code = getattr(result, 'error_code', None)
        if type(code) is not int or code < 0:
            return False, {'reason': '导航结果缺少合法错误码，不能确认成功'}
        return code == 0, {'reason': getattr(result, 'error_msg', '') or f'导航错误码：{code}'}
