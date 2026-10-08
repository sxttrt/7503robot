"""导航动作适配；同时提供对准动作可复用的异步取消处理。"""

from action_msgs.msg import GoalStatus
from mission_interfaces.action import NavigateToPoint
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
    """只发送点位名称；具体地图、坐标、朝向和路线由导航模块决定。"""

    def __init__(self, node, name, safety=None):
        super().__init__(node, NavigateToPoint, name, safety)

    def start(self, request, payload, callback):
        goal = NavigateToPoint.Goal()
        goal.target_id = payload['target_id']
        self.send(request, goal, callback)

    def decode(self, result):
        # 队友只返回完成结果和原因，不再要求导航错误码或额外到点反馈。
        success = getattr(result, 'success', None) is True
        return success, {'reason': getattr(result, 'message', '') or ('已到达目标点' if success else '导航失败')}
