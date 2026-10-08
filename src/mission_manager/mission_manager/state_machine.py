"""不依赖 ROS 的任务状态机，集中管理固定顺序、超时和货物状态。

后端负责实际通信，提供 start、cancel、stop、poll、ready、healthy 方法。
所有完成回调只进入事件队列，统一在 tick 中处理，防止同步回调导致状态重入。
"""

from collections import deque
import time
import uuid


FINAL_STATES = {'FINISHED', 'FAULT', 'STOPPED'}
ACTION_STATES = {'NAV_RACK', 'WAIT_RACK_QR', 'DOCK_ENTER', 'LIFT_UP',
                 'NAV_END', 'WAIT_END_QR', 'LIFT_DOWN', 'DOCK_EXIT', 'WAIT_START'}


class Mission:
    """一个实例代表一轮比赛；终止后必须重新启动，禁止自动恢复运动。"""

    def __init__(self, config, backend, clock=time.monotonic, on_transition=None):
        self.config = config
        self.policy = config['mission']
        self.targets = config['targets']
        self.backend = backend
        self.clock = clock
        self.on_transition = on_transition or (lambda *_: None)
        self.state = 'IDLE'
        self.index = 0
        self.cargo = 'EMPTY'
        self.completed = []
        self.delivered = []
        self.started_at = None
        self.deadline = None
        self.active_request = None
        self.active_kind = None
        self.stop_request = None
        self.stop_plan = None
        self.reason = ''
        self.attempts = {}
        self.events = deque()
        self.history = []

    @property
    def rack(self):
        """只按固定列表读取货架，不做自动重排或故障跳过。"""
        order = self.policy['rack_order']
        return order[self.index] if self.index < len(order) else None

    def arm(self):
        """操作员启用后才观察 START；启用本身不启动比赛计时。"""
        if self.state != 'IDLE':
            return False, '当前不是待启用状态，本轮不允许再次启用'
        if not self.backend.ready():
            return False, '模块或接口尚未就绪，请检查任务状态中的诊断信息'
        self.enter('WAIT_START')
        return True, '已启用，等待有效 START'

    def enqueue(self, request_id, success, details=None):
        """携带原请求编号入队；旧回调不能影响新的动作。"""
        self.events.append((request_id, bool(success), details or {}))

    def transition(self, state, reason=''):
        previous = self.state
        self.state, self.reason = state, reason
        self.history.append({'from': previous, 'to': state, 'rack': self.rack,
                             'reason': reason, 'time': self.clock()})
        self.on_transition(previous, state, reason)

    def enter(self, state, retry=False):
        """先设置请求和状态，再发起通信，兼容立即完成的模拟回调。"""
        self.active_request = uuid.uuid4().hex
        self.deadline = self.clock() + self.policy['timeouts'][state]
        if not retry:
            self.attempts[state] = 0
        self.transition(state)
        rack = self.rack
        item = self.targets['racks'].get(rack, {})
        if state in ('WAIT_START', 'WAIT_RACK_QR', 'WAIT_END_QR'):
            self.active_kind = 'qr'
            expected = {'WAIT_START': 'START', 'WAIT_END_QR': 'END'}.get(state, item.get('qr'))
            payload = {'expected_qr': expected}
        elif state in ('NAV_RACK', 'NAV_END'):
            self.active_kind = 'navigation'
            payload = {'pose': item['approach_pose'] if state == 'NAV_RACK'
                       else self.targets['destination_pose'], 'frame_id': self.targets['frame_id']}
        elif state in ('DOCK_ENTER', 'DOCK_EXIT'):
            self.active_kind = 'docking'
            payload = {'operation': 'ENTER' if state == 'DOCK_ENTER' else 'EXIT',
                       'rack_id': rack, 'expected_qr': item['qr'],
                       'max_duration_sec': self.policy['timeouts'][state]}
        else:
            self.active_kind = 'lift'
            # 动作发出后，到位前均不能声称货物状态已被确认。
            self.cargo = 'UNKNOWN'
            payload = {'up': state == 'LIFT_UP'}
        request = self.active_request
        try:
            self.backend.start(self.active_kind, request, payload, self.enqueue)
        except Exception as exc:
            self.enqueue(request, False, {'reason': f'接口调用异常：{exc}'})

    def invalidate_action(self):
        """使请求先失效再取消；取消过程中到达的旧成功结果会被忽略。"""
        request, kind = self.active_request, self.active_kind
        self.active_request = self.active_kind = None
        if request:
            try:
                self.backend.cancel(kind, request)
            except Exception as exc:
                self.reason = f'取消请求异常：{exc}'

    def stopping(self, reason, target=None, terminal='FAULT'):
        """任何重试先停机确认，禁止两个运动动作同时获得控制权。"""
        self.invalidate_action()
        self.stop_plan = (target, terminal, reason)
        self.stop_request = uuid.uuid4().hex
        self.deadline = self.clock() + self.policy['stop_timeout_sec']
        self.transition('STOPPING', reason)
        try:
            self.backend.stop(self.stop_request, self.enqueue)
        except Exception as exc:
            self.enqueue(self.stop_request, False, {'reason': f'停止接口异常：{exc}'})

    def manual_stop(self):
        if self.state == 'STOPPING':
            # 人工停止优先于已经安排的重试，关闭程序时也不能重新发起运动。
            self.stop_plan = (None, 'STOPPED', '操作员要求停止')
        elif self.state not in FINAL_STATES:
            self.stopping('操作员要求停止', terminal='STOPPED')

    def failed(self, reason):
        """只有空载前往货架及扫码等待允许自动重试，其他失败直接终止。"""
        allowed = self.state in ('NAV_RACK', 'WAIT_RACK_QR', 'WAIT_END_QR')
        tries = self.attempts.get(self.state, 0)
        limit = self.policy['retries'].get(self.state, 0)
        target = self.state if allowed and tries < limit else None
        if target:
            self.attempts[target] = tries + 1
        self.stopping(reason, target=target)

    def success(self, details):
        """各阶段校验完成条件；二维码身份不等价于可托举。"""
        state = self.state
        self.active_request = self.active_kind = None
        if state == 'WAIT_START':
            self.started_at = self.clock()
            self.enter('NAV_RACK')
        elif state == 'NAV_RACK':
            self.enter('WAIT_RACK_QR')
        elif state == 'WAIT_RACK_QR':
            self.enter('DOCK_ENTER')
        elif state == 'DOCK_ENTER':
            if details.get('ready_to_lift') is not True:
                self.failed('对准模块未明确确认可托举')
            else:
                self.enter('LIFT_UP')
        elif state in ('LIFT_UP', 'LIFT_DOWN'):
            expected = state == 'LIFT_UP'
            if details.get('is_up') is not expected:
                self.failed('升降完成结果与请求方向不一致')
            else:
                self.cargo = 'UP' if expected else 'EMPTY'
                if expected:
                    self.enter('NAV_END')
                else:
                    # 到目标区并确认放下，才记录已交付；退出成功另记完整闭环。
                    self.delivered.append(self.rack)
                    self.enter('DOCK_EXIT')
        elif state == 'NAV_END':
            self.enter('WAIT_END_QR' if self.policy['confirm_end_qr'] else 'LIFT_DOWN')
        elif state == 'WAIT_END_QR':
            self.enter('LIFT_DOWN')
        elif state == 'DOCK_EXIT':
            if details.get('exited') is not True:
                self.failed('退出模块未明确确认已脱离货架')
            else:
                self.completed.append(self.rack)
                self.index += 1
                if self.rack is None:
                    self.stopping('A、B、C、D 全部完成', terminal='FINISHED')
                else:
                    self.enter('NAV_RACK')

    def tick(self):
        """定期处理事件和截止时间；每次调用都不等待外部动作结束。"""
        self.backend.poll()
        now = self.clock()
        if self.state in FINAL_STATES or self.state == 'IDLE':
            self.events.clear()
            return
        time_up = self.started_at is not None and now - self.started_at >= self.policy['game_duration_sec']
        if time_up:
            if self.state == 'STOPPING':
                # 时间到优先于待执行的重试，但不重复发停止命令。
                self.stop_plan = (None, 'FINISHED', '比赛时间到，停止新增任务')
            else:
                self.stopping('比赛时间到，停止新增任务', terminal='FINISHED')
        if self.state != 'STOPPING' and not self.backend.healthy():
            self.stopping('必需模块断联、状态过期或报告故障')
        # 截止时间先于结果处理，截止之后的成功不能继续推进流程。
        if self.deadline is not None and now >= self.deadline:
            if self.state == 'STOPPING':
                self.stop_request = None
                self.transition('FAULT', '停止未得到确认，请人工检查；自动流程已终止')
            elif self.state == 'WAIT_START':
                # START 尚未出现时不启动比赛，只报告等待超时并重新观察。
                self.invalidate_action()
                self.enter('WAIT_START')
            else:
                self.failed(f'{self.state} 等待超时')
        while self.events:
            request, ok, details = self.events.popleft()
            if self.state == 'STOPPING' and request == self.stop_request:
                self.stop_request = None
                target, terminal, reason = self.stop_plan
                if not ok:
                    self.transition('FAULT', '停止失败：' + details.get('reason', '未说明原因'))
                elif target:
                    self.enter(target, retry=True)
                else:
                    self.transition(terminal, reason)
            elif self.state in ACTION_STATES and request == self.active_request:
                if ok:
                    self.success(details)
                else:
                    self.failed(details.get('reason', '模块返回失败，未说明原因'))

    def status(self):
        """用于终端、模拟模块和日志的统一状态快照。"""
        elapsed = 0.0 if self.started_at is None else max(0.0, self.clock() - self.started_at)
        return {'schema_version': 1, 'team_number': 2, 'mode': self.config['mode'],
                'state': self.state, 'rack': self.rack, 'request_id': self.active_request,
                'cargo': self.cargo, 'completed': list(self.completed),
                'delivered': list(self.delivered), 'remaining_sec': max(
                    0.0, self.policy['game_duration_sec'] - elapsed),
                'attempt': self.attempts.get(self.state, 0), 'reason': self.reason}
