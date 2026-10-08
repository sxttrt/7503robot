"""不依赖 ROS 的任务状态机，集中管理固定顺序、超时和货物状态。

后端负责通信及状态读取，提供 start、cancel、stop、poll、ready、healthy、
stationary、cargo_observation、cargo_matches 方法。
所有完成回调只进入事件队列，统一在 tick 中处理，防止同步回调导致状态重入。
"""

from collections import deque
import time
import uuid


FINAL_STATES = {'FINISHED', 'FAULT', 'STOPPED'}
ACTION_STATES = {'NAV_RACK', 'WAIT_RACK_QR', 'DOCK_ENTER', 'LIFT_UP',
                 'NAV_END', 'WAIT_END_QR', 'LIFT_DOWN', 'DOCK_EXIT', 'WAIT_START'}
STATIONARY_STATES = {'WAIT_START', 'WAIT_RACK_QR', 'WAIT_END_QR'}


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
        self.fatal_error = ''
        # 最新停止请求的结果独立于任务结束原因，故障保留时仍能看清是否停稳。
        self.stop_status = 'NOT_REQUESTED'
        self.stop_result_reason = ''
        self.last_stop_request = None

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
        return (True, '已启用，等待有效 START') if self.state == 'WAIT_START' else (
            False, self.reason or '启用期间检查失败，正在停止')

    def enqueue(self, request_id, success, details=None):
        """携带原请求编号入队；旧回调不能影响新的动作。"""
        if not isinstance(details, dict) and details is not None:
            success, details = False, {'reason': '模块完成结果格式错误'}
        self.events.append((request_id, success is True, details or {}))

    def transition(self, state, reason='', before_notify=None):
        previous = self.state
        self.state, self.reason = state, reason
        self.history.append({'from': previous, 'to': state, 'rack': self.rack,
                             'reason': reason, 'time': self.clock()})
        # 停止命令必须先发出，日志、终端显示或状态发布再慢也不能挡住停机。
        if before_notify is not None:
            before_notify()
        try:
            self.on_transition(previous, state, reason)
        except Exception as exc:
            # 日志或显示异常不能打断真正的停止请求，也不能继续发起新动作。
            self.fatal_error = f'状态记录或发布异常：{exc}'

    def enter(self, state, retry=False):
        """先设置请求和状态，再发起通信，兼容立即完成的模拟回调。"""
        if self.fatal_error:
            self.fail_safe(self.fatal_error)
            return
        if self.started_at is not None and self.clock() - self.started_at >= self.policy['game_duration_sec']:
            self.stopping('比赛时间到，停止新增任务', terminal='FINISHED')
            return
        self.active_request = uuid.uuid4().hex
        self.deadline = self.clock() + self.policy['timeouts'][state]
        if not retry:
            self.attempts[state] = 0
        self.transition(state)
        if self.fatal_error:
            self.active_request = self.active_kind = None
            self.fail_safe(self.fatal_error)
            return
        # 记录或发布状态也可能耗时；发出动作之前再次检查全部启动条件。
        now = self.clock()
        if self.started_at is not None and now - self.started_at >= self.policy['game_duration_sec']:
            self.active_request = self.active_kind = None
            self.stopping('比赛时间到，停止新增任务', terminal='FINISHED')
            return
        if now >= self.deadline:
            self.active_request = self.active_kind = None
            self.stopping('进入状态期间超过步骤时限，禁止发起动作')
            return
        try:
            if not self.backend.healthy():
                self.active_request = self.active_kind = None
                self.stopping('发起动作前模块健康检查失败')
                return
            if not self.backend.stationary():
                self.active_request = self.active_kind = None
                self.stopping('发起动作前尚未确认静止，禁止交接控制权')
                return
            if self.cargo != 'UNKNOWN' and not self.backend.cargo_matches(self.cargo):
                self.cargo = 'UNKNOWN'
                self.active_request = self.active_kind = None
                self.stopping('发起动作前升降状态与携货记录不一致')
                return
        except Exception as exc:
            self.active_request = self.active_kind = None
            self.fail_safe(f'发起动作前状态检查异常：{exc}')
            return
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
        self.last_stop_request = self.stop_request
        self.stop_status = 'PENDING'
        self.stop_result_reason = '等待停止响应、新鲜静止状态和旧动作结束'
        self.deadline = self.clock() + self.policy['stop_timeout_sec']
        request = self.stop_request

        def dispatch_stop():
            try:
                self.backend.stop(request, self.enqueue)
            except Exception as exc:
                self.enqueue(request, False, {'reason': f'停止接口异常：{exc}'})

        self.transition('STOPPING', reason, before_notify=dispatch_stop)

    def manual_stop(self, reissue=False):
        """显式人工请求可在终态重发停止；退出循环只调用幂等的默认形式。

        重发仅重新确认停止，保留原有结束状态和原因，绝不重新启用比赛。
        已经等待停止时不重复下发，也不延长原来的截止时间。
        """
        if self.state == 'STOPPING':
            # 人工停止优先于已经安排的重试，关闭程序时也不能重新发起运动。
            target, terminal, reason = self.stop_plan
            if self.fatal_error:
                self.stop_plan = (None, 'FAULT', self.fatal_error)
            elif target is not None:
                self.stop_plan = (None, 'STOPPED', '操作员要求停止，已撤销自动重试')
            # 原本就在终止（尤其是 FAULT）的计划必须保留，不能改写故障原因。
            return True, '停止确认已在进行，未重复下发；请观察任务状态'
        elif self.state in FINAL_STATES:
            if reissue:
                self.stopping(self.reason or '操作员再次确认停止', terminal=self.state)
                return True, '已重新请求停止；保留本轮结束结果，不会恢复任务'
            return False, '本轮已经结束，退出检查未重复下发停止'
        elif self.state not in FINAL_STATES:
            self.stopping('操作员要求停止', terminal='STOPPED')
            return True, '已请求停止；响应不表示已静止，请观察任务状态'

    def prepare_shutdown(self):
        """退出入口只执行一次；未确认停止时再做一次有时限的停止尝试。

        定时器仍调用幂等的 manual_stop，不重置截止时间，也不会恢复任何任务。
        """
        if self.state in FINAL_STATES:
            try:
                confirmed = self.stop_status == 'CONFIRMED' and self.backend.stationary()
            except Exception:
                confirmed = False
            if not confirmed:
                self.manual_stop(reissue=True)
        else:
            self.manual_stop()

    def check_terminal_safety(self):
        """结束后继续监督已确认的静止；异常只触发一次新的停止握手。

        新握手失败或超时后保留未确认结果，不在每个 tick 无限重发停止。
        """
        if self.state not in FINAL_STATES or self.stop_status != 'CONFIRMED':
            return
        if not self.backend.stationary():
            reason = '任务结束后检测到运动或静止反馈失效，重新停止并等待人工检查'
            if self.reason:
                reason = self.reason + '；' + reason
            self.stopping(reason)

    def check_cargo_observation(self):
        """即使停机中或已结束，也不能把机构变化后的旧携货记录当作已确认。

        只使用新鲜且来源符合配置的升降观察；没有有效观察时保留最后记录。
        UNKNOWN 必须等原升降动作明确完成或人工检查，不能靠一个采样自动恢复。
        """
        if self.cargo == 'UNKNOWN' or self.state == 'IDLE':
            return
        observed = self.backend.cargo_observation()
        if observed is None or observed == self.cargo:
            return
        self.cargo = 'UNKNOWN'
        reason = '升降实际状态与携货记录不一致，停止并等待人工检查'
        if self.state == 'STOPPING':
            previous_reason = self.stop_plan[2]
            reason = previous_reason + '；' + reason if previous_reason else reason
            self.stop_plan = (None, 'FAULT', reason)
            # 原停止请求继续生效，禁止重置截止时间或继续之前的重试。
            self.transition('STOPPING', reason)
        else:
            if self.state in FINAL_STATES and self.reason:
                reason = self.reason + '；' + reason
            self.stopping(reason)

    def fail_safe(self, reason):
        """异常必须撤销重试；已经停止中的异常不得重置停止截止时间。"""
        self.fatal_error = reason
        if self.state == 'STOPPING':
            self.stop_plan = (None, 'FAULT', reason)
        elif self.state not in FINAL_STATES:
            self.stopping(reason)

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
        try:
            self.backend.poll()
        except Exception as exc:
            self.fail_safe(f'接口轮询异常：{exc}')
        now = self.clock()
        try:
            self.check_cargo_observation()
            self.check_terminal_safety()
        except Exception as exc:
            self.fail_safe(f'升降观察检查异常：{exc}')
        if self.state in FINAL_STATES or self.state == 'IDLE':
            self.events.clear()
            return
        time_up = self.started_at is not None and now - self.started_at >= self.policy['game_duration_sec']
        if time_up:
            if self.state == 'STOPPING':
                # 时间到优先于待执行的重试，但不重复发停止命令。
                if self.stop_plan[0] is not None:
                    self.stop_plan = (None, 'FINISHED', '比赛时间到，停止新增任务')
            else:
                self.stopping('比赛时间到，停止新增任务', terminal='FINISHED')
        if self.fatal_error:
            self.fail_safe(self.fatal_error)
        if self.state != 'STOPPING':
            try:
                if not self.backend.healthy():
                    self.stopping('必需模块断联、状态过期或报告故障')
                elif self.state in STATIONARY_STATES and not self.backend.stationary():
                    self.stopping('等待二维码期间检测到未静止，禁止进入下一动作')
            except Exception as exc:
                self.fail_safe(f'健康或升降状态检查异常：{exc}')
        # 截止时间先于结果处理，截止之后的成功不能继续推进流程。
        # 上面的停机记录或观察回调可能耗时，不能继续使用进入 tick 时的旧时间。
        now = self.clock()
        if self.deadline is not None and now >= self.deadline:
            if self.state == 'STOPPING':
                self.stop_request = None
                self.stop_status = 'TIMED_OUT'
                self.stop_result_reason = '停止未在时限内得到确认，请人工检查'
                self.transition('FAULT', self.stop_plan[2] + '；停止未得到确认，请人工检查；自动流程已终止')
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
                    self.stop_status = 'FAILED'
                    self.stop_result_reason = details.get('reason', '停止服务未说明失败原因')
                    self.transition('FAULT', reason + '；停止失败：' + details.get('reason', '未说明原因'))
                else:
                    self.stop_status = 'CONFIRMED'
                    self.stop_result_reason = '本次停止响应、新鲜静止状态及旧动作结束均已确认'
                if ok and target:
                    # 静止已确认仍不等于可以恢复；有故障或机构状态变化时禁止重试。
                    if not self.backend.healthy() or not self.backend.cargo_matches(self.cargo):
                        self.transition('FAULT', '停止已确认，但模块或升降状态不满足重试条件')
                    else:
                        self.enter(target, retry=True)
                elif ok:
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
                'attempt': self.attempts.get(self.state, 0), 'reason': self.reason,
                'stop': {'status': self.stop_status, 'request_id': self.last_stop_request,
                         'reason': self.stop_result_reason}}
