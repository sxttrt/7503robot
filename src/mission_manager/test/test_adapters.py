"""验证 QR 确认和动作取消竞态，使用消息时间与可控异步对象。"""

from types import SimpleNamespace

from mission_manager.adapters.navigation import AsyncAction
from mission_manager.adapters.qr import QR


def qr_reader():
    # 直接构造纯判定部分，不启动摄像头或 ROS 订阅。
    reader = QR.__new__(QR)
    reader.policy = {'qr_max_age_sec': 1.0, 'qr_future_tolerance_sec': 0.1,
                     'qr_confirm_frames': 3}
    outcomes = []
    reader.pending = ('request1', 'RACKA_TEAM2', lambda *args: outcomes.append(args), 10_000_000_000)
    reader.last_frame, reader.count = None, 0
    return reader, outcomes


def observation(ns, raw='RACKA_TEAM2'):
    return {'image_stamp': {'sec': ns // 1_000_000_000, 'nanosec': ns % 1_000_000_000},
            'status': 'ok', 'detections': [] if raw is None else [{'raw': raw}]}


def test_only_distinct_fresh_correct_frames_confirm():
    reader, outcomes = qr_reader()
    for _ in range(8):
        reader.observe(observation(10_100_000_000), 10_200_000_000)
    assert reader.count == 1 and not outcomes
    reader.observe(observation(10_200_000_000), 10_300_000_000)
    reader.observe(observation(10_300_000_000), 10_400_000_000)
    assert len(outcomes) == 1 and outcomes[0][1] is True


def test_wrong_team_missing_and_old_frame_never_confirm():
    reader, outcomes = qr_reader()
    for index in range(6):
        stamp = 10_100_000_000 + index * 100_000_000
        reader.observe(observation(stamp, 'RACKA_ANOTHERTEAM'), stamp)
    reader.observe(observation(1_000_000_000), 10_800_000_000)
    reader.observe(observation(10_900_000_000, None), 10_900_000_000)
    assert not outcomes and reader.count == 0


def test_empty_observation_resets_confirmation():
    reader, outcomes = qr_reader()
    reader.observe(observation(10_100_000_000), 10_100_000_000)
    reader.observe(observation(10_200_000_000), 10_200_000_000)
    reader.observe(observation(10_300_000_000, None), 10_300_000_000)
    reader.observe(observation(10_400_000_000), 10_400_000_000)
    assert reader.count == 1 and not outcomes


def test_future_frame_and_previous_wait_window_are_rejected():
    reader, outcomes = qr_reader()
    reader.observe(observation(20_000_000_000), 10_000_000_000)
    reader.observe(observation(9_990_000_000), 10_000_000_000)
    assert not outcomes and reader.count == 0


def test_cancelled_qr_wait_cannot_trigger():
    reader, outcomes = qr_reader()
    reader.cancel('request1')
    for index in range(5):
        reader.observe(observation(10_100_000_000 + index), 10_100_000_000 + index)
    assert not outcomes


def test_late_goal_acceptance_still_cancels_old_action():
    action = AsyncAction.__new__(AsyncAction)
    cancelled = []
    class Future:
        def add_done_callback(self, callback):
            self.callback = callback
    result_future = Future()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result_future,
                             cancel_goal_async=lambda: cancelled.append(True))
    action.requests = {'old': {'handle': None, 'cancelled': False, 'callback': lambda *_: None}}
    action.cancel('old')
    assert not action.idle()
    action.accepted('old', SimpleNamespace(result=lambda: handle))
    assert cancelled == [True] and not action.idle()


def test_rejected_old_action_is_removed_without_success():
    action = AsyncAction.__new__(AsyncAction)
    outcomes = []
    action.requests = {'old': {'handle': None, 'cancelled': True,
                               'callback': lambda *args: outcomes.append(args)}}
    action.accepted('old', SimpleNamespace(result=lambda: SimpleNamespace(accepted=False)))
    assert action.idle() and outcomes[0][1] is False
