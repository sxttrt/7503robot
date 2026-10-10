"""框架只传点位名；坐标、放置顺序和内部 Pose 动作归导航模块管理。"""
import math

from geometry_msgs.msg import PoseStamped
from mission_interfaces.action import NavigateToPoint
from nav2_msgs.action import NavigateToPose


class PointTargets:
    def __init__(self, data):
        if not isinstance(data, dict) or type(data.get('simulation_only')) is not bool:
            raise ValueError('导航点位配置必须明确 simulation_only')
        self.simulation_only = data['simulation_only']
        self.points = data.get('points', {})
        self.drops = data.get('drop_by_rack', {})
        if not isinstance(self.points, dict) or not isinstance(self.drops, dict):
            raise ValueError('points 和 drop_by_rack 必须为字典')
        for table in (self.points, self.drops):
            for pose in table.values():
                if pose is not None:
                    self.checked_pose(pose)

    @staticmethod
    def checked_pose(pose):
        if (not isinstance(pose, (list, tuple)) or len(pose) != 3 or
                not all(type(x) in (int, float) and math.isfinite(x) for x in pose)):
            raise ValueError('点位必须填写 [x米, y米, yaw弧度]，三个有限数字')
        return tuple(pose)

    def resolve(self, target_id, *, carrying, context=None, simulation=False):
        if self.simulation_only and not simulation:
            raise ValueError('实机禁止使用仿真导航点位')
        allowed = {'A', 'B', 'C', 'D', 'DROP_OFF', 'START_SCAN', 'FINAL_SCAN'}
        if target_id not in allowed:
            raise ValueError('未知点位名：' + str(target_id))
        if target_id == 'DROP_OFF':
            if not carrying:
                raise ValueError('DROP_OFF 要求已确认载货')
            if self.drops:
                # 同一个 DROP_OFF 请求按当前货架选择里到外、左到右的落点。
                # 上下文来自主节点当前状态，不能按调用次数猜测当前货物。
                context = context or {}
                if (context.get('state') != 'NAV_END' or context.get('cargo') != 'UP'
                        or context.get('rack') not in self.drops):
                    raise ValueError('DROP_OFF 缺少新鲜的 NAV_END 载货上下文')
                pose = self.drops[context['rack']]
            else:
                pose = self.points.get(target_id)
        else:
            if carrying:
                raise ValueError('货架接近点和扫描点只能空载导航')
            pose = self.points.get(target_id)
        if pose is None:
            raise ValueError('导航点位尚未实测填写：' + target_id)
        return self.checked_pose(pose)


def stamped_target(target, stamp):
    result = PoseStamped()
    result.header.frame_id = 'map'
    result.header.stamp = stamp
    result.pose.position.x, result.pose.position.y = map(float, target[:2])
    result.pose.orientation.z = math.sin(target[2] / 2)
    result.pose.orientation.w = math.cos(target[2] / 2)
    return result


class PointGoalHandle:
    """复用原有规划和实测停止流程；对外仍返回 NavigateToPoint 类型。"""
    def __init__(self, handle, pose):
        self.handle = handle
        self.request = NavigateToPose.Goal()
        self.request.pose = pose

    @property
    def is_cancel_requested(self):
        return self.handle.is_cancel_requested

    def publish_feedback(self, feedback):
        value = NavigateToPoint.Feedback()
        value.phase = f'前往 {self.handle.request.target_id}，剩余 {feedback.distance_remaining:.3f} m'
        self.handle.publish_feedback(value)

    def succeed(self):
        self.handle.succeed()

    def abort(self):
        self.handle.abort()

    def canceled(self):
        self.handle.canceled()


def point_result(pose_result):
    result = NavigateToPoint.Result()
    result.success = pose_result.error_code == 0
    result.message = pose_result.error_msg or ('已到达并实测停稳' if result.success else '导航失败')
    return result
