"""进退架几何与闭环执行边界；不控制升降，不读取仿真真值或私有 QR 缓存。

仅已标定架口内允许不使用 costmap 占据判定。几何走廊、场界、实测
位置/航向仍持续检查；其它导航仍由正常地图规划器负责。
"""
import math
import re

from .core import effective_half_size, route_distance, wrap
from .point_targets import PointTargets


class DockGeometry:
    def __init__(self, data, cfg):
        self.cfg = cfg
        self.points = PointTargets(data)
        self.racks = data.get('docking', {})
        if not isinstance(self.racks, dict):
            raise ValueError('docking 必须是按货架填写的字典')

    def request(self, request, simulation=False):
        if self.points.simulation_only and not simulation:
            raise ValueError('实机禁止使用仿真进退架几何')
        if (not request.request_id or request.rack_id not in 'ABCD' or
                len(request.rack_id) != 1 or request.operation not in ('ENTER', 'EXIT')):
            raise ValueError('invalid Dock request')
        if not re.fullmatch(r'RACK' + request.rack_id + r'_[A-Za-z0-9]+', request.expected_qr):
            raise ValueError('Dock requires full expected rack QR')
        if not math.isfinite(request.max_duration_sec) or not 0 < request.max_duration_sec <= 600:
            raise ValueError('invalid Dock duration')
        spec = self.racks.get(request.rack_id)
        if not isinstance(spec, dict):
            raise ValueError('进退架几何尚未实测填写：' + request.rack_id)
        bounds = spec.get('bounds')
        if (not isinstance(bounds, list) or len(bounds) != 4 or
                not all(type(v) in (int, float) and math.isfinite(v) for v in bounds)):
            raise ValueError('bounds 必须为 [xmin,xmax,ymin,ymax]')
        x0, x1, y0, y1 = bounds
        if not (0 < x0 < x1 < self.cfg['field_size'][0] and
                0 < y0 < y1 < self.cfg['field_size'][1]):
            raise ValueError('rack bounds outside field or inverted')
        for key in ('leg_thickness', 'exit_distance', 'max_entry_distance'):
            value = spec.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('docking.' + key + ' 必须为正有限数')
        if 2 * spec['leg_thickness'] >= y1 - y0:
            raise ValueError('rack mouth has no clearance')
        approach = self.points.checked_pose(self.points.points.get(request.rack_id))
        drop = self.points.checked_pose(self.points.drops.get(request.rack_id))
        if any(abs(wrap(p[2] - self.cfg['travel_yaw'])) > 1e-6 for p in (approach, drop)):
            raise ValueError('Dock points must use travel heading')
        return spec, approach, drop

    def plan(self, request, pose, *, carrying, context, simulation=False):
        spec, approach, drop = self.request(request, simulation)
        if carrying:
            raise ValueError('Dock requires measured platform down and no cargo')
        if (not context or context.get('state') != 'DOCK_' + request.operation or
                context.get('rack') != request.rack_id or context.get('cargo') != 'EMPTY' or
                context.get('request_id') != request.request_id):
            raise ValueError('waiting for current Dock mission context')
        if abs(wrap(pose[2] - self.cfg['travel_yaw'])) > self.cfg['yaw_tolerance']:
            raise ValueError('Dock heading must be restored before entering corridor')
        bounds = spec['bounds']
        center = ((bounds[0] + bounds[1]) / 2, (bounds[2] + bounds[3]) / 2)
        start = approach[:2] if request.operation == 'ENTER' else drop[:2]
        if math.dist(pose[:2], start) > self.cfg['position_tolerance']:
            raise ValueError('Dock start is not the measured approach/drop point')
        if request.operation == 'ENTER':
            # Lateral correction takes place outside the legs; then translate along x.
            points = [pose[:2], (pose[0], center[1]), center]
            if approach[0] <= bounds[1]:
                raise ValueError('approach must be outside positive-x rack mouth')
            if sum(math.dist(a, b) for a, b in zip(points, points[1:])) > spec['max_entry_distance']:
                raise ValueError('entry exceeds calibrated corridor length')
            legs = bounds
        else:
            # Released rack is now at its drop point, not its original scene position.
            dx, dy = drop[0] - center[0], drop[1] - center[1]
            legs = [bounds[0] + dx, bounds[1] + dx, bounds[2] + dy, bounds[3] + dy]
            points = [pose[:2], (pose[0], drop[1]), (drop[0] + spec['exit_distance'], drop[1])]
        corridor = DockCorridor(points, legs, spec['leg_thickness'], self.cfg,
                                exiting=request.operation == 'EXIT')
        return corridor


class DockCorridor:
    def __init__(self, points, bounds, thickness, cfg, *, exiting=False):
        self.points = [tuple(p) for i, p in enumerate(points) if i == 0 or tuple(p) != tuple(points[i-1])]
        if len(self.points) == 1:
            self.points.append(self.points[0])
        self.cfg = cfg
        x0, x1, y0, y1 = bounds
        # A conservative full-depth strip includes all four feet of this rack.
        self.legs = [(x0, x1, y0, y0 + thickness), (x0, x1, y1 - thickness, y1)]
        reserve = cfg['margin'] + cfg['position_tolerance']
        for a, b in zip(self.points, self.points[1:]):
            if a[0] != b[0] and a[1] != b[1]:
                raise ValueError('Dock route must be axis aligned')
            self.check_sweep(a, b, reserve)
        hx, _hy = effective_half_size(cfg, False)
        if exiting and self.points[-1][0] - hx - reserve <= x1:
            raise ValueError('exit distance does not fully clear rack')

    def check_sweep(self, a, b, margin):
        hx, hy = effective_half_size(self.cfg, False)
        box = (min(a[0], b[0]) - hx - margin, max(a[0], b[0]) + hx + margin,
               min(a[1], b[1]) - hy - margin, max(a[1], b[1]) + hy + margin)
        if not (box[0] >= 0 and box[1] <= self.cfg['field_size'][0] and
                box[2] >= 0 and box[3] <= self.cfg['field_size'][1]):
            raise ValueError('Dock footprint exceeds field boundary')
        for x0, x1, y0, y1 in self.legs:
            if box[0] < x1 and box[1] > x0 and box[2] < y1 and box[3] > y0:
                raise ValueError('Dock footprint intersects rack legs/clearance')

    def validate(self, pose, remaining):
        if abs(wrap(pose[2] - self.cfg['travel_yaw'])) > self.cfg['yaw_tolerance']:
            raise ValueError('Dock heading left calibrated axis')
        if route_distance(pose, remaining) > self.cfg['position_tolerance']:
            raise ValueError('tracker left Dock corridor')
        self.check_sweep(pose[:2], pose[:2], self.cfg['margin'])


class DockGoalHandle:
    """把同一个 TrackPath 的距离反馈转为框架 Dock phase。"""
    def __init__(self, handle):
        self.handle = handle

    @property
    def is_cancel_requested(self):
        return self.handle.is_cancel_requested

    def publish_feedback(self, feedback):
        from mission_interfaces.action import Dock
        value = Dock.Feedback()
        value.phase = f'{self.handle.request.operation}: {feedback.distance_remaining:.3f} m remaining'
        self.handle.publish_feedback(value)
