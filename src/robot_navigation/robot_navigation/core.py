"""无 ROS 的实机校验与几何；复用项目内已验证算法，不导入仿真执行桥。"""
import math
import sys
import numpy as np
import yaml
from mission_manager.project_paths import navigation_root

sys.path.insert(0, str(navigation_root() / 'scripts'))
from lidar_geometry import WallLocalizer, scan_points, wrap
from grid_planner import GridMap, Planner
from costmap_cache import CostmapCache
from published_costmap_cache import PublishedCostmapCache

def load_settings(path):
    with open(path, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict) or type(cfg.get('calibration_verified')) is not bool:
        raise ValueError('calibration_verified 必须显式填写布尔值')
    for name, size in [('field_size',2),('initial_pose',3),('laser_pose',4),
                       ('empty_half_size',2),('loaded_half_size',2)]:
        value = cfg.get(name)
        if not isinstance(value,list) or len(value)!=size or not all(
                type(x) in (int,float) and math.isfinite(x) for x in value):
            raise ValueError(name + ' 必须填写有限数字数组')
    for name in ('field_size','empty_half_size','loaded_half_size'):
        if min(cfg[name]) <= 0: raise ValueError(name+' 必须大于零')
    if any(a>b for a,b in zip(cfg['empty_half_size'],cfg['loaded_half_size'])):
        raise ValueError('载货包络必须包含空车')
    if not (0<cfg['initial_pose'][0]<cfg['field_size'][0] and
            0<cfg['initial_pose'][1]<cfg['field_size'][1]):
        raise ValueError('initial_pose 必须在四壁场地内')
    for name in ('max_linear_speed','max_angular_speed','position_tolerance',
                 'yaw_tolerance','stop_linear_speed','stop_angular_speed',
                 'settle_duration_sec','max_duration_sec','feedback_timeout_sec',
                 'cancel_timeout_sec','costmap_max_age_sec','tracking_frequency',
                 'tracking_linear_acceleration','tracking_angular_acceleration',
                 'tracking_command_timeout_sec'):
        x=cfg.get(name)
        if type(x) not in (int,float) or not math.isfinite(x) or x<=0:
            raise ValueError(name+' 必须是正有限数')
    # Defaults preserve older deployed configurations; explicit values are validated.
    for name,default in (('tracking_progress_timeout_sec',8.),
                         ('tracking_progress_distance',.005),('tracking_progress_angle',.02)):
        cfg.setdefault(name,default)
        if type(cfg[name]) not in (int,float) or not math.isfinite(cfg[name]) or cfg[name]<=0:
            raise ValueError(name+' 必须是正有限数')
    if cfg['tracking_progress_distance']>=cfg['position_tolerance']:
        raise ValueError('进展距离必须小于到位容差')
    if cfg['tracking_progress_angle']>=cfg['yaw_tolerance']:
        raise ValueError('进展角度必须小于朝向容差')
    if cfg['tracking_frequency']<10 or cfg['tracking_frequency']>100:
        raise ValueError('tracking_frequency 必须在 10~100Hz')
    if cfg['tracking_command_timeout_sec']<=2/cfg['tracking_frequency']:
        raise ValueError('控制超时必须大于两个控制周期')
    if cfg['max_linear_speed']>.3 or cfg['max_angular_speed']>.375:
        raise ValueError('速度超出当前雷达定位器验证范围')
    if type(cfg.get('margin')) not in (int,float) or not math.isfinite(cfg['margin']) or cfg['margin']<0:
        raise ValueError('margin 必须非负')
    if type(cfg.get('travel_yaw')) not in (int,float) or not math.isfinite(cfg['travel_yaw']):
        raise ValueError('travel_yaw 必须为有限弧度值')
    if abs(math.sin(cfg['travel_yaw']))>1e-6:
        raise ValueError('当前半长/半宽约定要求车头沿场地 x 轴，travel_yaw=0 或 pi')
    if cfg['yaw_tolerance']>.1:
        raise ValueError('yaw_tolerance 不能超过 0.1rad，矩形包络只用于轴向车身')
    cfg.setdefault('allow_unknown_in_field',False)
    if type(cfg['allow_unknown_in_field']) is not bool:
        raise ValueError('allow_unknown_in_field 必须是布尔值')
    if not isinstance(cfg.get('costmap_topic'),str) or not cfg['costmap_topic']:
        raise ValueError('costmap_topic 必须是完整快照的非空话题名')
    if cfg['frame_id']!='map' or cfg['odom_frame']!='odom' or cfg['base_frame']!='base_link':
        raise ValueError('当前框架统一使用 map/odom/base_link')
    if not isinstance(cfg['laser_frame'],str) or not cfg['laser_frame']:
        raise ValueError('laser_frame 必须非空')
    return cfg

def stamp_seconds(stamp):
    """Validate JSON ROS Time before advancing any stream watermark."""
    try:sec=stamp['sec'];ns=stamp['nanosec']
    except (TypeError,KeyError):raise ValueError('invalid source stamp') from None
    if (type(sec) is not int or type(ns) is not int or
            not -2**31<=sec<2**31 or not 0<=ns<10**9):
        raise ValueError('invalid source stamp')
    return sec+ns*1e-9

def laser_to_base(points, extrinsic):
    """laser_pose=[x,y,z,yaw]；二维雷达必须水平安装，roll/pitch=0。"""
    x,y,_z,yaw=extrinsic
    c,s=math.cos(yaw),math.sin(yaw)
    return np.asarray(points) @ np.array([[c,s],[-s,c]]) + np.array([x,y])

def pose_values(pose):
    q=pose.orientation
    values=(pose.position.x,pose.position.y,q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in values): raise ValueError('nonfinite pose')
    if abs(sum(v*v for v in values[2:])-1)>.01 or abs(q.x)+abs(q.y)>1e-5:
        raise ValueError('pose must have a normalized planar quaternion')
    return values[0],values[1],2*math.atan2(q.z,q.w)

def health_valid(data, now, timeout, require_lift_settled=True):
    try:
        stamp=data['stamp'];sec=stamp['sec'];ns=stamp['nanosec']
        if type(sec) is not int or type(ns) is not int or not 0<=ns<10**9:return False
        age=now-(sec+ns*1e-9)
        if 'lift_is_up' not in data:return False
        settled=data.get('lift_settled');up=data.get('lift_is_up')
        if 'lift_settled' in data and type(settled) is not bool:return False
        # null 是机构尚未到位/状态未确认，不是模块断联。健康与允许运动分别判定。
        known=type(up) is bool and settled is not False
        measured_state=(type(up) is bool or (up is None and settled is not True))
        return (-.05<=age<timeout and data['modules'].get('base') is True and
                not data.get('fault') and type(data.get('stopped')) is bool and
                measured_state and (known or not require_lift_settled) and
                data.get('lift_state_source')=='measured' and
                data['modules'].get('lift') is True)
    except (KeyError,TypeError,AttributeError):return False

def effective_half_size(cfg, carrying):
    # 小偏航也扩大包络，避免仅用轴向车身尺寸漏检角点。
    x,y=cfg['loaded_half_size' if carrying else 'empty_half_size']
    delta=cfg['yaw_tolerance']
    return x+y*math.sin(delta), y+x*math.sin(delta)

def planner_for(cfg, message, carrying, rotating=False, *, for_tracking=False):
    """Nominal routes reserve the permitted tracking error in addition to clearance.

    Actual measured footprint checks use only physical clearance: applying the
    tracking reserve there again would count the same deviation twice.
    """
    if message.header.frame_id!='map': raise ValueError('costmap frame must be map')
    half=effective_half_size(cfg,carrying)
    if rotating:
        radius=math.hypot(*cfg['empty_half_size']);half=(radius,radius)
    return Planner(GridMap(message.metadata,message.data,
                           unknown_is_free=cfg.get('allow_unknown_in_field',False)),half,cfg['field_size'],
                   margin=cfg['margin']+(cfg['position_tolerance'] if for_tracking else 0.))

class WaitingForObservedCostmap(ValueError):
    """地图响应存在，但还未完成可用于启动的雷达观测；保持停车等待。"""


def costmap_observation_ready(planner, point, initialized=False):
    """首次放行需要可观测的本车包络；服务存在/时间戳新鲜不代表首帧已更新。

    初始化时当前包络不得被策略认定为障碍，且包络外要有实际观测格，
    排除全未知初始图和仅自体清除的空图。允许未知时，包络中的未知也允许；
    保守模式则要求完整包络已观测。Dock/举升合法足迹暂态由执行入口等待新图，
    运动时的碰撞始终由 check_route 检查，不把动作暂态当全模块健康丢失。
    """
    grid=planner.grid
    if not np.any(grid.costs<254):
        raise WaitingForObservedCostmap('waiting for first radar-observed costmap: no free observed cells')
    if initialized:return True
    reason=planner.segment_blocked_reason(point,point)
    if reason:
        cells=grid.blocking_world_rects((point[0]-planner.hx-planner.margin,
            point[0]+planner.hx+planner.margin,point[1]-planner.hy-planner.margin,
            point[1]+planner.hy+planner.margin))
        # 真障碍也不可放行；此处仅区分暂未观测与已观测的碰撞诊断，不删除格。
        for cell in cells:
            x=(cell[0]+cell[1])/2;y=(cell[2]+cell[3])/2
            i=math.floor((x-grid.ox)/grid.res);j=math.floor((y-grid.oy)/grid.res)
            if 0<=i<grid.w and 0<=j<grid.h and grid.costs[j,i]==254:
                raise ValueError('current footprint blocked in startup costmap: '+reason)
        raise WaitingForObservedCostmap('waiting for observed startup footprint: '+reason)
    mx=planner.hx+planner.margin;my=planner.hy+planner.margin;x,y=point
    i0=max(0,math.floor((x-mx-grid.ox)/grid.res));i1=min(grid.w,math.floor((x+mx-grid.ox)/grid.res)+1)
    j0=max(0,math.floor((y-my-grid.oy)/grid.res));j1=min(grid.h,math.floor((y+my-grid.oy)/grid.res)+1)
    outside=(np.any(grid.costs[:j0,:]<254) or np.any(grid.costs[j1:,:]<254) or
             np.any(grid.costs[j0:j1,:i0]<254) or np.any(grid.costs[j0:j1,i1:]<254))
    if not outside:
        raise WaitingForObservedCostmap('waiting for radar-cleared space beyond the startup footprint')
    return True


def settled(pose, velocity, goal, cfg):
    return (math.dist(pose[:2],goal[:2])<=cfg['position_tolerance'] and
            abs(wrap(pose[2]-goal[2]))<=cfg['yaw_tolerance'] and
            math.hypot(*velocity[:2])<cfg['stop_linear_speed'] and
            abs(velocity[2])<cfg['stop_angular_speed'])


def route_distance(pose, points):
    """到折线的距离，含单点停留路径；监测跟踪器是否离开规划走廊。"""
    if len(points)==1:return math.dist(pose[:2],points[0])
    best=math.inf
    for a,b in zip(points,points[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];norm=dx*dx+dy*dy
        t=0 if norm==0 else max(0,min(1,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/norm))
        best=min(best,math.dist(pose[:2],(a[0]+t*dx,a[1]+t*dy)))
    return best


def measured_base_stopped(data, now, timeout):
    """Stop evidence is independent of navigation/lift readiness and cargo.

    A sticky operational fault must prohibit new motion, but does not invalidate
    fresh measured wheel-stop feedback. Unknown/stale/nonmeasured feedback does.
    Legacy owners may use stopped; new owners report base_motion_stopped so a
    moving/unknown lift cannot prevent confirming that the chassis has stopped.
    """
    try:
        sec=data['stamp']['sec'];ns=data['stamp']['nanosec']
        if type(sec) is not int or type(ns) is not int or not 0<=ns<10**9:return False
        if not -.05<=now-(sec+ns*1e-9)<timeout:return False
        if data.get('lift_state_source')!='measured':return False
        return data.get('base_motion_stopped',data.get('stopped')) is True
    except (KeyError,TypeError):return False
