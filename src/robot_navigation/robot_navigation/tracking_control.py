"""纯路径控制器：每次用实测位姿反馈，不积分命令推算位置。"""
import math
from .core import wrap, pose_values

def validate_goal(goal,cfg):
    if goal.path.header.frame_id!='map' or not goal.path.poses:
        raise ValueError('nonempty map path required')
    if not isinstance(goal.request_id,str) or not goal.request_id:
        raise ValueError('request_id required')
    poses=[]
    for item in goal.path.poses:
        if item.header.frame_id!='map':raise ValueError('each pose must be map')
        p=pose_values(item.pose)
        if not (0<p[0]<cfg['field_size'][0] and 0<p[1]<cfg['field_size'][1]):
            raise ValueError('path outside field')
        poses.append(p)
    names=('max_linear_speed','max_angular_speed','position_tolerance','yaw_tolerance',
           'stop_linear_speed','stop_angular_speed','settle_duration_sec','max_duration_sec')
    for name in names:
        value=getattr(goal,name)
        if not math.isfinite(value) or value<=0:raise ValueError('invalid '+name)
        if value>cfg[name]+1e-9:raise ValueError('request exceeds configured '+name)
    if goal.carrying and goal.allow_in_place_rotation:raise ValueError('loaded rotation forbidden')
    yaw=poses[-1][2]
    if any(abs(wrap(p[2]-yaw))>1e-6 for p in poses):raise ValueError('mixed headings require separate actions')
    for a,b in zip(poses,poses[1:]):
        if abs(a[0]-b[0])>1e-8 and abs(a[1]-b[1])>1e-8:raise ValueError('diagonal path forbidden')
    if goal.allow_in_place_rotation:
        if any(math.dist(p[:2],poses[0][:2])>1e-8 for p in poses):
            raise ValueError('rotation must be in place')
    elif abs(wrap(yaw-cfg['travel_yaw']))>goal.yaw_tolerance:
        raise ValueError('translation heading must follow field x axis')
    return poses

def body_velocity(world_vx,world_vy,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return c*world_vx+s*world_vy,-s*world_vx+c*world_vy

class PathController:
    def __init__(self,goal,cfg):
        self.goal=goal;self.cfg=cfg;self.poses=validate_goal(goal,cfg)
        self.index=0 if len(self.poses)==1 else 1
        self.stable_since=None;self.stable_frames=0;self.last_stamp=None
        self.correcting=False;self.previous=(0.,0.,0.);self.previous_world=(0.,0.)
        self.done=False;self.phase='TRACKING'
        self.progress_index=None;self.progress_best=math.inf;self.progress_elapsed=0.

    def step(self,pose,velocity,stamp,base_stopped,dt):
        """输出 base_link vx/vy/wz；移动严格沿 map 单轴，转动只在原地。"""
        g=self.goal;target=self.poses[self.index]
        dx,dy=target[0]-pose[0],target[1]-pose[1]
        yaw_error=wrap(target[2]-pose[2])
        if g.allow_in_place_rotation:
            if math.hypot(dx,dy)>g.position_tolerance:raise ValueError('rotation position drift')
            wz=0. if abs(yaw_error)<=g.yaw_tolerance*.5 else max(-g.max_angular_speed,
                min(g.max_angular_speed,2.*yaw_error-.4*velocity[2]))
            command=(0.,0.,wz)
        else:
            if abs(yaw_error)>g.yaw_tolerance:raise ValueError('translation heading drift')
            a=self.poses[max(0,self.index-1)]
            axis=0 if abs(target[0]-a[0])>1e-8 else 1
            cross_error=dy if axis==0 else dx
            enter=g.position_tolerance*.4;leave=enter*.5
            if abs(cross_error)>enter:self.correcting=True
            elif abs(cross_error)<=leave:self.correcting=False
            active_axis=1-axis if self.correcting else axis
            error=dx if active_axis==0 else dy
            deadband=min(.003,g.position_tolerance*.2)
            speed=0. if abs(error)<deadband else max(-g.max_linear_speed,min(g.max_linear_speed,2.*error))
            wx,wy=(speed,0.) if active_axis==0 else (0.,speed)
            bx,by=body_velocity(wx,wy,pose[2]);command=(bx,by,0.)
        # 限制加速；换轴先发零并等待实测静止，不能把两轴速度混成斜线。
        old=self.previous
        world_old=self.previous_world
        world_new=(math.cos(pose[2])*command[0]-math.sin(pose[2])*command[1],
                   math.sin(pose[2])*command[0]+math.cos(pose[2])*command[1])
        changing_axis=(abs(world_old[0])>1e-7 and abs(world_new[1])>1e-7 or
                       abs(world_old[1])>1e-7 and abs(world_new[0])>1e-7)
        moving=math.hypot(*velocity[:2])>=g.stop_linear_speed or abs(velocity[2])>=g.stop_angular_speed
        # A zero command does not mean the wheels have stopped. Check measured
        # world velocity as well, including direction reversal after overshoot.
        c,s=math.cos(pose[2]),math.sin(pose[2])
        measured=(c*velocity[0]-s*velocity[1],s*velocity[0]+c*velocity[1])
        reverse=any(a*b < -1e-10 for a,b in zip(world_old,world_new))
        coasting=any(abs(measured[1-axis])>=g.stop_linear_speed or
                     measured[axis]*world_new[axis] < -1e-10 and
                     abs(measured[axis])>=g.stop_linear_speed
                     for axis in (0,1) if abs(world_new[axis])>1e-7)
        angular_reverse=(command[2]*velocity[2]<0 and abs(velocity[2])>=g.stop_angular_speed)
        if changing_axis or reverse or coasting or angular_reverse or (self.phase=='AXIS_STOP' and (moving or not base_stopped)):
            command=(0.,0.,0.);self.phase='AXIS_STOP'
        else:
            self.phase='TRACKING'
            linear=math.hypot(*command[:2]);old_linear=math.hypot(*old[:2])
            limit=old_linear+self.cfg['tracking_linear_acceleration']*min(max(dt,0),.1)
            if linear>limit and linear>0:command=(command[0]*limit/linear,command[1]*limit/linear,command[2])
            angular=self.cfg['tracking_angular_acceleration']*min(max(dt,0),.1)
            command=(command[0],command[1],max(old[2]-angular,min(old[2]+angular,command[2])))
        close=math.hypot(dx,dy)<=g.position_tolerance and abs(yaw_error)<=g.yaw_tolerance
        # 对所有拐点先停车，完成后再开始下一条轴向段。
        if close:
            command=(0.,0.,0.);self.phase='SETTLING'
            if not moving and base_stopped:
                if self.last_stamp is None or stamp>self.last_stamp:
                    if self.stable_since is None:self.stable_since=stamp;self.stable_frames=0
                    self.stable_frames+=1;self.last_stamp=stamp
                    duration=g.settle_duration_sec if self.index==len(self.poses)-1 else min(g.settle_duration_sec,.2)
                    if self.stable_frames>=3 and stamp-self.stable_since>=duration:
                        if self.index==len(self.poses)-1:self.done=True;self.phase='COMPLETED'
                        else:self.index+=1;self.stable_since=None;self.stable_frames=0
            else:self.stable_since=None;self.stable_frames=0
        else:self.stable_since=None;self.stable_frames=0
        # Detect a stuck base or sustained oscillation from measured progress,
        # not from integrating requested speed. Count control wall time: missing
        # odometry/loop stalls are separately rejected by PathTracker.
        metric=abs(yaw_error) if g.allow_in_place_rotation else math.hypot(dx,dy)
        minimum=self.cfg.get('tracking_progress_angle',.02) if g.allow_in_place_rotation else self.cfg.get('tracking_progress_distance',.005)
        if self.progress_index!=self.index:
            # A corner may have advanced index earlier in this tick. The new
            # segment's baseline is its own target, not zero distance to the old.
            next_target=self.poses[self.index]
            self.progress_index=self.index
            self.progress_best=(abs(wrap(next_target[2]-pose[2])) if g.allow_in_place_rotation
                                else math.dist(pose[:2],next_target[:2]))
            self.progress_elapsed=0.
        elif metric<=self.progress_best-minimum:
            self.progress_best=metric;self.progress_elapsed=0.
        elif not close:
            self.progress_elapsed+=max(dt,0.)
            if self.progress_elapsed>=self.cfg.get('tracking_progress_timeout_sec',8.):
                raise ValueError('measured trajectory made no progress; check chassis tracking')
        else:self.progress_elapsed=0.
        self.previous=command
        self.previous_world=(math.cos(pose[2])*command[0]-math.sin(pose[2])*command[1],
                             math.sin(pose[2])*command[0]+math.cos(pose[2])*command[1])
        return command

    def distance_remaining(self,pose):
        return math.dist(pose[:2],self.poses[self.index][:2])+sum(
            math.dist(a[:2],b[:2]) for a,b in zip(self.poses[self.index:],self.poses[self.index+1:]))
