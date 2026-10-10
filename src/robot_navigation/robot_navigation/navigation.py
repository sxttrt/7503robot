"""NavigateToPose -> 四连通规划 -> TrackPath；本节点不发布 cmd_vel。

本包 path_tracker 提供 TrackPath 服务端；底盘队友负责 ESP32 接口及轮速伺服。本节点持续检查雷达/地图/实测
反馈，异常取消跟踪；取消无最终确认则锁定故障，不继续发新轨迹。
"""
import json
import signal
import math
import threading
import time
import uuid
from pathlib import Path
import numpy as np
from mission_manager.project_paths import project_root
from .observed_planning import observed_route, endpoint_summary
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, Polygon, Point32
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from mission_interfaces.action import TrackPath, NavigateToPoint, Dock
from .docking import DockGeometry, DockGoalHandle
from .point_targets import PointTargets, PointGoalHandle, stamped_target, point_result
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mission_manager.adapters.navigation import AsyncAction
from .core import (load_settings, pose_values, health_valid, settled, planner_for,
                   CostmapCache, PublishedCostmapCache, wrap, route_distance, costmap_observation_ready,
                   WaitingForObservedCostmap, measured_base_stopped, stamp_seconds)

class RouteChanged(ValueError):
    """A fresh map invalidated the remaining nominal route; requires stopped replan."""

class Tracking(AsyncAction):
    def on_feedback(self,request,feedback):
        super().on_feedback(request,feedback)
        if request in self.requests and type(feedback.segment_index) is int:
            self.feedback['segment_index']=feedback.segment_index

    def decode(self,result):
        return result.success and result.stopped, {'result':result,'reason':result.message}

class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        self.declare_parameter('config_file','');self.declare_parameter('interfaces_file','')
        self.declare_parameter('points_file','')
        self.cfg=load_settings(self.get_parameter('config_file').value)
        with open(self.get_parameter('interfaces_file').value,encoding='utf-8') as f:self.names=yaml.safe_load(f)
        self.point_targets=None;self.point_pose=None;self.mission_context=None;self.mission_seen=0.
        self.dock_geometry=None;self.dock_corridor=None
        points_file=self.get_parameter('points_file').value
        if points_file:
            with open(points_file,encoding='utf-8') as f:data=yaml.safe_load(f)
            self.point_targets=PointTargets(data);self.dock_geometry=DockGeometry(data,self.cfg)
        self.create_subscription(String,self.names['mission_status_topic'],self.on_mission,10)
        self.lock=threading.Lock();self.busy=False;self.stopping=False;self.abort_event=threading.Event();self.fault=''
        self.odom=None;self.odom_received=0.;self.odom_stamp=-math.inf
        self.base=None;self.stop_base=None;self.base_received=0.;self.base_stamp=-math.inf
        self.payload_stamp=-math.inf;self.costmap_observed=False
        self.lidar=None;self.lidar_received=0.;self.lidar_stamp=-math.inf;self.closing=False
        self.track=Tracking(self,TrackPath,self.names['tracking_action'])
        self.cache=PublishedCostmapCache(expected_frame=self.cfg['frame_id'])
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Costmap,self.cfg['costmap_topic'],
            lambda msg:self.cache.accept(msg,self.now(),self.cfg['costmap_max_age_sec']),qos)
        self.last_footprint=None
        self.footprint_pub=self.create_publisher(Polygon,self.cfg['footprint_topic'],10)
        self.create_timer(.2,self.publish_footprint)
        self.path_pub=self.create_publisher(Path,'navigation/path',10)
        self.status_pub=self.create_publisher(String,self.names['navigation_status_topic'],10)
        self.create_subscription(Odometry,self.cfg['odom_topic'],self.on_odom,10)
        self.create_subscription(String,self.names.get('base_feedback_topic',self.names['health_topic']),self.on_base,10)
        self.create_subscription(String,self.cfg['localization_status_topic'],self.on_lidar,10)
        group=ReentrantCallbackGroup()
        self.point_server=ActionServer(self,NavigateToPoint,self.names['navigation_action'],
            execute_callback=self.execute_point,goal_callback=self.accept_point,
            cancel_callback=self.cancel,callback_group=group)
        self.server=ActionServer(self,NavigateToPose,self.names['pose_navigation_action'],
            execute_callback=self.execute,goal_callback=self.accept,
            cancel_callback=self.cancel,callback_group=group)
        self.dock_server=ActionServer(self,Dock,self.names['docking_action'],
            execute_callback=self.execute_dock,goal_callback=self.accept_dock,
            cancel_callback=self.cancel,callback_group=group)
        self.stop_server=self.create_service(Trigger,self.names['navigation_stop_service'],self.stop,callback_group=group)
        self.create_timer(.2,self.publish_status)

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def on_mission(self,msg):
        try:
            data=json.loads(msg.data);stamp=data['stamp_ns']
            if type(stamp) is not int or not -.05<=self.now()-stamp*1e-9<.6:return
            self.mission_context=data;self.mission_seen=time.monotonic()
        except (ValueError,TypeError,KeyError):return

    def accept_point(self,request):
        try:
            if self.point_targets is None:raise ValueError('missing navigation points_file')
            if self.point_targets.simulation_only and not self.get_parameter('use_sim_time').value:
                raise ValueError('实机禁止使用仿真导航点位')
            allowed={'A','B','C','D','DROP_OFF','START_SCAN','FINAL_SCAN'}
            if request.target_id not in allowed:raise ValueError('unknown target_id')
            self.inputs(require_payload_map=False,allow_lift_transition=True)
            with self.lock:
                if self.busy or self.stopping or self.closing:return GoalResponse.REJECT
                self.busy=True;self.abort_event.clear()
            return GoalResponse.ACCEPT
        except ValueError as exc:
            self.get_logger().warn('拒绝点位导航：'+str(exc));return GoalResponse.REJECT

    def execute_point(self,handle):
        # 机构完成响应、详细反馈和状态话题可能以不同顺序到达。
        # 已接收点位目标先停车等待新鲜实测数据，不能把消息先后差异当作导航失败。
        limit=time.monotonic()+self.cfg['cancel_timeout_sec']
        try:
            while rclpy.ok() and time.monotonic()<limit:
                if handle.is_cancel_requested or self.abort_event.is_set():raise ValueError('navigation cancelled')
                try:
                    (_pose,_vel),carrying,_map=self.inputs(require_payload_map=False)
                    context=self.mission_context if time.monotonic()-self.mission_seen<.6 else None
                    target=self.point_targets.resolve(handle.request.target_id,carrying=carrying,context=context,
                        simulation=bool(self.get_parameter('use_sim_time').value))
                    if not (0<target[0]<self.cfg['field_size'][0] and 0<target[1]<self.cfg['field_size'][1]):
                        raise ValueError('target outside field')
                    pose=stamped_target(target,self.get_clock().now().to_msg())
                    return point_result(self.execute(PointGoalHandle(handle,pose)))
                except ValueError as exc:
                    # 只等待动作交接的合法暂态；缺项或故障立即终止。
                    waiting=('fresh measured base/lift feedback required',
                             'DROP_OFF 要求已确认载货',
                             'DROP_OFF 缺少新鲜的 NAV_END 载货上下文')
                    if str(exc) not in waiting:raise
                time.sleep(.02)
            raise ValueError('point navigation handoff not ready before deadline')
        except Exception as exc:
            result=NavigateToPoint.Result();result.success=False;result.message=str(exc)
            if handle.is_cancel_requested:handle.canceled()
            else:handle.abort()
            with self.lock:self.busy=False
            return result

    def on_odom(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        try:
            if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':return
            if stamp<=self.odom_stamp or not -.05<=self.now()-stamp<.5:return
            pose=pose_values(msg.pose.pose)
            v=msg.twist.twist
            vel=(v.linear.x,v.linear.y,v.angular.z)
            if not all(math.isfinite(x) for x in vel):return
            self.odom=(pose,vel);self.odom_stamp=stamp;self.odom_received=time.monotonic()
        except ValueError:return

    def accept_dock(self,request):
        try:
            if self.dock_geometry is None:raise ValueError('missing Dock geometry')
            self.dock_geometry.request(request,bool(self.get_parameter('use_sim_time').value))
            # 升降完成响应和反馈可能先后到达；接受后停车等待，不发速度。
            self.inputs(require_payload_map=False,allow_lift_transition=True)
            with self.lock:
                if self.busy or self.stopping or self.closing:return GoalResponse.REJECT
                self.busy=True;self.abort_event.clear()
            return GoalResponse.ACCEPT
        except ValueError as exc:
            self.get_logger().warn('拒绝进退架：'+str(exc));return GoalResponse.REJECT

    def execute_dock(self,handle):
        result=Dock.Result();request=handle.request
        deadline=time.monotonic()+min(request.max_duration_sec,self.cfg['max_duration_sec'])
        try:
            # 与普通导航共用运动所有权、取消/停车握手、雷达和实测底盘反馈。
            limit=min(deadline,time.monotonic()+self.cfg['cancel_timeout_sec'])
            while rclpy.ok() and time.monotonic()<limit:
                if self.abort_event.is_set() or handle.is_cancel_requested:raise ValueError('Dock cancelled')
                try:
                    (pose,_vel),carrying,_message=self.inputs(require_payload_map=False)
                    context=self.mission_context if time.monotonic()-self.mission_seen<.6 else None
                    if carrying:raise ValueError('waiting for measured platform down')
                    corridor=self.dock_geometry.plan(request,pose,carrying=carrying,context=context,
                        simulation=bool(self.get_parameter('use_sim_time').value))
                    break
                except ValueError as exc:
                    if str(exc) not in ('fresh measured base/lift feedback required',
                        'waiting for measured platform down','waiting for current Dock mission context'):raise
                    time.sleep(.02)
            else:raise ValueError('Dock handoff not ready before deadline')
            self.wait_handoff(handle,deadline)
            # Recompute from the latest stopped radar pose, not the pre-handoff sample.
            (pose,_vel),carrying,_message=self.inputs(require_payload_map=False)
            context=self.mission_context if time.monotonic()-self.mission_seen<.6 else None
            self.dock_corridor=self.dock_geometry.plan(request,pose,carrying=carrying,context=context,
                simulation=bool(self.get_parameter('use_sim_time').value))
            yaw=self.cfg['travel_yaw'];points=self.dock_corridor.points
            self.run_track(DockGoalHandle(handle),self.path_message(points,yaw),
                           (*points[-1],yaw),False,deadline)
            if handle.is_cancel_requested or self.abort_event.is_set():raise ValueError('Dock cancelled')
            result.success=True;result.ready_to_lift=request.operation=='ENTER'
            result.exited=request.operation=='EXIT';result.message='Dock arrival and measured stop confirmed'
            handle.succeed()
        except Exception as exc:
            result.success=False;result.ready_to_lift=False;result.exited=False
            result.error_code='DOCK_EXECUTION_FAILED';result.message=str(exc)
            if handle.is_cancel_requested:handle.canceled()
            else:handle.abort()
            self.get_logger().error('进退架结束：'+str(exc))
        finally:
            self.dock_corridor=None
            with self.lock:self.busy=False
        return result

    def on_base(self,msg):
        try:
            data=json.loads(msg.data)
            stamp=stamp_seconds(data['stamp'])
            if stamp<=self.base_stamp or not -.05<=self.now()-stamp<self.cfg['feedback_timeout_sec']:return
            self.stop_base=data
            if not health_valid(data,self.now(),self.cfg['feedback_timeout_sec'],require_lift_settled=False):
                self.base=None;self.base_stamp=stamp;return
            if self.base is None or self.base['lift_is_up']!=data['lift_is_up']:
                self.payload_stamp=math.inf
            self.base=data;self.base_stamp=stamp;self.base_received=time.monotonic()
        except (ValueError,TypeError,KeyError):return

    def publish_footprint(self):
        if not self.cfg['calibration_verified'] or not self.base:return
        if not health_valid(self.base,self.now(),self.cfg['feedback_timeout_sec']):return
        # 新鲜实测携货状态驱动 costmap 自体清除，避免把自己的架腿当路障。
        hx,hy=self.cfg['loaded_half_size' if self.base['lift_is_up'] else 'empty_half_size']
        if self.footprint_pub.get_subscription_count()==0:
            self.last_footprint=None;self.payload_stamp=math.inf;return
        key=(hx,hy)
        if getattr(self,'last_footprint',None)==key and self.payload_stamp!=math.inf:return
        # Same footprint at 5Hz causes full reinflation on every update in Nav2.
        # The timer now only watches for measured payload changes/reconnection.
        polygon=Polygon()
        for x,y in ((hx,hy),(-hx,hy),(-hx,-hy),(hx,-hy)):
            point=Point32();point.x=float(x);point.y=float(y);polygon.points.append(point)
        self.footprint_pub.publish(polygon);self.last_footprint=key
        if self.payload_stamp==math.inf:self.payload_stamp=self.now()

    def on_lidar(self,msg):
        try:
            data=json.loads(msg.data)
            stamp=stamp_seconds(data['stamp'])
            if (stamp<=getattr(self,'lidar_stamp',-math.inf) or
                    not -.05<=self.now()-stamp<self.cfg['feedback_timeout_sec']):return
            self.lidar=data;self.lidar_received=time.monotonic();self.lidar_stamp=stamp
        except (ValueError,TypeError,KeyError):return

    def inputs(self,require_payload_map=True,allow_lift_transition=False):
        cfg=self.cfg;wall=time.monotonic()
        if not cfg['calibration_verified']:raise ValueError('calibration not verified')
        if self.fault:raise ValueError(self.fault)
        if getattr(self,'closing',False):raise ValueError('navigation closing')
        if (not self.odom or wall-self.odom_received>cfg['feedback_timeout_sec'] or
                not -.05<=self.now()-self.odom_stamp<.5):raise ValueError('radar odom stale')
        if (not self.lidar or self.lidar.get('valid') is not True or
                wall-self.lidar_received>cfg['feedback_timeout_sec']):raise ValueError('localization invalid')
        if (not self.base or wall-self.base_received>cfg['feedback_timeout_sec'] or
                not health_valid(self.base,self.now(),cfg['feedback_timeout_sec'],
                                 require_lift_settled=not allow_lift_transition)):
            raise ValueError('fresh measured base/lift feedback required')
        if not self.track.available():raise ValueError('TrackPath server unavailable')
        message,received,_sequence=self.cache.snapshot()
        if message is None or wall-received>cfg['costmap_max_age_sec']:raise ValueError('costmap response stale')
        stamp=message.metadata.update_time.sec+message.metadata.update_time.nanosec*1e-9
        if not -.05<=self.now()-stamp<cfg['costmap_max_age_sec']:raise ValueError('costmap source stale')
        if require_payload_map and stamp<=self.payload_stamp:
            raise ValueError('waiting for costmap after payload change')
        # 当前四壁定位直接输出场地 odom，map->odom 固定为 identity。
        # 外部定位替换本节点时必须遵守此坐标约定或扩展坐标变换，不能仅改 frame 字符串。
        if (not getattr(self,'costmap_observed',False) and
                not health_valid(self.base,self.now(),cfg['feedback_timeout_sec'])):
            raise WaitingForObservedCostmap('waiting for measured lift state before navigation startup')
        planner=planner_for(self.cfg,message,self.base['lift_is_up'])
        self.costmap_observed=costmap_observation_ready(planner,self.odom[0][:2],
                                                       getattr(self,'costmap_observed',False))
        return self.odom,self.base['lift_is_up'],message

    def publish_status(self):
        try:
            self.inputs(require_payload_map=False,allow_lift_transition=True)
            healthy=True
            ready=health_valid(self.base,self.now(),self.cfg['feedback_timeout_sec'])
            reason='' if ready else 'lift state unconfirmed; navigation motion paused'
        except ValueError as exc:healthy=False;ready=False;reason=str(exc)
        stamp=self.get_clock().now().to_msg()
        self.status_pub.publish(String(data=json.dumps({'schema_version':1,
            'stamp':{'sec':stamp.sec,'nanosec':stamp.nanosec},'healthy':healthy,'ready':ready,
            'active':self.busy,'fault':self.fault,'reason':reason,
            'costmap_pipeline':self.cache.diagnostic()})))

    def accept(self,request):
        try:
            (_pose,_vel),_carrying,_map=self.inputs(require_payload_map=False)
            if request.pose.header.frame_id!='map':return GoalResponse.REJECT
            goal=pose_values(request.pose.pose)
            if not (0<goal[0]<self.cfg['field_size'][0] and 0<goal[1]<self.cfg['field_size'][1]):
                return GoalResponse.REJECT
            # Action-result and health samples travel independently. Accept a
            # valid request, but do not dispatch tracking until the handoff gate
            # has observed fresh idle/stopped feedback after this acceptance.
            with self.lock:
                if self.busy or self.stopping:return GoalResponse.REJECT
                self.busy=True;self.abort_event.clear()
            return GoalResponse.ACCEPT
        except ValueError as exc:
            self.get_logger().warn('拒绝导航目标：'+str(exc));return GoalResponse.REJECT

    def cancel(self,_handle):
        self.abort_event.set();return CancelResponse.ACCEPT

    def stop(self,_request,response):
        # 整机 safety/stop 的实现方先调用此服务，再确认 Dock/底盘/升降停止。
        with self.lock:
            self.stopping=True;self.abort_event.set()
        deadline=time.monotonic()+self.cfg['cancel_timeout_sec'];begin=self.now()
        try:
            while rclpy.ok() and time.monotonic()<deadline:
                evidence=getattr(self,'stop_base',None) or self.base
                if (not self.busy and self.track.idle() and evidence and
                        self.base_stamp>=begin and self.odom and
                        self.odom_stamp>=begin and -.05<=self.now()-self.odom_stamp<.5 and
                        time.monotonic()-self.odom_received<self.cfg['feedback_timeout_sec'] and
                        math.hypot(*self.odom[1][:2])<self.cfg['stop_linear_speed'] and
                        abs(self.odom[1][2])<self.cfg['stop_angular_speed'] and
                        measured_base_stopped(evidence,self.now(),self.cfg['feedback_timeout_sec'])):
                    response.success=True;response.message='navigation idle and fresh measured stopped';return response
                time.sleep(.02)
            response.success=False;response.message=self.fault or 'navigation stop not confirmed';return response
        finally:
            with self.lock:self.stopping=False

    def path_message(self,points,yaw):
        path=Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        for x,y in points:
            p=PoseStamped();p.header=path.header;p.pose.position.x=float(x);p.pose.position.y=float(y)
            p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
            path.poses.append(p)
        return path

    def check_route(self,points,carrying,rotating=False):
        corridor=getattr(self,'dock_corridor',None)
        if corridor is not None:
            (pose,_v),current,_message=self.inputs(require_payload_map=False)
            if carrying or current or rotating:raise ValueError('Dock requires empty aligned chassis')
            corridor.validate(pose,points)
            return
        (pose,_v),current,message=self.inputs()
        if current!=carrying:raise ValueError('payload changed while navigation active')
        planner=planner_for(self.cfg,message,carrying,rotating)
        nominal=planner_for(self.cfg,message,carrying,rotating,for_tracking=True)
        # 名义路线预留跟踪容差，实测车体只检查安全间距，避免重复计入误差。
        # 已完成段由 TrackPath 进度去除；当前段只检查投影点之后的剩余扫掠。
        # 位姿必须仍在已授权走廊内，投影不能掩盖跟踪偏离。
        if not planner.segment_free(pose[:2],pose[:2]):raise ValueError('current footprint blocked')
        if route_distance(pose,points)>self.cfg['position_tolerance']:
            raise ValueError('tracker left planned corridor')
        if not rotating and abs(wrap(pose[2]-self.cfg['travel_yaw']))>self.cfg['yaw_tolerance']:
            raise ValueError('body heading left axis alignment')
        remaining=points
        if len(points)>1:
            a,b=points[:2];dx,dy=b[0]-a[0],b[1]-a[1];norm=dx*dx+dy*dy
            t=0. if norm==0. else max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/norm))
            remaining=[(a[0]+t*dx,a[1]+t*dy)]+points[1:]
        for a,b in zip(remaining,remaining[1:]):
            if not nominal.segment_free(a,b):
                detail=self.save_planning_failure(nominal,pose,(*b,self.cfg['travel_yaw']))
                self.get_logger().warn('remaining radar route changed: '+detail+
                    ' reason='+str(nominal.segment_blocked_reason(a,b))+' remaining='+str(remaining))
                raise RouteChanged('live radar route blocked')

    def run_track(self,handle,path,target,carrying,deadline,rotating=False):
        done=threading.Event();outcome={}
        def complete(_request,success,details):
            outcome.update(success=success,details=details);done.set()
        goal=TrackPath.Goal();goal.request_id=uuid.uuid4().hex;goal.path=path;goal.carrying=carrying
        goal.allow_in_place_rotation=rotating and not carrying
        for name in ('max_linear_speed','max_angular_speed','position_tolerance','yaw_tolerance',
                     'stop_linear_speed','stop_angular_speed','settle_duration_sec'):
            setattr(goal,name,float(self.cfg[name]))
        goal.max_duration_sec=max(.1,deadline-time.monotonic())
        points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
        self.check_route(points,carrying,rotating)
        self.path_pub.publish(path)
        self.track.send(goal.request_id,goal,complete)
        failure=None;cause=None;segment_index=0
        while rclpy.ok() and not done.is_set():
            try:
                if handle.is_cancel_requested or self.abort_event.is_set():raise ValueError('navigation cancelled')
                if time.monotonic()>deadline:raise ValueError('navigation deadline exceeded')
                progress=getattr(self.track,'feedback',{})
                if progress.get('request_id')==goal.request_id:
                    index=progress.get('segment_index',segment_index)
                    if type(index) is not int or not segment_index<=index<max(1,len(points)-1):
                        raise ValueError('invalid tracker segment progress')
                    segment_index=index
                self.check_route(points[segment_index:],carrying,rotating)
            except ValueError as exc:failure=str(exc);cause=exc;break
            feedback=NavigateToPose.Feedback();feedback.distance_remaining=float(math.dist(self.odom[0][:2],target[:2]))
            handle.publish_feedback(feedback);done.wait(.05)
        if failure or not done.is_set():
            self.track.cancel(goal.request_id)
            if (not done.wait(self.cfg['cancel_timeout_sec']) or
                    not getattr(outcome.get('details',{}).get('result'), 'stopped',False)):
                self.fault='TrackPath cancellation/result unconfirmed; restart only after physical stop'
            if isinstance(cause,RouteChanged) and not self.fault:raise cause
            raise ValueError(failure or 'navigation shutdown')
        if not outcome['success']:
            if not getattr(outcome['details'].get('result'),'stopped',False):
                self.fault='TrackPath failed without measured stop confirmation'
            raise ValueError(outcome['details'].get('reason') or 'tracking failed')
        result=outcome['details']['result']
        measured=pose_values(result.final_pose.pose)
        stamp=result.final_pose.header.stamp.sec+result.final_pose.header.stamp.nanosec*1e-9
        if (result.final_pose.header.frame_id!='map' or not -.05<=self.now()-stamp<.5 or
                not settled(measured,(0,0,0),target,self.cfg)):
            raise ValueError('TrackPath final measured pose invalid')
        # 对自己的雷达连续新帧再确认，避免把队友 ACK 或缓存 stopped 当完成。
        stable_since=None;last_stamp=-math.inf;stable_frames=0
        while rclpy.ok() and time.monotonic()<deadline:
            if handle.is_cancel_requested or self.abort_event.is_set():raise ValueError('navigation cancelled')
            self.check_route(points[-2:],carrying,rotating)
            pose,vel=self.odom
            if settled(pose,vel,target,self.cfg) and self.base['stopped']:
                if self.odom_stamp>last_stamp:
                    if stable_since is None:stable_since=self.odom_stamp;stable_frames=0
                    stable_frames+=1
                    if stable_frames>=3 and self.odom_stamp-stable_since>=self.cfg['settle_duration_sec']:return
                    last_stamp=self.odom_stamp
            else:stable_since=None
            time.sleep(.02)
        raise ValueError('radar measured arrival/stop not confirmed')

    def wait_handoff(self,handle,deadline):
        begin=self.now();limit=min(deadline,time.monotonic()+self.cfg['cancel_timeout_sec'])
        frames=0;last=-math.inf;first=None
        while rclpy.ok() and time.monotonic()<limit:
            if self.abort_event.is_set() or handle.is_cancel_requested:raise ValueError('navigation cancelled')
            (_pose,vel),_carrying,_message=self.inputs(require_payload_map=False)
            if (self.base_stamp>begin and self.odom_stamp>begin and self.base['stopped'] and
                    math.hypot(*vel[:2])<self.cfg['stop_linear_speed'] and
                    abs(vel[2])<self.cfg['stop_angular_speed']):
                if self.odom_stamp>last:
                    first=self.odom_stamp if first is None else first
                    last=self.odom_stamp;frames+=1
                if frames>=3 and last-first>=self.cfg['settle_duration_sec']:return
            else:frames=0;first=None
            time.sleep(.02)
        raise ValueError('fresh stopped handoff not confirmed; no tracking goal dispatched')

    def wait_payload_map(self,handle,deadline):
        # 合法升降后的地图足迹切换不是健康故障，也不是旧路径执行中重规划。
        # 接受目标后保持未派发状态，直到足迹已发布且新的地图更新已收到。
        while rclpy.ok():
            if self.abort_event.is_set() or handle.is_cancel_requested:raise ValueError('navigation cancelled')
            if time.monotonic()>=deadline:raise ValueError('navigation deadline exceeded')
            try:return self.inputs()
            except WaitingForObservedCostmap:
                pass  # 初始化图不是无路线；未派发任何轨迹时停车等待第一份有效观测。
            except ValueError as exc:
                if str(exc)!='waiting for costmap after payload change':raise
            time.sleep(.02)
        raise ValueError('navigation shutdown')

    def save_planning_failure(self,planner,pose,target):
        detail=dict(start=endpoint_summary(planner,pose[:2]),
                    goal=endpoint_summary(planner,target[:2]))
        try:
            directory=project_root()/'runtime_logs/navigation';directory.mkdir(parents=True,exist_ok=True)
            path=directory/('blocked_'+str(time.time_ns())+'.npz')
            grid=planner.grid
            np.savez_compressed(path,costs=grid.costs,origin=[grid.ox,grid.oy],
                resolution=grid.res,half_size=[planner.hx,planner.hy],margin=planner.margin,
                field=planner.field,start=pose,target=target)
            detail['snapshot']=str(path)
        except OSError as exc:detail['snapshot_error']=str(exc)
        return json.dumps(detail,ensure_ascii=False)

    def wait_post_stop_map(self,handle,deadline,carrying):
        """No new trajectory until post-stop scan and a map at/after that scan."""
        arrival=self.now();new_scan_stamp=None
        while rclpy.ok() and time.monotonic()<deadline:
            if handle.is_cancel_requested or self.abort_event.is_set():
                raise ValueError('navigation cancelled')
            (pose,_vel),current,message=self.inputs()
            if current!=carrying:raise ValueError('payload changed while waiting for fresh map')
            # Localization health stamp is a periodic heartbeat, not acquisition
            # time. /odom retains the timestamp of the actual matched LaserScan.
            scan_stamp=self.odom_stamp
            update=message.metadata.update_time;map_stamp=update.sec+update.nanosec*1e-9
            if new_scan_stamp is None and scan_stamp>arrival+.2:new_scan_stamp=scan_stamp
            if new_scan_stamp is not None and map_stamp>=new_scan_stamp:break
            time.sleep(.02)
        else:raise ValueError('fresh post-stop observation deadline exceeded')
        return pose,message

    def execute(self,handle):
        result=NavigateToPose.Result();deadline=time.monotonic()+self.cfg['max_duration_sec']
        try:
            target=pose_values(handle.request.pose.pose)
            self.wait_handoff(handle,deadline)
            (pose,_vel),carrying,message=self.wait_payload_map(handle,deadline)
            yaw=self.cfg['travel_yaw']
            if carrying and (abs(wrap(pose[2]-yaw))>self.cfg['yaw_tolerance'] or
                             abs(wrap(target[2]-yaw))>self.cfg['yaw_tolerance']):
                raise ValueError('loaded rotation forbidden')
            # 空车先在原地回正，再平移，最后在目标点原地转到扫码角度。
            if abs(wrap(pose[2]-yaw))>self.cfg['yaw_tolerance']:
                self.run_track(handle,self.path_message([pose[:2],pose[:2]],yaw),
                               (*pose[:2],yaw),False,deadline,True)
            (pose,_vel),carrying,message=self.inputs()
            staging=[]
            while True:
                planner=planner_for(self.cfg,message,carrying,for_tracking=True)
                route,complete=observed_route(planner,pose[:2],target[:2])
                if route is None:
                    detail=self.save_planning_failure(planner,pose,target)
                    raise ValueError('no observed collision-free route: '+detail)
                endpoint=target[:2] if complete else route[-1]
                if not complete:
                    if len(staging)>=16 or any(math.dist(endpoint,p)<.05 for p in staging):
                        detail=self.save_planning_failure(planner,pose,target)
                        raise ValueError('observation staging made no progress: '+detail)
                    self.get_logger().info('观测分段：仅执行已确认自由空间至 '+str(endpoint))
                try:
                    self.run_track(handle,self.path_message([pose[:2]]+route,yaw),
                                   (*endpoint,yaw),carrying,deadline)
                except RouteChanged:
                    # A dispatched old action must confirm stopped before this is raised;
                    # pre-dispatch rejection has no old action to cancel.
                    # Keep the NavigateToPose request/target and original deadline.
                    self.get_logger().info('地图更新阻断剩余路线；已取消并确认停车，等待新图重规划')
                    self.wait_handoff(handle,deadline)
                    pose,message=self.wait_post_stop_map(handle,deadline,carrying)
                    continue
                if complete:break
                staging.append(endpoint)
                pose,message=self.wait_post_stop_map(handle,deadline,carrying)
            if abs(wrap(target[2]-yaw))>self.cfg['yaw_tolerance']:
                self.run_track(handle,self.path_message([target[:2],target[:2]],target[2]),
                               target,False,deadline,True)
            handle.succeed()
        except Exception as exc:
            result.error_code=1;result.error_msg=str(exc)
            if handle.is_cancel_requested:handle.canceled()
            else:handle.abort()
            self.get_logger().error('导航结束：'+str(exc))
        finally:
            with self.lock:self.busy=False
        return result

def main(args=None):
    quitting=threading.Event()
    old={s:signal.signal(s,lambda _s,_f:quitting.set()) for s in (signal.SIGINT,signal.SIGTERM)}
    node=None;executor=None
    try:
        rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
        executor=MultiThreadedExecutor(num_threads=4);node=NavigationServer();executor.add_node(node)
        while rclpy.ok() and not quitting.is_set():executor.spin_once(timeout_sec=.05)
    finally:
        try:
            if node and rclpy.ok():
                node.closing=True;node.abort_event.set()
                # Cancellation and terminal result callbacks need a live executor
                # and context. Shutdown only after they finish or the bounded wait.
                until=time.monotonic()+node.cfg['cancel_timeout_sec']+.5
                while (node.busy or not node.track.idle()) and rclpy.ok() and time.monotonic()<until:
                    executor.spin_once(timeout_sec=.05)
                if node.busy or not node.track.idle():
                    node.get_logger().error('navigation shutdown: TrackPath stop/result unconfirmed')
            if executor:executor.shutdown(timeout_sec=1.)
            if node:node.destroy_node()
        finally:
            rclpy.try_shutdown()
            for sig,handler in old.items():signal.signal(sig,handler)
