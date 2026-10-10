"""树莓派 ROS TrackPath 服务端，闭环读取雷达位姿并发布车体 Twist。

只在持有 TrackPath 动作或确认停止期间发布；闲置时不抢 Dock 控制权。
ESP32 仍需独立命令看门狗和编码器轮速闭环。当前 map->odom 为 identity。
"""
import json
import math
import signal
import threading
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mission_interfaces.action import TrackPath
from .core import load_settings,pose_values,health_valid,measured_base_stopped,stamp_seconds
from .tracking_control import PathController,validate_goal

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')
        self.declare_parameter('config_file','');self.declare_parameter('interfaces_file','')
        self.cfg=load_settings(self.get_parameter('config_file').value)
        with open(self.get_parameter('interfaces_file').value,encoding='utf-8') as f:self.names=yaml.safe_load(f)
        self.lock=threading.RLock();self.busy=False;self.active=None;self.closing=False;self.fault=''
        self.odom=None;self.odom_stamp=-math.inf;self.odom_received=0.
        self.base=None;self.stop_base=None;self.base_stamp=-math.inf;self.base_received=0.
        self.lidar=None;self.nav=None;self.lidar_received=0.;self.nav_received=0.
        self.lidar_stamp=-math.inf;self.nav_stamp=-math.inf
        self.command=self.create_publisher(Twist,self.cfg['cmd_vel_topic'],1)
        self.status=self.create_publisher(String,'base/tracking_status',10)
        self.create_subscription(Odometry,self.cfg['odom_topic'],self.on_odom,10)
        self.create_subscription(String,self.names.get('base_feedback_topic',self.names['health_topic']),self.on_base,10)
        self.create_subscription(String,self.cfg['localization_status_topic'],lambda m:self.on_health(m,'lidar'),10)
        self.create_subscription(String,self.names['navigation_status_topic'],lambda m:self.on_health(m,'nav'),10)
        self.server=ActionServer(self,TrackPath,self.names['tracking_action'],
            goal_callback=self.accept,cancel_callback=self.cancel,execute_callback=self.execute,
            callback_group=ReentrantCallbackGroup())
        self.last_tick=time.monotonic();self.create_timer(1./self.cfg['tracking_frequency'],self.tick)
        self.create_timer(.2,self.publish_status)

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def on_odom(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        try:
            if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':return
            if stamp<=self.odom_stamp or not -.05<=self.now()-stamp<.5:return
            pose=pose_values(msg.pose.pose);v=msg.twist.twist
            velocity=(v.linear.x,v.linear.y,v.angular.z)
            if not all(math.isfinite(x) for x in velocity):return
            with self.lock:
                self.odom=(pose,velocity);self.odom_stamp=stamp;self.odom_received=time.monotonic()
        except ValueError:return

    def on_base(self,msg):
        try:
            data=json.loads(msg.data);stamp=stamp_seconds(data['stamp'])
            if stamp<=self.base_stamp or not -.05<=self.now()-stamp<self.cfg['feedback_timeout_sec']:return
            with self.lock:
                self.stop_base=data
                self.base=data if health_valid(data,self.now(),self.cfg['feedback_timeout_sec']) else None
                self.base_stamp=stamp;self.base_received=time.monotonic()
        except (ValueError,KeyError,TypeError):return

    def on_health(self,msg,name):
        try:
            data=json.loads(msg.data);sec=data['stamp']['sec'];ns=data['stamp']['nanosec']
            if type(sec) is not int or type(ns) is not int or not 0<=ns<10**9:return
            stamp=sec+ns*1e-9
            if stamp<=getattr(self,name+'_stamp') or not -.05<=self.now()-stamp<self.cfg['feedback_timeout_sec']:return
            with self.lock:
                setattr(self,name,data);setattr(self,name+'_received',time.monotonic())
                setattr(self,name+'_stamp',stamp)
        except (ValueError,KeyError,TypeError):return

    def feedback_inputs(self,require_nav=True):
        cfg=self.cfg;wall=time.monotonic()
        if not cfg['calibration_verified']:raise ValueError('calibration not verified')
        if self.fault:raise ValueError(self.fault)
        if self.closing:raise ValueError('tracker closing')
        if (self.odom is None or wall-self.odom_received>cfg['feedback_timeout_sec'] or
                not -.05<=self.now()-self.odom_stamp<.5):raise ValueError('radar odom stale')
        if (not self.base or wall-self.base_received>cfg['feedback_timeout_sec'] or
                not health_valid(self.base,self.now(),cfg['feedback_timeout_sec'])):
            raise ValueError('measured base/lift feedback unavailable')
        if (not self.lidar or self.lidar.get('valid') is not True or
                wall-self.lidar_received>cfg['feedback_timeout_sec'] or
                not -.05<=self.now()-self.lidar_stamp<cfg['feedback_timeout_sec']):raise ValueError('localization invalid')
        if require_nav and (not self.nav or self.nav.get('ready') is not True or
                wall-self.nav_received>cfg['feedback_timeout_sec'] or self.nav.get('fault') or
                not -.05<=self.now()-self.nav_stamp<cfg['feedback_timeout_sec']):
            raise ValueError('navigation heartbeat unavailable or unhealthy')
        return self.odom,self.base

    def accept(self,request):
        with self.lock:
            try:
                poses=validate_goal(request,self.cfg);(pose,vel),base=self.feedback_inputs()
                if self.busy or base['lift_is_up']!=request.carrying or not base['stopped']:return GoalResponse.REJECT
                if math.hypot(*vel[:2])>=request.stop_linear_speed or abs(vel[2])>=request.stop_angular_speed:
                    return GoalResponse.REJECT
                if math.dist(pose[:2],poses[0][:2])>request.position_tolerance:return GoalResponse.REJECT
                if not request.allow_in_place_rotation and abs(math.atan2(math.sin(pose[2]-poses[0][2]),
                        math.cos(pose[2]-poses[0][2])))>request.yaw_tolerance:return GoalResponse.REJECT
                self.busy=True;return GoalResponse.ACCEPT
            except ValueError as exc:
                self.get_logger().warn('拒绝轨迹：'+str(exc));return GoalResponse.REJECT

    def zero(self):self.command.publish(Twist())

    def cancel(self,_handle):
        with self.lock:
            if self.active:self.begin_stop(self.active,'cancelled')
            self.zero()
        return CancelResponse.ACCEPT

    def begin_stop(self,record,reason):
        if not record['reason']:
            record['reason']=reason;record['stop_started']=time.monotonic()
            record['stop_stamp']=self.now();record['stable_since']=None
            record['stable_frames']=0;record['last_stamp']=-math.inf
        self.zero()

    def stopped_after(self,record):
        # 停止确认仅依靠请求后新鲜雷达＋实际底盘测量，不依赖导航健康位。
        evidence=getattr(self,'stop_base',None) or self.base
        if (not self.odom or not evidence or self.odom_stamp<=record['stop_stamp'] or
                self.base_stamp<=record['stop_stamp'] or
                time.monotonic()-self.odom_received>self.cfg['feedback_timeout_sec'] or
                not -.05<=self.now()-self.odom_stamp<.5 or
                not measured_base_stopped(evidence,self.now(),self.cfg['feedback_timeout_sec'])):
            record['stable_since']=None;record['stable_frames']=0;return False
        velocity=self.odom[1];g=record['handle'].request
        if (math.hypot(*velocity[:2])>=g.stop_linear_speed or
                abs(velocity[2])>=g.stop_angular_speed):
            record['stable_since']=None;record['stable_frames']=0;return False
        if self.odom_stamp>record['last_stamp']:
            if record['stable_since'] is None:record['stable_since']=self.odom_stamp
            record['stable_frames']+=1;record['last_stamp']=self.odom_stamp
        return (record['stable_frames']>=3 and
                self.odom_stamp-record['stable_since']>=g.settle_duration_sec)

    def finish(self,record,success,stopped,message):
        self.zero();result=TrackPath.Result();result.success=success;result.stopped=stopped
        result.error_code='' if success else 'TRACKING_FAILED';result.message=message
        if self.odom:
            p=PoseStamped();p.header.frame_id='map'
            p.header.stamp.sec=int(self.odom_stamp);p.header.stamp.nanosec=int((self.odom_stamp%1)*1e9)
            x,y,yaw=self.odom[0];p.pose.position.x=x;p.pose.position.y=y
            p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
            result.final_pose=p
        record['result']=result;record['done'].set()

    def tick(self):
        with self.lock:
            record=self.active;now=time.monotonic();dt=now-self.last_tick;self.last_tick=now
            if not record or record['done'].is_set():return
            handle=record['handle'];g=handle.request
            if not record['reason']:
                try:
                    if handle.is_cancel_requested:raise ValueError('cancelled')
                    if now-record['started']>g.max_duration_sec:raise ValueError('trajectory timeout')
                    if dt>self.cfg['tracking_command_timeout_sec']:raise ValueError('control loop stalled')
                    (pose,vel),base=self.feedback_inputs()
                    if base['lift_is_up']!=g.carrying:raise ValueError('payload changed')
                    # 跟踪仅在导航拥有当前运动并持续检查地图时执行。
                    # 动作开始与周期心跳不同步，先保持零速度等待首次 active 确认。
                    if self.nav.get('active') is not True or self.nav_stamp<record.get('start_stamp',-math.inf):
                        self.zero()
                        if record.get('nav_started') or now-record['started']>self.cfg['feedback_timeout_sec']:
                            raise ValueError('navigation path monitor not active')
                        return
                    record['nav_started']=True
                    controller=record['controller']
                    vx,vy,wz=controller.step(pose,vel,self.odom_stamp,base['stopped'],dt)
                    cmd=Twist();cmd.linear.x=vx;cmd.linear.y=vy;cmd.angular.z=wz
                    self.command.publish(cmd)
                    feedback=TrackPath.Feedback();feedback.segment_index=max(0,controller.index-1)
                    feedback.distance_remaining=float(controller.distance_remaining(pose));feedback.phase=controller.phase
                    handle.publish_feedback(feedback)
                    if controller.done:self.finish(record,True,True,'radar pose and measured base settled')
                except (ValueError,ArithmeticError) as exc:self.begin_stop(record,str(exc))
            if record['reason']:
                self.zero()
                if self.stopped_after(record):self.finish(record,False,True,record['reason'])
                elif now-record['stop_started']>self.cfg['cancel_timeout_sec']:
                    self.fault='physical stop unconfirmed; inspect base before restarting'
                    self.finish(record,False,False,record['reason']+'; '+self.fault)

    def execute(self,handle):
        with self.lock:
            record={'handle':handle,'controller':PathController(handle.request,self.cfg),
                'done':threading.Event(),'started':time.monotonic(),'start_stamp':self.now(),'reason':'','result':None}
            self.active=record;self.last_tick=time.monotonic()
        try:
            while not record['done'].wait(.05):
                # During orderly shutdown timers/subscriptions keep running to
                # confirm the requested physical stop. Context loss cannot certify it.
                if not rclpy.ok():
                    with self.lock:self.finish(record,False,False,'tracker shutdown; stop not confirmed')
            result=record['result']
            if handle.is_cancel_requested:result.success=False;handle.canceled()
            elif result.success:handle.succeed()
            else:handle.abort()
            return result
        finally:
            with self.lock:
                self.zero();self.active=None;self.busy=False

    def publish_status(self):
        with self.lock:
            try:self.feedback_inputs(require_nav=False);ready=True;reason=''
            except ValueError as exc:ready=False;reason=str(exc)
            stamp=self.get_clock().now().to_msg()
            self.status.publish(String(data=json.dumps({'stamp':{'sec':stamp.sec,'nanosec':stamp.nanosec},
                'ready':ready,'active':self.busy,'fault':self.fault,'reason':reason,
                'phase':self.active['controller'].phase if self.active else 'IDLE',
                'segment_index':max(0,self.active['controller'].index-1) if self.active else None})))

def main(args=None):
    quitting=threading.Event()
    old={s:signal.signal(s,lambda _s,_f:quitting.set()) for s in (signal.SIGINT,signal.SIGTERM)}
    node=None;executor=None
    try:
        rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
        executor=MultiThreadedExecutor(num_threads=4);node=PathTracker();executor.add_node(node)
        while rclpy.ok() and not quitting.is_set():executor.spin_once(timeout_sec=.05)
    finally:
        try:
            if node and rclpy.ok():
                with node.lock:
                    node.closing=True
                    if node.active:node.begin_stop(node.active,'tracker shutdown')
                    elif node.busy:node.zero()
                # Keep the ROS context, stop timer and incoming measurements alive.
                # Executor.shutdown first would disable the very callbacks needed
                # to confirm stop, then the action worker could only time out.
                until=time.monotonic()+node.cfg['cancel_timeout_sec']+.5
                while node.busy and rclpy.ok() and time.monotonic()<until:
                    executor.spin_once(timeout_sec=.05)
                if node.busy:
                    with node.lock:
                        if node.active and not node.active['done'].is_set():
                            node.finish(node.active,False,False,'shutdown stop unconfirmed')
                    node.get_logger().error('shutdown stop unconfirmed; check chassis watchdog')
            if executor:executor.shutdown(timeout_sec=1.)
            if node:node.destroy_node()
        finally:
            rclpy.try_shutdown()
            for sig,handler in old.items():signal.signal(sig,handler)
