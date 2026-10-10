"""Gazebo execution endpoints for the 7503 framework; no task loop here.

Hardware handoff: this is the Gazebo composite bridge, not a hardware node.
Hybrid navigation and Dock use robot_navigation/TrackPath; this bridge only
substitutes Gazebo lift and actuator feedback. Ordinary legacy rack simulation
can still enable its own navigation/Dock. See docs/导航接入与部署.md.

ROS callbacks run in an executor. Blocking control routines run only in separate
action/service callback groups and must never spin a second executor.
"""
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

from .project_paths import navigation_root
RACK_ROOT=navigation_root()
sys.path.insert(0,str(RACK_ROOT/'scripts'))
import scene
from mission_v3 import MissionV3, yaw, stamp_seconds
from frame_geometry import apply, inverse, wrap
import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Path as ROSPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool, Float64
from std_srvs.srv import SetBool, Trigger
from mission_interfaces.action import Dock, NavigateToPoint
from robot_navigation.point_targets import PointTargets, PointGoalHandle, stamped_target, point_result


class FrameworkExecution(MissionV3):
    def __init__(self):
        self.abort_event=threading.Event();self.claim_lock=threading.Lock()
        self.busy=False;self.initialized=False;self.active_handle=None
        self.camera_seen=0.
        self.image_lock=threading.Lock();self.pending_image=None
        self.decode_closing=threading.Event()
        self.entered_rack=None;self.last_frame_ns=None;self.operation_deadline=None
        super().__init__(subscribe_next=False)
        self.accepting=False;self.released=True
        self.declare_parameter('interface_prefix','/team2/sim')
        prefix=self.get_parameter('interface_prefix').value.rstrip('/')
        self.path_pub=self.create_publisher(ROSPath,prefix+'/navigation/path',10)
        self.qr_pub=self.create_publisher(String,prefix+'/qr/detections',10)
        self.health_pub=self.create_publisher(String,prefix+'/robot/status',10)
        self.up_pub=self.create_publisher(Bool,prefix+'/lift/is_up',10)
        self.base_feedback_pub=self.create_publisher(String,prefix+'/base/feedback',10)
        self.create_subscription(String,prefix+'/mission/status',self.on_mission_context,10)
        self.mission_context=None;self.mission_seen=0.;self.point_pose=None
        self.point_targets=PointTargets({'simulation_only':True,
            'points':{**{k:[*scene.scan(k),scene.MODEL_YAW] for k in scene.RACKS},
                      'START_SCAN':[*scene.START_SCAN,scene.MODEL_YAW+math.pi/6],
                      'FINAL_SCAN':[*scene.END_SCAN,scene.MODEL_YAW+math.pi/6]},
            'drop_by_rack':{k:[*scene.DROP[k],scene.MODEL_YAW] for k in scene.RACKS}})
        group=ReentrantCallbackGroup()
        self.declare_parameter('provide_navigation',True)
        self.declare_parameter('provide_docking',True)
        self.nav_server=None
        if self.get_parameter('provide_navigation').value:
            self.point_server=ActionServer(self,NavigateToPoint,prefix+'/navigate_to_point',
                self.navigate_point,goal_callback=self.claim_point,cancel_callback=self.cancel,callback_group=group)
            self.nav_server=ActionServer(self,NavigateToPose,prefix+'/navigation/navigate_to_pose',
                self.navigate,goal_callback=self.claim,cancel_callback=self.cancel,callback_group=group)
        self.dock_server=None
        if self.get_parameter('provide_docking').value:
            self.dock_server=ActionServer(self,Dock,prefix+'/docking/execute',
                self.dock,goal_callback=self.claim,cancel_callback=self.cancel,callback_group=group)
        self.create_service(SetBool,prefix+'/lift/set_up',self.lift,callback_group=group)
        self.create_service(Trigger,prefix+'/safety/stop',self.stop_motion,callback_group=group)
        self.create_timer(.1,self.health)
        self.declare_parameter('use_builtin_qr',False)
        self.use_builtin_qr=self.get_parameter('use_builtin_qr').value
        self.qr_worker=None
        if self.use_builtin_qr:
            self.qr_worker=threading.Thread(target=self.decode_images,daemon=True)
            self.qr_worker.start()

    def on_mission_context(self,msg):
        try:
            data=json.loads(msg.data);stamp=data['stamp_ns']
            now=self.get_clock().now().nanoseconds*1e-9
            if type(stamp) is not int or not -.05<=now-stamp*1e-9<.6:return
            self.mission_context=data;self.mission_seen=time.monotonic()
        except (ValueError,TypeError,KeyError):return

    def claim_point(self,request):
        try:
            context=self.mission_context if time.monotonic()-self.mission_seen<.6 else None
            pose=self.point_targets.resolve(request.target_id,carrying=bool(self.current_payload),
                context=context,simulation=True)
            answer=self.claim(request)
            if answer==GoalResponse.ACCEPT:self.point_pose=stamped_target(pose,self.get_clock().now().to_msg())
            return answer
        except ValueError as exc:
            self.get_logger().warn(str(exc));return GoalResponse.REJECT

    def navigate_point(self,handle):
        return point_result(self.navigate(PointGoalHandle(handle,self.point_pose)))

    def spin(self,seconds=.02):
        # Subscriptions and service futures are serviced by MultiThreadedExecutor.
        time.sleep(max(.001,seconds))
        handle=self.active_handle
        if handle is not None and isinstance(handle.request,NavigateToPose.Goal) and self.pose and self.goal is not None:
            feedback=NavigateToPose.Feedback()
            feedback.distance_remaining=float(math.dist(self.pose[:2],self.goal))
            handle.publish_feedback(feedback)

    def fresh(self,radar=True):
        return (MissionV3.fresh(self,radar) and
                not (self.busy and (self.abort_event.is_set() or
                    (self.operation_deadline is not None and time.monotonic()>self.operation_deadline))))

    def claim(self,request):
        with self.claim_lock:
            if self.busy or not self.initialized or not MissionV3.fresh(self):return GoalResponse.REJECT
            if isinstance(request,Dock.Goal):
                if request.rack_id not in scene.RACKS or request.operation not in ('ENTER','EXIT'):return GoalResponse.REJECT
                if request.expected_qr!=scene.RACKS[request.rack_id][5]:return GoalResponse.REJECT
                if not 0<request.max_duration_sec<=600:return GoalResponse.REJECT
            self.busy=True;self.abort_event.clear()
        return GoalResponse.ACCEPT

    def cancel(self,handle):
        self.abort_event.set();self.pub_vel()
        return CancelResponse.ACCEPT

    def finish(self,handle,result,ok):
        self.pub_vel();self.active_handle=None;self.operation_deadline=None
        try:
            if handle.is_cancel_requested:handle.canceled()
            elif ok:handle.succeed()
            else:handle.abort()
        finally:
            with self.claim_lock:self.busy=False
        return result

    def map_from_odom(self,timeout=2.):
        deadline=time.monotonic()+timeout;reported=False;reason='map->odom unavailable'
        while rclpy.ok():
            try:
                transform=self.tf.lookup_transform(scene.MAP_FRAME,scene.ODOM_FRAME,Time())
                age=self.get_clock().now().nanoseconds*1e-9-stamp_seconds(transform.header.stamp)
                if stamp_seconds(transform.header.stamp)==0. or -.5<=age<1.5:
                    t=transform.transform
                    return t.translation.x,t.translation.y,yaw(t.rotation)
                reason=f'map->odom transform stale: {age:.3f}s'
            except Exception as error:
                reason=f'map->odom unavailable: {error}'
            # A transient TF gap at action boundaries is not an action failure.
            # Keep stopped while callbacks receive an actually fresh transform;
            # never restamp cached TF or relax its age limit.
            self.pub_vel()
            if not self.fresh() or time.monotonic()>=deadline:
                raise ValueError(reason+'; fresh TF wait failed')
            if not reported:
                self.get_logger().warn(reason+'; stopped, waiting up to 2s for fresh TF')
                reported=True
            self.spin(.05)
        self.pub_vel()
        raise ValueError(reason+'; ROS context closed')

    def plan_for(self,target):
        points=super().plan_for(target)
        path=ROSPath();path.header.frame_id=scene.MAP_FRAME
        path.header.stamp=self.get_clock().now().to_msg()
        if points is not None:
            try:
                transform=self.map_from_odom()
                for point in [self.plan_origin,*points]:
                    x,y=apply(transform,point)
                    pose=PoseStamped();pose.header=path.header
                    pose.pose.position.x=float(x);pose.pose.position.y=float(y)
                    heading=wrap(self.pose[2]+transform[2])
                    pose.pose.orientation.z=math.sin(heading/2)
                    pose.pose.orientation.w=math.cos(heading/2)
                    path.poses.append(pose)
            except Exception as error:
                self.get_logger().error(f'Path map conversion failed: {error}')
                path.poses=[];points=None
        self.path_pub.publish(path)
        return points

    def turn(self,target):
        if self.current_payload:
            # Loaded navigation may verify heading but must never rotate cargo.
            # Use the existing .02 transport admission limit, not scan-turn .012.
            self.pub_vel()
            if abs(math.atan2(math.sin(target-scene.MODEL_YAW),math.cos(target-scene.MODEL_YAW)))>.02:
                self.get_logger().error('载货禁止扫描转向');return False
            center=tuple(self.pose[:2])
            if not self.fresh(radar=False) or not self.wait_still(center):
                self.get_logger().error('载货停稳/数据确认失败');return False
            error=abs(math.atan2(math.sin(self.pose[2]-scene.MODEL_YAW),
                                 math.cos(self.pose[2]-scene.MODEL_YAW)))
            if error>.02:
                self.get_logger().error(f'载货航向超出运输容差: error={error:.6f}rad；停车，不旋转')
                return False
            return True
        deadline=time.monotonic()+30
        center=tuple(self.pose[:2])
        while rclpy.ok() and time.monotonic()<deadline:
            if not self.fresh(radar=False):self.pub_vel();return False
            error=math.atan2(math.sin(target-self.pose[2]),math.cos(target-self.pose[2]))
            if abs(error)<.006:
                self.pub_vel()
                if self.wait_still(center):
                    final_error=math.atan2(math.sin(target-self.pose[2]),math.cos(target-self.pose[2]))
                    if abs(final_error)<.006:return True
                continue
            from geometry_msgs.msg import Twist
            command=Twist();command.angular.z=math.copysign(min(scene.TURN_SPEED,abs(error)*scene.TURN_GAIN),error)
            self.cmd_pub.publish(command);self.spin(.02)
        self.pub_vel();return False

    def navigate(self,handle):
        result=NavigateToPose.Result();ok=False;self.active_handle=handle
        self.operation_deadline=time.monotonic()+300.
        try:
            pose=handle.request.pose
            if pose.header.frame_id!=scene.MAP_FRAME:raise ValueError('navigation targets must use map frame')
            heading=yaw(pose.pose.orientation)
            if not all(math.isfinite(v) for v in (pose.pose.position.x,pose.pose.position.y,heading)):
                raise ValueError('non-finite navigation goal')
            allowed=(scene.MODEL_YAW,scene.MODEL_YAW+math.pi/6)
            if min(abs(math.atan2(math.sin(heading-v),math.cos(heading-v))) for v in allowed)>.02:
                raise ValueError('only transport heading or left-30-degree scan heading is allowed')
            transform=self.map_from_odom()
            goal_odom=apply(inverse(transform),(pose.pose.position.x,pose.pose.position.y))
            final_heading_odom=wrap(heading-transform[2])
            self.get_logger().info(f'坐标确认: goal_frame=map goal_map_xy={(pose.pose.position.x,pose.pose.position.y)} '
                f'goal_odom_xy={goal_odom} map_from_odom={transform} odom_pose={self.pose}')
            # START confirmation already dispatches this action automatically.
            # Restore transport heading and verify stop before any translation.
            if self.current_payload and abs(math.atan2(math.sin(self.pose[2]-scene.MODEL_YAW),
                    math.cos(self.pose[2]-scene.MODEL_YAW)))>.02:
                raise RuntimeError('loaded chassis is not in transport heading')
            if not self.turn(scene.MODEL_YAW):raise RuntimeError('transport heading restore failed')
            self.get_logger().info(f'回正确认，立即开始导航: odom_yaw={self.pose[2]:.6f}')
            # Restore may take seconds; use fresh map conversion before motion.
            transform=self.map_from_odom()
            goal_odom=apply(inverse(transform),(pose.pose.position.x,pose.pose.position.y))
            target={'name':'navigation','xy':goal_odom}
            if self.current_payload:target['name']='drop_'+self.current_payload
            self.goal=self.target_goal(target);self.released=True;self.error=None
            if not self.move_target(target):raise RuntimeError(self.error or 'navigation failed')
            final_heading_odom=wrap(heading-self.map_from_odom()[2])
            if not self.turn(final_heading_odom):raise RuntimeError('loaded heading/stop verification failed' if self.current_payload else 'scan turn failed')
            ok=True;result.error_code=0
        except Exception as error:
            result.error_code=1;result.error_msg=str(error);self.get_logger().error(str(error))
        return self.finish(handle,result,ok)

    def dock(self,handle):
        result=Dock.Result();ok=False;self.active_handle=handle
        try:
            request=handle.request;name=request.rack_id
            self.operation_deadline=time.monotonic()+request.max_duration_sec
            if request.operation=='ENTER':
                if self.current_payload:raise RuntimeError('cannot enter while carrying')
                # Framework released ENTER for this target (QR result or configured simulation timeout).
                # Do not require a private camera-decoder cache.
                if not self.turn(scene.MODEL_YAW):raise RuntimeError('heading restore failed')
                self.goal=scene.rack_model_center(name);self.released=True
                if not self.move_target({'name':'under_'+name}):raise RuntimeError('entry failed')
                if not self.select_payload(name):raise RuntimeError('payload selection not acknowledged')
                self.entered_rack=name;result.ready_to_lift=True
            else:
                if self.current_payload or self.status.get('engaged'):raise RuntimeError('payload not released')
                if not self.exit_rack():raise RuntimeError('exit failed')
                result.exited=True;self.entered_rack=None
            ok=True;result.success=True
        except Exception as error:
            result.success=False;result.error_code='EXECUTION_FAILED';result.message=str(error)
        return self.finish(handle,result,ok)

    def lift(self,request,response):
        with self.claim_lock:
            if self.busy or not self.initialized:
                response.success=False;response.message='execution busy or not ready';return response
            self.busy=True;self.abort_event.clear()
        try:
            self.operation_deadline=time.monotonic()+75.
            name=self.prepare_lift_up() if request.data else self.current_payload
            if not name:raise RuntimeError('no confirmed rack for lift request')
            if not self.wait_lift(bool(request.data)):
                raise RuntimeError('measured lift/payload confirmation failed: '+str(self.status.get('fault') or self.data_diagnostic()))
            if request.data:self.current_payload=name
            else:
                center=self.status.get('payload_center')
                if center is None or math.dist(apply(self.map_from_odom(),center[:2]),scene.DROP[name])>.025:
                    raise RuntimeError('actual drop position mismatch in map frame')
                self.placed[name]=tuple(center[:2]);self.current_payload=None
                if not self.select_payload(None):raise RuntimeError('release selection not acknowledged')
            if not self.set_footprint():raise RuntimeError('footprint update failed')
            response.success=True;response.message='joint and actual payload pose confirmed'
        except Exception as error:response.success=False;response.message=str(error)
        finally:
            self.pub_vel()
            self.operation_deadline=None
            with self.claim_lock:self.busy=False
        return response

    def prepare_lift_up(self):
        # 普通历史仿真由自身 Dock 选货；联合验证覆写为模拟升降执行器准备。
        return self.entered_rack

    def wait_measured_stop(self,deadline):
        # A fault/readiness flag is not stop evidence. Require independently
        # measured chassis + joint speeds, fresh radar, and multiple new frames.
        begin=self.get_clock().now().nanoseconds*1e-9;first=None;last=-math.inf;frames=0
        while rclpy.ok() and time.monotonic()<deadline:
            self.pub_vel();self.spin(.02)
            now=self.get_clock().now().nanoseconds*1e-9
            stamp=self.status.get('stamp',{});source=stamp.get('sec',-math.inf)+stamp.get('nanosec',0)*1e-9
            measured=(time.monotonic()-self.status_seen<.6 and self.status.get('actuator_feedback_fresh') is True and
                      source>begin and -.05<=now-source<.6 and self.status.get('base_motion_stopped') is True and
                      self.status.get('lift_motion_stopped') is True)
            radar=(self.pose is not None and time.monotonic()-self.odom_seen<.6 and
                   self.speed<.01 and self.angular_speed<.01)
            if measured and radar and source>last:
                first=source if first is None else first;last=source;frames+=1
                if frames>=3 and source-first>=.4:return True
            elif not (measured and radar):first=None;frames=0
        return False

    def stop_motion(self,request,response):
        self.abort_event.set();self.pub_vel()
        # Hold the measured physical joint; never automatically lower cargo on stop.
        self.lift_pub.publish(Float64(data=float(self.status.get('lift',scene.LIFT_DOWN))))
        deadline=time.monotonic()+4.
        while rclpy.ok() and time.monotonic()<deadline and self.busy:time.sleep(.02)
        # Reassert hold after the previous operation has left its command loop.
        # Otherwise a lift command already in flight could overwrite the first hold.
        self.lift_pub.publish(Float64(data=float(self.status.get('lift',scene.LIFT_DOWN))))
        response.success=not self.busy and self.wait_measured_stop(deadline=time.monotonic()+4.)
        response.message='stopped and old operation ended' if response.success else 'stop not confirmed'
        return response

    def on_image(self,message):
        self.camera_seen=time.monotonic()
        if getattr(self,'use_builtin_qr',False):
            with self.image_lock:self.pending_image=message

    def decode_images(self):
        while not self.decode_closing.wait(.01):
            with self.image_lock:
                message=self.pending_image;self.pending_image=None
            if message is not None:
                try:self.process_image(message)
                except Exception as error:
                    if not self.decode_closing.is_set():self.get_logger().error(f'QR worker: {error}')

    def process_image(self,message):
        codes=[];status='ok'
        ns=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if self.last_frame_ns==ns:return
        self.last_frame_ns=ns
        try:
            channels={'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'mono8':1}[message.encoding]
            data=np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)
            image=data[:,:message.width*channels].reshape(message.height,message.width,channels)
            conversions={'rgb8':cv2.COLOR_RGB2GRAY,'bgr8':cv2.COLOR_BGR2GRAY,
                         'rgba8':cv2.COLOR_RGBA2GRAY,'bgra8':cv2.COLOR_BGRA2GRAY}
            image=cv2.cvtColor(image,conversions[message.encoding]) if channels>1 else image[:,:,0]
            found,decoded,_,_=self.qr_detector.detectAndDecodeMulti(image)
            codes=[value for value in decoded if value] if found else []
            if not codes:
                value,_,_=self.qr_detector.detectAndDecode(image)
                if value:codes=[value]
        except (ValueError,KeyError,cv2.error):status='decode_error'
        if hasattr(self,'qr_pub') and not self.decode_closing.is_set():
            self.qr_pub.publish(String(data=json.dumps({'image_stamp':{'sec':message.header.stamp.sec,
                'nanosec':message.header.stamp.nanosec},'status':status,
                'detections':[{'raw':code} for code in codes]})))

    def health(self):
        # Reuse the same asynchronous reader as planning and control.
        grid=self.get_grid()
        base=MissionV3.fresh(self,radar=False)
        navigation=base and self.initialized and grid is not None
        stamp=self.get_clock().now().to_msg()
        stopped=(not self.busy and self.pose is not None and time.monotonic()-self.odom_seen<.6
                 and time.monotonic()-self.status_seen<.6 and self.speed<.01 and self.angular_speed<.01
                 and self.status.get('actuator_feedback_fresh') is True
                 and self.status.get('base_motion_stopped') is True
                 and self.status.get('lift_motion_stopped') is True)
        measured_up=bool(self.status.get('lift_settled') and self.status.get('carrying'))
        feedback=String(data=json.dumps({'schema_version':1,
            'stamp':{'sec':stamp.sec,'nanosec':stamp.nanosec},
            'modules':{'navigation':bool(navigation),'qr':time.monotonic()-self.camera_seen<2.,
                       'docking':bool(base),'lift':bool(base),'base':bool(base)},
            'stopped':bool(stopped),'lift_is_up':measured_up if self.status.get('lift_settled') else None,'lift_state_source':'measured',
            'lift_settled':bool(self.status.get('lift_settled',False)),
            'base_motion_stopped':bool(self.status.get('actuator_feedback_fresh') and self.status.get('base_motion_stopped')),
            'lift_motion_stopped':bool(self.status.get('actuator_feedback_fresh') and self.status.get('lift_motion_stopped')),
            'fault':self.status.get('fault') or '',
            'navigation_diagnostic':self.map_error or '',
            'costmap_pipeline':self.costmaps.diagnostic()}))
        self.base_feedback_pub.publish(feedback)
        # 主框架只使用 ready/fault；详细实测状态属于导航内部反馈。
        details=json.loads(feedback.data)
        ready=bool(navigation and base and self.initialized and not details['fault'])
        self.health_pub.publish(String(data=json.dumps({'ready':ready,'fault':details['fault']})))
        self.up_pub.publish(Bool(data=measured_up))

    def warmup(self):
        if self.wait_ready() and self.set_footprint():
            self.initialized=True;self.set_state('EXECUTION_READY')
            self.get_logger().info('FRAMEWORK_READY: Gazebo execution interfaces and real data ready')
        else:self.get_logger().error('FRAMEWORK_STARTUP_FAILED: '+getattr(self,'startup_diagnostic','footprint confirmation failed'))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=FrameworkExecution();executor=MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    import signal
    closing=threading.Event()
    def stop(signum,frame):node.abort_event.set();node.decode_closing.set();closing.set();node.pub_vel()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    worker=threading.Thread(target=node.warmup,daemon=True);worker.start()
    try:
        while rclpy.ok() and not closing.is_set():executor.spin_once(timeout_sec=.05)
        until=time.monotonic()+5.
        while node.busy and time.monotonic()<until:executor.spin_once(timeout_sec=.02)
    finally:
        node.decode_closing.set()
        if node.qr_worker is not None:node.qr_worker.join(timeout=2.)
        node.pub_vel();executor.shutdown(timeout_sec=2.);node.destroy_node();rclpy.try_shutdown()
