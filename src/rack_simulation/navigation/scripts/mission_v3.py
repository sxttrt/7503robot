#!/usr/bin/env python3
"""13 manual target releases; automatic QR/lift/drop actions after fresh settled odom.

World coordinates are the absolute odom coordinates estimated from radar/fixed walls.
Measured map cells are transformed through map<-odom before footprint collision tests.
Only entry/exit under a rack are explicitly permitted to bypass the costmap.
"""
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import traceback

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import try_shutdown
from rclpy.time import Time
from geometry_msgs.msg import Twist, Polygon, Point32
from nav_msgs.msg import Odometry
from nav2_msgs.srv import GetCostmap
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float64, String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from tf2_ros import Buffer, TransformListener

import scene
from targets import build
from grid_planner import GridMap, Planner
from costmap_cache import CostmapCache

MAX_RETRY=3
POSITION_TOL=.012
STILL_SECONDS=.4
DATA_TIMEOUT=1.0


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def stamp_seconds(s):
    return s.sec+s.nanosec*1e-9


class MissionV3(Node):
    def __init__(self,subscribe_next=True):
        super().__init__('mission_v3')
        self.state='STARTING';self.error=None;self.goal_i=0
        self.targets=build();self.pose=None;self.odom_seen=0.;self.odom_seq=0
        self.speed=math.inf;self.angular_speed=math.inf;self.last_odom_stamp=None
        self.scan_seen=0.;self.scan_stamp=None
        self.status={};self.status_seen=0.;self.carry=False
        self.next_flag=False;self.accepting=False;self.released=False
        self.retry=0;self.n_plans=self.n_segs=self.n_turns=0
        self.placed={};self.current_payload=None;self.goal=None
        self.qr_expected=None;self.qr_hits=0;self.qr_seen=0.;self.qr_detector=cv2.QRCodeDetector()
        self.cmd_pub=self.create_publisher(Twist,'cmd_vel',10)
        self.lift_pub=self.create_publisher(Float64,'lift_cmd',10)
        self.payload_pub=self.create_publisher(String,'payload_name',10)
        self.fp_pubs=[self.create_publisher(Polygon,f'/{n}/footprint',10)
                      for n in ('local_costmap','global_costmap')]
        self.state_pub=self.create_publisher(String,'/rack/state',10)
        self.create_subscription(Odometry,'odom',self.on_odom,10)
        self.create_subscription(LaserScan,'scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Image,'camera/image_raw',self.on_image,qos_profile_sensor_data)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool,'carry_state',lambda m:setattr(self,'carry',bool(m.data)),latched)
        self.create_subscription(String,'payload_status',self.on_status,latched)
        if subscribe_next:self.create_subscription(Bool,'/rack/next',self.on_next,10)
        self.cm_cli=None
        self.costmaps=self.make_costmap_reader()
        self.create_timer(.1,self.costmaps.poll)
        self.setp_clis=[self.create_client(SetParameters,f'/{n}/{n}/set_parameters')
                       for n in ('local_costmap','global_costmap')]
        self.tf=Buffer();self.tf_listener=TransformListener(self.tf,self)
        self.fp_msg=None
        self.create_timer(.5,self.republish_footprint)
        self.create_timer(.5,lambda:self.state_pub.publish(String(data=self.state)))

    def make_costmap_reader(self):
        self.cm_cli=self.create_client(GetCostmap,'/global_costmap/get_costmap')
        return CostmapCache(self.cm_cli,GetCostmap.Request)

    def set_state(self,value):
        self.get_logger().info(f'[{self.state} -> {value}]')
        self.state=value
        self.state_pub.publish(String(data=value))

    def on_odom(self,m):
        if m.header.frame_id!=scene.ODOM_FRAME or m.child_frame_id!=scene.BASE_FRAME:return
        stamp=stamp_seconds(m.header.stamp)
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if not -.05<=age<.5 or (self.last_odom_stamp is not None and stamp<=self.last_odom_stamp):return
        values=(m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation),
                m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z)
        if not all(math.isfinite(v) for v in values):return
        self.last_odom_stamp=stamp
        self.pose=values[:3]
        self.speed=math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)
        self.angular_speed=abs(m.twist.twist.angular.z)
        self.odom_seen=time.monotonic();self.odom_seq+=1

    def on_scan(self,m):
        stamp=stamp_seconds(m.header.stamp)
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if m.header.frame_id!=scene.LASER_FRAME or not -.05<=age<.5:return
        if self.scan_stamp is not None and stamp<=stamp_seconds(self.scan_stamp):return
        if any(math.isfinite(v) and m.range_min<=v<=m.range_max for v in m.ranges):
            self.scan_seen=time.monotonic();self.scan_stamp=m.header.stamp

    def on_status(self,m):
        try:self.status=json.loads(m.data);self.status_seen=time.monotonic()
        except (ValueError,TypeError):pass

    def on_next(self,m):
        if m.data and self.accepting and not self.next_flag:
            self.next_flag=True
            # Close immediately so queued duplicates cannot leak to another target.
            self.accepting=False
        elif m.data:
            self.get_logger().warn(f'忽略重复或非等待阶段触发: {self.state}')

    def on_image(self,m):
        if not self.qr_expected:return
        try:
            channels={'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'mono8':1}.get(m.encoding)
            if channels is None:return
            rows=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)
            image=rows[:,:m.width*channels].reshape(m.height,m.width,channels)
            if m.encoding=='rgb8':image=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
            elif m.encoding=='bgr8':image=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            elif m.encoding=='rgba8':image=cv2.cvtColor(image,cv2.COLOR_RGBA2GRAY)
            elif m.encoding=='bgra8':image=cv2.cvtColor(image,cv2.COLOR_BGRA2GRAY)
            else:image=image[:,:,0]
            codes=[]
            if hasattr(self.qr_detector,'detectAndDecodeMulti'):
                found,decoded,_,_=self.qr_detector.detectAndDecodeMulti(image)
                if found:codes.extend(decoded)
            if not codes:
                code,_,_=self.qr_detector.detectAndDecode(image);codes.append(code)
            if self.qr_expected in codes:
                self.qr_hits+=1;self.qr_seen=time.monotonic()
            else:self.qr_hits=0
        except (ValueError,cv2.error) as e:
            self.get_logger().warn(f'相机帧解码失败: {e}')

    def spin(self,seconds=.02):
        rclpy.spin_once(self,timeout_sec=seconds)

    def request(self,cli,req,timeout=2.):
        if not cli.service_is_ready():return None
        future=cli.call_async(req);deadline=time.monotonic()+timeout
        while rclpy.ok() and not future.done() and time.monotonic()<deadline:self.spin()
        if not future.done():
            cli.remove_pending_request(future);future.cancel();return None
        return future.result()

    def fresh(self,radar=True):
        now=time.monotonic()
        return (self.pose is not None and now-self.odom_seen<DATA_TIMEOUT and
                now-self.status_seen<DATA_TIMEOUT and not self.status.get('fault') and
                self.status.get('ready',False) and
                (not radar or now-self.scan_seen<DATA_TIMEOUT))

    def get_grid(self):
        self.map_error=None
        if not self.fresh():self.map_error='sensor/driver data stale or fault';return None
        cm,received,_=self.costmaps.snapshot()
        if cm is None:self.map_error=self.costmaps.diagnostic()['error'];return None
        receive_age=time.monotonic()-received
        if receive_age>=1.5:self.map_error=f'costmap receive age={receive_age:.3f}s';return None
        now=self.get_clock().now().nanoseconds*1e-9
        age=now-stamp_seconds(cm.metadata.update_time)
        if not (0<=age<1.5):self.map_error=f'costmap age={age:.3f}s';return None
        try:
            transform=self.tf.lookup_transform(cm.header.frame_id,scene.ODOM_FRAME,Time())
            transform_age=now-stamp_seconds(transform.header.stamp)
            # A static transform (TF stamp zero) is timeless; dynamic TF must be fresh.
            if cm.header.frame_id!=scene.ODOM_FRAME and stamp_seconds(transform.header.stamp)!=0. and not (-.5<=transform_age<1.5):
                self.map_error=f'TF age={transform_age:.3f}s';return None
            t=transform.transform
            return GridMap(cm.metadata,cm.data,(t.translation.x,t.translation.y,yaw(t.rotation)))
        except Exception as e:
            self.map_error=str(e)
            self.get_logger().warn(f'地图/TF 未就绪: {e}')
            return None

    def half_size(self):
        hx,hy=scene.ROBOT_L/2,scene.ROBOT_W/2
        if self.current_payload:
            dx,dy=self.status.get('relative_xy',[0.,0.])
            n=self.current_payload
            x0,x1,y0,y1=scene.RACKS[n][:4]
            hx=max(hx,abs(dx)+(x1-x0)/2)
            hy=max(hy,abs(dy)+(y1-y0)/2)
        # Heading is fixed; include the measured small yaw error conservatively.
        heading=self.pose[2] if self.pose else scene.MODEL_YAW
        return abs(math.cos(heading))*hx+abs(math.sin(heading))*hy,abs(math.sin(heading))*hx+abs(math.cos(heading))*hy

    def obstacle_rects(self):
        result=[]
        for n in scene.RACKS:
            if n==self.current_payload:continue
            center=self.placed.get(n,scene.rack_model_center(n))
            if self.current_payload:
                x0,x1,y0,y1=scene.RACKS[n][:4]
                cx,cy=center
                result.append((cx-(x1-x0)/2,cx+(x1-x0)/2,cy-(y1-y0)/2,cy+(y1-y0)/2))
            else:result.extend(scene.rack_leg_rects(n,center))
        return result

    def planner(self,for_motion=False):
        grid=self.get_grid()
        if grid is None:return None
        return Planner(grid,self.half_size(),(scene.FIELD_X,scene.FIELD_Y),
                       self.obstacle_rects()+list(getattr(self,'observed_blockers',())),scene.COLLISION_MARGIN +
                       (0. if for_motion else scene.TRACKING_TOL))

    def republish_footprint(self):
        if self.fp_msg:
            for pub in self.fp_pubs:pub.publish(self.fp_msg)

    def set_footprint(self):
        hx,hy=scene.ROBOT_L/2,scene.ROBOT_W/2
        if self.current_payload:
            dx,dy=self.status.get('relative_xy',[0.,0.])
            x0,x1,y0,y1=scene.RACKS[self.current_payload][:4]
            hx=max(hx,abs(dx)+(x1-x0)/2);hy=max(hy,abs(dy)+(y1-y0)/2)
        self.fp_msg=Polygon(points=[Point32(x=float(x),y=float(y),z=0.)
                                  for x,y in ((hx,hy),(-hx,hy),(-hx,-hy),(hx,-hy))])
        req=SetParameters.Request(parameters=[Parameter(name='inflation_layer.inflation_radius',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=scene.inflation_radius(hx,hy)))])
        deadline=time.monotonic()+8.
        while time.monotonic()<deadline and rclpy.ok():
            self.republish_footprint()
            results=[self.request(cli,req,1.) for cli in self.setp_clis]
            if all(r is not None and all(v.successful for v in r.results) for r in results):
                self.get_logger().info(f'足迹确认: {2*hx:.3f} x {2*hy:.3f}')
                return True
            self.spin(.1)
        return False

    def pub_vel(self,vx=0.,vy=0.):
        # 单位 m/s，vx/vy 已是 base_link 速度；ESP32 不应再交换地图轴或增加180度。
        msg=Twist();msg.linear.x=float(vx);msg.linear.y=float(vy)
        self.cmd_pub.publish(msg)

    def abort(self,code,why):
        self.error=code;self.accepting=False
        self.pub_vel();self.set_state('ABORT')
        self.get_logger().error(f'{code}: {why}')
        return False

    def wait_ready(self,timeout=90.):
        deadline=time.monotonic()+timeout;last_report=-float('inf')
        while rclpy.ok() and time.monotonic()<deadline:
            self.spin(.05)
            fresh=self.fresh()
            if fresh and self.get_grid() is not None:
                self.get_logger().info('地图就绪: scan/odom/driver/costmap/TF 均有效')
                return True
            now=time.monotonic()
            if now-last_report>=5.:
                last_report=now
                self.startup_diagnostic=(f'odom_age={now-self.odom_seen:.3f}s '
                    f'scan_age={now-self.scan_seen:.3f}s status_age={now-self.status_seen:.3f}s '
                    f'driver_ready={self.status.get("ready",False)} '
                    f'driver_fault={self.status.get("fault")} '
                    f'driver_waiting={self.status.get("waiting_for",[])} '
                    f'costmap_service_ready={self.cm_cli.service_is_ready() if self.cm_cli is not None else "published snapshots"} '
                    f'costmap_pipeline={self.costmaps.diagnostic()} '
                    f'map_error={self.map_error if fresh else "sensor/driver not ready"}')
                self.get_logger().warn('STARTUP_WAIT: '+self.startup_diagnostic)
        return False

    def wait_next(self,name):
        # Drain messages queued before this target opened for release.
        for _ in range(3):self.spin(0.)
        self.next_flag=False;self.accepting=True
        self.set_state(f'GO_{name}')
        self.get_logger().info(f'等待开关: {name} (本目标只需一次)')
        deadline=time.monotonic()+600.
        while rclpy.ok() and time.monotonic()<deadline and not self.next_flag:self.spin(.05)
        self.accepting=False
        if not self.next_flag:return False
        self.next_flag=False;self.released=True
        return True

    def release_target(self,tgt):
        if tgt.get('auto_release',False):
            self.accepting=False;self.next_flag=False;self.released=True
            self.get_logger().info(f'卸货退架后自动前往扫码点: {tgt["name"]}')
            return True
        return self.wait_next(tgt['name'])

    def wait_still(self,goal,timeout=8.):
        self.pub_vel();deadline=time.monotonic()+timeout
        stable_since=None;last_seq=self.odom_seq
        while rclpy.ok() and time.monotonic()<deadline:
            self.spin(.02)
            if not self.fresh(radar=False):return False
            if self.odom_seq==last_seq:continue
            last_seq=self.odom_seq
            settled=(math.dist(self.pose[:2],goal)<=POSITION_TOL and self.speed<.01 and self.angular_speed<.01)
            if not settled:stable_since=None
            elif stable_since is None:stable_since=time.monotonic()
            elif time.monotonic()-stable_since>=STILL_SECONDS:return True
        if self.pose is not None:
            self.get_logger().warn(f'停稳超时: position_error={math.dist(self.pose[:2],goal):.4f}m '
                                   f'speed={self.speed:.4f}m/s angular={self.angular_speed:.4f}rad/s '
                                   f'pose={self.pose[:2]} goal={goal}')
        return False

    def remember_blockers(self,probe,start,end):
        mx,my=probe.hx+probe.margin,probe.hy+probe.margin
        box=(min(start[0],end[0])-mx,max(start[0],end[0])+mx,
             min(start[1],end[1])-my,max(start[1],end[1])+my)
        cells=probe.grid.blocking_world_rects(box)
        if not hasattr(self,'observed_blockers'):self.observed_blockers=set()
        self.observed_blockers.update(cells)
        self.get_logger().warn(f'保留本目标雷达阻塞格: new={len(cells)} '
                               f'total={len(self.observed_blockers)} cells={cells[:8]} '
                               f'map_from_odom={(probe.grid.tx,probe.grid.ty,math.atan2(probe.grid.s,probe.grid.c))}')

    def drive_seg(self,axis,target,blind=False):
        # 当前上位机跟踪器：输出车体 Twist。队友接管 TrackPath 时必须关闭本速度输出。
        distance=abs(target-self.pose[0 if axis=='x' else 1])
        sim_start=self.get_clock().now().nanoseconds*1e-9
        sim_limit=max(10.,distance/scene.CONTROL_SPEED+8.)
        wall_limit=time.monotonic()+max(30.,sim_limit*5)
        checked_at=0.;probe=None
        while rclpy.ok() and time.monotonic()<wall_limit:
            self.spin()
            if not self.fresh(radar=not blind):
                self.pub_vel();self.get_logger().warn(f'运动数据失效: axis={axis} target={target} {self.data_diagnostic()}')
                return False
            index=0 if axis=='x' else 1
            err=target-self.pose[index]
            if abs(err)<=scene.TRACKING_TOL:self.pub_vel();return True
            if not blind and time.monotonic()-checked_at>.25:
                # This only reads the asynchronous cache: no service wait in control.
                probe=self.planner(for_motion=True);checked_at=time.monotonic()
                end=list(self.pose[:2]);end[index]=target
                if probe is None:
                    self.pub_vel()
                    self.get_logger().warn(f'运动地图不可用: {self.data_diagnostic()}');return False
                reason=probe.segment_blocked_reason(self.pose[:2],end)
                if reason:
                    self.pub_vel()
                    if reason.startswith('radar costmap'):
                        self.remember_blockers(probe,self.pose[:2],end)
                    self.get_logger().warn(f'运动碰撞检查拒绝: {reason} start={self.pose[:2]} '
                                           f'end={end} half_size={(probe.hx,probe.hy)} margin={probe.margin}')
                    return False
            v=math.copysign(min(scene.CONTROL_SPEED,abs(err)*scene.CONTROL_GAIN),err)
            wx,wy=(v,0.) if axis=='x' else (0.,v)
            c,s=math.cos(self.pose[2]),math.sin(self.pose[2])
            self.pub_vel(wx*c+wy*s,-wx*s+wy*c)
            if self.get_clock().now().nanoseconds*1e-9-sim_start>sim_limit:break
        self.pub_vel()
        self.get_logger().warn(f'运动超时: axis={axis} target={target} pose={self.pose}')
        return False

    def data_diagnostic(self):
        now=time.monotonic()
        return (f'odom_age={now-self.odom_seen:.3f}s status_age={now-self.status_seen:.3f}s '
                f'scan_age={now-self.scan_seen:.3f}s ready={self.status.get("ready")} '
                f'fault={self.status.get("fault")} map={getattr(self,"map_error",None)}')

    def plan_for(self,tgt):
        gx,gy=self.goal
        if tgt['name'].startswith('under'):
            self.plan_origin=tuple(self.pose[:2])
            # Authorized rack entry: align with its opening, then enter without map.
            pts=[]
            entry_y=self.plan_origin[1]
            if abs(entry_y-gy)>.008:
                pts.append((self.plan_origin[0],gy));entry_y=gy
            # If alignment is already within tolerance, retain that actual lane.
            # Substituting exact gy here would create a diagonal planned edge.
            if abs(self.plan_origin[0]-gx)>.008:pts.append((gx,entry_y))
            return pts
        p=self.planner();self.n_plans+=1
        self.plan_origin=tuple(self.pose[:2])
        return None if p is None else p.plan(self.plan_origin,self.goal)

    def move_target(self,tgt):
        # Preserve actual collision observations also across framework retries.
        # A new target (after cargo changes) starts a new observation lifetime.
        goal_key=(tgt['name'],tuple(self.goal),self.current_payload if hasattr(self,'current_payload') else None)
        if getattr(self,'blocker_goal',None)!=goal_key:
            self.blocker_goal=goal_key;self.observed_blockers=set()
        for attempt in range(MAX_RETRY+1):
            self.retry=attempt
            pts=self.plan_for(tgt)
            if pts is None:
                self.get_logger().warn(f'规划失败 {attempt+1}/{MAX_RETRY+1}: {tgt["name"]}')
                if attempt<MAX_RETRY:self.spin(.2);continue
                return self.abort('E_PLAN','目标被占、地图不可用或无安全路径；没有移动目标')
            self.get_logger().info(f'轨迹 {pts}')
            origin=getattr(self,'plan_origin',tuple(self.pose[:2]))
            if not self.released and not self.release_target(tgt):return self.abort('E_TIMEOUT','等待触发超时')
            self.set_state(f'MOVING_{tgt["name"]}')
            failed=math.dist(self.pose[:2],origin)>POSITION_TOL
            if failed:self.get_logger().warn(f'放行后偏离规划起点: pose={self.pose[:2]} origin={origin}')
            last_axis=None;planned_start=origin
            for point in pts:
                if failed:break
                # Planned edges are axis aligned; never split a diagonal connector.
                dx,dy=point[0]-planned_start[0],point[1]-planned_start[1]
                if abs(dx)>1e-8 and abs(dy)>1e-8:
                    self.get_logger().error(f'规划段非轴对齐: start={planned_start} end={point}')
                    failed=True;break
                if abs(dx)+abs(dy)<1e-8:continue
                axis='x' if abs(dx)>=abs(dy) else 'y'
                cross=1 if axis=='x' else 0
                if abs(self.pose[cross]-planned_start[cross])>scene.TRACKING_TOL+1e-9:
                    self.get_logger().warn(f'偏离规划走廊: pose={self.pose[:2]} start={planned_start} axis={axis}')
                    failed=True;break
                if last_axis is not None and axis!=last_axis:self.n_turns+=1
                last_axis=axis
                if not self.drive_seg(axis,point[0 if axis=='x' else 1],blind=tgt['name'].startswith('under')):
                    failed=True;break
                self.n_segs+=1
                planned_start=point
            if not failed and self.wait_still(self.goal):return True
            self.pub_vel()
            self.get_logger().warn(f'运动/停稳失败，保留本目标放行并重规划 {attempt+1}/{MAX_RETRY+1}')
            if attempt<MAX_RETRY:
                # Consume fresh feedback while stopped before another map/plan request.
                self.wait_still(tuple(self.pose[:2]),timeout=2.)
        return self.abort('E_STUCK','重试上限已到')

    def select_payload(self,name,timeout=5.):
        value=f'rack_{name.lower()}' if name else ''
        deadline=time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<deadline:
            self.payload_pub.publish(String(data=value));self.spin(.05)
            if time.monotonic()-self.status_seen<DATA_TIMEOUT and self.status.get('selected')==value:return True
        return False

    def wait_lift(self,want,timeout=12.):
        value=scene.LIFT_UP if want else scene.LIFT_DOWN
        # Controller runs in simulation time; slow rendering must not consume its budget.
        sim_start=self.get_clock().now().nanoseconds*1e-9
        deadline=time.monotonic()+max(60.,timeout*5);last_report=0.
        while rclpy.ok() and time.monotonic()<deadline:
            if not self.fresh(radar=False):return False
            self.lift_pub.publish(Float64(data=value));self.spin(.05)
            if not self.fresh(radar=False):return False
            measured=self.status.get('lift');acknowledged=self.status.get('lift_target')
            joint_matches=(isinstance(measured,(int,float)) and isinstance(acknowledged,(int,float)) and
                           math.isfinite(measured) and math.isfinite(acknowledged) and
                           abs(acknowledged-value)<.0001 and abs(measured-value)<=scene.LIFT_TOL)
            released=want or not self.status.get('engaged',True)
            if (joint_matches and released and self.carry==want and self.status.get('lift_settled') and
                    self.status.get('carrying')==want):
                return True
            if time.monotonic()-last_report>2.:
                self.get_logger().info(f'等待升降: target={value:.3f} joint={self.status.get("lift")} '
                                       f'settled={self.status.get("lift_settled")} carrying={self.status.get("carrying")} '
                                       f'payload={self.status.get("payload_center")}')
                last_report=time.monotonic()
            if self.get_clock().now().nanoseconds*1e-9-sim_start>timeout:break
        self.get_logger().error(f'升降超时: target={value:.3f} status={self.status}')
        return False

    def exit_rack(self):
        target=self.pose[0]+scene.EXIT_DIST
        if target+scene.ROBOT_L/2>scene.FIELD_X:return False
        self.get_logger().info(f'许可盲走退架: x {self.pose[0]:.3f} -> {target:.3f}')
        return self.drive_seg('x',target,blind=True) and self.wait_still((target,self.pose[1]))

    def do_action(self,tgt):
        kind=tgt['name'].split('_')[0];name=tgt.get('rack')
        if kind=='scan':
            self.qr_expected=tgt['qr'];self.qr_hits=0
            deadline=time.monotonic()+12.
            while rclpy.ok() and time.monotonic()<deadline:
                self.spin(.05)
                if not self.fresh():break
                if self.qr_hits>=3 and time.monotonic()-self.qr_seen<.5:
                    self.qr_expected=None
                    self.get_logger().info(f'实际相机连续确认 QR: {tgt["qr"]}')
                    return True
            self.qr_expected=None;return self.abort('E_QR',f'相机没有连续识别指定码 {tgt["qr"]}')
        if kind=='under':
            if not self.select_payload(name) or not self.wait_lift(True):return self.abort('E_LIFT','举升/载荷确认失败')
            self.current_payload=name
            if not self.set_footprint():return self.abort('E_FOOTPRINT','载货足迹更新失败')
            # A carried rack moves with the robot; its opening imposes no exit leg.
            # The next drop target plans directly from this measured pickup pose.
        elif kind=='drop':
            if not self.wait_lift(False):return self.abort('E_LIFT','放下未完成，保留载货足迹')
            center=self.status.get('payload_center')
            if center is None or math.dist(center[:2],scene.DROP[name])>.025:return self.abort('E_DROP','货架实际落点偏差过大')
            self.placed[name]=tuple(center[:2]);self.current_payload=None
            if not self.select_payload(None) or not self.set_footprint():return self.abort('E_FOOTPRINT','空车足迹/载荷清除失败')
            if not self.exit_rack():return self.abort('E_STUCK','卸货退架失败')
        return True

    def target_goal(self,tgt):
        gx,gy=tgt['xy']
        if tgt['name'].startswith('drop'):
            dx,dy=self.status.get('relative_xy',[0.,0.])
            c,s=math.cos(self.pose[2]),math.sin(self.pose[2])
            gx-=c*dx-s*dy;gy-=s*dx+c*dy
        return gx,gy

    def run(self):
        if not self.wait_ready():return self.abort('E_READY','启动数据或地图超时，禁止继续')
        if not self.set_footprint():return self.abort('E_FOOTPRINT','初始足迹配置失败')
        for i,tgt in enumerate(self.targets):
            self.goal_i=i;self.released=False;self.retry=0
            self.goal=self.target_goal(tgt);self.set_state(f'PLAN_{tgt["name"]}')
            if not self.move_target(tgt):return False
            self.set_state(f'AT_{tgt["name"]}')
            if not self.do_action(tgt):return False
        self.pub_vel();self.set_state('DONE');return True


def main():
    n=None;ok=False;received_signal=None;previous={}
    lock_path=Path(__file__).resolve().parent.parent/'log/mission.lock'
    lock_path.parent.mkdir(parents=True,exist_ok=True)
    lock=lock_path.open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        print('mission_v3 already running',file=sys.stderr);return 3
    def stop(signum,_frame):
        nonlocal received_signal
        print(f'[EXIT] signal={signal.Signals(signum).name} pid={os.getpid()} time={time.time():.6f}',file=sys.stderr,flush=True)
        if received_signal is None:received_signal=signum;raise KeyboardInterrupt
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):previous[sig]=signal.signal(sig,stop)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        n=MissionV3();ok=n.run()
    except KeyboardInterrupt:pass
    except Exception:traceback.print_exc()
    finally:
        try:
            if n is not None:
                if n.context.ok():n.pub_vel()
                print(f'{"DONE" if ok else "FAILED"} state={n.state} error={n.error} target={n.goal_i+1}/13 plans={n.n_plans} segments={n.n_segs} turns={n.n_turns}',file=sys.stderr,flush=True)
                n.destroy_node()
        finally:
            try_shutdown()
            for sig,handler in previous.items():signal.signal(sig,handler)
            lock.close()
    return 0 if ok else 1


if __name__=='__main__':sys.exit(main())
