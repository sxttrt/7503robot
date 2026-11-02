#!/usr/bin/env python3
"""ROS gate for the Gazebo planar actuator; no pose RPCs or local physics state.

The simulation-step plugin owns constrained chassis execution and payload
coupling. This node requires fresh measured plugin feedback and radar validity
before forwarding body Twist. The plugin independently times out commands.
Private Gazebo measurements are never published as navigation odometry or TF.
"""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import traceback
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import try_shutdown
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import scene

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

class KinematicDriver(Node):
    def __init__(self,rate=30.):
        super().__init__('kinematic_driver')
        self.declare_parameter('allow_scan_rotation',False)
        self.allow_scan_rotation=self.get_parameter('allow_scan_rotation').value
        self.cmd=Twist();self.last_cmd=0.;self.fault=None
        self.actuator=None;self.actuator_seen=0.;self.actuator_stamp=-math.inf
        self.lidar_seen=0.;self.lidar_valid=False;self.lidar_status_seen=0.
        self.create_subscription(Twist,'cmd_vel',self.on_cmd,1)
        self.create_subscription(String,'/simulation/actuator/status',self.on_actuator,1)
        self.create_subscription(Odometry,'/odom',self.on_lidar_odom,10)
        self.create_subscription(String,'/lidar/localization_status',self.on_lidar_status,10)
        self.command_pub=self.create_publisher(Twist,'/simulation/chassis_cmd',1)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.carry_pub=self.create_publisher(Bool,'carry_state',qos)
        self.status_pub=self.create_publisher(String,'payload_status',qos)
        self.create_timer(1./rate,self.tick)
        self.get_logger().info('driver starting: constrained planar actuator; radar-gated Twist, no set_pose service')

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def on_cmd(self,msg):
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if not all(math.isfinite(v) for v in values):self.set_fault('nonfinite chassis command');return
        if any(abs(v)>.0001 for v in (msg.linear.z,msg.angular.x,msg.angular.y)):
            self.set_fault('nonplanar chassis command');return
        if math.hypot(msg.linear.x,msg.linear.y)>.31 or abs(msg.angular.z)>scene.TURN_COMMAND_LIMIT:
            self.set_fault('chassis speed limit exceeded');return
        if abs(msg.angular.z)>.0001:
            if (not self.allow_scan_rotation or (self.actuator or {}).get('engaged',True) or
                    abs(msg.linear.x)+abs(msg.linear.y)>.0001):
                self.set_fault('rotation command rejected');return
        self.last_cmd=time.monotonic();self.cmd=msg

    def on_actuator(self,msg):
        try:
            data=json.loads(msg.data)
            sec=data['stamp']['sec'];ns=data['stamp']['nanosec'];stamp=sec+ns*1e-9
            if type(sec) is not int or type(ns) is not int or not 0<=ns<10**9:return
            if stamp<=self.actuator_stamp or not -.05<=self.now()-stamp<.6:return
            if data.get('source')!='gazebo_planar_actuator':return
            for key in ('carrying','engaged','lift_settled','base_motion_stopped','lift_motion_stopped','ready'):
                if type(data[key]) is not bool:return
            for key,length in (('base_pose',4),('relative_xy',2),('payload_center',3)):
                if len(data[key])!=length or not all(type(v) in (int,float) and math.isfinite(v) for v in data[key]):return
            if not all(type(data[k]) in (int,float) and math.isfinite(data[k]) for k in ('lift','lift_target')):return
            if data['selected'] not in {f'rack_{n.lower()}' for n in scene.RACKS}|{''}:return
            self.actuator=data;self.actuator_seen=time.monotonic();self.actuator_stamp=stamp
            if data['fault']:self.set_fault(data['fault'])
        except (ValueError,KeyError,TypeError):return

    def on_lidar_odom(self,msg):
        if msg.header.frame_id!=scene.ODOM_FRAME or msg.child_frame_id!=scene.BASE_FRAME:return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if -.05<=self.now()-stamp<.5:self.lidar_seen=time.monotonic()

    def on_lidar_status(self,msg):
        try:
            data=json.loads(msg.data);s=data['stamp'];stamp=s['sec']+s['nanosec']*1e-9
            self.lidar_valid=(data.get('source')=='lidar_fixed_walls' and data.get('valid') is True
                              and -.05<=self.now()-stamp<.6)
            if self.lidar_valid:self.lidar_status_seen=time.monotonic()
        except (ValueError,KeyError,TypeError):self.lidar_valid=False

    def set_fault(self,reason):
        if self.fault is None:self.get_logger().error(reason)
        self.fault=self.fault or reason;self.cmd=Twist();self.command_pub.publish(Twist())

    def actuator_fresh(self):
        return (self.actuator is not None and time.monotonic()-self.actuator_seen<.6 and
                -.05<=self.now()-self.actuator_stamp<.6)

    def ready(self):
        now=time.monotonic()
        return (not self.fault and self.actuator_fresh() and self.actuator['ready'] and
                self.lidar_valid and now-self.lidar_seen<.6 and now-self.lidar_status_seen<.6)

    def publish_status(self):
        status=dict(self.actuator or {})
        status.update(actuator_feedback_fresh=bool(self.actuator_fresh()),ready=bool(self.ready()),fault=self.fault or status.get('fault') or None,
                      localization_source='lidar_fixed_walls',waiting_for=[])
        if not self.actuator_fresh():
            status.update(lift_settled=False,base_motion_stopped=False,lift_motion_stopped=False)
            status['waiting_for'].append('fresh measured planar actuator')
        if not self.lidar_valid:status['waiting_for'].append('valid radar localization')
        self.carry_pub.publish(Bool(data=bool(status.get('carrying',False))))
        self.status_pub.publish(String(data=json.dumps(status)))

    def tick(self):
        if time.monotonic()-self.last_cmd>.30 or not self.ready():self.cmd=Twist()
        self.command_pub.publish(self.cmd)
        self.publish_status()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--rate',type=float,default=30.)
    args,_=parser.parse_known_args();node=None;received=None;previous={}
    lock_path=Path(__file__).resolve().parent.parent/'log/driver.lock';lock_path.parent.mkdir(parents=True,exist_ok=True)
    lock=lock_path.open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:print('driver already running',file=sys.stderr);return 3
    def stop(signum,_frame):
        nonlocal received
        print(f'[EXIT] signal={signal.Signals(signum).name} pid={os.getpid()} time={time.time():.6f}',file=sys.stderr,flush=True)
        if received is None:received=signum;raise KeyboardInterrupt
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):previous[sig]=signal.signal(sig,stop)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=KinematicDriver(args.rate);rclpy.spin(node)
    except KeyboardInterrupt:pass
    except Exception:traceback.print_exc();return 1
    finally:
        try:
            if node is not None:
                node.command_pub.publish(Twist());node.destroy_node()
        finally:
            try_shutdown()
            for sig,handler in previous.items():signal.signal(sig,handler)
            lock.close()
    return 0

if __name__=='__main__':sys.exit(main())
