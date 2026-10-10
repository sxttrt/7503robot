"""底盘测试脚本：只测实机 /cmd_vel → 通信 → 底盘，绕过任务和路径跟踪。

与 path_tracker 使用相同 Twist、话题、QoS 和周期；始终只发 base_link +x。
本节点不做定位导航，不伪造健康状态或动作完成，不确认实物已停稳。
"""
import csv
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mission_manager.project_paths import project_root
from .core import load_settings, pose_values


class ForwardProfile:
    """Pure wall-time schedule; duration includes acceleration and deceleration."""
    def __init__(self, speed, duration, acceleration, wait_timeout, command_timeout):
        for name,value in [('speed',speed),('duration',duration),('acceleration',acceleration),
                           ('wait_timeout',wait_timeout),('command_timeout',command_timeout)]:
            if type(value) not in (float,int) or not math.isfinite(value) or value<=0:
                raise ValueError(name+' must be a positive finite number')
        if speed>.10:raise ValueError('slow chassis test speed must be <= 0.10 m/s')
        if duration>30:raise ValueError('chassis test duration must be <= 30 seconds')
        self.speed=speed;self.duration=duration;self.acceleration=acceleration
        self.wait_timeout=wait_timeout;self.command_timeout=command_timeout
        self.created=None;self.last=None;self.started=None;self.stopping=None
        self.phase='WAITING';self.reason='';self.failed=False;self.done=False

    def stop(self, now, reason, failed=False):
        if self.stopping is None:self.stopping=now
        if reason and (not self.reason or failed):self.reason=reason
        self.failed=self.failed or failed;self.phase='STOPPING'

    def step(self, now, subscribers, publishers, interrupted=False):
        if self.created is None:self.created=now
        if self.last is not None and now-self.last>self.command_timeout:
            self.stop(now,'control loop stalled',True)
        self.last=now
        if interrupted:self.stop(now,'operator interrupted')
        if publishers>1:self.stop(now,'another publisher owns the velocity topic',True)
        if self.started is not None and publishers<1:
            self.stop(now,'velocity graph ownership unavailable',True)
        if self.started is not None and subscribers==0:
            self.stop(now,'velocity subscriber disconnected',True)
        if self.stopping is not None:
            if now-self.stopping>=.5:self.done=True;self.phase='DONE'
            return 0.
        if self.started is None:
            if now-self.created>=self.wait_timeout:
                self.stop(now,'no velocity subscriber before wait deadline',True)
                return 0.
            # Allow ROS graph discovery before the first nonzero command.
            if subscribers<1 or publishers!=1 or now-self.created<.5:return 0.
            self.started=now
        elapsed=now-self.started
        if elapsed>=self.duration:
            self.stop(now,'command schedule completed; physical stop not certified')
            return 0.
        speed=min(self.speed,self.acceleration*elapsed,self.acceleration*(self.duration-elapsed))
        self.phase=('ACCELERATING' if self.acceleration*elapsed<self.speed else
                    'DECELERATING' if self.acceleration*(self.duration-elapsed)<self.speed else 'CRUISING')
        return max(0.,speed)


def forward_command(speed):
    """Same base_link Twist contract as path_tracker; never a map-axis command."""
    if not math.isfinite(speed) or not 0.<=speed<=.10:
        raise ValueError('test command must be finite and within 0..0.10 m/s')
    msg=Twist();msg.linear.x=float(speed)
    return msg  # all five other components remain exactly zero


class ChassisTest(Node):
    def __init__(self):
        super().__init__('chassis_test')  # ROS graph names use ASCII; entry is 底盘测试脚本.sh.
        self.declare_parameter('config_file',str(project_root()/'src/robot_bringup/config/navigation_robot.yaml'))
        self.declare_parameter('speed',.05)
        self.declare_parameter('duration_sec',5.)
        self.declare_parameter('wait_timeout_sec',5.)
        filename=project_root()/'runtime_logs/chassis_test'/('test_'+str(time.time_ns())+'.csv')
        self.declare_parameter('csv_file',str(filename))
        if self.get_parameter('use_sim_time').value:
            raise ValueError('chassis_test uses real wall time; use_sim_time must be false')
        self.cfg=load_settings(self.get_parameter('config_file').value)
        self.profile=ForwardProfile(self.get_parameter('speed').value,
            self.get_parameter('duration_sec').value,self.cfg['tracking_linear_acceleration'],
            self.get_parameter('wait_timeout_sec').value,self.cfg['tracking_command_timeout_sec'])
        if self.profile.speed>self.cfg['max_linear_speed']:
            raise ValueError('test speed exceeds configured max_linear_speed')
        # Exact publication contract of the real path_tracker, including depth=1.
        self.command=self.create_publisher(Twist,self.cfg['cmd_vel_topic'],1)
        self.topic=self.command.topic_name  # use resolved name, also for graph checks
        self.odom=None;self.odom_received=0.;self.odom_stamp=-math.inf
        self.create_subscription(Odometry,self.cfg['odom_topic'],self.on_odom,10)
        self.origin=time.monotonic();self.last_report=0.;self.interrupted=False
        filename=Path(self.get_parameter('csv_file').value).expanduser()
        filename.parent.mkdir(parents=True,exist_ok=True)
        self.stream=filename.open('w',encoding='utf-8',newline='')
        self.writer=csv.writer(self.stream)
        self.writer.writerow(('wall_elapsed_sec','ros_stamp_sec','phase','cmd_vx_mps','cmd_vy_mps',
            'cmd_wz_radps','subscription_count','odom_fresh','odom_stamp_sec','odom_receive_age_sec',
            'odom_x_m','odom_y_m','odom_yaw_rad','odom_vx_mps','odom_vy_mps','odom_wz_radps','reason'))
        self.stream.flush()
        self.create_timer(1./self.cfg['tracking_frequency'],self.tick)
        self.get_logger().info('底盘测试脚本: topic='+self.topic+' Twist base_link +x; speed='+
            str(self.profile.speed)+' m/s, duration='+str(self.profile.duration)+' s, frequency='+
            str(self.cfg['tracking_frequency'])+' Hz; CSV='+str(filename))
        self.get_logger().info('Only start the base receiver/servo, optionally lidar localization; '+
            'no mission, path_tracker, Dock or another cmd_vel publisher. Odom is optional observation.')

    def on_odom(self,msg):
        try:
            if msg.header.frame_id!=self.cfg['odom_frame'] or msg.child_frame_id!=self.cfg['base_frame']:return
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            now=self.get_clock().now().nanoseconds*1e-9
            if stamp<=self.odom_stamp or not -.05<=now-stamp<self.cfg['feedback_timeout_sec']:return
            pose=pose_values(msg.pose.pose);v=msg.twist.twist
            velocity=(v.linear.x,v.linear.y,v.angular.z)
            if not all(math.isfinite(x) for x in velocity):return
            self.odom=(pose,velocity);self.odom_stamp=stamp;self.odom_received=time.monotonic()
        except ValueError:return

    def publish(self,speed,now,subscribers):
        self.command.publish(forward_command(speed))
        ros_now=self.get_clock().now().nanoseconds*1e-9
        fresh=(self.odom is not None and now-self.odom_received<self.cfg['feedback_timeout_sec'] and
               -.05<=ros_now-self.odom_stamp<self.cfg['feedback_timeout_sec'])
        values=(*self.odom[0],*self.odom[1]) if fresh else ('',)*6
        self.writer.writerow((now-self.origin,ros_now,self.profile.phase,speed,0.,0.,subscribers,
            fresh,self.odom_stamp if self.odom else '',now-self.odom_received if self.odom else '',
            *values,self.profile.reason))
        self.stream.flush()  # every published test command is recorded, not 10Hz sampling

    def tick(self):
        now=time.monotonic();subscribers=self.command.get_subscription_count()
        speed=self.profile.step(now,subscribers,self.count_publishers(self.topic),self.interrupted)
        self.publish(speed,now,subscribers)
        if now-self.last_report>=1.:
            self.last_report=now
            self.get_logger().info(self.profile.phase+' vx='+format(speed,'.3f')+' m/s; '+self.profile.reason)

    def close(self):
        # Signals are handled without shutting down the rcl context first.
        # Best-effort zero burst; it never claims hardware has stopped.
        self.profile.stop(time.monotonic(),'test node exiting')
        try:
            if rclpy.ok():
                for _ in range(10):
                    try:self.publish(0.,time.monotonic(),self.command.get_subscription_count())
                    except OSError as exc:
                        # publish() sends zero before writing CSV; a disk failure
                        # must not prevent the remaining best-effort stop burst.
                        self.get_logger().error('CSV write failed during zero burst: '+str(exc))
                    time.sleep(1./self.cfg['tracking_frequency'])
        finally:self.stream.flush();self.stream.close()


def main(args=None):
    node=None;exit_code=0;received_signal=[]
    def interrupt(sig,_frame):
        received_signal.append(sig)
        if node:node.interrupted=True
    previous={sig:signal.signal(sig,interrupt) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
        node=ChassisTest()
        if received_signal:node.interrupted=True
        while rclpy.ok() and not node.profile.done:rclpy.spin_once(node,timeout_sec=.02)
        if node.profile.failed:exit_code=2
        elif received_signal:exit_code=128+received_signal[-1]
        node.get_logger().info('Test command publication ended; verify actual stop on hardware. '+node.profile.reason)
    except Exception as exc:
        print('底盘测试脚本失败: '+str(exc),file=sys.stderr);exit_code=2
    finally:
        try:
            if node:
                try:node.close()
                finally:node.destroy_node()
        finally:
            rclpy.try_shutdown()
            for sig,handler in previous.items():signal.signal(sig,handler)
    return exit_code


if __name__=='__main__':raise SystemExit(main())
