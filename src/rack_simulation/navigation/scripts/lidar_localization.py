#!/usr/bin/env python3
"""Sole navigation odom/TF publisher: scan matching against known fixed walls."""
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import try_shutdown
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from numpy.linalg import LinAlgError
from lidar_geometry import WallLocalizer,scan_points
import scene


class LidarLocalization(Node):
    def __init__(self):
        super().__init__('lidar_localization')
        self.matcher=WallLocalizer((scene.FIELD_X,scene.FIELD_Y),scene.model_start_pose())
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.status_pub=self.create_publisher(String,'/lidar/localization_status',10)
        self.tfb=TransformBroadcaster(self)
        self.last_valid=0.;self.last_received=0.;self.last_message_stamp=None
        self.last_report=0.;self.invalid_reason='waiting for scan';self.valid=False
        # 实机交接：这里接 LaserScan 而非 PointCloud2；真实 x/y/yaw 外参必须用于点变换。
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_timer(.1,self.health)
        self.get_logger().info('LIDAR_ONLY: /scan -> fixed-wall point matching -> /odom + odom->base_link; no truth/IMU/commands')

    def health(self):
        now=time.monotonic()
        valid=self.valid and now-self.last_valid<.6
        self.status_pub.publish(String(data=json.dumps(dict(source='lidar_fixed_walls',valid=valid,
            reason=(self.invalid_reason or 'scan localization timed out') if not valid else '',age=now-self.last_valid))))

    def on_scan(self,message):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        # Duplicate frames must neither refresh odom nor declare a healthy source.
        if self.last_message_stamp is not None and stamp<=self.last_message_stamp:return
        self.last_message_stamp=stamp
        self.last_received=time.monotonic()
        try:
            if message.header.frame_id!=scene.LASER_FRAME:raise ValueError('unexpected scan frame')
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            if not -.05<=age<.5:raise ValueError('stale scan timestamp')
            if self.matcher.last_stamp is not None and stamp-self.matcher.last_stamp>.6:
                self.matcher.recover_after_gap()
            points=scan_points(message.ranges,message.angle_min,message.angle_increment,
                               message.range_min,message.range_max)
            estimate=self.matcher.update(points,stamp)
            pose=estimate['pose'];velocity=estimate['velocity'];cov=estimate['covariance']
            odom=Odometry();odom.header.stamp=message.header.stamp
            odom.header.frame_id=scene.ODOM_FRAME;odom.child_frame_id=scene.BASE_FRAME
            odom.pose.pose.position.x=float(pose[0]);odom.pose.pose.position.y=float(pose[1])
            odom.pose.pose.orientation.z=math.sin(pose[2]/2);odom.pose.pose.orientation.w=math.cos(pose[2]/2)
            c,s=math.cos(pose[2]),math.sin(pose[2])
            odom.twist.twist.linear.x=float(c*velocity[0]+s*velocity[1])
            odom.twist.twist.linear.y=float(-s*velocity[0]+c*velocity[1])
            odom.twist.twist.angular.z=float(velocity[2])
            for i,ii in enumerate((0,1,5)):
                for j,jj in enumerate((0,1,5)):odom.pose.covariance[ii*6+jj]=float(cov[i,j])
            self.odom_pub.publish(odom)
            tf=TransformStamped();tf.header=odom.header;tf.child_frame_id=scene.BASE_FRAME
            tf.transform.translation.x=float(pose[0]);tf.transform.translation.y=float(pose[1])
            tf.transform.rotation=odom.pose.pose.orientation;self.tfb.sendTransform(tf)
            self.last_valid=time.monotonic();self.valid=True;self.invalid_reason=''
            if time.monotonic()-self.last_report>2.:
                self.last_report=time.monotonic()
                self.get_logger().info(f'雷达定位: xy=({pose[0]:.4f},{pose[1]:.4f}) yaw={pose[2]:.5f} '
                                       f'inliers={estimate["inliers"]} rms={estimate["rms"]:.4f}m')
        except (ValueError,ArithmeticError,LinAlgError) as error:
            self.valid=False;self.invalid_reason=str(error)
            if time.monotonic()-self.last_report>1.:
                self.last_report=time.monotonic();self.get_logger().warn(f'雷达定位失效，停止更新位姿: {error}')
        self.health()


def main():
    node=None
    def stop(_signal,_frame):raise KeyboardInterrupt
    previous={sig:signal.signal(sig,stop) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=LidarLocalization();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        try_shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':main()
