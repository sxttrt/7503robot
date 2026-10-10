"""实机 LaserScan 定位入口；绝不读取 Gazebo、IMU、轮速或命令作为定位值。"""
import copy
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from numpy.linalg import LinAlgError
from .core import load_settings, WallLocalizer, scan_points, laser_to_base

class Localization(Node):
    def __init__(self):
        super().__init__('lidar_localization')
        self.declare_parameter('config_file','')
        self.cfg=cfg=load_settings(self.get_parameter('config_file').value)
        self.matcher=WallLocalizer(cfg['field_size'],cfg['initial_pose'])
        self.last_stamp=None;self.last_valid=0.;self.valid=False
        self.reason='waiting for scan' if cfg['calibration_verified'] else 'calibration not verified'
        self.publisher=self.create_publisher(Odometry,cfg['odom_topic'],10)
        self.status=self.create_publisher(String,cfg['localization_status_topic'],10)
        self.dynamic_tf=TransformBroadcaster(self)
        self.static_tf=StaticTransformBroadcaster(self)
        if cfg['calibration_verified']:
            identity=TransformStamped();identity.header.frame_id=cfg['frame_id']
            identity.child_frame_id=cfg['odom_frame'];identity.transform.rotation.w=1.
            laser=TransformStamped();laser.header.frame_id=cfg['base_frame'];laser.child_frame_id=cfg['laser_frame']
            x,y,z,yaw=cfg['laser_pose']
            laser.transform.translation.x=x;laser.transform.translation.y=y;laser.transform.translation.z=z
            laser.transform.rotation.z=math.sin(yaw/2);laser.transform.rotation.w=math.cos(yaw/2)
            self.static_tf.sendTransform([identity,laser])
        self.create_subscription(LaserScan,cfg['scan_topic'],self.on_scan,qos_profile_sensor_data)
        self.create_timer(.1,self.health)
        self.get_logger().info('实机雷达定位：四壁匹配；外参用于点变换；未确认标定时只等待')

    def health(self):
        valid=self.valid and time.monotonic()-self.last_valid<self.cfg['feedback_timeout_sec']
        stamp=self.get_clock().now().to_msg()
        self.status.publish(String(data=json.dumps({'valid':valid,'source':'lidar_fixed_walls',
            'stamp':{'sec':stamp.sec,'nanosec':stamp.nanosec},
            'scan_stamp':self.last_stamp,
            'reason':'' if valid else self.reason or 'scan timeout'})))

    def on_scan(self,msg):
        if not self.cfg['calibration_verified']:return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.last_stamp is not None and stamp<=self.last_stamp:return
        try:
            if msg.header.frame_id!=self.cfg['laser_frame']:raise ValueError('unexpected scan frame')
            if not -.05<=self.get_clock().now().nanoseconds*1e-9-stamp<.5:raise ValueError('stale scan')
            if self.matcher.last_stamp is not None and stamp-self.matcher.last_stamp>.6:self.matcher.recover_after_gap()
            pts=scan_points(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max)
            est=self.matcher.update(laser_to_base(pts,self.cfg['laser_pose']),stamp)
            # Commit only a validated, successfully matched scan. A rejected future
            # packet or wrong frame must never prevent later good scans recovering.
            self.last_stamp=stamp
            x,y,yaw=est['pose'];vx,vy,wz=est['velocity']
            odom=Odometry();odom.header=copy.deepcopy(msg.header);odom.header.frame_id=self.cfg['odom_frame']
            odom.child_frame_id=self.cfg['base_frame']
            odom.pose.pose.position.x=float(x);odom.pose.pose.position.y=float(y)
            odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
            odom.twist.twist.linear.x=float(math.cos(yaw)*vx+math.sin(yaw)*vy)
            odom.twist.twist.linear.y=float(-math.sin(yaw)*vx+math.cos(yaw)*vy)
            odom.twist.twist.angular.z=float(wz)
            for i,ii in enumerate((0,1,5)):
                for j,jj in enumerate((0,1,5)):odom.pose.covariance[ii*6+jj]=float(est['covariance'][i,j])
            self.publisher.publish(odom)
            tf=TransformStamped();tf.header=odom.header;tf.child_frame_id=self.cfg['base_frame']
            tf.transform.translation.x=float(x);tf.transform.translation.y=float(y)
            tf.transform.rotation=odom.pose.pose.orientation;self.dynamic_tf.sendTransform(tf)
            self.valid=True;self.reason='';self.last_valid=time.monotonic()
        except (ValueError,ArithmeticError,LinAlgError) as exc:
            self.valid=False;self.reason=str(exc)

def main(args=None):
    rclpy.init(args=args);node=None
    try:node=Localization();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        rclpy.try_shutdown()
