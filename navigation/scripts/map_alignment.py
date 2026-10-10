#!/usr/bin/env python3
"""Anchor public map coordinates to the configured field at stationary startup.

Navigation TF: map -> odom -> base_link -> laser.
The fixed-wall localizer already estimates absolute field coordinates, so map
and odom share that gauge. SLAM mapping publishes no navigation transform.
No simulator truth or QR inputs.
"""
import math
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import try_shutdown
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import scene


class MapAlignment(Node):
    def __init__(self):
        super().__init__('map_alignment')
        self.broadcaster=StaticTransformBroadcaster(self)
        self.odom=None;self.odom_seen=0.;self.calibrated=False
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.create_timer(.05,self.align)

    def on_odom(self,message):
        if message.header.frame_id==scene.ODOM_FRAME:
            self.odom=message;self.odom_seen=time.monotonic()

    def align(self):
        if self.calibrated or self.odom is None or time.monotonic()-self.odom_seen>.6:return
        velocity=self.odom.twist.twist
        if math.hypot(velocity.linear.x,velocity.linear.y)>.01 or abs(velocity.angular.z)>.01:return
        try:
            stamp=self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1e-9
            if not -.05<=self.get_clock().now().nanoseconds*1e-9-stamp<.5:return
            position=self.odom.pose.pose.position
            if not all(math.isfinite(value) for value in (position.x,position.y)):return
            x=y=angle=0.
            tf=TransformStamped();tf.header.stamp=self.get_clock().now().to_msg()
            tf.header.frame_id=scene.MAP_FRAME;tf.child_frame_id=scene.ODOM_FRAME
            tf.transform.translation.x=x;tf.transform.translation.y=y
            tf.transform.rotation.z=math.sin(angle/2);tf.transform.rotation.w=math.cos(angle/2)
            self.broadcaster.sendTransform(tf);self.calibrated=True
            self.get_logger().info(f'MAP_ALIGNED: radar field coordinates; static map->odom={(x,y,angle)}')
        except Exception:return


def main():
    node=None
    def stop(_signal,_frame):raise KeyboardInterrupt
    previous={sig:signal.signal(sig,stop) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=MapAlignment();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        try_shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':main()
