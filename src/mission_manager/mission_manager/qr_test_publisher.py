#!/usr/bin/env python3
"""User-run test publisher using the framework QR contract; not a recognizer."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser(description='Publish one confirmed QR result in the active QR wait window')
    parser.add_argument('code',help='START, END or the full expected RACKx_XXXX code')
    parser.add_argument('--topic',default='/team2/sim/qr/detections')
    parser.add_argument('--real-time',action='store_true',help='Use wall ROS clock for hardware; simulation clock is default')
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args)
    node=Node('qr_result_test_publisher',parameter_overrides=[Parameter('use_sim_time',value=not args.real_time)])
    publisher=node.create_publisher(String,args.topic,10)
    try:
        deadline=time.monotonic()+10.
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if publisher.get_subscription_count()>0 and node.get_clock().now().nanoseconds>0:break
        else:raise RuntimeError('No QR subscriber or clock; check startup and ROS_DOMAIN_ID/RMW settings')
        # Allow discovery to settle before constructing a fresh recognition stamp.
        until=time.monotonic()+.25
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        stamp=node.get_clock().now().to_msg()
        data={'image_stamp':{'sec':stamp.sec,'nanosec':stamp.nanosec},
              'status':'ok','detections':[{'raw':args.code}]}
        publisher.publish(String(data=json.dumps(data)))
        print('Published QR test result:',data)
        until=time.monotonic()+.2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
