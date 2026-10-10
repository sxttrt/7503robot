"""联合验证评估节点。Gazebo 真值仅在这里与估计比较，绝不发布 odom/TF/cmd。"""
import csv
import json
import math
import time
from collections import deque
from pathlib import Path as FilePath
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray,Twist
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import String
from .core import pose_values,route_distance
from .evaluation_core import TruthSeries,ErrorStats

class HybridEvaluator(Node):
    def __init__(self):
        super().__init__('hybrid_evaluator');self.declare_parameter('csv_file','hybrid_metrics.csv')
        filename=FilePath(self.get_parameter('csv_file').value);filename.parent.mkdir(parents=True,exist_ok=True)
        self.stream=filename.open('w',encoding='utf-8',newline='')
        columns=('stamp','estimate_x','estimate_y','estimate_yaw','truth_x','truth_y','truth_yaw',
            'position_error_m','yaw_error_rad','cross_track_error_m','cmd_vx','cmd_vy','cmd_wz',
            'tracking_active','state','rack','tracking_phase','odom_receive_wall_sec',
            'odom_receive_interval_wall_sec','odom_source_interval_sec','odom_receive_rtf')
        self.writer=csv.writer(self.stream);self.writer.writerow(columns);self.stream.flush()
        self.truth=TruthSeries();self.pending=deque(maxlen=200);self.stats=ErrorStats()
        self.path=[];self.tracking=False;self.tracking_seen=0.;self.command=(0.,0.,0.)
        self.state='';self.rack='';self.last_pair=0.;self.last_odom_stamp=-math.inf;self.skipped=0
        self.latest={};self.filename=str(filename)
        self.tracking_phase='IDLE';self.last_odom_received=None;self.receive_origin=time.monotonic()
        self.publisher=self.create_publisher(String,'evaluation/metrics',10)
        self.create_subscription(PoseArray,'/simulation/robot_pose',self.on_truth,10)
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.create_subscription(Path,'navigation/path',self.on_path,10)
        self.create_subscription(Twist,'/cmd_vel',self.on_command,10)
        self.create_subscription(String,'base/tracking_status',self.on_tracking,10)
        self.create_subscription(String,'mission/status',self.on_mission,10)
        self.create_timer(.5,self.report)
        self.get_logger().info('EVALUATION_ONLY: Gazebo truth vs lidar; CSV='+self.filename)

    def on_truth(self,msg):
        if len(msg.poses)!=1:return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        try:
            if self.truth.add(stamp,pose_values(msg.poses[0])):self.compare()
        except ValueError:return

    def on_odom(self,msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp<=self.last_odom_stamp:return
        try:
            estimate=pose_values(msg.pose.pose);received=time.monotonic()
            interval=None if self.last_odom_received is None else received-self.last_odom_received
            source_interval=None if not math.isfinite(self.last_odom_stamp) else stamp-self.last_odom_stamp
            # Receipt timing reveals slow simulation / bursts, not GUI render rate.
            timing=(received-self.receive_origin,interval,source_interval,
                    source_interval/interval if interval and source_interval is not None else None)
            self.last_odom_received=received;self.last_odom_stamp=stamp
            self.pending.append((stamp,estimate,timing));self.compare()
        except ValueError:return

    def on_path(self,msg):
        if msg.header.frame_id!='map':return
        try:self.path=[pose_values(p.pose)[:2] for p in msg.poses]
        except ValueError:self.path=[]

    def on_command(self,msg):
        value=(msg.linear.x,msg.linear.y,msg.angular.z)
        if all(math.isfinite(v) for v in value):self.command=value

    def on_tracking(self,msg):
        try:
            data=json.loads(msg.data);self.tracking=data.get('active') is True;self.tracking_seen=time.monotonic()
            self.tracking_phase=data.get('phase','UNKNOWN')
        except (ValueError,AttributeError):return

    def on_mission(self,msg):
        try:
            data=json.loads(msg.data);self.state=data.get('state','');self.rack=data.get('rack','')
        except (ValueError,AttributeError):return

    def compare(self):
        while self.pending and self.truth.samples:
            stamp,estimate,timing=self.pending[0];truth=self.truth.at(stamp)
            if truth is None:
                if self.truth.samples[-1][0]<=stamp:break
                self.pending.popleft();self.skipped+=1;continue
            self.pending.popleft();xy,angle=self.stats.add(estimate,truth)
            active=self.tracking and time.monotonic()-self.tracking_seen<.6
            cross=route_distance(truth,self.path) if active and self.path else None
            self.latest={'stamp_sec':stamp,'position_error_m':xy,'yaw_error_rad':angle,
                'cross_track_error_m':cross,'tracking_active':active,
                'cmd_vx':self.command[0],'cmd_vy':self.command[1],'cmd_wz':self.command[2],
                'state':self.state,'rack':self.rack,'tracking_phase':self.tracking_phase,
                'odom_receive_wall_sec':timing[0],'odom_receive_interval_wall_sec':timing[1],
                'odom_source_interval_sec':timing[2],'odom_receive_rtf':timing[3]}
            self.writer.writerow((stamp,*estimate,*truth,xy,angle,'' if cross is None else cross,
                                  *self.command,active,self.state,self.rack,self.tracking_phase,
                                  *(value if value is not None else '' for value in timing)))
            self.stream.flush();self.last_pair=time.monotonic()

    def report(self):
        result={**self.stats.report(),**self.latest,'valid':time.monotonic()-self.last_pair<.6,
                'unmatched_samples':self.skipped,'pending_samples':len(self.pending),'csv_file':self.filename,
                'truth_source':'Gazebo evaluation only; no control input'}
        self.publisher.publish(String(data=json.dumps(result)))

    def close(self):self.stream.flush();self.stream.close()

def main(args=None):
    rclpy.init(args=args);node=None
    try:node=HybridEvaluator();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.close();node.destroy_node()
        rclpy.try_shutdown()
