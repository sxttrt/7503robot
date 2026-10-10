"""联合验证专用：模拟平台/健康/整机停止，导航和 Dock 均由实机包执行。

禁止注册 NavigateToPose，禁止设置或重复发布 costmap 足迹。
Gazebo 状态只用于模拟机构完成／健康；定位来源始终是雷达。
"""
import json
import math
import signal
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String,Bool,Float64
from std_srvs.srv import Trigger
from nav2_msgs.msg import Costmap
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from .framework_execution import FrameworkExecution
from published_costmap_cache import PublishedCostmapCache
from mission_v3 import MissionV3
import scene

class HybridExecution(FrameworkExecution):
    def __init__(self):
        self.nav_status=None;self.nav_seen=0.;self.nav_stamp=-math.inf
        super().__init__()
        if self.get_parameter('provide_navigation').value:
            raise ValueError('hybrid requires provide_navigation=false')
        if self.get_parameter('provide_docking').value:
            raise ValueError('hybrid requires provide_docking=false')
        # 父类为普通历史仿真创建了速度出口；联合验证只能由真实跟踪器发布速度。
        self.destroy_publisher(self.cmd_pub);self.cmd_pub=None
        prefix=self.get_parameter('interface_prefix').value.rstrip('/')
        self.create_subscription(String,prefix+'/navigation/status',self.on_navigation,10)
        self.nav_stop=self.create_client(Trigger,prefix+'/navigation/stop')

    def pub_vel(self,vx=0.,vy=0.):
        # 父类的停稳/升降代码会请求零速度；导航交接已确认停车，无需第二发布者。
        # 整机停止必须经 navigation/stop 的取消与实测停车流程。
        if vx or vy:raise ValueError('hybrid actuator bridge cannot command chassis motion')

    def prepare_lift_up(self):
        # 选中 Gazebo 模型属于模拟升降执行器，不产生任何进退架运动。
        # Dock action 成功后，LIFT_UP 状态与 service 请求可能乱序到达。
        limit=time.monotonic()+5.
        while rclpy.ok() and time.monotonic()<limit:
            if self.abort_event.is_set():raise ValueError('lift preparation cancelled')
            context=self.mission_context if time.monotonic()-self.mission_seen<.6 else None
            if context and context.get('state')=='LIFT_UP' and context.get('rack') in scene.RACKS:
                name=context['rack']
                if self.current_payload:raise ValueError('cannot lift another rack while carrying')
                if (self.pose is None or time.monotonic()-self.odom_seen>=.6 or
                        math.dist(self.pose[:2],scene.rack_model_center(name))>.02 or
                        abs(math.atan2(math.sin(self.pose[2]-scene.MODEL_YAW),
                                       math.cos(self.pose[2]-scene.MODEL_YAW)))>.04):
                    raise ValueError('radar does not confirm robot under requested rack')
                if not self.wait_still(scene.rack_model_center(name)):
                    raise ValueError('base stop before simulated lift not confirmed')
                if not self.select_payload(name):raise ValueError('simulated payload selection not acknowledged')
                self.entered_rack=name
                return name
            time.sleep(.02)
        raise ValueError('fresh LIFT_UP mission context missing')

    def make_costmap_reader(self):
        cache=PublishedCostmapCache(expected_frame='map')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Costmap,'/global_costmap/costmap_raw',
            lambda msg:cache.accept(msg,self.get_clock().now().nanoseconds*1e-9),qos)
        return cache

    def on_navigation(self,msg):
        try:
            data=json.loads(msg.data);stamp=data['stamp']['sec']+data['stamp']['nanosec']*1e-9
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            if stamp<=self.nav_stamp or not -.05<=age<.6:return
            self.nav_status=data;self.nav_seen=time.monotonic();self.nav_stamp=stamp
        except (ValueError,KeyError,TypeError):return

    def set_footprint(self):
        # 实机 navigation_server 按实测状态更新足迹；这里不成为第二个发布者。
        return True

    def health(self):
        base=MissionV3.fresh(self,radar=False)
        ready=base and self.initialized
        navigation=(ready and self.nav_status is not None and time.monotonic()-self.nav_seen<.6
                    and self.nav_status.get('healthy',self.nav_status.get('ready')) is True)
        stamp=self.get_clock().now().to_msg()
        stopped=(not self.busy and self.pose is not None and time.monotonic()-self.odom_seen<.6
                 and time.monotonic()-self.status_seen<.6 and self.speed<.01 and self.angular_speed<.01
                 and self.status.get('actuator_feedback_fresh') is True
                 and self.status.get('base_motion_stopped') is True
                 and self.status.get('lift_motion_stopped') is True)
        measured_up=bool(self.status.get('lift_settled') and self.status.get('carrying'))
        fault=self.status.get('fault') or (self.nav_status or {}).get('fault') or ''
        feedback=String(data=json.dumps({'schema_version':1,
            'stamp':{'sec':stamp.sec,'nanosec':stamp.nanosec},
            'modules':{'navigation':bool(navigation),'qr':time.monotonic()-self.camera_seen<2.,
                'docking':bool(navigation),'lift':bool(base),'base':bool(base)},
            'stopped':bool(stopped),'lift_is_up':measured_up if self.status.get('lift_settled') else None,
            'lift_settled':bool(self.status.get('lift_settled',False)),
            'base_motion_stopped':bool(self.status.get('actuator_feedback_fresh') and self.status.get('base_motion_stopped')),
            'lift_motion_stopped':bool(self.status.get('actuator_feedback_fresh') and self.status.get('lift_motion_stopped')),
            'lift_state_source':'measured','fault':fault,
            'navigation_ready':bool((self.nav_status or {}).get('ready',False)),
            'navigation_diagnostic':(self.nav_status or {}).get('reason','waiting for real backend')}))
        self.base_feedback_pub.publish(feedback)
        # 主框架只使用 ready/fault；详细实测状态属于导航内部反馈。
        details=json.loads(feedback.data)
        ready=bool(navigation and base and self.initialized and not details['fault'])
        self.health_pub.publish(String(data=json.dumps({'ready':ready,'fault':details['fault']})))
        self.up_pub.publish(Bool(data=measured_up))

    def stop_motion(self,request,response):
        # 先停止实机导航/Dock/跟踪，再确认模拟平台停止。
        self.abort_event.set()
        # 先撤销自己的升降循环并保持实测平台，避免停止导航时仍等待机构动作。
        self.lift_pub.publish(Float64(data=float(self.status.get('lift',scene.LIFT_DOWN))))
        if not self.nav_stop.service_is_ready():
            response.success=False;response.message='real navigation stop unavailable';return response
        try:
            future=self.nav_stop.call_async(Trigger.Request());deadline=time.monotonic()+6.
            while rclpy.ok() and not future.done() and time.monotonic()<deadline:time.sleep(.02)
            if not future.done() or not future.result().success:
                response.success=False;response.message='real navigation stop unconfirmed';return response
        except Exception as exc:
            response.success=False;response.message=str(exc);return response
        return super().stop_motion(request,response)

    def warmup(self):
        super().warmup()
        if self.initialized:self.get_logger().info('HYBRID_ADAPTER_READY: real backend owns navigation/Dock; simulated lift ready')

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=HybridExecution();executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node)
    closing=threading.Event()
    def stop(_signal,_frame):node.abort_event.set();closing.set();node.pub_vel()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    worker=threading.Thread(target=node.warmup,daemon=True);worker.start()
    try:
        while rclpy.ok() and not closing.is_set():executor.spin_once(timeout_sec=.05)
        until=time.monotonic()+5.
        while node.busy and time.monotonic()<until:executor.spin_once(timeout_sec=.02)
    finally:
        node.pub_vel();executor.shutdown(timeout_sec=2.);node.destroy_node();rclpy.try_shutdown()
