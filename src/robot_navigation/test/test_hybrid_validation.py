"""联合入口与评估的离线测试；不启动任何 ROS/Gazebo 节点。"""
import ast
import importlib.util
import math
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import patch
import pytest
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mission_manager.project_paths import project_root
from mission_manager.hybrid_execution import HybridExecution
from mission_manager.framework_execution import FrameworkExecution
from robot_navigation.core import load_settings
from robot_navigation.evaluation_core import TruthSeries,ErrorStats

ROOT=project_root()

def test_truth_interpolation_and_yaw_wrap():
    data=TruthSeries();assert data.add(1.,(1.,1.,math.pi-.02))
    assert data.add(1.1,(1.2,1.,-math.pi+.02))
    # Floating representation of .1 is conservatively allowed by tolerance in the implementation.
    value=data.at(1.05)
    assert value is not None and value[0]==pytest.approx(1.1)
    assert abs(abs(value[2])-math.pi)<1e-9

def test_no_truth_extrapolation_or_stale_interpolation():
    data=TruthSeries();data.add(1.,(0,0,0));data.add(2.,(1,0,0))
    assert data.at(.9) is None and data.at(2.1) is None and data.at(1.5) is None
    assert not data.add(1.8,(0,0,0)) and not data.add(0.,(0,0,0))

def test_error_statistics_use_wrapped_yaw():
    stats=ErrorStats();xy,angle=stats.add((1.03,1.04,math.pi-.01),(1.,1.,-math.pi+.01))
    assert xy==pytest.approx(.05) and angle==pytest.approx(.02)
    assert stats.report()['position_rmse_m']==pytest.approx(.05)

def test_hybrid_geometry_does_not_change_real_calibration():
    cfg=load_settings(ROOT/'src/robot_bringup/config/navigation_hybrid.yaml')
    real=load_settings(ROOT/'src/robot_bringup/config/navigation_robot.yaml')
    assert cfg['calibration_verified'] is True and real['calibration_verified'] is False
    assert cfg['localization_status_topic']=='/lidar/localization_status'
    assert cfg['costmap_service']=='/global_costmap/get_costmap'

def test_hybrid_uses_real_costmap_unknown_policy():
    import yaml
    folder=ROOT/'src/robot_bringup/config'
    real=yaml.safe_load((folder/'costmap_robot.yaml').read_text())
    hybrid=yaml.safe_load((folder/'costmap_hybrid.yaml').read_text())
    params=hybrid['global_costmap']['global_costmap']['ros__parameters']
    assert params['use_sim_time'] is True
    assert params['track_unknown_space']==real['global_costmap']['global_costmap']['ros__parameters']['track_unknown_space']

def test_launch_has_one_real_control_chain_and_no_old_localizer():
    path=ROOT/'src/robot_bringup/launch/rack_hybrid.launch.py'
    spec=importlib.util.spec_from_file_location('hybrid_launch',path);module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module);ld=module.generate_launch_description()
    assert len(ld.entities)==8
    text=path.read_text()
    assert "('lidar_localization','navigation_server','path_tracker')" in text
    assert "'provide_navigation':False" in text and "'use_sim_time':True" in text
    assert 'framework_execution' not in text and 'mock_modules' not in text

def test_supervisor_hybrid_branch_starts_no_duplicate_tf_or_localizer():
    path=ROOT/'navigation/scripts/stack_manager.py'
    tree=ast.parse(path.read_text())
    branches=[n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=='args.hybrid']
    assert branches
    text='\n'.join(ast.unparse(n) for n in branches[0].body)
    assert 'kinematic_driver.py' in text
    assert 'lidar_localization.py' not in text and 'static_transform_publisher' not in text
    assert 'map_alignment.py' not in text and 'slam_toolbox' not in text

def test_adapter_not_another_footprint_controller():
    n=NS();assert HybridExecution.set_footprint(n) is True
    text=(ROOT/'src/mission_manager/mission_manager/framework_execution.py').read_text()
    assert "if self.get_parameter('provide_navigation').value:" in text

def test_adapter_stop_aborts_own_lift_before_nav_wait():
    publications=[]
    node=NS(abort_event=threading.Event(),status={'lift':.13},
        lift_pub=NS(publish=publications.append),nav_stop=NS(service_is_ready=lambda:False))
    result=HybridExecution.stop_motion(node,Trigger.Request(),Trigger.Response())
    assert node.abort_event.is_set() and not result.success
    assert publications[0].data==pytest.approx(.13)

def test_adapter_stop_orders_real_navigation_before_simulator_stop():
    calls=[];node=NS(abort_event=threading.Event(),status={'lift':.13},
        lift_pub=NS(publish=lambda m:calls.append('hold')),
        nav_stop=NS(service_is_ready=lambda:True,call_async=lambda r:NS(done=lambda:True,
            result=lambda:NS(success=True))))
    instance=HybridExecution.__new__(HybridExecution)
    instance.__dict__.update(vars(node))
    with patch.object(FrameworkExecution,'stop_motion',lambda *args:calls.append('sim_stop') or args[-1]):
        HybridExecution.stop_motion(instance,Trigger.Request(),Trigger.Response())
    assert calls==['hold','sim_stop']

def test_no_truth_input_in_real_control_nodes():
    for name in ('localization.py','navigation.py','path_tracker.py'):
        text=(ROOT/'src/robot_navigation/robot_navigation'/name).read_text()
        assert '/simulation/robot_pose' not in text and 'PoseArray' not in text

def test_evaluator_publishes_no_motion_or_localization():
    tree=ast.parse((ROOT/'src/robot_navigation/robot_navigation/hybrid_evaluator.py').read_text())
    pubs=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
          and n.func.attr=='create_publisher']
    assert len(pubs)==1 and pubs[0].args[1].value=='evaluation/metrics'


def test_csv_path_error_uses_truth_not_estimated_pose():
    from collections import deque
    from robot_navigation.hybrid_evaluator import HybridEvaluator
    data=TruthSeries();data.add(1.,(1.,1.,0));data.add(1.1,(1.2,1.,0))
    rows=[];node=NS(pending=deque([(1.05,(1.11,1.002,0),(10.,.2,.1,.5))]),truth=data,stats=ErrorStats(),
        tracking=True,tracking_seen=time.monotonic(),path=[(1,1),(2,1)],command=(.1,0,0),
        state='NAV_RACK',rack='A',tracking_phase='AXIS_STOP',writer=NS(writerow=rows.append),stream=NS(flush=lambda:None),
        skipped=0,latest={},last_pair=0.)
    HybridEvaluator.compare(node)
    assert node.stats.count==1 and node.latest['cross_track_error_m']==pytest.approx(0.)
    assert node.latest['position_error_m']>0.01 and len(rows)==1
    assert node.latest['tracking_phase']=='AXIS_STOP'
    assert node.latest['odom_receive_rtf']==pytest.approx(.5)
    assert len(rows[0])==21

def test_no_matched_samples_are_not_reported_as_zero_error():
    stats=ErrorStats().report()
    assert stats['samples']==0 and stats['position_rmse_m'] is None

def test_peripheral_health_bootstraps_without_faking_navigation_ready():
    from mission_v3 import MissionV3
    import json
    messages=[];node=NS(initialized=True,busy=False,nav_status=None,nav_seen=0.,
        camera_seen=time.monotonic(),speed=0.,angular_speed=0.,
        pose=(1.,1.,3.141592653589793),odom_seen=time.monotonic(),status_seen=time.monotonic(),
        status={'ready':True,'lift_settled':True,'carrying':False,'fault':'',
                'actuator_feedback_fresh':True,'base_motion_stopped':True,'lift_motion_stopped':True},
        get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS(sec=5,nanosec=0))),
        health_pub=NS(publish=lambda p:None),base_feedback_pub=NS(publish=messages.append),up_pub=NS(publish=lambda m:None))
    with patch.object(MissionV3,'fresh',lambda *a,**k:True):HybridExecution.health(node)
    data=json.loads(messages[0].data)
    assert data['modules']['base'] and data['modules']['lift']
    assert not data['modules']['docking'] and not data['modules']['navigation']
    assert data['stopped'] is True
    assert data['lift_is_up'] is False and data['lift_state_source']=='measured'

def test_odom_receipt_timing_preserved_until_truth_pairing():
    from collections import deque
    from unittest.mock import patch
    from nav_msgs.msg import Odometry
    from robot_navigation.hybrid_evaluator import HybridEvaluator
    node=NS(last_odom_stamp=1.,last_odom_received=10.,receive_origin=8.,pending=deque(),compare=lambda:None)
    msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_link'
    msg.header.stamp.sec=1;msg.header.stamp.nanosec=100000000;msg.pose.pose.orientation.w=1.
    with patch('robot_navigation.hybrid_evaluator.time.monotonic',return_value=10.2):
        HybridEvaluator.on_odom(node,msg)
    stamp,pose,timing=node.pending[0]
    assert stamp==pytest.approx(1.1)
    assert timing==pytest.approx((2.2,.2,.1,.5))
