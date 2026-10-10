"""使用真实进退架算法的离线契约验证，不初始化 ROS 或 Gazebo。"""
import copy
import math
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest
from mission_interfaces.action import Dock
from mission_manager.config_loader import read_yaml, validate
from mission_manager.project_paths import project_root
from mission_manager.hybrid_execution import HybridExecution
from mission_manager.state_machine import Mission
from robot_navigation.core import load_settings
from robot_navigation.docking import DockGeometry
from robot_navigation.navigation import NavigationServer

FOLDER = project_root() / 'src/robot_bringup/config'


def geometry():
    return DockGeometry(read_yaml(FOLDER/'navigation_points_rack.yaml'),
                        load_settings(FOLDER/'navigation_hybrid.yaml'))


def request(rack='A', operation='ENTER'):
    return NS(request_id='dock-1', rack_id=rack, operation=operation,
              expected_qr=f'RACK{rack}_XXXX', max_duration_sec=12.)


def context(req):
    return dict(request_id=req.request_id, state='DOCK_'+req.operation,
                rack=req.rack_id, cargo='EMPTY')


@pytest.mark.parametrize('rack',list('ABCD'))
@pytest.mark.parametrize('operation',['ENTER','EXIT'])
def test_each_corridor_clears_legs_and_field_and_is_axis_aligned(rack,operation):
    geo=geometry();req=request(rack,operation)
    start=geo.points.points[rack] if operation=='ENTER' else geo.points.drops[rack]
    corridor=geo.plan(req,start,carrying=False,context=context(req),simulation=True)
    for a,b in zip(corridor.points,corridor.points[1:]):
        assert a[0]==b[0] or a[1]==b[1]
        for t in (0.,.25,.5,.75,1.):
            pose=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),math.pi)
            corridor.validate(pose,corridor.points)
    if operation=='ENTER':
        bounds=geo.racks[rack]['bounds']
        assert corridor.points[-1]==((bounds[0]+bounds[1])/2,(bounds[2]+bounds[3])/2)
    else:
        assert corridor.points[-1]==pytest.approx((start[0]+.45,start[1]))


def test_c_aligns_y_outside_legs_before_entry():
    geo=geometry();req=request('C');start=geo.points.points['C']
    corridor=geo.plan(req,start,carrying=False,context=context(req),simulation=True)
    assert corridor.points[1][0]==start[0] > geo.racks['C']['bounds'][1]+.12
    assert corridor.points[1][1]==corridor.points[-1][1] != start[1]


@pytest.mark.parametrize('offset',[-.011,.011])
def test_stop_tolerance_allows_bounded_correction_at_drop(offset):
    geo=geometry();req=request('B','EXIT');drop=geo.points.drops['B']
    start=(drop[0],drop[1]+offset,drop[2])
    corridor=geo.plan(req,start,carrying=False,context=context(req),simulation=True)
    corridor.validate(start,corridor.points)


@pytest.mark.parametrize('change',[
    {'rack_id':'E'}, {'operation':'MOVE'}, {'expected_qr':'RACKB_XXXX'},
    {'request_id':''}, {'max_duration_sec':float('nan')}, {'max_duration_sec':0.},
])
def test_invalid_request_never_produces_corridor(change):
    geo=geometry();req=request();req.__dict__.update(change)
    with pytest.raises(ValueError):geo.request(req,True)


@pytest.mark.parametrize('change',[{'cargo':'UP'},{'rack':'B'},
    {'state':'LIFT_DOWN'},{'request_id':'previous-request'}])
def test_dock_requires_current_framework_operation(change):
    geo=geometry();req=request();ctx=context(req);ctx.update(change)
    with pytest.raises(ValueError,match='context'):
        geo.plan(req,geo.points.points['A'],carrying=False,context=ctx,simulation=True)


def test_real_template_has_no_simulation_fallback():
    cfg=load_settings(FOLDER/'navigation_robot.yaml')
    real=DockGeometry(read_yaml(FOLDER/'navigation_points_robot.yaml'),cfg)
    with pytest.raises(ValueError,match='实测'):real.request(request(),False)
    with pytest.raises(ValueError,match='实机'):geometry().request(request(),False)


@pytest.mark.parametrize('issue',['narrow','exit_short','exit_outside','entry_limit'])
def test_invalid_geometry_cannot_send_motion(issue):
    geo=geometry();req=request('A','EXIT' if issue.startswith('exit') else 'ENTER')
    if issue=='narrow':geo.racks['A']['leg_thickness']=.06
    if issue=='exit_short':geo.racks['A']['exit_distance']=.1
    if issue=='exit_outside':geo.racks['A']['exit_distance']=3.
    if issue=='entry_limit':geo.racks['A']['max_entry_distance']=.1
    start=geo.points.points['A'] if req.operation=='ENTER' else geo.points.drops['A']
    with pytest.raises(ValueError):geo.plan(req,start,carrying=False,context=context(req),simulation=True)


def test_corridor_monitor_stops_heading_and_position_deviation():
    geo=geometry();req=request();start=geo.points.points['A']
    corridor=geo.plan(req,start,carrying=False,context=context(req),simulation=True)
    with pytest.raises(ValueError,match='heading'):corridor.validate((*start[:2],math.pi+.05),corridor.points)
    with pytest.raises(ValueError,match='corridor'):corridor.validate((start[0],start[1]+.02,start[2]),corridor.points)
    with pytest.raises(ValueError,match='platform'):
        geo.plan(req,start,carrying=True,context=context(req),simulation=True)


def dock_node(operation='ENTER'):
    geo=geometry();req=request('A',operation)
    start=geo.points.points['A'] if operation=='ENTER' else geo.points.drops['A']
    node=NS(cfg=geo.cfg,dock_geometry=geo,dock_corridor=None,busy=True,lock=threading.Lock(),
        abort_event=threading.Event(),mission_context=context(req),mission_seen=time.monotonic(),
        get_parameter=lambda k:NS(value=True),get_logger=lambda:NS(error=lambda text:None))
    node.inputs=lambda **kw:((start,(0.,0.,0.)),False,None)
    node.path_message=lambda points,yaw:(points,yaw)
    calls=[]
    node.wait_handoff=lambda *a:calls.append('measured-stop')
    node.run_track=lambda h,path,target,carry,deadline:calls.append(('track',target,carry))
    events=[]
    handle=NS(request=req,is_cancel_requested=False,succeed=lambda:events.append('success'),
        abort=lambda:events.append('abort'),canceled=lambda:events.append('cancel'),publish_feedback=lambda m:None)
    return node,handle,calls,events


@pytest.mark.parametrize('operation',['ENTER','EXIT'])
def test_real_dock_dispatches_same_tracker_after_stop_and_returns_only_motion_result(operation):
    node,handle,calls,events=dock_node(operation)
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        result=NavigationServer.execute_dock(node,handle)
    assert result.success and events==['success'] and calls[0]=='measured-stop'
    assert calls[1][0]=='track' and calls[1][2] is False
    assert result.ready_to_lift==(operation=='ENTER') and result.exited==(operation=='EXIT')
    assert not node.busy and node.dock_corridor is None


def test_dock_track_failure_cannot_report_ready_to_lift():
    node,handle,calls,events=dock_node()
    node.run_track=lambda *a:(_ for _ in ()).throw(ValueError('measured stop unconfirmed'))
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        result=NavigationServer.execute_dock(node,handle)
    assert not result.success and not result.ready_to_lift and events==['abort'] and not node.busy


def test_cancel_before_handoff_never_dispatches_tracking():
    node,handle,calls,events=dock_node();handle.is_cancel_requested=True
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        result=NavigationServer.execute_dock(node,handle)
    assert not result.success and not calls and events==['cancel'] and not node.busy


def test_cancel_at_tracking_completion_cannot_report_ready_to_lift():
    node,handle,calls,events=dock_node()
    node.run_track=lambda *a:setattr(handle,'is_cancel_requested',True)
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        result=NavigationServer.execute_dock(node,handle)
    assert not result.success and not result.ready_to_lift and events==['cancel']


def test_missing_current_context_times_out_without_dispatch():
    node,handle,calls,events=dock_node();node.mission_context=None
    node.cfg['cancel_timeout_sec']=.001
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        result=NavigationServer.execute_dock(node,handle)
    assert not result.success and not calls and events==['abort'] and not node.busy


def test_platform_feedback_race_waits_without_moving():
    node,handle,calls,events=dock_node('EXIT');original=node.inputs;attempts=[]
    def inputs(**kw):
        attempts.append(True)
        if len(attempts)==1:raise ValueError('fresh measured base/lift feedback required')
        return original(**kw)
    node.inputs=inputs
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        result=NavigationServer.execute_dock(node,handle)
    assert result.success and len(attempts)==3 and len(calls)==2


def test_dock_and_navigation_share_busy_lock():
    node,handle,calls,events=dock_node();node.stopping=False;node.closing=False
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        assert NavigationServer.accept_dock(node,handle.request).name=='REJECT'
        node.busy=False
        assert NavigationServer.accept_dock(node,handle.request).name=='ACCEPT'
        assert NavigationServer.accept_dock(node,handle.request).name=='REJECT'


def test_only_active_dock_uses_calibrated_corridor_guard():
    node,handle,calls,events=dock_node();req=handle.request
    pose=node.inputs()[0][0];node.dock_corridor=node.dock_geometry.plan(
        req,pose,carrying=False,context=node.mission_context,simulation=True)
    node.inputs=lambda **kw:((pose,(0.,0.,0.)),False,None)
    NavigationServer.check_route(node,node.dock_corridor.points,False)
    with pytest.raises(ValueError):NavigationServer.check_route(node,node.dock_corridor.points,True)
    node.dock_corridor=None
    node.inputs=lambda **kw:(_ for _ in ()).throw(ValueError('normal map guard required'))
    with pytest.raises(ValueError,match='normal map'):
        NavigationServer.check_route(node,[(1.,1.),(2.,1.)],False)


def test_simulated_lift_selects_model_after_real_dock_without_driving():
    import scene
    selected=[]
    node=NS(mission_context={'state':'LIFT_UP','rack':'C'},mission_seen=time.monotonic(),
        abort_event=threading.Event(),current_payload=None,pose=(*scene.rack_model_center('C'),math.pi),
        odom_seen=time.monotonic(),wait_still=lambda goal:True,
        select_payload=lambda name:selected.append(name) or True)
    with patch('mission_manager.hybrid_execution.rclpy.ok',return_value=True):
        assert HybridExecution.prepare_lift_up(node)=='C'
    assert selected==['C'] and node.entered_rack=='C'


def test_joint_bridge_does_not_select_wrong_rack_pose():
    selected=[]
    node=NS(mission_context={'state':'LIFT_UP','rack':'B'},mission_seen=time.monotonic(),
        abort_event=threading.Event(),current_payload=None,pose=(1.,1.,math.pi),
        odom_seen=time.monotonic(),select_payload=lambda name:selected.append(name) or True)
    with patch('mission_manager.hybrid_execution.rclpy.ok',return_value=True),pytest.raises(ValueError,match='radar'):
        HybridExecution.prepare_lift_up(node)
    assert not selected


def test_hybrid_bridge_disables_duplicate_navigation_and_dock_servers():
    launch=(project_root()/'src/robot_bringup/launch/rack_hybrid.launch.py').read_text()
    assert "'provide_docking':False" in launch and "'provide_navigation':False" in launch


def test_hybrid_lift_bridge_cannot_publish_chassis_velocity():
    node=NS()
    HybridExecution.pub_vel(node)
    with pytest.raises(ValueError,match='cannot command'):
        HybridExecution.pub_vel(node,.01,0.)


def test_real_and_hybrid_success_sequences_are_identical():
    from test_framework_rack import Backend
    sequence=[]
    for mode,mission_file in [('robot','mission_robot.yaml'),('simulation','mission_rack.yaml')]:
        targets=read_yaml(FOLDER/'targets_rack.yaml');targets['simulation_only']=mode=='simulation'
        cfg=validate({'mission':read_yaml(FOLDER/mission_file),'interfaces':read_yaml(FOLDER/'interfaces.yaml'),
                      'targets':targets},mode)
        backend=Backend();mission=Mission(cfg,backend,clock=lambda:1.)
        assert mission.arm()[0];states=[]
        for _ in range(60):
            states.append((mission.state,mission.rack))
            if mission.state=='FINISHED':break
            if mission.state=='STOPPING':
                req,callback=backend.stops[-1];callback(req,True,{});mission.tick()
            else:backend.finish(mission)
        assert mission.state=='FINISHED' and mission.final_confirmed
        sequence.append(states)
    assert sequence[0]==sequence[1]
