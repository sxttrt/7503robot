"""离线验证遮挡探索；不启动 ROS/Gazebo。"""
import math
import numpy as np
from types import SimpleNamespace as NS
from robot_navigation.core import GridMap, Planner
from robot_navigation.observed_planning import observed_route, endpoint_summary

def planner(data=None,half=(.12,.08)):
    meta=NS(resolution=.025,size_x=120,size_y=80,
            origin=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.)))
    if data is None:data=np.zeros((80,120),dtype=np.uint8)
    return Planner(GridMap(meta,data.ravel()),half,(3.,2.),margin=.05)

def assert_safe(p,start,route):
    assert route
    for a,b in zip([start]+route,route):
        assert abs(a[0]-b[0])<1e-8 or abs(a[1]-b[1])<1e-8
        assert p.segment_free(a,b)

def test_complete_observed_route_unchanged():
    p=planner();start=(2.5,1.);goal=(1.5,.5)
    route,complete=observed_route(p,start,goal)
    assert complete and route==p.plan(start,goal)
    assert_safe(p,start,route)

def test_unknown_frontier_stops_before_unseen_space_then_replans():
    data=np.zeros((80,120),dtype=np.uint8);data[:,:60]=255
    p=planner(data);start=(2.5,1.);goal=(.5,1.)
    assert p.plan(start,goal) is None
    route,complete=observed_route(p,start,goal)
    assert not complete and route[-1][0]>1.5+.17+.049
    assert_safe(p,start,route)
    # 原地图未知分类保持不变；没有提前发布/执行候选后半程。
    assert np.all(p.grid.costs[:,:60]==255) and np.all(p.grid.blocked[:,:60])
    updated=planner();next_route,complete=observed_route(updated,route[-1],goal)
    assert complete and next_route[-1]==goal
    assert_safe(updated,route[-1],next_route)

def test_goal_real_obstacle_does_not_become_explorable():
    data=np.zeros((80,120),dtype=np.uint8);data[0,0]=255;data[36:44,16:24]=254
    assert observed_route(planner(data),(2.5,1.),(.5,1.))==(None,False)

def test_unknown_start_cannot_move():
    data=np.full((80,120),255,dtype=np.uint8)
    assert observed_route(planner(data),(2.5,1.),(.5,1.))==(None,False)

def test_lethal_barrier_still_blocks_candidate():
    data=np.zeros((80,120),dtype=np.uint8);data[:,58:62]=254;data[:,:30]=255
    assert observed_route(planner(data),(2.5,1.),(.5,1.))==(None,False)

def test_loaded_footprint_applies_to_staging():
    data=np.zeros((80,120),dtype=np.uint8);data[:,:60]=255
    p=planner(data,half=(.12,.19));start=(2.5,1.)
    route,complete=observed_route(p,start,(.5,1.))
    assert not complete;assert_safe(p,start,route)

def test_boundary_cannot_be_explored():
    data=np.zeros((80,120),dtype=np.uint8);data[0,0]=255
    assert observed_route(planner(data),(2.5,1.),(.05,1.))==(None,False)

def test_unknown_near_start_without_safe_progress_stops():
    data=np.zeros((80,120),dtype=np.uint8);data[:,:92]=255
    assert observed_route(planner(data),(2.5,1.),(.5,1.))==(None,False)

def test_diagnostics_classify_unknown_and_lethal():
    data=np.zeros((80,120),dtype=np.uint8);data[40,60]=255;data[41,60]=254
    info=endpoint_summary(planner(data),(1.5,1.))
    assert info['lethal']==1 and info['unknown']==1 and info['reason']

def test_candidate_with_real_obstacle_detour_keeps_every_segment_safe():
    data=np.zeros((80,120),dtype=np.uint8);data[32:48,72:80]=254;data[:,:40]=255
    p=planner(data);start=(2.5,1.)
    route,complete=observed_route(p,start,(.5,1.))
    assert not complete;assert_safe(p,start,route)


def test_actual_scene_a_visibility_and_observed_staging():
    import importlib.util
    from mission_manager.project_paths import project_root
    path=project_root()/'tools/replay_a_visibility.py'
    spec=importlib.util.spec_from_file_location('replay_visibility',path)
    replay=importlib.util.module_from_spec(spec);spec.loader.exec_module(replay)
    poses=[(2.72-i*.008,1.,math.pi) for i in range(21)]
    poses += [(2.559,1.,a) for a in np.linspace(math.pi,math.pi+math.pi/6,80)]
    meta,data=replay.replay(poses)
    p=Planner(GridMap(meta,data.ravel()),(.1232,.0848),(3.02,2),margin=.05)
    start=(2.559,1.);goal=replay.scene.scan('A')
    assert p.plan(start,goal) is None
    assert endpoint_summary(p,goal)['unknown']>0 and endpoint_summary(p,goal)['lethal']==0
    route,complete=observed_route(p,start,goal)
    assert not complete;assert_safe(p,start,route)
    # 移到观测前缀终点后增加新扫描，必须严格地图可达才执行剩余段。
    poses.extend([(*route[-1],math.pi)])
    meta,data=replay.replay(poses)
    updated=Planner(GridMap(meta,data.ravel()),(.1232,.0848),(3.02,2),margin=.05)
    finish,complete=observed_route(updated,route[-1],goal)
    assert complete and finish[-1]==goal
    assert_safe(updated,route[-1],finish)


def run_staged_navigation(cancel=False):
    import threading
    from unittest.mock import patch
    from test_real_navigation import config,grid,stamped
    from robot_navigation.navigation import NavigationServer
    cfg=config();cfg['allow_unknown_in_field']=False;message=grid();data=np.asarray(message.data,dtype=np.uint8).reshape(80,122)
    data[:,:60]=255;message.data=data.ravel().tolist()
    calls=[];outcomes=[];start=(2.5,1.,math.pi)
    node=NS(cfg=cfg,busy=True,lock=threading.Lock(),abort_event=threading.Event(),
        now=lambda:7.,get_logger=lambda:NS(error=lambda text:None,info=lambda text:None),
        odom_stamp=8.,lidar={'stamp':{'sec':8,'nanosec':0}},path_message=lambda points,yaw:(points,yaw))
    node.inputs=lambda:((start,(0,0,0)),False,message)
    node.wait_handoff=lambda *args:None
    node.wait_post_stop_map=lambda *args:NavigationServer.wait_post_stop_map(node,*args)
    node.wait_payload_map=lambda *args:node.inputs()
    def run(_handle,path,target,carrying,deadline,rotating=False):
        calls.append((path,target))
        assert_safe(planner(data[:,:120]),start[:2],path[0][1:]) if len(calls)==1 else None
        updated=grid();updated.metadata.update_time.sec=8
        node.inputs=lambda:((target,(0,0,0)),False,updated)
        if cancel:node.abort_event.set()
    node.run_track=run
    handle=NS(request=NS(pose=stamped(x=.5,y=1.,yaw=math.pi)),is_cancel_requested=False,
        succeed=lambda:outcomes.append(True),abort=lambda:outcomes.append(False))
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True):
        result=NavigationServer.execute(node,handle)
    return calls,outcomes,result

def test_same_navigation_request_stages_then_completes_original_goal():
    calls,outcomes,result=run_staged_navigation()
    assert outcomes==[True] and result.error_code==0 and len(calls)==2
    assert calls[0][1][0]>.5 and calls[-1][1]==(.5,1.,math.pi)

def test_cancellation_after_staging_prevents_next_segment():
    calls,outcomes,result=run_staged_navigation(cancel=True)
    assert outcomes==[False] and len(calls)==1 and 'cancelled' in result.error_msg

def test_failure_snapshot_keeps_unknown_and_geometry(tmp_path):
    from unittest.mock import patch
    from robot_navigation.navigation import NavigationServer
    import json
    data=np.zeros((80,120),dtype=np.uint8);data[40,60]=255
    with patch('robot_navigation.navigation.project_root',return_value=tmp_path):
        detail=json.loads(NavigationServer.save_planning_failure(None,planner(data),
                          (2.5,1.,math.pi),(1.5,1.,math.pi)))
    saved=np.load(detail['snapshot'])
    assert saved['costs'][40,60]==255 and saved['margin']==.05
    assert detail['goal']['unknown']==1
