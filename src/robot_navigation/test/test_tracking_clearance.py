"""Replay the observed D approach failure without ROS nodes or Gazebo."""
import math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from mission_manager.project_paths import project_root
from robot_navigation.core import load_settings,planner_for
from robot_navigation.navigation import NavigationServer

def inputs():
    d=np.load(Path(__file__).parent/'data/d_approach_costmap.npz')
    h,w=d['costs'].shape
    message=NS(header=NS(frame_id='map'),metadata=NS(resolution=float(d['resolution']),size_x=w,size_y=h,
        origin=NS(position=NS(x=float(d['origin'][0]),y=float(d['origin'][1])),
                  orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=d['costs'].reshape(-1).tolist())
    cfg=load_settings(project_root()/'src/robot_bringup/config/navigation_robot.yaml')
    return cfg,message

def test_old_nominal_d_lane_passes_but_permitted_offset_collides():
    cfg,msg=inputs();physical=planner_for(cfg,msg,False)
    assert physical.segment_free((.997,.39),(2.34,.39))
    assert not physical.segment_free((1.6977,.39133),(1.6977,.39133))
    assert .39133-.39<cfg['position_tolerance']

@pytest.mark.parametrize('carrying,rotating',[(False,False),(True,False),(False,True)])
def test_tracking_reserve_is_explicit_and_not_double_counted(carrying,rotating):
    cfg,msg=inputs()
    measured=planner_for(cfg,msg,carrying,rotating)
    nominal=planner_for(cfg,msg,carrying,rotating,for_tracking=True)
    assert measured.margin==pytest.approx(.05)
    assert nominal.margin==pytest.approx(.062)
    assert (measured.hx,measured.hy)==(nominal.hx,nominal.hy)

def test_recorded_map_reserved_route_is_safe_under_allowed_deviation():
    cfg,msg=inputs();physical=planner_for(cfg,msg,False)
    reserved=planner_for(cfg,msg,False,for_tracking=True)
    start=(.997,.708);goal=(2.34,.39);route=reserved.plan(start,goal)
    assert route and route[-1]==goal
    # The square reserve contains the full radial tolerance disk, including corners.
    for a,b in zip([start]+route,route):
        assert a[0]==b[0] or a[1]==b[1]
        for dx in (-cfg['position_tolerance'],0,cfg['position_tolerance']):
            for dy in (-cfg['position_tolerance'],0,cfg['position_tolerance']):
                assert physical.segment_free((a[0]+dx,a[1]+dy),(b[0]+dx,b[1]+dy))

def test_monitor_rejects_zero_error_lane_without_reserve():
    cfg,msg=inputs()
    node=NS(cfg=cfg,inputs=lambda:(((1.69,.39,math.pi),(0,0,0)),False,msg),
        save_planning_failure=lambda *args:'snapshot',get_logger=lambda:NS(warn=lambda msg:None))
    with pytest.raises(ValueError,match='live radar route blocked'):
        NavigationServer.check_route(node,[(.997,.39),(2.34,.39)],False)

def test_actual_footprint_monitor_does_not_apply_reserve_twice():
    cfg,msg=inputs();route=planner_for(cfg,msg,False,for_tracking=True).plan((.997,.708),(2.34,.39))
    node=NS(cfg=cfg,inputs=lambda:(((1.4,.725-.011,math.pi),(0,0,0)),False,msg))
    NavigationServer.check_route(node,[(.997,.708)]+route,False)
