"""Startup map regressions; pure helpers and uninitialized node doubles only.

No rclpy.init, node construction, ROS process, Gazebo process or live topic access.
"""
import json
import math
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import pytest

from mission_manager.config_loader import load_config
from mission_manager.mission_node import MissionNode
from mission_manager.project_paths import project_root
from mission_manager.state_machine import Mission
from robot_navigation.core import (
    WaitingForObservedCostmap, costmap_observation_ready,
    load_settings, planner_for,
)
from robot_navigation.navigation import NavigationServer


def config():
    cfg = load_settings(project_root()/'src/robot_bringup/config/navigation_robot.yaml')
    cfg['calibration_verified'] = True
    return cfg


def message(fill=0):
    return NS(header=NS(frame_id='map'),
        metadata=NS(resolution=.025, size_x=122, size_y=80,
            origin=NS(position=NS(x=0., y=0.),
                orientation=NS(x=0., y=0., z=0., w=1.)),
            update_time=NS(sec=5, nanosec=0)), data=[fill]*(122*80))


def measured_health():
    return {'stamp': {'sec': 5, 'nanosec': 0},
        'modules': {'base': True, 'lift': True}, 'stopped': True,
        'lift_is_up': False, 'lift_state_source': 'measured', 'fault': ''}


def node_double(msg):
    maps={'message':msg}; published=[]
    node=NS(cfg=config(), fault='', odom=((1.,1.,math.pi),(0.,0.,0.)),
        odom_stamp=5., odom_received=time.monotonic(),
        lidar={'valid':True}, lidar_received=time.monotonic(),
        base=measured_health(), base_received=time.monotonic(), payload_stamp=4.,
        track=NS(available=lambda:True),
        cache=NS(snapshot=lambda:(maps['message'],time.monotonic(),1),
            diagnostic=lambda: {'sequence':1}),
        now=lambda:5., costmap_observed=False, busy=False,
        get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS(sec=5,nanosec=0))),
        status_pub=NS(publish=published.append), abort_event=threading.Event())
    node.inputs=lambda require_payload_map=True,allow_lift_transition=False: NavigationServer.inputs(node,require_payload_map,allow_lift_transition)
    return node,maps,published


@pytest.mark.parametrize('initialized', [False, True])
def test_entirely_unobserved_map_is_waiting_not_a_route_failure(initialized):
    planner=planner_for(config(),message(255),False)
    with pytest.raises(WaitingForObservedCostmap):
        costmap_observation_ready(planner,(1.,1.),initialized=initialized)
    assert np.all(planner.grid.costs==255)  # Policy never erases unknown cells.


def test_start_must_be_observed_even_if_map_has_remote_free_cells():
    msg=message(255);msg.data[5*122+5]=0
    cfg=config();cfg['allow_unknown_in_field']=False
    planner=planner_for(cfg,msg,False)
    with pytest.raises(WaitingForObservedCostmap):
        costmap_observation_ready(planner,(1.,1.))


def test_single_unknown_in_start_safety_envelope_defers_start():
    msg=message();msg.data[40*122+40]=255
    cfg=config();cfg['allow_unknown_in_field']=False
    planner=planner_for(cfg,msg,False)
    with pytest.raises(WaitingForObservedCostmap):
        costmap_observation_ready(planner,(1.,1.))


def test_first_complete_start_observation_is_ready():
    assert costmap_observation_ready(planner_for(config(),message(),False),(1.,1.))


def test_only_self_footprint_cleared_does_not_prove_scan_observation():
    msg=message(255);planner=planner_for(config(),msg,False)
    mx,my=planner.hx+planner.margin,planner.hy+planner.margin
    i0=math.floor((1.-mx)/.025);i1=math.floor((1.+mx)/.025)+1
    j0=math.floor((1.-my)/.025);j1=math.floor((1.+my)/.025)+1
    cells=np.full((80,122),255,dtype=np.uint8);cells[j0:j1,i0:i1]=0
    msg.data=cells.reshape(-1).tolist();planner=planner_for(config(),msg,False)
    assert planner.segment_free((1.,1.),(1.,1.))
    with pytest.raises(WaitingForObservedCostmap):
        costmap_observation_ready(planner,(1.,1.))


def test_real_obstacle_in_start_is_not_hidden_by_initialization_wait():
    msg=message();msg.data[40*122+40]=254
    with pytest.raises(ValueError) as error:
        costmap_observation_ready(planner_for(config(),msg,False),(1.,1.))
    assert not isinstance(error.value,WaitingForObservedCostmap)
    assert 'blocked' in str(error.value)


def test_initialized_status_does_not_prevent_payload_clearance_handshake():
    msg=message();msg.data[40*122+40]=254
    planner=planner_for(config(),msg,True)
    assert costmap_observation_ready(planner,(1.,1.),initialized=True)
    assert not planner.segment_free((1.,1.),(1.,1.))


def test_initial_map_wait_does_not_latch_success_or_publish_ready():
    node,_maps,published=node_double(message(255))
    with pytest.raises(WaitingForObservedCostmap): node.inputs(require_payload_map=False)
    assert not node.costmap_observed
    NavigationServer.publish_status(node)
    assert json.loads(published[-1].data)['ready'] is False
    assert not node.costmap_observed


def test_valid_scan_map_releases_ready_without_fake_health_dependency():
    node,maps,published=node_double(message(255))
    NavigationServer.publish_status(node)
    assert not json.loads(published[-1].data)['ready']
    maps['message']=message()
    NavigationServer.publish_status(node)
    assert json.loads(published[-1].data)['ready'] and node.costmap_observed


def test_after_initialization_current_lethal_still_fails_actual_route_check():
    node,maps,_published=node_double(message())
    node.inputs();assert node.costmap_observed
    maps['message'].data[40*122+40]=254
    node.inputs(require_payload_map=False)  # Module remains available during Dock/lift.
    with pytest.raises(ValueError,match='blocked'):
        NavigationServer.check_route(node,[(1.,1.),(1.1,1.)],False)


def test_initialized_backend_does_not_accept_total_map_loss():
    node,maps,published=node_double(message())
    node.inputs();maps['message']=message(255)
    NavigationServer.publish_status(node)
    assert json.loads(published[-1].data)['ready'] is False


def test_execute_initial_map_waits_for_observed_map_without_dispatch():
    node,maps,_published=node_double(message(255));calls=[]
    original=node.inputs
    def inputs():
        calls.append('read')
        if len(calls)==2: maps['message']=message()
        return original()
    node.inputs=inputs
    handle=NS(is_cancel_requested=False)
    with patch('robot_navigation.navigation.rclpy.ok',return_value=True), \
            patch('robot_navigation.navigation.time.sleep',lambda seconds:None):
        _odom,_carrying,msg=NavigationServer.wait_payload_map(node,handle,time.monotonic()+1)
    assert calls==['read','read'] and msg is maps['message'] and node.costmap_observed


def test_auto_arm_remains_idle_until_first_observed_map():
    node,maps,published=node_double(message(255));calls=[]
    folder=project_root()/'src/robot_bringup/config'
    cfg=load_config(folder/'mission_rack.yaml',folder/'interfaces.yaml',
        folder/'targets_rack.yaml','simulation')
    ready={'value':False}
    backend=NS(ready=lambda:ready['value'], healthy=lambda:ready['value'],poll=lambda:None,
        start=lambda *args:calls.append(args),cancel=lambda *args:None,stop=lambda *args:None)
    mission=Mission(cfg,backend,lambda:100.)
    mission_node=NS(auto_arm=True,mission=mission,closing=False,config=cfg)
    for _ in range(4):
        NavigationServer.publish_status(node)
        ready['value']=json.loads(published[-1].data)['ready']
        MissionNode.on_tick(mission_node)
    assert mission.state=='IDLE' and not calls
    maps['message']=message();NavigationServer.publish_status(node)
    ready['value']=json.loads(published[-1].data)['ready']
    MissionNode.on_tick(mission_node)
    assert mission.state=='NAV_START' and len(calls)==1 and calls[0][0]=='navigation'
