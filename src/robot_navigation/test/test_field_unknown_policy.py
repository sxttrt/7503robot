"""Validate allowed unknown routes, known obstacles and exact field limits offline."""
import copy
import math
from types import SimpleNamespace as NS
import numpy as np
import pytest
import yaml
from robot_navigation.core import planner_for,load_settings,costmap_observation_ready,WaitingForObservedCostmap
from robot_navigation.observed_planning import observed_route
from robot_navigation.navigation import NavigationServer
from mission_manager.project_paths import project_root
from test_costmap_startup import message
from test_real_navigation import config


@pytest.mark.parametrize('carrying,rotating',[(False,False),(True,False),(False,True)])
def test_unknown_is_allowed_inside_field_for_all_navigation_footprints(carrying,rotating):
    m=message(255);p=planner_for(config(),m,carrying,rotating,for_tracking=True)
    start=(2.5,1.);end=start if rotating else (.5,1.)
    assert p.segment_free(start,end)
    assert np.all(p.grid.costs==255) and not np.any(p.grid.blocked)


def test_unknown_destination_gets_full_route_without_frontier_staging():
    m=message();a=np.asarray(m.data).reshape(80,122);a[:,:60]=255;m.data=a.ravel().tolist()
    p=planner_for(config(),m,False,for_tracking=True)
    route,complete=observed_route(p,(2.5,1.),(.5,1.))
    assert complete and route[-1]==(.5,1.)
    n=NS(cfg=config(),inputs=lambda:(((2.,1.,math.pi),(0,0,0)),False,m))
    NavigationServer.check_route(n,[(2.5,1.),(.5,1.)],False)


def test_seen_lethal_barrier_still_blocks_route_through_unknown():
    m=message(255);a=np.asarray(m.data).reshape(80,122);a[:,60]=254;m.data=a.ravel().tolist()
    p=planner_for(config(),m,False,for_tracking=True)
    assert not p.segment_free((2.5,1.),(.5,1.))
    assert observed_route(p,(2.5,1.),(.5,1.))==(None,False)


@pytest.mark.parametrize('carrying',[False,True])
def test_known_obstacle_at_unknown_destination_is_not_erased(carrying):
    m=message(255);m.data[40*122+20]=254
    p=planner_for(config(),m,carrying,for_tracking=True)
    assert p.plan((2.5,1.),(.5,1.)) is None


@pytest.mark.parametrize('point',[(-.5,1.),(3.2,1.),(1.,-.1),(1.,2.1),(.05,1.),(1.,.05)])
def test_unknown_never_allows_center_or_full_footprint_outside_field(point):
    p=planner_for(config(),message(255),False,for_tracking=True)
    assert p.plan((2.5,1.),point) is None


def test_outside_received_map_is_not_assumed_free():
    m=message(255);m.metadata.size_x=60;m.data=[255]*(80*60)
    p=planner_for(config(),m,False)
    assert not p.segment_free((1.,1.),(2.,1.))


def test_startup_requires_observed_map_even_when_routes_allow_unknown():
    p=planner_for(config(),message(255),False)
    with pytest.raises(WaitingForObservedCostmap):costmap_observation_ready(p,(1.,1.))
    m=message();m.data[40*122+40]=255;p=planner_for(config(),m,False)
    assert costmap_observation_ready(p,(1.,1.))
    assert p.segment_free((1.,1.),(1.,1.)) and not p.grid.blocked[40,40]


def test_setting_false_restores_conservative_unknown_policy():
    c=config();c['allow_unknown_in_field']=False
    assert planner_for(c,message(255),False).plan((2.5,1.),(.5,1.)) is None


def test_fixed_real_and_generated_hybrid_maps_cover_field_and_preserve_raw_unknown():
    for name in ('robot','hybrid'):
        folder=project_root()/'src/robot_bringup/config'
        cfg=load_settings(folder/f'navigation_{name}.yaml')
        params=yaml.safe_load((folder/f'costmap_{name}.yaml').read_text())['global_costmap']['global_costmap']['ros__parameters']
        assert cfg['allow_unknown_in_field'] is True
        assert params['rolling_window'] is False
        assert params['origin_x']==params['origin_y']==0.
        assert params['width']>=cfg['field_size'][0] and params['height']>=cfg['field_size'][1]
        assert params['width']==math.ceil(cfg['field_size'][0]) and params['height']==math.ceil(cfg['field_size'][1])
        assert params['track_unknown_space'] is True


def test_unknown_policy_requires_boolean(tmp_path):
    c=config();c['allow_unknown_in_field']='true';p=tmp_path/'cfg.yaml';p.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError,match='allow_unknown_in_field'):load_settings(p)
