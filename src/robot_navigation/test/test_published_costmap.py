"""Complete publication/cache/footprint regressions; never initializes ROS."""
import copy
import json
import math
import time
from types import SimpleNamespace as NS
import pytest
from robot_navigation.core import PublishedCostmapCache,health_valid,load_settings
from robot_navigation.navigation import NavigationServer
from mission_manager.project_paths import project_root
from test_costmap_startup import node_double,message,measured_health

def published(stamp=5.,fill=0):
    m=message(fill);m.header.stamp=NS(sec=int(stamp),nanosec=int(round((stamp%1)*1e9)))
    m.metadata.update_time=NS(sec=0,nanosec=0)  # Actual Jazzy raw publisher layout.
    return m

def test_cache_preserves_complete_bytes_and_uses_publication_time():
    cache=PublishedCostmapCache(clock=lambda:100.)
    m=published();m.data[40*122+40]=254;m.data[40*122+41]=255
    assert cache.accept(m,5.1)
    out,received,seq=cache.snapshot()
    assert received==100 and seq==1 and out.metadata.update_time.sec==5
    assert out.data[40*122+40:40*122+42]==[254,255]
    m.data[:]=[255]*len(m.data)
    assert out.data[0]==0  # Input/callback buffers cannot mutate an accepted version.

@pytest.mark.parametrize('kind',['stale','future','empty','shape','frame'])
def test_cache_rejects_invalid_or_expired_snapshots(kind):
    m=published()
    if kind=='stale':m.header.stamp.sec=1
    if kind=='future':m.header.stamp.sec=6
    if kind=='empty':m.header.stamp.sec=0
    if kind=='shape':m.data=[]
    if kind=='frame':m.header.frame_id='odom'
    cache=PublishedCostmapCache();assert not cache.accept(m,5.1)
    assert cache.snapshot()[0] is None

def test_duplicate_cannot_refresh_received_age_or_overwrite_grid():
    now=[100.];cache=PublishedCostmapCache(clock=lambda:now[0]);cache.accept(published(),5.1)
    now[0]=105.;assert not cache.accept(published(fill=255),5.2)
    assert cache.snapshot()[1]==100. and cache.snapshot()[0].data[0]==0

def test_complete_unknown_publication_is_not_hidden_as_old_safe_map():
    cache=PublishedCostmapCache();cache.accept(published(),5.)
    cache.accept(published(5.5,255),5.5)
    assert all(x==255 for x in cache.snapshot()[0].data)

def test_service_reset_intermediate_not_a_published_snapshot_and_no_false_exit_fault():
    n,maps,packets=node_double(message());n.costmap_observed=True;n.now=lambda:5.2
    cache=PublishedCostmapCache();cache.accept(published(5.1),5.2);n.cache=cache
    # A service request could see the live reset buffer. It is not supplied to this pipeline.
    service_reset=published(5.15,255)
    NavigationServer.publish_status(n)
    assert json.loads(packets[-1].data)['healthy'] and service_reset.data[0]==255
    cache.accept(published(5.2),5.2);NavigationServer.publish_status(n)
    assert json.loads(packets[-1].data)['healthy']

def test_published_source_age_expires_even_when_receipt_is_fresh():
    n,maps,packets=node_double(message());n.costmap_observed=True
    cache=PublishedCostmapCache();cache.accept(published(),5.);n.cache=cache
    n.now=lambda:6.6;n.odom_stamp=6.6;n.base['stamp']['sec']=6;n.base['stamp']['nanosec']=600_000_000
    NavigationServer.publish_status(n)
    assert not json.loads(packets[-1].data)['healthy']

def footprint_node():
    sends=[];subs=[1];n=NS(cfg=load_settings(project_root()/'src/robot_bringup/config/navigation_robot.yaml'),
        base=measured_health(),payload_stamp=math.inf,last_footprint=None,now=lambda:5.,
        footprint_pub=NS(publish=sends.append,get_subscription_count=lambda:subs[0]))
    n.cfg['calibration_verified']=True
    return n,sends,subs

def test_footprint_is_published_once_per_measured_payload_transition():
    n,sends,subs=footprint_node()
    for _ in range(20):NavigationServer.publish_footprint(n)
    assert len(sends)==1 and n.payload_stamp==5.
    n.base['lift_is_up']=True;n.payload_stamp=math.inf
    for _ in range(20):NavigationServer.publish_footprint(n)
    assert len(sends)==2 and sends[-1].points[0].y==pytest.approx(.19)
    n.base['lift_is_up']=False;n.payload_stamp=math.inf
    for _ in range(20):NavigationServer.publish_footprint(n)
    assert len(sends)==3 and sends[-1].points[0].y==pytest.approx(.08)

def test_no_early_mark_when_subscriber_absent_and_resend_on_reconnect():
    n,sends,subs=footprint_node();subs[0]=0
    NavigationServer.publish_footprint(n);assert not sends and n.last_footprint is None
    subs[0]=1;NavigationServer.publish_footprint(n);assert len(sends)==1
    subs[0]=0;NavigationServer.publish_footprint(n)
    subs[0]=1;NavigationServer.publish_footprint(n);assert len(sends)==2

def test_unknown_lift_does_not_publish_empty_footprint():
    n,sends,subs=footprint_node();n.base.update(lift_is_up=None,lift_settled=False)
    NavigationServer.publish_footprint(n);assert not sends

def test_transport_and_full_publication_configuration():
    import yaml
    root=project_root();real=load_settings(root/'src/robot_bringup/config/navigation_robot.yaml')
    hybrid=load_settings(root/'src/robot_bringup/config/navigation_hybrid.yaml')
    assert real['costmap_topic']=='global_costmap/costmap_raw'
    assert hybrid['costmap_topic']=='/global_costmap/costmap_raw'
    for name in ('robot','hybrid'):
        p=yaml.safe_load((root/f'src/robot_bringup/config/costmap_{name}.yaml').read_text())
        assert p['global_costmap']['global_costmap']['ros__parameters']['always_send_full_costmap'] is True
    source=(root/'src/robot_navigation/robot_navigation/navigation.py').read_text()
    assert 'self.create_client(GetCostmap' not in source
    assert 'TRANSIENT_LOCAL' in source
