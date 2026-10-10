"""Offline contract checks; never initializes a ROS context or launches nodes."""
from pathlib import Path
from types import SimpleNamespace as NS
import math
import pytest
from mission_manager.config_loader import load_config
from mission_manager.state_machine import Mission
from mission_manager.manual_backend import ManualBackend
from mission_manager.config_loader import read_yaml
from robot_navigation.point_targets import PointTargets

ROOT=Path(__file__).resolve().parents[3]
CONFIG=ROOT/'src/robot_bringup/config'


def config():
    return load_config(CONFIG/'mission_rack.yaml',CONFIG/'interfaces.yaml',CONFIG/'targets_rack.yaml','simulation')


class Backend:
    def __init__(self):self.calls=[];self.cancelled=[];self.stops=[]
    def ready(self):return True
    def healthy(self):return True
    def poll(self):pass
    def start(self,*args):self.calls.append(args)
    def cancel(self,*args):self.cancelled.append(args)
    def stop(self,*args):self.stops.append(args)
    def finish(self,mission,details=None):
        kind,request,payload,callback=self.calls[-1]
        if details is None:details={'ready_to_lift':True,'exited':True,'is_up':payload.get('up')}
        callback(request,True,details);mission.tick()


def points():return read_yaml(CONFIG/'navigation_points_rack.yaml')


def test_actual_framework_config_and_per_rack_drop_targets():
    c=config();assert c['mission']['rack_order']==['A','B','C','D']
    assert all(set(v)=={'qr'} for v in c['targets']['racks'].values())
    assert c['mission']['manual_step'] is False
    assert c['mission']['confirm_end_qr'] is False
    assert math.isclose(points()['points']['START_SCAN'][2],math.pi+math.pi/6)
    assert len({tuple(item) for item in points()['drop_by_rack'].values()})==4


def test_start_and_final_scan_navigation_and_delivery_sequence():
    backend=Backend();mission=Mission(config(),backend,clock=lambda:1.)
    assert mission.arm()[0];assert mission.state=='NAV_START'
    backend.finish(mission);assert mission.state=='WAIT_START'
    backend.finish(mission);assert mission.state=='NAV_RACK'
    for name in ['A','B','C','D']:
        assert mission.rack==name
        backend.finish(mission);assert mission.state=='WAIT_RACK_QR'
        backend.finish(mission);assert mission.state=='DOCK_ENTER'
        backend.finish(mission);assert mission.state=='LIFT_UP'
        backend.finish(mission);assert mission.state=='NAV_END'
        assert backend.calls[-1][2]['target_id']=='DROP_OFF'
        backend.finish(mission);assert mission.state=='LIFT_DOWN'
        backend.finish(mission);assert mission.state=='DOCK_EXIT'
        backend.finish(mission)
    assert mission.state=='NAV_FINAL'
    backend.finish(mission);assert mission.state=='WAIT_FINAL_QR'
    backend.finish(mission);assert mission.state=='STOPPING'
    request,callback=backend.stops[-1];callback(request,True,{});mission.tick()
    assert mission.state=='FINISHED';assert mission.completed==['A','B','C','D']
    assert mission.final_confirmed is True


def manual_backend():
    backend=ManualBackend.__new__(ManualBackend)
    backend.waiting=None;backend.node=NS(closing=False)
    backend.navigation=NS(node=NS(get_logger=lambda:NS(info=lambda *args:None)))
    operations=[];backend.modules={'navigation':NS(start=lambda *a:operations.append(a)),
                                 'qr':NS(start=lambda *a:operations.append(a)),
                                 'docking':NS(start=lambda *a:operations.append(a))}
    return backend,operations


def test_start_qr_always_uses_framework_topic_even_in_motion_debug_mode():
    backend,operations=manual_backend();results=[]
    backend.start('qr','start-1',{'phase':'WAIT_START','expected_qr':'START'},lambda *a:results.append(a))
    assert len(operations)==1 and not results and backend.waiting is None
    assert not backend.release()
    operations[-1][2]('start-1',True,{'raw':'START'})
    assert results[0][2]['raw']=='START'


def test_next_rack_navigation_is_automatic_and_duplicates_do_not_queue():
    backend,operations=manual_backend()
    backend.start('navigation','first',{'phase':'NAV_RACK'},lambda *a:None)
    assert len(operations)==1 and backend.waiting is None
    assert not backend.release()
    backend.start('navigation','second',{'phase':'NAV_RACK'},lambda *a:None)
    assert len(operations)==2 and backend.waiting is None


def test_start_confirmation_immediately_dispatches_first_rack_navigation():
    backend,operations=manual_backend()
    backend.ready=lambda:True;backend.healthy=lambda:True;backend.poll=lambda:None
    mission=Mission(config(),backend,clock=lambda:1.)
    assert mission.arm()[0];assert backend.waiting[2]['phase']=='NAV_START'
    assert backend.release()
    request,payload,callback=operations[-1];callback(request,True,{});mission.tick()
    assert mission.state=='WAIT_START' and backend.waiting is None
    request,payload,callback=operations[-1];callback(request,True,{'raw':'START'});mission.tick()
    assert mission.state=='NAV_RACK' and backend.waiting is None
    assert operations[-1][1]['phase']=='NAV_RACK'
    assert not backend.release()


def test_drop_slots_are_left_to_right_inner_before_outer():
    targets=points()['drop_by_rack']
    assert [targets[n][:2] for n in 'ABCD']==[[.25,.42],[.25,1.05],[.55,.7],[.55,1.42]]


def test_cancelling_gate_cannot_release_old_request():
    backend,operations=manual_backend()
    backend.start('navigation','old',{'phase':'NAV_START'},lambda *a:None)
    backend.cancel('navigation','old')
    assert not backend.release() and not operations


def test_scene_coordinates_and_model_heading_are_not_map_or_spec_yaw():
    from mission_manager.framework_execution import scene
    targets=points()
    assert scene.MAP_FRAME=='map'
    for name in 'ABCD':
        assert targets['points'][name][:2]==list(scene.scan(name))
        assert targets['drop_by_rack'][name][:2]==list(scene.DROP[name])
        assert targets['points'][name][2]==scene.MODEL_YAW==math.pi


def test_unloaded_exit_waits_for_trigger_then_next_navigation_is_automatic():
    backend,operations=manual_backend()
    backend.start('docking','exit-A',{'phase':'DOCK_EXIT','operation':'EXIT'},lambda *a:None)
    assert not operations and backend.waiting is not None
    assert backend.release();assert operations[-1][0]=='exit-A'
    backend.start('navigation','next-B',{'phase':'NAV_RACK'},lambda *a:None)
    assert backend.waiting is None and operations[-1][0]=='next-B'
    assert not backend.release()


def test_final_navigation_auto_runs_but_end_confirmation_waits():
    backend,operations=manual_backend();results=[]
    backend.start('navigation','final',{'phase':'NAV_FINAL'},lambda *a:None)
    assert backend.waiting is None and operations[-1][0]=='final'
    backend.start('qr','end',{'phase':'WAIT_FINAL_QR','expected_qr':'END'},lambda *a:results.append(a))
    assert not results and backend.waiting is None
    assert operations[-1][0]=='end'
    operations[-1][2]('end',True,{'raw':'END'})
    assert results[0][0]=='end' and results[0][1] is True
    assert not backend.release()


def test_time_limit_does_not_count_as_operator_final_confirmation():
    backend=Backend();mission=Mission(config(),backend,clock=lambda:1.)
    assert mission.final_confirmed is False
    mission.started_at=-10000.;mission.state='NAV_RACK'
    mission.tick();assert mission.state=='STOPPING'
    request,callback=backend.stops[-1];callback(request,True,{});mission.tick()
    assert mission.state=='FINISHED' and mission.final_confirmed is False


def test_invalid_drop_coordinate_rejected():
    c=points();c['drop_by_rack']['A']=[1.,float('nan'),0.]
    with pytest.raises(ValueError):PointTargets(c)
