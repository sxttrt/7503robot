import time
from types import SimpleNamespace as NS
import pytest
from mission_manager.adapters.safety import Safety
from test_state_machine import advance_to
from test_qr_timeout_advance import system

def safety_sample(**changes):
    data={'stamp':{'sec':10,'nanosec':0},'stopped':True,'lift_state_source':'measured',
          'base_motion_stopped':True,'lift_motion_stopped':True,'lift_is_up':None,
          'modules':{'navigation':False,'qr':True,'docking':False,'base':False,'lift':False},
          'fault':'drive fault'}
    data.update(changes);return data

def safety_double(changes=None):
    events=[]
    safety=Safety.__new__(Safety)
    safety.latest={'ready':False,'fault':'drive fault'}
    safety.received_at=time.monotonic();safety.diagnostic=''
    safety.policy={'health_timeout_sec':.6}
    safety.pending={'request':'stop1','callback':lambda *a:events.append(a)}
    return safety,events

def test_faulted_modules_can_confirm_independent_measured_stop():
    safety,events=safety_double();assert not safety.healthy()
    record=safety.pending
    safety.on_stop_response(record,NS(result=lambda:NS(success=True,message='physical stop confirmed')))
    assert len(events)==1 and events[0][1] is True and safety.pending is None
    assert not safety.healthy()  # Stop completion is not permission to move.

@pytest.mark.parametrize('changes',[{'base_motion_stopped':False},{'lift_motion_stopped':False},
    {'lift_state_source':'estimated'},{'stopped':False}])
def test_stop_needs_both_actual_motion_measurements(changes):
    # 测量检查属于执行端；框架只接受执行端的实际完成响应。
    safety,events=safety_double(changes)
    safety.on_stop_response(safety.pending,NS(result=lambda:NS(success=False,message='execution stop unconfirmed')))
    assert events[0][1] is False and safety.pending is None

def test_old_stop_sample_or_unfinished_actions_cannot_complete_stop():
    safety,events=safety_double();old=safety.pending
    safety.pending={'request':'new','callback':lambda *a:events.append(a)}
    safety.on_stop_response(old,NS(result=lambda:NS(success=True,message='old result')))
    assert not events and safety.pending['request']=='new'

def test_stop_failure_preserves_original_action_fault_reason():
    mission,backend,clock=system();advance_to(mission,backend,'LIFT_DOWN')
    backend.finish(False,{'reason':'physical lift fault z=0.0263'});mission.tick()
    assert mission.state=='STOPPING'
    backend.confirm_stop(False);mission.tick()
    assert mission.state=='FAULT' and 'physical lift fault z=0.0263' in mission.reason and '停止失败' in mission.reason

def test_successful_stop_cannot_retry_while_module_remains_faulted():
    mission,backend,clock=system();advance_to(mission,backend,'NAV_RACK')
    backend.finish(False,{'reason':'navigation feedback error'});mission.tick()
    assert mission.state=='STOPPING';before=len(backend.calls)
    backend.is_healthy=False;backend.confirm_stop();mission.tick()
    assert mission.state=='FAULT' and len(backend.calls)==before
