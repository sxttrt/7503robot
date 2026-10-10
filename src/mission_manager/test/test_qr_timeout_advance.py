"""仿真扫码超时推进的行为验证；不创建 ROS 节点。"""
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from mission_manager.config_loader import load_config,validate
from mission_manager.state_machine import Mission
from mission_manager.mission_node import MissionNode
from test_state_machine import Clock,Backend,advance_to

CONFIG=Path(__file__).resolve().parents[2]/'robot_bringup/config'

def system():
    config=load_config(CONFIG/'mission_rack.yaml',CONFIG/'interfaces.yaml',CONFIG/'targets_rack.yaml','simulation')
    clock=Clock();backend=Backend();mission=Mission(config,backend,clock)
    return mission,backend,clock

@pytest.mark.parametrize('state,next_state',[('WAIT_START','NAV_RACK'),('WAIT_RACK_QR','DOCK_ENTER'),
    ('WAIT_END_QR','LIFT_DOWN'),('WAIT_FINAL_QR','STOPPING')])
def test_wait_timeout_advances_and_cancels_old_qr_only(state,next_state):
    mission,backend,clock=system();mission.policy['confirm_end_qr']=True
    advance_to(mission,backend,state);old=backend.calls[-1]
    clock.now=mission.deadline;mission.tick()
    assert mission.state==next_state and ('qr',old[1]) in backend.cancelled
    assert '超时' in mission.reason
    if state=='WAIT_START':assert mission.started_at==clock.now
    if state=='WAIT_FINAL_QR':
        assert not mission.final_confirmed and mission.final_wait_timed_out
        assert mission.state!='FINISHED'
        backend.confirm_stop();mission.tick();assert mission.state=='FINISHED'
    # 旧扫码回调不会再越过一个阶段；正常下一阶段完成仍要自己的请求编号。
    current=mission.state;backend.finish(call=old);mission.tick()
    assert mission.state==current

def test_four_racks_finish_in_order_without_publishing_any_qr():
    mission,backend,clock=system();mission.arm()
    for _ in range(70):
        if mission.state=='FINISHED':break
        if mission.state.startswith('WAIT_'):clock.now=mission.deadline
        elif mission.state=='STOPPING':backend.confirm_stop()
        else:backend.finish()
        mission.tick()
    assert mission.state=='FINISHED' and mission.completed==['A','B','C','D']
    assert mission.delivered==['A','B','C','D'] and mission.cargo=='EMPTY'
    assert mission.final_wait_timed_out and not mission.final_confirmed

def test_qr_before_timeout_still_immediately_advances():
    mission,backend,clock=system();advance_to(mission,backend,'WAIT_RACK_QR')
    backend.finish();mission.tick()
    assert mission.state=='DOCK_ENTER' and not backend.stops and not backend.cancelled

@pytest.mark.parametrize('state',['NAV_RACK','DOCK_ENTER','LIFT_UP','NAV_END','LIFT_DOWN','DOCK_EXIT'])
def test_motion_timeout_cannot_be_treated_as_completion(state):
    mission,backend,clock=system();advance_to(mission,backend,state)
    clock.now=mission.deadline;mission.tick()
    assert mission.state=='STOPPING' and backend.stops

def test_health_fault_beats_qr_timeout():
    mission,backend,clock=system();advance_to(mission,backend,'WAIT_RACK_QR')
    backend.is_healthy=False;clock.now=mission.deadline;mission.tick()
    assert mission.state=='STOPPING' and '必需模块' in mission.reason

def test_stop_failure_at_end_is_fault_not_fake_finished():
    mission,backend,clock=system();advance_to(mission,backend,'WAIT_FINAL_QR')
    clock.now=mission.deadline;mission.tick();backend.confirm_stop(False);mission.tick()
    assert mission.state=='FAULT'

def test_strict_policy_still_retries_qr_failure():
    mission,backend,clock=system();mission.policy['qr_timeout_advance']=False
    advance_to(mission,backend,'WAIT_RACK_QR');clock.now=mission.deadline;mission.tick()
    assert mission.state=='STOPPING' and mission.stop_plan[0]=='WAIT_RACK_QR'

def test_node_closes_auto_finish_only_after_stop_confirmation():
    mission,backend,clock=system();advance_to(mission,backend,'WAIT_FINAL_QR')
    clock.now=mission.deadline;mission.tick();logs=[]
    node=NS(auto_arm=False,closing=False,config=mission.config,mission=mission,
            publish_status=lambda:None,get_logger=lambda:NS(info=logs.append))
    MissionNode.on_tick(node);assert not node.closing
    backend.confirm_stop();MissionNode.on_tick(node)
    assert node.closing and 'END 等待超时' in logs[-1]

def test_auto_advance_flag_rejects_nonboolean_and_robot_mode():
    mission,backend,clock=system();mission.policy['qr_timeout_advance']='true'
    with pytest.raises(ValueError,match='qr_timeout_advance'):validate(mission.config,'simulation')
    mission.policy['qr_timeout_advance']=True
    mission.policy['game_duration_sec']=180
    with pytest.raises(ValueError,match='超时放行'):validate(mission.config,'robot')
