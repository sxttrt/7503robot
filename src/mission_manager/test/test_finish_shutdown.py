from types import SimpleNamespace as NS
from mission_manager.mission_node import MissionNode


def node(state,confirmed):
    markers=[]
    result=NS(auto_arm=False,closing=False,config={'mode':'simulation','mission':{'shutdown_on_finish':True}},
              mission=NS(state=state,final_confirmed=confirmed,final_wait_timed_out=False,
                         stop_status='CONFIRMED',reason='END 已确认',tick=lambda:None),
              publish_status=lambda:None,get_logger=lambda:NS(info=markers.append))
    return result,markers


def test_final_confirmation_closes_only_after_stopped_finished_state():
    instance,markers=node('STOPPING',True)
    MissionNode.on_tick(instance)
    assert not instance.closing and not markers
    instance.mission.state='FINISHED'
    MissionNode.on_tick(instance)
    assert instance.closing and markers[0].startswith('FRAMEWORK_FINISHED:')


def test_fault_or_time_limit_does_not_close_run_as_confirmed_finish():
    for state,confirmed in [('FAULT',True),('FINISHED',False)]:
        instance,markers=node(state,confirmed)
        MissionNode.on_tick(instance)
        assert not instance.closing and not markers
