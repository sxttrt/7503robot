"""Offline checks only: no rclpy.init, nodes, publishers or hardware processes."""
import ast
import math
from types import SimpleNamespace as NS
from pathlib import Path
from unittest.mock import patch
import pytest
from robot_navigation.chassis_test import ForwardProfile,forward_command,ChassisTest
from mission_manager.project_paths import project_root


def profile(**changes):
    values=dict(speed=.05,duration=5.,acceleration=.3,wait_timeout=5.,command_timeout=.3)
    values.update(changes);return ForwardProfile(**values)


@pytest.mark.parametrize('change',[{'speed':-.05},{'speed':float('nan')},{'speed':True},
    {'speed':.11},{'duration':31.},{'duration':0.},{'acceleration':float('inf')},
    {'wait_timeout':0.},{'command_timeout':-.1}])
def test_invalid_test_motion_rejected(change):
    with pytest.raises(ValueError):profile(**change)


def test_forward_schedule_ramps_runs_brakes_and_holds_zero():
    p=profile();samples=[]
    for i in range(131):
        stamp=i*.05;samples.append((stamp,p.step(stamp,1,1)))
    assert p.done and not p.failed
    assert all(0<=speed<=.05 for _,speed in samples)
    assert all(speed==0 for stamp,speed in samples if stamp<.5 or stamp>=5.5)
    assert samples[11][1]==pytest.approx(.015)
    assert samples[20][1]==pytest.approx(.05)
    assert samples[109][1]==pytest.approx(.015)
    # Total expected displacement is bounded and close to 24 cm, not a measured distance.
    distance=sum((a[1]+b[1])*.5*(b[0]-a[0]) for a,b in zip(samples,samples[1:]))
    assert .23<distance<.25


def test_absent_subscriber_never_gets_nonzero_command():
    p=profile(wait_timeout=1.)
    outputs=[p.step(i*.05,0,1) for i in range(33)]
    assert set(outputs)=={0.} and p.failed and p.done


@pytest.mark.parametrize('condition', ['conflict','disconnected','interrupt','stall'])
def test_active_motion_aborts_to_zero(condition):
    p=profile()
    for i in range(21):p.step(i*.05,1,1)
    assert p.step(1.05,1,1)>0.
    now=1.5 if condition=='stall' else 1.1
    assert p.step(now,0 if condition=='disconnected' else 1,
        2 if condition=='conflict' else 1,condition=='interrupt')==0.
    assert p.phase=='STOPPING' and p.reason
    assert p.failed==(condition!='interrupt')
    for i in range(1,15):assert p.step(now+i*.05,1,1)==0.
    assert p.done


def test_conflicting_publisher_rejected_before_movement():
    p=profile()
    assert p.step(0.,1,2)==0.
    assert p.started is None and p.failed


def test_command_contract_is_body_forward_only():
    msg=forward_command(.05)
    assert msg.linear.x==.05
    assert (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)==(0.,)*5


def test_csv_records_each_command_and_blanks_stale_feedback():
    rows=[];commands=[]
    node=NS(command=NS(publish=commands.append),get_clock=lambda:NS(now=lambda:NS(nanoseconds=10**10)),
        cfg={'feedback_timeout_sec':.6},odom=((1.,2.,math.pi),(.04,0.,0.)),
        odom_received=5.,odom_stamp=9.9,origin=4.,profile=NS(phase='CRUISING',reason=''),
        writer=NS(writerow=rows.append),stream=NS(flush=lambda:None))
    ChassisTest.publish(node,.05,5.1,1)
    ChassisTest.publish(node,.05,5.8,1)
    assert len(commands)==len(rows)==2 and all(len(row)==17 for row in rows)
    assert rows[0][3:6]==(.05,0.,0.) and rows[0][7] is True
    assert rows[0][10:16]==(1.,2.,math.pi,.04,0.,0.)
    assert rows[1][7] is False and rows[1][10:16]==('',)*6


def test_final_zero_burst_precedes_file_close_without_ros_shutdown():
    commands=[];events=[];rows=[]
    node=NS(profile=profile(),command=NS(publish=commands.append,get_subscription_count=lambda:1),
        cfg={'tracking_frequency':20.,'feedback_timeout_sec':.6},odom=None,odom_received=0.,
        odom_stamp=-math.inf,origin=0.,get_clock=lambda:NS(now=lambda:NS(nanoseconds=10**10)),
        writer=NS(writerow=rows.append),stream=NS(flush=lambda:events.append('flush'),close=lambda:events.append('close')))
    node.publish=lambda *args:ChassisTest.publish(node,*args)
    with patch('robot_navigation.chassis_test.rclpy.ok',return_value=True),patch('robot_navigation.chassis_test.time.sleep'):
        ChassisTest.close(node)
    assert len(commands)==10 and all(c.linear.x==0 for c in commands)
    assert len(rows)==10 and all(row[3:6]==(0.,0.,0.) for row in rows)
    assert events[-2:]==['flush','close']

def test_graph_discovery_must_confirm_unique_publisher_before_motion():
    p=profile()
    for i in range(20):assert p.step(i*.05,1,0)==0.
    assert p.started is None
    assert p.step(1.,1,1)==0.
    assert p.step(1.05,1,1)>0.

@pytest.mark.parametrize('speed',[float('nan'),float('inf'),-.01,.11])
def test_invalid_forward_command_rejected(speed):
    with pytest.raises(ValueError):forward_command(speed)

def test_main_cleans_up_while_context_still_valid():
    from robot_navigation.chassis_test import main
    events=[]
    fake=NS(profile=NS(done=True,failed=False,reason='completed'),
        close=lambda:events.append('zero_burst'),destroy_node=lambda:events.append('destroy'),
        get_logger=lambda:NS(info=lambda text:None))
    with patch('robot_navigation.chassis_test.signal.signal'), \
         patch('robot_navigation.chassis_test.rclpy.init',side_effect=lambda **kw:events.append('init')), \
         patch('robot_navigation.chassis_test.rclpy.ok',return_value=True), \
         patch('robot_navigation.chassis_test.ChassisTest',return_value=fake), \
         patch('robot_navigation.chassis_test.rclpy.try_shutdown',side_effect=lambda:events.append('shutdown')):
        assert main()==0
    assert events==['init','zero_burst','destroy','shutdown']
