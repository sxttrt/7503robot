"""Check installed ROS schemas and execution methods without rclpy.init()."""
import math
import threading
import time
from types import SimpleNamespace as NS
from pathlib import Path
import sys
import unittest

from mission_manager.framework_execution import FrameworkExecution
from mission_manager.manual_backend import ManualBackend
from nav2_msgs.action import NavigateToPose
from mission_interfaces.action import Dock
from mission_v3 import MissionV3
from geometry_msgs.msg import Twist
import scene


class EndpointTests(unittest.TestCase):
    def server(self):
        node=FrameworkExecution.__new__(FrameworkExecution)
        node.abort_event=threading.Event();node.claim_lock=threading.Lock()
        node.initialized=True;node.busy=False;node.operation_deadline=None
        node.pose=(2.72,1.,math.pi);node.odom_seen=node.status_seen=node.scan_seen=time.monotonic()
        node.status={'ready':True,'fault':None};node.pub_vel=lambda *a:None
        node.map_from_odom=lambda:(0.,0.,0.)
        node.get_logger=lambda:NS(error=lambda *a:None,info=lambda *a:None)
        return node

    def test_only_one_motion_or_docking_goal_can_claim_chassis(self):
        node=self.server();nav=NavigateToPose.Goal()
        self.assertEqual(node.claim(nav).name,'ACCEPT')
        self.assertEqual(node.claim(nav).name,'REJECT')

    def test_wrong_rack_qr_or_operation_rejected(self):
        node=self.server();request=Dock.Goal()
        request.rack_id='A';request.operation='ENTER';request.max_duration_sec=10.
        request.expected_qr='RACKB_XXXX'
        self.assertEqual(node.claim(request).name,'REJECT');self.assertFalse(node.busy)
        request.expected_qr=scene.RACKS['A'][5];request.operation='TURN'
        self.assertEqual(node.claim(request).name,'REJECT')

    def test_cancel_stops_execution_and_result_releases_slot(self):
        node=self.server();node.busy=True
        calls=[];handle=NS(is_cancel_requested=True,canceled=lambda:calls.append('cancelled'))
        node.cancel(handle);self.assertTrue(node.abort_event.is_set())
        result=Dock.Result();self.assertIs(node.finish(handle,result,False),result)
        self.assertEqual(calls,['cancelled']);self.assertFalse(node.busy)

    def test_result_publication_exception_cannot_keep_chassis_claimed(self):
        node=self.server();node.busy=True
        def fail():raise RuntimeError('transport disconnected')
        handle=NS(is_cancel_requested=False,abort=fail)
        with self.assertRaises(RuntimeError):node.finish(handle,Dock.Result(),False)
        self.assertFalse(node.busy)

    def test_bad_frame_returns_real_navigation_failure(self):
        node=self.server();node.busy=True
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='unknown'
        calls=[];handle=NS(request=goal,is_cancel_requested=False,abort=lambda:calls.append('abort'))
        result=node.navigate(handle)
        self.assertNotEqual(result.error_code,0);self.assertIn('map',result.error_msg)
        self.assertEqual(calls,['abort']);self.assertFalse(node.busy)

    def test_heading_restore_precedes_translation_after_start(self):
        node=self.server();node.busy=True;node.current_payload=None
        node.pose=(2.55,1.,-math.pi+math.pi/6)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.pose.position.x=1.541;goal.pose.pose.position.y=.54
        goal.pose.pose.orientation.z=1.;goal.pose.pose.orientation.w=0.
        events=[]
        node.move_target=lambda target:events.append(('translate',node.pose[2])) or True
        def turn(target):
            events.append(('turn',target));node.pose=(*node.pose[:2],target);return True
        node.turn=turn
        handle=NS(request=goal,is_cancel_requested=False,succeed=lambda:None)
        result=node.navigate(handle)
        self.assertEqual(result.error_code,0)
        self.assertEqual([item[0] for item in events],['turn','translate','turn'])
        self.assertAlmostEqual(events[0][1],math.pi)
        self.assertAlmostEqual(events[1][1],math.pi)

    def test_failed_heading_restore_cannot_start_translation(self):
        node=self.server();node.busy=True;node.current_payload=None
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.pose.orientation.z=1.;goal.pose.pose.orientation.w=0.
        node.turn=lambda target:False
        node.move_target=lambda target:self.fail('translated without confirmed heading')
        handle=NS(request=goal,is_cancel_requested=False,abort=lambda:None)
        result=node.navigate(handle)
        self.assertNotEqual(result.error_code,0)
        self.assertIn('heading restore',result.error_msg)

    def test_image_callback_only_keeps_latest_frame_without_decoding(self):
        node=self.server();node.image_lock=threading.Lock();node.pending_image=None;node.use_builtin_qr=True
        node.process_image=lambda message:self.fail('decode blocked ROS callback')
        first=object();second=object()
        node.on_image(first);node.on_image(second)
        self.assertIs(node.pending_image,second)

    def test_loaded_heading_noise_at_old_threshold_never_commands_rotation(self):
        for error in (.0118,.0121,.0125,.019):
            node=self.server();node.current_payload='B'
            node.pose=(.2632,1.0495,-math.pi+error)
            node.wait_still=lambda center:True
            node.cmd_pub=NS(publish=lambda msg:self.fail('loaded rotation published'))
            node.spin=lambda *a:self.fail('loaded heading entered rotation loop')
            self.assertTrue(node.turn(math.pi),error)

    def test_loaded_heading_outside_transport_limit_stops_without_rotating(self):
        node=self.server();node.current_payload='B'
        node.pose=(.2632,1.0495,-math.pi+.03);node.wait_still=lambda center:True
        node.cmd_pub=NS(publish=lambda msg:self.fail('loaded rotation published'))
        self.assertFalse(node.turn(math.pi))

    def test_loaded_scan_turn_and_failed_stop_cannot_succeed(self):
        node=self.server();node.current_payload='B'
        node.cmd_pub=NS(publish=lambda msg:self.fail('loaded rotation published'))
        node.wait_still=lambda center:True
        self.assertFalse(node.turn(math.pi+math.pi/6))
        node.wait_still=lambda center:False
        self.assertFalse(node.turn(math.pi))

    def test_operation_timeout_blocks_further_velocity(self):
        node=self.server();node.busy=True;node.operation_deadline=time.monotonic()-1
        self.assertFalse(node.fresh());node.busy=False
        self.assertTrue(node.fresh())


if __name__=='__main__':unittest.main()
