"""Offline regression tests; no ROS imports or simulator processes."""
import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT/'scripts'))
import scene
from targets import build,fit_ok
from grid_planner import GridMap,Planner


def grid(blocks=(),transform=(0.,0.,0.),origin=(0.,0.),size=(140,100)):
    meta=NS(resolution=.025,origin=NS(position=NS(x=origin[0],y=origin[1]),
             orientation=NS(x=0.,y=0.,z=0.,w=1.)),size_x=size[0],size_y=size[1])
    data=np.zeros((size[1],size[0]),dtype=np.uint8)
    for x,y,cost in blocks:data[y,x]=cost
    return GridMap(meta,data.ravel(),transform)


class PlannerTests(unittest.TestCase):
    def planner(self,blocks=()):return Planner(grid(blocks),(.12,.08),(3.02,2.))
    def assert_path(self,p,start,goal,path):
        self.assertIsNotNone(path)
        for point in path:
            self.assertTrue(p.segment_free(start,point),(start,point))
            self.assertTrue(abs(start[0]-point[0])<1e-8 or abs(start[1]-point[1])<1e-8)
            start=point
        self.assertEqual(start,goal)
    def test_free_map_exact_two_segments(self):
        p=self.planner();start=(2.720,1.000);goal=(1.541,1.460)
        path=p.plan(start,goal);self.assertEqual(len(path),2)
        self.assert_path(p,start,goal,path)
    def test_already_at_goal(self):
        self.assertEqual(self.planner().plan((.525,.525),(.525,.525)),[])
    def test_blocked_goal_never_snapped(self):
        p=self.planner([(29,21,254)])
        self.assertIsNone(p.plan((.225,.525),(.725,.525)))
    def test_full_footprint_front_collision(self):
        p=self.planner([(25,20,254)])
        # Obstacle 0.10m ahead, outside empty inscribed radius .08 but inside front .12.
        self.assertFalse(p.segment_free((.525,.525),(.525,.525)))
    def test_soft_inflation_not_wall(self):
        self.assertTrue(self.planner([(25,20,186)]).segment_free((.525,.525),(.525,.525)))
    def test_unknown_rejected(self):
        self.assertFalse(self.planner([(25,20,255)]).segment_free((.525,.525),(.525,.525)))
    def test_exact_swept_corridor(self):
        blocks=[(40,y,254) for y in range(10,50)]
        p=self.planner(blocks);start=(.4,.8);goal=(1.6,.8)
        path=p.plan(start,goal);self.assert_path(p,start,goal,path)
        self.assertGreater(len(path),2)
    def test_refresh_detects_new_obstacle(self):
        a,b=(.4,.8),(1.6,.8)
        self.assertTrue(self.planner().segment_free(a,b))
        self.assertFalse(self.planner([(40,32,254)]).segment_free(a,b))
    def test_witnessed_obstacles_prevent_alternating_corridor_retry(self):
        start=(1.0076,.71425);goal=(2.34,.39)
        low=grid([(70,16,254)]);high=grid([(70,29,254)])
        # Each changing snapshot individually offers the other corridor.
        blockers=[]
        for observed,lane in ((low,.39),(high,.725)):
            box=(start[0]-.17,goal[0]+.17,lane-.13,lane+.13)
            blockers.extend(observed.blocking_world_rects(box))
        p=Planner(grid(),(.12,.08),(3.02,2.),blockers,margin=.058)
        self.assertFalse(p.segment_free((start[0],.39),(goal[0],.39)))
        self.assertFalse(p.segment_free((start[0],.725),(goal[0],.725)))
        path=p.plan(start,goal)
        self.assert_path(p,start,goal,path)

    def test_witnessed_cells_transform_back_from_rotated_map(self):
        g=grid([(80,60,254)],transform=(2.5,1.,math.pi/2))
        cells=g.blocking_world_rects((.35,.7,.25,.7))
        self.assertEqual(cells,[(.5,.525,.475,.5)])

    def test_map_world_translation(self):
        p=Planner(grid([(60,40,254)],transform=(1.,.5,0.)),(.12,.08),(3.02,2.))
        self.assertFalse(p.segment_free((.525,.525),(.525,.525)))
    def test_rotated_map_world(self):
        p=Planner(grid([(80,60,254)],transform=(2.5,1.,math.pi/2)),(.12,.08),(3.02,2.))
        self.assertFalse(p.segment_free((.525,.475),(.525,.475)))
    def test_field_boundary_footprint(self):
        self.assertIsNone(self.planner().plan((.10,1.),(.6,1.)))


class SceneTests(unittest.TestCase):
    def test_racks_reflect_about_initial_forward_axis(self):
        original={
            'A':(1.051,1.221,1.271,1.649,1.46),
            'B':(1.756,1.920,.430,.811,.6205),
            'C':(1.252,1.422,.130,.511,.28),
            'D':(1.853,2.020,1.421,1.799,1.61),
        }
        axis=scene.START_POSE[1]
        world=ET.parse(ROOT/'worlds/rack_world.sdf').getroot().find('world')
        targets=build()
        for name,(x0,x1,y0,y1,sy) in original.items():
            actual=scene.RACKS[name]
            expected=(x0,x1,2*axis-y1,2*axis-y0,2*axis-sy)
            for a,b in zip(actual[:5],expected):self.assertAlmostEqual(a,b)
            model=world.find(f"model[@name='rack_{name.lower()}']")
            position=list(map(float,model.findtext('pose').split()))
            self.assertAlmostEqual(position[0],(x0+x1)/2)
            self.assertAlmostEqual(position[1],2*axis-(y0+y1)/2)
            for kind in ('scan','under'):
                target=next(t for t in targets if t['name']==f'{kind}_{name}')
                expected_y=2*axis-sy if kind=='scan' else 2*axis-(y0+y1)/2
                self.assertAlmostEqual(target['xy'][1],expected_y)
        self.assertEqual(scene.START_POSE[:2],(2.72,1.))
        self.assertEqual(scene.DROP,{'A':(.25,.42),'B':(.25,1.05),'C':(.55,.7),'D':(.55,1.42)})

    def test_framework_identity_order_is_abcd(self):
        targets=build();drops=[t for t in targets if t['name'].startswith('drop')]
        self.assertEqual([t['rack'] for t in drops],['A','B','C','D'])
        self.assertEqual([t['xy'][0] for t in drops],[.25,.25,.55,.55])
        self.assertEqual([t['xy'][1] for t in drops],[.42,1.05,.7,1.42])
        scans=[t for t in targets if t['name'].startswith('scan')]
        self.assertEqual([t['auto_release'] for t in scans],[False,True,True,True])
        self.assertFalse(any(t.get('auto_release') for t in targets if not t['name'].startswith('scan')))

    def test_car_and_payload_clearance_exceeds_rack_outline(self):
        hx,hy=max(scene.ROBOT_L,scene.RACK_D)/2,scene.RACK_L/2
        self.assertGreater(scene.inflation_radius(hx,hy),math.hypot(scene.RACK_D/2,scene.RACK_L/2))
        self.assertEqual(scene.COLLISION_MARGIN,.05)

    def test_targets(self):
        targets=build();self.assertEqual(len(targets),13)
        for t in targets:
            if t['name'].startswith('drop'):self.assertEqual(t['xy'],scene.DROP[t['rack']])
        x,y=targets[-1]['xy'];self.assertAlmostEqual(x,scene.END_ZONE[1]+scene.ROBOT_L/2+.01)
        self.assertTrue(scene.END_ZONE[2]<y<scene.END_ZONE[3]);self.assertTrue(fit_ok())
    def test_all_thirteen_targets_with_placed_shelves(self):
        # All shelf geometry included, even if hidden from radar in this synthetic map.
        position=scene.START_POSE[:2];placed={};payload=None;offset=(0.,0.)
        for t in build():
            kind=t['name'].split('_')[0];name=t.get('rack')
            hx,hy=scene.ROBOT_L/2,scene.ROBOT_W/2
            if payload:
                r=scene.RACKS[payload];hx=max(hx,(r[1]-r[0])/2+abs(offset[0]));hy=max(hy,(r[3]-r[2])/2+abs(offset[1]))
            obstacles=[]
            for n,r in scene.RACKS.items():
                if n==payload:continue
                cx,cy=placed.get(n,scene.rack_model_center(n))
                if payload:obstacles.append((cx-(r[1]-r[0])/2,cx+(r[1]-r[0])/2,cy-(r[3]-r[2])/2,cy+(r[3]-r[2])/2))
                else:obstacles.extend(scene.rack_leg_rects(n,(cx,cy)))
            p=Planner(grid(),(hx,hy),(scene.FIELD_X,scene.FIELD_Y),obstacles,
                      margin=scene.COLLISION_MARGIN+scene.TRACKING_TOL)
            goal=t['xy']
            if kind=='drop':goal=(goal[0]-offset[0],goal[1]-offset[1])
            if kind!='under':
                path=p.plan(position,goal)
                self.assertIsNotNone(path,t['name'])
                PlannerTests().assert_path(p,position,goal,path)
            position=goal
            if kind=='under':
                cx,cy=scene.rack_model_center(name);offset=(cx-position[0],cy-position[1]);payload=name
            elif kind=='drop':
                placed[name]=(position[0]+offset[0],position[1]+offset[1]);payload=None
                self.assertEqual(placed[name],scene.DROP[name]);position=(position[0]+scene.EXIT_DIST,position[1])
    def test_final_scan_rotation_with_all_delivered_racks(self):
        angle=math.pi/6
        hx=scene.ROBOT_L/2*math.cos(angle)+scene.ROBOT_W/2*math.sin(angle)
        hy=scene.ROBOT_L/2*math.sin(angle)+scene.ROBOT_W/2*math.cos(angle)
        obstacles=[rect for name in scene.ORDER for rect in scene.rack_leg_rects(name,scene.DROP[name])]
        p=Planner(grid(),(hx,hy),(scene.FIELD_X,scene.FIELD_Y),obstacles,
                  margin=scene.COLLISION_MARGIN+scene.TRACKING_TOL)
        self.assertTrue(p.segment_free(scene.END_SCAN,scene.END_SCAN))

    def test_generated_world(self):
        world=ET.parse(ROOT/'worlds/rack_world.sdf').getroot().find('world')
        robot=world.find("model[@name='robot']")
        lift=robot.find("link[@name='lift_platform']")
        z=float(lift.findtext('pose').split()[2]);h=float(lift.findtext('collision/geometry/box/size').split()[2])
        self.assertAlmostEqual(z+h/2,scene.PLATFORM_TOP)
        self.assertNotIn('imu',[n.attrib['type'] for n in robot.findall('.//sensor')])
        for n in scene.RACKS:
            rack=world.find(f"model[@name='rack_{n.lower()}']")
            self.assertEqual(len(rack.findall('joint')),len(rack.findall('link'))-1)
            for link in rack.findall('link'):self.assertEqual(link.findtext('gravity'),'false')
            texture=Path(rack.findtext("link[@name='qr']/visual/material/pbr/metal/albedo_map"))
            self.assertFalse(texture.is_absolute())
            texture=(ROOT/'worlds'/texture).resolve()
            self.assertTrue(texture.is_relative_to(ROOT));self.assertTrue(texture.exists())
    def test_no_double_control_contact_or_zone_steps(self):
        world=ET.parse(ROOT/'worlds/rack_world.sdf').getroot().find('world')
        robot=world.find("model[@name='robot']")
        mask=robot.findtext("link[@name='lift_platform']/collision/surface/contact/collide_bitmask")
        for n in scene.RACKS:
            rack=world.find(f"model[@name='rack_{n.lower()}']")
            for collision in rack.findall('.//collision'):
                other=collision.findtext('surface/contact/collide_bitmask')
                self.assertEqual(int(mask,16)&int(other,16),0)
        for name in ('start_zone','end_zone','drop_a','drop_b','drop_c','drop_d'):
            self.assertEqual(world.find(f"model[@name='{name}']").findall('.//collision'),[])
    def test_qr_textures_decode(self):
        import cv2
        detector=cv2.QRCodeDetector()
        for n,r in scene.RACKS.items():
            decoded,_,_=detector.detectAndDecode(cv2.imread(str(ROOT/f'materials/qr_{n.lower()}.png')))
            self.assertEqual(decoded,r[5])


class FakeTime:
    def __init__(self):self.now=0.
    def monotonic(self):return self.now


def twist():return NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))


def extracted_class(filename,class_name,clock):
    tree=ast.parse((ROOT/'scripts'/filename).read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==class_name)
    cls.bases=[]
    cls.body=[n for n in cls.body if not (isinstance(n,ast.FunctionDef) and n.name=='__init__')]
    namespace=dict(time=clock,math=math,scene=scene,np=np,cv2=cv2,Planner=Planner,
                   MAX_RETRY=3,POSITION_TOL=.012,STILL_SECONDS=.4,DATA_TIMEOUT=1.,
                   rclpy=NS(ok=lambda:True),Twist=twist,String=lambda **k:NS(**k),
                   Float64=lambda **k:NS(**k),Bool=lambda **k:NS(**k),
                   wrap=lambda a:math.atan2(math.sin(a),math.cos(a)),
                   yaw=lambda q:math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                   SetEntityPose=NS(Request=lambda:NS(entity=NS(),pose=NS(position=NS(),orientation=NS()))))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'<offline-methods>','exec'),namespace)
    return namespace[class_name]


class ControlTests(unittest.TestCase):
    def mission(self):
        clock=FakeTime();m=extracted_class('mission_v3.py','MissionV3',clock)()
        m.get_logger=lambda:NS(info=lambda *a:None,warn=lambda *a:None,error=lambda *a:None)
        m.pub_vel=lambda *a:None;m.set_state=lambda value:setattr(m,'state',value)
        m.fresh=lambda **k:True;m.odom_seq=0;m.pose=(1.,1.,math.pi)
        m.speed=0.;m.angular_speed=0.;m.state='TEST';m.error=None
        def spin(*args):clock.now+=.1;m.odom_seq+=1
        m.spin=spin
        return m,clock
    def test_rejected_radar_cell_survives_new_free_map_for_same_goal(self):
        m,_=self.mission();m.observed_blockers=set()
        probe=Planner(grid([(70,16,254)]),(.12,.08),(3.02,2.),margin=.05)
        m.remember_blockers(probe,(1.0076,.39),(2.34,.39))
        m.get_grid=lambda:grid();m.half_size=lambda:(.12,.08);m.obstacle_rects=lambda:[]
        p=m.planner()
        self.assertFalse(p.segment_free((1.0076,.39),(2.34,.39)))
        self.assertTrue(p.segment_free((1.0076,1.),(2.34,1.)))

    def test_blocker_history_clears_when_target_changes(self):
        m,_=self.mission();m.goal=(2.,1.);m.released=True
        m.n_turns=m.n_segs=0;m.current_payload=None
        m.observed_blockers={(1.,1.025,.4,.425)}
        m.blocker_goal=('navigation',(2.,1.),None)
        m.plan_for=lambda tgt:[]
        m.wait_still=lambda *a,**k:True
        self.assertTrue(m.move_target(dict(name='navigation')))
        self.assertTrue(m.observed_blockers)
        m.goal=(1.,1.)
        self.assertTrue(m.move_target(dict(name='navigation')))
        self.assertEqual(m.observed_blockers,set())

    def test_stale_odom_cannot_pass_still(self):
        m,clock=self.mission();m.spin=lambda *a:setattr(clock,'now',clock.now+.1)
        self.assertFalse(m.wait_still((1.,1.),timeout=1.))
    def test_moving_odom_cannot_pass_still(self):
        m,clock=self.mission();m.speed=.1
        def spin(*a):clock.now+=.1;m.odom_seq+=1
        m.spin=spin;self.assertFalse(m.wait_still((1.,1.),timeout=1.))
    def test_still_requires_multiple_fresh_frames(self):
        m,clock=self.mission()
        def spin(*a):clock.now+=.1;m.odom_seq+=1
        m.spin=spin;self.assertTrue(m.wait_still((1.,1.),timeout=1.))
        self.assertGreaterEqual(m.odom_seq,5)
    def test_next_scan_auto_release_without_waiting_for_topic(self):
        m,_=self.mission();m.released=False;m.accepting=True;m.next_flag=True
        m.wait_next=lambda name:self.fail('scan unnecessarily waits for topic')
        self.assertTrue(m.release_target(dict(name='scan_D',auto_release=True)))
        self.assertTrue(m.released);self.assertFalse(m.accepting);self.assertFalse(m.next_flag)

    def test_manual_targets_still_wait_once(self):
        m,_=self.mission();calls=[];m.wait_next=lambda name:calls.append(name) or True
        for name in ('scan_A','under_A','drop_A','END'):
            self.assertTrue(m.release_target(dict(name=name)))
        self.assertEqual(calls,['scan_A','under_A','drop_A','END'])

    def test_motion_retry_bounded_one_release(self):
        m,_=self.mission();m.released=False;m.goal=(2.,1.);m.n_turns=m.n_segs=0
        attempts=[];releases=[]
        def plan(t):attempts.append(t);return [(2.,1.)]
        def release(n):releases.append(n);m.released=True;return True
        m.plan_for=plan;m.wait_next=release;m.drive_seg=lambda *a,**k:False
        self.assertFalse(m.move_target(dict(name='scan_A')))
        self.assertEqual(len(attempts),4);self.assertEqual(releases,['scan_A']);self.assertEqual(m.retry,3)
    def test_segment_axis_comes_from_plan_not_residual_error(self):
        m,_=self.mission();m.pose=(2.,1.,math.pi);m.released=True;m.goal=(1.,1.0085)
        m.plan_origin=(2.,1.);m.n_turns=m.n_segs=0
        m.plan_for=lambda t:[(1.,1.),(1.,1.0085)]
        axes=[]
        def drive(axis,target,**k):
            axes.append(axis)
            if axis=='x':m.pose=(1.0079,1.,math.pi)
            else:m.pose=(1.0079,1.0085,math.pi)
            return True
        m.drive_seg=drive;m.wait_still=lambda *a:True
        self.assertTrue(m.move_target(dict(name='drop_A')))
        self.assertEqual(axes,['x','y'])
    def test_under_entry_small_alignment_residual_is_axis_aligned(self):
        for residual in (-.0079,-.006,.006,.0079):
            m,_=self.mission();m.pose=(1.548,1.46+residual,math.pi)
            m.goal=(1.136,1.46);m.released=True;m.n_turns=m.n_segs=0
            axes=[]
            def drive(axis,target,**kw):
                self.assertTrue(kw['blind']);axes.append(axis)
                coords=list(m.pose);coords[0 if axis=='x' else 1]=target;m.pose=tuple(coords)
                return True
            m.drive_seg=drive
            m.wait_still=lambda goal:math.dist(m.pose[:2],goal)<=.012
            self.assertTrue(m.move_target(dict(name='under_A')),residual)
            self.assertEqual(axes,['x'])

    def test_under_entry_large_residual_aligns_then_enters(self):
        m,_=self.mission();m.pose=(1.548,1.48,math.pi);m.goal=(1.136,1.46)
        path=m.plan_for(dict(name='under_A'))
        self.assertEqual(path,[(1.548,1.46),(1.136,1.46)])

    def test_task_failure_retains_simulation_but_core_failure_does_not(self):
        import tempfile
        tree=ast.parse((ROOT/'scripts/stack_manager.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('alive','mark_mission_failed')]
        namespace={}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'<supervisor>','exec'),namespace)
        with tempfile.TemporaryDirectory() as folder:
            log=Path(folder)/'mission.log';log.write_text('FAILED state=ABORT error=E_STUCK')
            stack=namespace['Stack']();stack.stop=False;stack.ready=True;stack.mission_failed=False
            stack.run=Path(folder);messages=[];stack.say=messages.append
            stack.children=[('mission',NS(poll=lambda:1),log),('driver',NS(poll=lambda:None),log)]
            self.assertTrue(stack.alive());self.assertTrue(stack.alive())
            self.assertEqual(len(messages),1)
            self.assertEqual((stack.run/'status').read_text(),'MISSION_FAILED\n')
            stack.children.append(('gz_srv',NS(poll=lambda:1),log))
            with self.assertRaises(RuntimeError):stack.alive()
            stack.children=stack.children[:1];stack.ready=False
            with self.assertRaises(RuntimeError):stack.alive()

    def test_gui_exit_does_not_stop_core_stack(self):
        tree=ast.parse((ROOT/'scripts/stack_manager.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='alive']
        namespace={}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'<supervisor-method>','exec'),namespace)
        stack=namespace['Stack']();stack.stop=False;stack.gui_closed=False
        stack.say=lambda s:None
        stack.children=[('gz_gui',NS(poll=lambda:0),None),('driver',NS(poll=lambda:None),None)]
        self.assertTrue(stack.alive());self.assertTrue(stack.gui_closed)
    def test_driver_signal_handler_dependencies_defined(self):
        tree=ast.parse((ROOT/'scripts/kinematic_driver.py').read_text())
        imports={alias.asname or alias.name for node in tree.body if isinstance(node,ast.Import) for alias in node.names}
        self.assertIn('os',imports)
    def test_trigger_outside_wait_or_duplicate_is_ignored(self):
        m,_=self.mission();m.accepting=False;m.next_flag=False
        m.on_next(NS(data=True));self.assertFalse(m.next_flag)
        m.accepting=True;m.on_next(NS(data=True));self.assertTrue(m.next_flag);self.assertFalse(m.accepting)
        m.on_next(NS(data=True));self.assertTrue(m.next_flag)
    def test_drop_failure_keeps_footprint_and_never_retreats(self):
        m,_=self.mission();m.current_payload='A';m.wait_lift=lambda want:False
        m.set_footprint=lambda:self.fail('changed footprint after failed drop')
        m.exit_rack=lambda:self.fail('retreated after failed drop')
        self.assertFalse(m.do_action(dict(name='drop_A',rack='A')))
        self.assertEqual(m.current_payload,'A')
    def test_lift_plans_drop_without_forced_retreat(self):
        m,_=self.mission();m.select_payload=lambda *a:True;m.wait_lift=lambda want:True
        m.set_footprint=lambda:True
        m.exit_rack=lambda:self.fail('unnecessary loaded retreat')
        original_pose=m.pose
        self.assertTrue(m.do_action(dict(name='under_A',rack='A')))
        self.assertEqual(m.current_payload,'A');self.assertEqual(m.pose,original_pose)

    def test_drop_still_exits_from_placed_rack(self):
        m,_=self.mission();m.wait_lift=lambda want:True
        m.status={'payload_center':[*scene.DROP['A'],0.]};m.placed={};m.current_payload='A'
        m.select_payload=lambda n:True;m.set_footprint=lambda:True
        exits=[];m.exit_rack=lambda:exits.append(True) or True
        self.assertTrue(m.do_action(dict(name='drop_A',rack='A')))
        self.assertEqual(exits,[True]);self.assertIsNone(m.current_payload)

    def test_planning_reserves_tracking_error_execution_checks_actual_footprint(self):
        m,_=self.mission();m.get_grid=lambda:grid();m.current_payload=None
        m.half_size=lambda:(.12,.08)
        # 0.006m planned clearance: previously accepted then rejected after a 0.008m residual.
        m.obstacle_rects=lambda:[(.5,.8,0.,1.-.08-scene.COLLISION_MARGIN-.001)]
        start,end=(.25,1.),(1.10,1.)
        planning=m.planner();execution=m.planner(for_motion=True)
        self.assertFalse(planning.segment_free(start,end))
        self.assertTrue(execution.segment_free(start,end))
        path=planning.plan(start,end);self.assertIsNotNone(path)
        for point in path:
            cross=1 if start[0]!=point[0] else 0
            for delta in (-scene.TRACKING_TOL,scene.TRACKING_TOL):
                a=list(start);b=list(point);a[cross]+=delta;b[cross]+=delta
                self.assertTrue(execution.segment_free(a,b),(a,b))
            start=point

    def test_wrong_qr_never_succeeds(self):
        m,clock=self.mission()
        m.qr_hits=0;m.qr_seen=0.
        def spin(*a):clock.now+=1.
        m.spin=spin
        self.assertFalse(m.do_action(dict(name='scan_A',rack='A',qr='RACKA_XXXX')))
        self.assertEqual(m.error,'E_QR')




    def test_valid_map_checks_do_not_interrupt_velocity_tracking(self):
        m,clock=self.mission();velocity=[0.];commands=[];checks=[]
        m.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock.now*1e9)))
        def publish(vx=0.,vy=0.):
            commands.append((vx,vy));velocity[0]=-vx
        m.pub_vel=publish
        def spin(seconds=.02):
            clock.now+=seconds;m.pose=(m.pose[0]+velocity[0]*seconds,m.pose[1],m.pose[2])
        m.spin=spin
        def planner(**kwargs):
            checks.append(clock.now)
            return NS(segment_blocked_reason=lambda start,end:None)
        m.planner=planner
        self.assertTrue(m.drive_seg('x',1.30))
        self.assertGreaterEqual(len(checks),3)
        self.assertEqual(sum(vx==vy==0. for vx,vy in commands),1)

    def test_expired_map_or_new_obstacle_stops_before_more_velocity(self):
        for failure in ('expired','obstacle'):
            m,clock=self.mission();velocity=[0.];commands=[];checks=[]
            m.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock.now*1e9)))
            m.data_diagnostic=lambda:'injected map fault'
            def publish(vx=0.,vy=0.):
                commands.append((vx,vy));velocity[0]=-vx
            m.pub_vel=publish
            def spin(seconds=.02):
                clock.now+=seconds;m.pose=(m.pose[0]+velocity[0]*seconds,m.pose[1],m.pose[2])
            m.spin=spin
            def planner(**kwargs):
                checks.append(clock.now)
                if len(checks)>1 and failure=='expired':return None
                return NS(hx=.12,hy=.08,margin=.05,segment_blocked_reason=lambda start,end:'new obstacle' if len(checks)>1 else None)
            m.planner=planner
            self.assertFalse(m.drive_seg('x',1.30))
            self.assertEqual(commands[-1],(0.,0.));self.assertLess(m.pose[0],1.30)

    def test_lift_completion_requires_current_target_and_measured_joint_position(self):
        for want,initial in [(False,dict(lift=scene.LIFT_UP,lift_target=scene.LIFT_UP,lift_settled=True,carrying=False,engaged=True)),
                              (True,dict(lift=scene.LIFT_DOWN,lift_target=scene.LIFT_DOWN,lift_settled=True,carrying=True,engaged=True))]:
            m,clock=self.mission();m.status=initial;m.carry=want
            m.get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock.now*1e9)))
            commands=[];m.lift_pub=NS(publish=lambda msg:commands.append(msg.data))
            iterations=[]
            def spin(seconds):
                clock.now+=seconds;iterations.append(clock.now)
                if len(iterations)>=3:
                    value=scene.LIFT_UP if want else scene.LIFT_DOWN
                    m.status.update(lift=value,lift_target=value,lift_settled=True,carrying=want,engaged=want)
            m.spin=spin
            self.assertTrue(m.wait_lift(want));self.assertEqual(len(iterations),3)
            self.assertEqual(len(commands),3)

    def test_lift_command_not_reissued_after_data_or_cancel_failure(self):
        m,clock=self.mission();m.fresh=lambda **kw:False
        m.get_clock=lambda:NS(now=lambda:NS(nanoseconds=0))
        m.lift_pub=NS(publish=lambda msg:self.fail('lift command after invalid/cancelled operation'))
        self.assertFalse(m.wait_lift(True))



class ProcessIdentityTests(unittest.TestCase):
    def test_recycled_pid_never_signaled(self):
        import process_control as pc
        record=dict(pid=123,start='10',pgid=123,group_owned=True)
        with patch.object(pc,'identity',return_value=dict(pid=123,start='20')),patch.object(pc.os,'killpg') as kill:
            pc.terminate(record);kill.assert_not_called()
    def test_orphaned_session_is_cleaned(self):
        import process_control as pc
        record=dict(pid=123,start='10',pgid=123,group_owned=True)
        with patch.object(pc,'identity',return_value=None),patch.object(pc,'group_members',return_value=[{'pid':124}]),patch.object(pc.os,'killpg') as kill:
            pc.terminate(record);kill.assert_called_once()
    def test_unowned_group_not_signaled(self):
        import process_control as pc
        record=dict(pid=123,start='10',pgid=999,group_owned=False)
        with patch.object(pc,'same',return_value=True),patch.object(pc.os,'kill') as kill,patch.object(pc.os,'killpg') as group:
            pc.terminate(record);kill.assert_called_once();group.assert_not_called()


if __name__=='__main__':unittest.main(verbosity=2)
