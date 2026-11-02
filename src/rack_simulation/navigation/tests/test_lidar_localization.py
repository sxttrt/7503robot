"""Offline scan synthesis tests. Truth is used only to generate test observations."""
import ast
import math
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import scene
from lidar_geometry import WallLocalizer,scan_points,wrap
from grid_planner import Planner,GridMap
from targets import build
from types import SimpleNamespace as NS


def synthetic_scan(pose,obstacles=(),seed=0):
    angle_min=-math.pi;increment=2*math.pi/719
    angles=angle_min+np.arange(720)*increment
    headings=angles+pose[2];dx,dy=np.cos(headings),np.sin(headings)
    distances=np.full(720,8.)
    for axis,values in ((0,(0.,scene.FIELD_X)),(1,(0.,scene.FIELD_Y))):
        direction=(dx,dy)[axis]
        for value in values:
            with np.errstate(divide='ignore',invalid='ignore'):
                t=(value-pose[axis])/direction
            distances=np.minimum(distances,np.where(t>0,t,np.inf))
    for x0,x1,y0,y1 in obstacles:
        with np.errstate(divide='ignore',invalid='ignore'):
            a=(x0-pose[0])/dx;b=(x1-pose[0])/dx
            c=(y0-pose[1])/dy;d=(y1-pose[1])/dy
        entry=np.maximum(np.minimum(a,b),np.minimum(c,d))
        leave=np.minimum(np.maximum(a,b),np.maximum(c,d))
        hit=np.where(entry>0,entry,leave)
        distances=np.minimum(distances,np.where((leave>=np.maximum(entry,0))&(hit>0),hit,np.inf))
    distances+=np.random.default_rng(seed).normal(0,.005,len(distances))
    return scan_points(distances,angle_min,increment,.05,8.)


class LidarTests(unittest.TestCase):
    def test_start_pose_measured_from_scan_not_copied_from_initial_guess(self):
        true=(2.70,1.025,math.pi+.025)
        matcher=WallLocalizer((scene.FIELD_X,scene.FIELD_Y),scene.model_start_pose())
        estimate=matcher.update(synthetic_scan(true),.1)
        self.assertLess(np.linalg.norm(estimate['pose'][:2]-true[:2]),.003)
        self.assertLess(abs(wrap(estimate['pose'][2]-true[2])),.003)

    def test_translations_and_scan_rotation_with_moving_rack_returns(self):
        matcher=WallLocalizer((scene.FIELD_X,scene.FIELD_Y),scene.model_start_pose())
        stamp=0.;pose=np.array(scene.model_start_pose());max_error=0.
        for step in range(220):
            if step<60:pose[0]-=.030
            elif step<90:pose[1]-=.020
            elif step<120:pose[2]+=(math.pi/6)/30
            elif step<150:pose[2]-=(math.pi/6)/30
            obstacles=scene.rack_leg_rects('C')+[(.3+step*.002,.47+step*.002,.8,1.18)]
            stamp+=.1
            estimate=matcher.update(synthetic_scan(pose,obstacles,step),stamp)
            max_error=max(max_error,np.linalg.norm(estimate['pose'][:2]-pose[:2]))
            self.assertLess(abs(wrap(estimate['pose'][2]-pose[2])),.004)
        self.assertLess(max_error,.004)
        self.assertLess(np.linalg.norm(estimate['velocity'][:2]),.01)
        self.assertLess(abs(estimate['velocity'][2]),.01)

    def test_all_mission_poses_include_rack_occlusions_and_loaded_motion(self):
        matcher=WallLocalizer((scene.FIELD_X,scene.FIELD_Y),scene.model_start_pose())
        pose=np.array(scene.model_start_pose());stamp=0.;placed={};payload=None
        meta=NS(resolution=.025,size_x=140,size_y=100,
                origin=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.)))
        grid=GridMap(meta,np.zeros(140*100,dtype=np.uint8))
        errors=[]
        def observations():
            return [rect for n in scene.ORDER if n!=payload
                    for rect in scene.rack_leg_rects(n,placed.get(n,scene.rack_model_center(n)))]
        def drive(points):
            nonlocal stamp,pose
            for target in points:
                start=pose[:2].copy();count=max(1,math.ceil(np.linalg.norm(np.array(target)-start)/.030))
                for step in range(1,count+1):
                    pose[:2]=start+(np.array(target)-start)*step/count;stamp+=.1
                    result=matcher.update(synthetic_scan(pose,observations(),int(stamp*100)),stamp)
                    errors.append(float(np.linalg.norm(result['pose'][:2]-pose[:2])))
        for target in build():
            kind=target['name'].split('_')[0];name=target.get('rack')
            hx,hy=(.12,.1905) if payload else (.12,.08)
            obstacles=observations()
            if payload:
                obstacles=[]
                for n in scene.ORDER:
                    if n==payload:continue
                    r=scene.RACKS[n];cx,cy=placed.get(n,scene.rack_model_center(n))
                    obstacles.append((cx-(r[1]-r[0])/2,cx+(r[1]-r[0])/2,cy-(r[3]-r[2])/2,cy+(r[3]-r[2])/2))
            if kind=='under':points=[(pose[0],target['xy'][1]),target['xy']]
            else:points=Planner(grid,(hx,hy),(scene.FIELD_X,scene.FIELD_Y),obstacles,margin=.058).plan(pose[:2],target['xy'])
            self.assertIsNotNone(points,target['name']);drive(points)
            if kind=='under':payload=name
            elif kind=='drop':
                placed[name]=tuple(pose[:2]);payload=None
                drive([(pose[0]+scene.EXIT_DIST,pose[1])])
        self.assertLess(max(errors),.005)

    def test_single_wall_is_rejected_without_overwriting_pose(self):
        matcher=WallLocalizer((scene.FIELD_X,scene.FIELD_Y),scene.model_start_pose())
        y=np.linspace(.2,1.8,100)
        points=np.column_stack((np.full(100,.30),1.-y))
        original=matcher.pose.copy()
        with self.assertRaises(ValueError):matcher.update(points,.1)
        np.testing.assert_equal(matcher.pose,original)
        self.assertIsNone(matcher.last_stamp)

    def test_invalid_ranges_and_duplicate_scan_do_not_update_pose(self):
        points=scan_points([np.nan,np.inf,-1.,.01,9.,1.],0.,.1,.05,8.)
        self.assertEqual(len(points),1)
        matcher=WallLocalizer((scene.FIELD_X,scene.FIELD_Y),scene.model_start_pose())
        scan=synthetic_scan(scene.model_start_pose());matcher.update(scan,.1)
        with self.assertRaises(ValueError):matcher.update(scan,.1)

    def test_node_subscribes_only_to_scan_driver_has_no_odom_or_tf_publisher(self):
        tree=ast.parse((ROOT/'scripts/lidar_localization.py').read_text())
        topics=[]
        for call in ast.walk(tree):
            if isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute) and call.func.attr=='create_subscription':
                topics.append(call.args[1].value)
        self.assertEqual(topics,['/scan'])
        driver=(ROOT/'scripts/kinematic_driver.py').read_text()
        self.assertNotIn('create_publisher(Odometry',driver)
        self.assertNotIn('TransformBroadcaster',driver)
        self.assertNotIn('on_imu',driver)


if __name__=='__main__':unittest.main()
