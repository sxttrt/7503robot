#!/usr/bin/env python3
"""从 scene 生成联合验证配置；不修改真实标定文件，不启动节点。"""
from pathlib import Path
import sys
import math
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'navigation/scripts'))
import scene

def generate():
    folder=ROOT/'src/robot_bringup/config'
    cfg=yaml.safe_load((folder/'navigation_robot.yaml').read_text())
    cfg.update(calibration_verified=True,field_size=[scene.FIELD_X,scene.FIELD_Y],
        initial_pose=list(scene.model_start_pose()),laser_pose=[0.,0.,scene.LIDAR_Z,0.],
        empty_half_size=[scene.ROBOT_L/2,scene.ROBOT_W/2],
        loaded_half_size=[max(scene.ROBOT_L/2,max((r[1]-r[0])/2 for r in scene.RACKS.values())),
                          max(scene.ROBOT_W/2,max((r[3]-r[2])/2 for r in scene.RACKS.values()))],
        travel_yaw=scene.MODEL_YAW,localization_status_topic='/lidar/localization_status',
        costmap_service='/global_costmap/get_costmap',costmap_topic='/global_costmap/costmap_raw',
        footprint_topic='/global_costmap/footprint')
    (folder/'navigation_hybrid.yaml').write_text('# GENERATED: simulation geometry only; never real calibration.\n'+yaml.safe_dump(cfg,sort_keys=False))
    # Preserve real costmap algorithm and unknown-space policy, only change clock and measured simulation geometry.
    costmap=yaml.safe_load((folder/'costmap_robot.yaml').read_text())
    costmap['planner_server']['ros__parameters']['use_sim_time']=True
    params=costmap['global_costmap']['global_costmap']['ros__parameters']
    params.update(use_sim_time=True,rolling_window=False,origin_x=0.,origin_y=0.,
                  width=math.ceil(cfg['field_size'][0]),height=math.ceil(cfg['field_size'][1]))
    hx,hy=cfg['empty_half_size']
    params['footprint']=str([[hx,hy],[-hx,hy],[-hx,-hy],[hx,-hy]])
    (folder/'costmap_hybrid.yaml').write_text('# GENERATED: real costmap backend, simulation clock.\n'+yaml.safe_dump(costmap,sort_keys=False))
    print('hybrid configs generated from scene; navigation_robot.yaml unchanged')

if __name__=='__main__':generate()
