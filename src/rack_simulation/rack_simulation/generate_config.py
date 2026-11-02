"""Generate simulation-only settings from the packaged scene; never alter hardware calibration."""
import importlib.util
import math
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from robot_navigation.resources import navigation_config


def generate(folder, assets=None, hardware=None, base=None):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    assets=Path(assets or (Path(get_package_share_directory('rack_simulation'))/'navigation'))
    base=Path(base or (Path(get_package_share_directory('robot_bringup'))/'config'))
    spec=importlib.util.spec_from_file_location('rack_scene_parameters',assets/'scripts/scene.py')
    scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)
    def write(name,value):
        (folder/name).write_text(yaml.safe_dump(value,allow_unicode=True,sort_keys=False),encoding='utf-8')
    write('targets_rack.yaml',{'simulation_only':True,'racks':{k:{'qr':v[5]} for k,v in scene.RACKS.items()}})
    write('navigation_points_rack.yaml',{'simulation_only':True,
        'points':{**{k:[*scene.scan(k),scene.MODEL_YAW] for k in scene.RACKS},
                  'START_SCAN':[*scene.START_SCAN,scene.MODEL_YAW+math.pi/6],
                  'FINAL_SCAN':[*scene.END_SCAN,scene.MODEL_YAW+math.pi/6]},
        'drop_by_rack':{k:[*scene.DROP[k],scene.MODEL_YAW] for k in scene.RACKS},
        'docking':{k:{'bounds':list(v[:4]),'leg_thickness':scene.LEG_T,
                      'exit_distance':scene.EXIT_DIST,'max_entry_distance':.6} for k,v in scene.RACKS.items()}})
    cfg=yaml.safe_load(Path(hardware or navigation_config()).read_text(encoding='utf-8'))
    cfg.update(calibration_verified=True,field_size=[scene.FIELD_X,scene.FIELD_Y],
        initial_pose=list(scene.model_start_pose()),laser_pose=[0.,0.,scene.LIDAR_Z,0.],
        empty_half_size=[scene.ROBOT_L/2,scene.ROBOT_W/2],
        loaded_half_size=[max(scene.ROBOT_L/2,max((r[1]-r[0])/2 for r in scene.RACKS.values())),
                          max(scene.ROBOT_W/2,max((r[3]-r[2])/2 for r in scene.RACKS.values()))],
        travel_yaw=scene.MODEL_YAW,localization_status_topic='/lidar/localization_status',
        costmap_service='/global_costmap/get_costmap',costmap_topic='/global_costmap/costmap_raw',
        footprint_topic='/global_costmap/footprint')
    write('navigation_hybrid.yaml',cfg)
    costmap=yaml.safe_load((base/'costmap_robot.yaml').read_text(encoding='utf-8'))
    costmap['planner_server']['ros__parameters']['use_sim_time']=True
    params=costmap['global_costmap']['global_costmap']['ros__parameters']
    params.update(use_sim_time=True,rolling_window=False,origin_x=0.,origin_y=0.,
                  width=math.ceil(cfg['field_size'][0]),height=math.ceil(cfg['field_size'][1]))
    hx,hy=cfg['empty_half_size'];params['footprint']=str([[hx,hy],[-hx,hy],[-hx,-hy],[hx,-hy]])
    write('costmap_hybrid.yaml',costmap)
