"""Export scene.py coordinates into the framework; never starts a node."""
from pathlib import Path
import os
import sys
import math
import yaml
root=Path(__file__).resolve().parents[1]/'navigation'
sys.path.insert(0,str(root/'scripts'))
import scene
target={'simulation_only':True,'frame_id':scene.MAP_FRAME,
        'start_pose':[*scene.START_SCAN,scene.MODEL_YAW+math.pi/6],
        'final_pose':[*scene.END_SCAN,scene.MODEL_YAW+math.pi/6],
        'destination_pose':[*scene.END_SCAN,scene.MODEL_YAW],
        'racks':{name:{'qr':values[5],'approach_pose':[*scene.scan(name),scene.MODEL_YAW],
                       'drop_pose':[*scene.DROP[name],scene.MODEL_YAW]} for name,values in scene.RACKS.items()}}
out=Path(__file__).resolve().parents[1]/'src/robot_bringup/config/targets_rack.yaml'
out.write_text(yaml.safe_dump({'simulation_only':True,'racks':{k:{'qr':v['qr']} for k,v in target['racks'].items()}},allow_unicode=True,sort_keys=False))
points={'simulation_only':True,'points':{k:v['approach_pose'] for k,v in target['racks'].items()},
        'drop_by_rack':{k:v['drop_pose'] for k,v in target['racks'].items()}}
points['points'].update(START_SCAN=target['start_pose'],FINAL_SCAN=target['final_pose'])
points['docking']={k:{'bounds':list(v[:4]),'leg_thickness':scene.LEG_T,
                     'exit_distance':scene.EXIT_DIST,'max_entry_distance':.60}
                   for k,v in scene.RACKS.items()}
(out.parent/'navigation_points_rack.yaml').write_text(yaml.safe_dump(points,sort_keys=False))
print('Framework targets generated from scene.py:',out)
