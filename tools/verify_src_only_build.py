"""Isolated build/import proof; no rclpy.init, Node construction, launch execution or hardware."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

repo=Path(__file__).resolve().parents[1]
root=Path(tempfile.mkdtemp(prefix='7503robot-src-only-'))
packages=('mission_interfaces','mission_manager','robot_navigation','robot_bringup')
for name in packages:
    shutil.copytree(repo/'src'/name,root/'src'/name,
        ignore=shutil.ignore_patterns('__pycache__','*.pyc','*.egg-info','.pytest_cache'))
env=dict(os.environ)
for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','COLCON_CURRENT_PREFIX',
            'PYTHONPATH','LD_LIBRARY_PATH','ROS_PACKAGE_PATH','ROBOT_PROJECT_ROOT','ROBOT_RUNTIME_DIR',
            'RACK_V3_ROOT','RACK_SIM_WORKSPACE','RACK_SIM_CONFIG_DIR'):
    env.pop(key,None)
print('SRC_ONLY_WORKSPACE='+str(root),flush=True)
subprocess.run(['bash','--noprofile','--norc','-c',
    'source /opt/ros/jazzy/setup.bash && colcon build --base-paths src'],
    cwd=root,env=env,check=True)
# A normal (not symlink) install must load even with copied src taken out of the way.
(root/'src').rename(root/'source_saved')
check=root/'check_installed.py'
check.write_text('''import importlib, importlib.abc, importlib.util, json, sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
class NoSimulation(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('rack_simulation','cv2','qrcode','ros_gz_interfaces','scene','mission_v3'):
            raise ImportError('hardware import attempted simulation dependency: '+fullname)
sys.meta_path.insert(0,NoSimulation())
assert 'rack_simulation' not in get_packages_with_prefixes()
loaded={}
for name in ('mission_manager.mission_node','mission_manager.qr_test_publisher',
             'robot_navigation.localization','robot_navigation.navigation','robot_navigation.path_tracker',
             'robot_navigation.velocity_json_bridge','robot_navigation.chassis_test'):
    module=importlib.import_module(name);loaded[name]=module.__file__
    assert '/install/' in module.__file__ and '/source_saved/' not in module.__file__
from robot_navigation.resources import navigation_config
from robot_navigation.core import load_settings
from mission_manager.project_paths import runtime_directory
assert load_settings(navigation_config())['calibration_verified'] is False
folder=Path(get_package_share_directory('robot_bringup'))
for name in ('robot.launch.py','rack_hybrid_backend.launch.py','simulation.launch.py'):
    spec=importlib.util.spec_from_file_location('installed_launch_'+name,folder/'launch'/name)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    assert module.generate_launch_description().entities
from mission_manager.config_loader import load_config
try:load_config(folder/'config/mission_robot.yaml',folder/'config/interfaces.yaml',folder/'config/targets_robot.yaml','robot')
except ValueError as exc:assert '尚未填写' in str(exc)
else:raise AssertionError('unmeasured hardware QR unexpectedly passed')
assert (folder/'config/navigation_points_robot.yaml').is_file()
assert (folder/'config/navigation_points_rack.yaml').is_file()
print(json.dumps({'loaded':loaded,'config':str(navigation_config()),'logs':str(runtime_directory()),
    'simulation_package_absent':True,'source_root_absent':True,'hardware_calibration_false':True}))
''')
result=subprocess.run(['bash','--noprofile','--norc','-c',
    'source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 check_installed.py'],
    cwd=root,env=env,check=True,capture_output=True,text=True)
print(result.stdout,flush=True)
(root/'validation.json').write_text(json.dumps(dict(root=str(root),
    packages=list(packages),result=json.loads(result.stdout)),indent=2))
