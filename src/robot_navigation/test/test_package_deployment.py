"""Installed resources and PC/Pi launch partition; never initialize ROS or start processes."""
import importlib
import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch
import xml.etree.ElementTree as ET

import pytest
import yaml

from mission_manager.project_paths import project_root, runtime_directory
from robot_navigation import core
from robot_navigation.dependency_check import check_dependencies
from robot_navigation.resources import navigation_config

ROOT=project_root()


def test_real_algorithms_are_packaged_without_changing_import_path():
    before=list(sys.path)
    importlib.reload(core)
    assert sys.path==before
    assert core.WallLocalizer.__module__=='robot_navigation.algorithms.lidar_geometry'
    assert core.Planner.__module__=='robot_navigation.algorithms.grid_planner'


def test_log_defaults_do_not_require_source_root(tmp_path,monkeypatch):
    monkeypatch.delenv('ROBOT_RUNTIME_DIR',raising=False)
    monkeypatch.setenv('ROBOT_PROJECT_ROOT','/unavailable/old-project')
    monkeypatch.setenv('ROS_HOME',str(tmp_path/'ros-user-data'))
    assert runtime_directory()==tmp_path/'ros-user-data/7503robot/runtime_logs'
    monkeypatch.setenv('ROBOT_RUNTIME_DIR',str(tmp_path/'chosen-logs'))
    assert runtime_directory()==tmp_path/'chosen-logs'


def test_hardware_config_installed_with_navigation_package_stays_uncalibrated():
    assert navigation_config().is_file()
    cfg=yaml.safe_load(navigation_config().read_text())
    assert cfg['calibration_verified'] is False
    for name in ('targets_robot.yaml','navigation_points_robot.yaml'):
        cfg=yaml.safe_load((ROOT/'src/robot_bringup/config'/name).read_text())
        if name.startswith('targets'):assert all(v['qr'] is None for v in cfg['racks'].values())
        else:assert all(p is None for section in ('points','drop_by_rack','docking') for p in cfg[section].values())


def executables(name):
    path=ROOT/'src/robot_bringup/launch'/name
    spec=importlib.util.spec_from_file_location('test_'+name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result=[]
    for entity in module.generate_launch_description().entities:
        value=getattr(entity,'node_executable',None)
        if value is not None:result.append(value if isinstance(value,str) else ''.join(p.text for p in value))
    return result


def test_pi_backend_has_all_control_nodes_and_no_simulator():
    backend=executables('rack_hybrid_backend.launch.py')
    assert set(backend)=={'lidar_localization','navigation_server','path_tracker','velocity_json_bridge',
                         'planner_server','lifecycle_manager','mission_node'}
    frontend=executables('rack_hybrid_frontend.launch.py')
    assert set(frontend)=={'hybrid_execution','hybrid_evaluator'}
    assert not set(frontend).intersection(backend)


def test_real_dependency_graph_does_not_install_simulation_stack():
    external=set()
    for package in ('mission_interfaces','mission_manager','robot_navigation','robot_bringup'):
        tree=ET.parse(ROOT/'src'/package/'package.xml').getroot()
        external.update(n.text for n in tree if n.tag in ('depend','exec_depend','build_depend','buildtool_depend'))
    assert not any(name.startswith(('gz_','ros_gz')) for name in external)
    assert not external.intersection({'rack_simulation','python3-opencv','python3-qrcode','slam_toolbox'})


def test_missing_dependencies_are_reported_without_launching_any_node():
    seen=[]
    def lookup(name):
        seen.append(name)
        if name=='nav2_planner':raise LookupError(name)
        return '/installed/'+name
    with patch('robot_navigation.dependency_check.get_package_prefix',side_effect=lookup),\
         patch('robot_navigation.dependency_check.importlib.util.find_spec',return_value=object()),\
         patch('robot_navigation.dependency_check.shutil.which',return_value='/bin/tool'):
        result=check_dependencies('robot')
    assert result==['ROS package: nav2_planner']
    assert 'rack_simulation' not in seen and 'ros_gz_bridge' not in seen


def test_generated_simulation_settings_never_modify_hardware_configuration(tmp_path):
    from rack_simulation.generate_config import generate
    hardware=ROOT/'src/robot_navigation/config/navigation_robot.yaml'
    before=hardware.read_bytes()
    generate(tmp_path,assets=ROOT/'src/rack_simulation/navigation',hardware=hardware,
             base=ROOT/'src/robot_bringup/config')
    assert hardware.read_bytes()==before
    assert yaml.safe_load((tmp_path/'navigation_hybrid.yaml').read_text())['calibration_verified'] is True
    points=yaml.safe_load((tmp_path/'navigation_points_rack.yaml').read_text())
    assert points['simulation_only'] is True and set(points['docking'])==set('ABCD')


def test_simulation_preparation_rejects_writing_into_installed_assets(tmp_path):
    from rack_simulation.workspace import prepare
    (tmp_path/'navigation/scripts').mkdir(parents=True)
    (tmp_path/'navigation/scripts/scene.py').write_text('# asset')
    with patch('rack_simulation.workspace.get_package_share_directory',return_value=str(tmp_path)),\
         pytest.raises(ValueError,match='重叠'):
        prepare(tmp_path/'navigation')


def test_pc_frontend_recovers_when_pi_backend_starts_later():
    import threading
    from types import SimpleNamespace as NS
    from rack_simulation.hybrid_execution import HybridExecution
    outcomes=iter([False,False,True]);calls=[];logs=[]
    def ready(timeout):
        calls.append(timeout);return next(outcomes)
    node=NS(initialized=False,abort_event=threading.Event(),wait_ready=ready,
        set_footprint=lambda:True,set_state=lambda state:None,get_logger=lambda:NS(info=logs.append))
    with patch('rack_simulation.hybrid_execution.rclpy.ok',return_value=True):
        HybridExecution.warmup(node)
    assert node.initialized and len(calls)==3 and 'HYBRID_ADAPTER_READY' in logs[-1]


def test_frontend_initialization_wait_can_be_stopped_before_pi_connects():
    import threading
    from types import SimpleNamespace as NS
    from rack_simulation.hybrid_execution import HybridExecution
    node=NS(initialized=False,abort_event=threading.Event())
    node.abort_event.set()
    with patch('rack_simulation.hybrid_execution.rclpy.ok',return_value=True):
        HybridExecution.warmup(node)
    assert not node.initialized
