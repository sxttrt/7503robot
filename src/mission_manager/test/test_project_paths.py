"""Pure source relocation tests; no nodes, processes or simulator startup."""
import importlib.util
from pathlib import Path
import shutil
from mission_manager.project_paths import project_root,navigation_root


def test_runtime_source_is_bundled_in_same_project():
    root=project_root()
    assert navigation_root()==root/'navigation'
    assert (root/'navigation/scripts/mission_v3.py').is_file()
    assert (root/'navigation/worlds/rack_world.sdf').is_file()


def test_project_resolves_after_copy_to_different_path_with_spaces(tmp_path,monkeypatch):
    root=project_root();target=tmp_path/'other computer'/'7503robot-main'
    (target/'navigation/scripts').mkdir(parents=True)
    (target/'.7503robot-project').write_text('test')
    (target/'navigation/scripts/scene.py').write_text('test')
    location=target/'src/mission_manager/mission_manager/project_paths.py'
    location.parent.mkdir(parents=True);shutil.copy2(root/'src/mission_manager/mission_manager/project_paths.py',location)
    spec=importlib.util.spec_from_file_location('relocated_project_paths',location)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setenv('ROBOT_PROJECT_ROOT',str(root))
    assert module.project_root()==target
    assert module.navigation_root()==target/'navigation'
    installed=target/'install/robot_bringup/share/robot_bringup/config'
    installed.mkdir(parents=True)
    assert module.project_root(installed)==target


def test_world_texture_uris_are_relative_and_remain_in_project():
    import xml.etree.ElementTree as ET
    root=project_root();world=root/'navigation/worlds/rack_world.sdf'
    for node in ET.parse(world).findall('.//albedo_map'):
        path=Path(node.text)
        assert not path.is_absolute()
        resolved=(world.parent/path).resolve()
        assert resolved.is_relative_to(root/'navigation') and resolved.is_file()
