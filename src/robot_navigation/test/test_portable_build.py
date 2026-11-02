"""Run build/export wrappers in temporary projects with a fake colcon; no ROS nodes."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def project(tmp_path):
    root = tmp_path / 'copied project'
    (root/'scripts').mkdir(parents=True)
    (root/'src').mkdir()
    (root/'.7503robot-project').write_text('7503robot\n')
    shutil.copy2(ROOT/'.gitignore', root/'.gitignore')
    for name in ('build.sh', 'export_source.sh'):
        shutil.copy2(ROOT/'scripts'/name, root/'scripts'/name)
    binary = tmp_path/'fake-bin'
    binary.mkdir()
    stub = binary/'colcon'
    stub.write_text(f'#!{sys.executable}\n' + '''import json, os, sys
from pathlib import Path
Path('build_call.json').write_text(json.dumps({'args':sys.argv[1:], 'env':{
    key:os.environ.get(key,'') for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH',
    'COLCON_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH')}}))
if os.environ.get('TEST_BUILD_FAIL'): sys.exit(7)
Path('install').mkdir(exist_ok=True)
Path('install/setup.bash').write_text('# freshly built\\n')
''')
    stub.chmod(0o755)
    env = dict(os.environ, PATH=str(binary)+os.pathsep+os.environ['PATH'])
    for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH'):
        env[key] = '/old_machine/install'
    return root, env


def build(project, *args):
    root, env = project
    return subprocess.run(['bash', 'scripts/build.sh', *args], cwd=root, env=env,
                          capture_output=True, text=True, timeout=30)


def old_outputs(root, marker):
    for folder in ('build', 'install', 'log', 'runtime_logs', 'integration_backups'):
        (root/folder).mkdir()
        (root/folder/'keep.txt').write_text('old '+folder)
    (root/'.build-root').write_text(marker+'\n')


@pytest.mark.parametrize('mode',[(), ('--robot',)])
def test_moved_project_rebuilds_and_preserves_only_old_outputs_in_backup(project, mode):
    root, _env = project
    old_outputs(root, '/other/computer/7503robot')
    result=build(project,*mode)
    assert result.returncode==0, result.stderr
    assert (root/'.build-root').read_text().strip()==str(root.resolve())
    assert not (root/'install/keep.txt').exists()
    backups=list((root/'.build_cache_backups').glob('build_*'))
    assert len(backups)==1 and (backups[0]/'install/keep.txt').read_text()=='old install'
    assert (backups[0]/'build/keep.txt').is_file()
    assert (root/'.build_cache_backups/COLCON_IGNORE').is_file()
    for folder in ('log','runtime_logs','integration_backups'):
        assert (root/folder/'keep.txt').is_file()
    call=json.loads((root/'build_call.json').read_text())
    assert call['args'][:4]==['build','--base-paths','src','--symlink-install']
    assert ('--packages-skip' in call['args'])==bool(mode)
    assert all('/old_machine/' not in value for value in call['env'].values())


def test_clean_rebuild_also_replaces_stale_outputs_at_same_path(project):
    root, _env=project
    old_outputs(root,str(root.resolve()))
    result=build(project,'--robot','--clean')
    assert result.returncode==0, result.stderr
    assert not (root/'install/keep.txt').exists()
    assert (root/'install/setup.bash').is_file()


def test_incremental_build_keeps_outputs_but_failed_build_invalidates_marker(project):
    root, env=project
    old_outputs(root,str(root.resolve()))
    env['TEST_BUILD_FAIL']='1'
    result=build(project)
    assert result.returncode==7
    assert not (root/'.build-root').exists()
    assert (root/'install/keep.txt').is_file()
    assert not (root/'.build_cache_backups').exists()


@pytest.mark.parametrize('arg,code',[('--wrong-option',2),('--help',0)])
def test_invalid_argument_and_help_never_move_old_build(project,arg,code):
    root, _env=project
    old_outputs(root,'/other/project')
    result=build(project,arg)
    assert result.returncode==code
    assert (root/'install/keep.txt').is_file()
    assert (root/'.build-root').read_text().strip()=='/other/project'
    assert not (root/'build_call.json').exists()


def test_backup_symlink_cannot_move_outputs_outside_project(project,tmp_path):
    root,_env=project
    old_outputs(root,'/other/project')
    outside=tmp_path/'outside';outside.mkdir()
    (root/'.build_cache_backups').symlink_to(outside,target_is_directory=True)
    result=build(project,'--clean')
    assert result.returncode!=0 and not list(outside.iterdir())
    assert (root/'install/keep.txt').is_file()


@pytest.mark.parametrize('script',['run_robot.sh','run_rack_hybrid.sh','run_rack_sim.sh','底盘测试脚本.sh'])
def test_launch_guard_uses_same_physical_root_when_accessed_by_symlink(project,tmp_path,script):
    root,env=project
    shutil.copy2(ROOT/'scripts'/script,root/'scripts'/script)
    (root/'install/rack_simulation/lib').mkdir(parents=True)
    (root/'install/rack_simulation/lib/librack_hybrid_actuator.so').touch()
    # Stop at the setup boundary; never reach ros2, Gazebo or chassis_test.
    (root/'install/setup.bash').write_text('exit 33\n')
    (root/'.build-root').write_text(str(root.resolve())+'\n')
    alias=tmp_path/'project alias';alias.symlink_to(root,target_is_directory=True)
    result=subprocess.run(['bash',str(alias/'scripts'/script)],env=env,
                          capture_output=True,text=True,timeout=20)
    assert result.returncode==33, result.stdout+result.stderr


def git(root,*args):
    return subprocess.run(['git',*args],cwd=root,check=True,capture_output=True,text=True)


def committed_project(project):
    root,_env=project
    (root/'src/navigation_points_robot.yaml').write_text('points: {}\n')
    git(root,'init','-q')
    git(root,'add','.')
    git(root,'-c','user.name=Test','-c','user.email=test@example.invalid','commit','-qm','source')
    return root


def export(root, output):
    return subprocess.run(['bash','scripts/export_source.sh',str(output)],cwd=root,
                          capture_output=True,text=True,timeout=20)


def test_export_includes_source_and_never_copies_ignored_builds_or_logs(project,tmp_path):
    root=committed_project(project)
    old_outputs(root,'/old/machine')
    output=tmp_path/'portable.zip'
    result=export(root,output)
    assert result.returncode==0, result.stdout+result.stderr
    with zipfile.ZipFile(output) as archive:
        names=archive.namelist()
        assert '7503robot/src/navigation_points_robot.yaml' in names
        assert '7503robot/scripts/build.sh' in names
        assert not any(n.startswith('7503robot/'+p) for n in names for p in
                       ('build/','install/','log/','runtime_logs/','integration_backups/','.build-root'))


@pytest.mark.parametrize('problem',['uncommitted','tracked_build','existing_output','parent_repo'])
def test_export_refuses_ambiguous_or_dirty_source(project,tmp_path,problem):
    root=committed_project(project);output=tmp_path/'portable.zip'
    if problem=='uncommitted':(root/'src/uncommitted.py').write_text('uncommitted\n')
    elif problem=='tracked_build':
        (root/'build').mkdir();(root/'build/cache.txt').write_text('absolute path')
        git(root,'add','-f','build/cache.txt')
        git(root,'-c','user.name=Test','-c','user.email=test@example.invalid','commit','-qm','bad output')
    elif problem=='existing_output':output.write_bytes(b'preserve')
    else:
        nested=root/'nested';(nested/'scripts').mkdir(parents=True)
        shutil.copy2(root/'scripts/export_source.sh',nested/'scripts/export_source.sh')
        root=nested
    result=export(root,output)
    assert result.returncode!=0
    assert output.read_bytes()==b'preserve' if problem=='existing_output' else not output.exists()
