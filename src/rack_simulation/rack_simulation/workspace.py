"""Prepare writable simulation files from installed resources, including src-only workspaces."""
import argparse
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from mission_manager.project_paths import runtime_directory
from robot_navigation.dependency_check import check_dependencies


def workspace_root():
    return Path(os.environ.get('RACK_SIM_WORKSPACE',str(runtime_directory()/'simulation'))).expanduser().resolve()


def prepare(root):
    """Copy immutable package resources. Caller holds prepare lock and has verified stack idle."""
    assets=Path(get_package_share_directory('rack_simulation'))/'navigation'
    config=Path(get_package_share_directory('robot_bringup'))/'config'
    if not (assets/'scripts/scene.py').is_file():raise RuntimeError('rack_simulation resources missing; rebuild package')
    if root.resolve().is_relative_to(assets.resolve()) or assets.resolve().is_relative_to(root.resolve()):
        raise ValueError('仿真运行目录不能与安装资源目录重叠')
    shutil.copytree(assets,root,dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__','*.pyc','log','tests'))
    framework_config=root/'framework_config'
    shutil.copytree(config,framework_config,dirs_exist_ok=True)
    from .generate_config import generate
    generate(framework_config,assets=root,base=config)
    return framework_config


def main():
    parser=argparse.ArgumentParser(description='Gazebo runner using installed ROS packages')
    parser.add_argument('--hybrid',action='store_true',help='use real localization/navigation/tracking')
    parser.add_argument('--remote-backend',action='store_true',help='PC Gazebo side; backend runs on Raspberry Pi')
    parser.add_argument('--headless',action='store_true')
    args=parser.parse_args()
    if args.remote_backend and not args.hybrid:parser.error('--remote-backend requires --hybrid')
    problems=check_dependencies('hybrid' if args.hybrid else 'simulation')
    if problems:raise SystemExit('缺少运行依赖，请按 README 安装：\n  '+'\n  '.join(problems))
    root=workspace_root();(root/'log').mkdir(parents=True,exist_ok=True)
    with (root/'log/prepare.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('当前仿真正在准备启动')
        with (root/'log/stack.lock').open('a') as stack_lock:
            try:fcntl.flock(stack_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:raise SystemExit('已有仿真运行，请先 ros2 run rack_simulation stop_simulation')
            config=prepare(root)
        env=dict(os.environ,RACK_SIM_WORKSPACE=str(root),RACK_SIM_CONFIG_DIR=str(config))
        cmd=['bash',str(root/'scripts/run.sh'),'--framework',str(root)]
        if args.hybrid:cmd.append('--hybrid')
        if args.remote_backend:cmd.append('--remote-backend')
        if args.headless:cmd.append('--headless')
        if args.remote_backend:
            print('电脑仿真端；树莓派另起 rack_hybrid_backend.launch.py。',flush=True)
            print('本次场景配置：'+str(config)+'；修改场景后请同步到树莓派并指定 config_dir。',flush=True)
        raise SystemExit(subprocess.call(cmd,cwd=root,env=env))


def stop():
    root=workspace_root()
    script=root/'scripts/process_control.py'
    if not script.is_file():
        print('当前用户没有已准备的仿真目录');return
    raise SystemExit(subprocess.call([sys.executable,str(script)],cwd=root))
