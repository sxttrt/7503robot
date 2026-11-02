"""Optional developer checkout lookup and source-independent runtime paths."""
import os
from pathlib import Path


def project_root(start=None):
    path=Path(start or __file__).resolve()
    candidates=([path] if path.is_dir() else [])+list(path.parents)
    # A stale environment from another checkout cannot override this source tree.
    override=os.environ.get('ROBOT_PROJECT_ROOT')
    if override:candidates.append(Path(override).expanduser().resolve())
    for candidate in candidates:
        if (candidate/'.7503robot-project').is_file() and (candidate/'src').is_dir():
            return candidate
    raise RuntimeError('当前不在完整开发仓库；已安装节点使用 ROS 包资源，无需项目根目录')


def navigation_root(start=None):
    """Gazebo assets only. Real navigation never calls this function."""
    workspace=os.environ.get('RACK_SIM_WORKSPACE')
    if workspace:
        path=Path(workspace).expanduser().resolve()
        if (path/'scripts/scene.py').is_file():return path
        raise RuntimeError('RACK_SIM_WORKSPACE 缺少仿真场景资源')
    try:
        from ament_index_python.packages import get_package_share_directory
        path=Path(get_package_share_directory('rack_simulation'))/'navigation'
        if (path/'scripts/scene.py').is_file():return path
    except LookupError:
        pass
    # Developer tools may run before building. This fallback is never needed by hardware nodes.
    path=project_root(start)/'src/rack_simulation/navigation'
    if (path/'scripts/scene.py').is_file():return path
    raise RuntimeError('缺少 rack_simulation 资源，请构建仿真包')


def runtime_directory():
    """Writable logs independent of checkout/install location; no directory creation at import."""
    override=os.environ.get('ROBOT_RUNTIME_DIR')
    if override:return Path(override).expanduser().resolve()
    return Path(os.environ.get('ROS_HOME',str(Path.home()/'.ros'))).expanduser()/'7503robot/runtime_logs'
