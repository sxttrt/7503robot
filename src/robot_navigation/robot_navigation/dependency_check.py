"""Read-only dependency diagnostics. Imports metadata, never creates ROS nodes or runs Gazebo."""
import argparse
import importlib.util
import shutil
from ament_index_python.packages import get_package_prefix


def check_dependencies(profile='robot'):
    packages=['mission_interfaces','mission_manager','robot_navigation','robot_bringup',
              'rclpy','tf2_ros','nav2_msgs','nav2_common','nav2_costmap_2d','nav2_planner',
              'nav2_navfn_planner','nav2_lifecycle_manager']
    modules=['numpy','yaml'];executables=['ros2']
    if profile in ('hybrid','simulation'):
        packages+=['rack_simulation','ros_gz_bridge','ros_gz_interfaces','ros_gz_sim','gz_sim_vendor',
                   'gz_transport_vendor','gz_plugin_vendor','gz_tools_vendor']
        modules+=['qrcode','PIL','cv2'];executables+=['gz','setsid','flock']
    if profile=='simulation':packages+=['slam_toolbox','nav2_controller','nav2_mppi_controller']
    missing=[]
    for package in packages:
        try:get_package_prefix(package)
        except LookupError:missing.append('ROS package: '+package)
    for module in modules:
        if importlib.util.find_spec(module) is None:missing.append('Python module: '+module)
    for executable in executables:
        if shutil.which(executable) is None:missing.append('executable: '+executable)
    return missing


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--profile',choices=('robot','hybrid','simulation'),default='robot')
    args=parser.parse_args();missing=check_dependencies(args.profile)
    if missing:raise SystemExit('依赖缺失：\n  '+'\n  '.join(missing))
    print('依赖检查通过：'+args.profile+'（未启动节点；不代表实机标定或设备已就绪）')
