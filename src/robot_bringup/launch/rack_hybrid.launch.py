"""Single-PC convenience: compose the same PC frontend and Raspberry Pi backend."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    folder=Path(get_package_share_directory('robot_bringup'))/'launch'
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(folder/name)))
        for name in ('rack_hybrid_backend.launch.py','rack_hybrid_frontend.launch.py')])
