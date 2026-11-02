"""Actual Gazebo framework execution; starts no mock_modules or legacy mission loop."""
from pathlib import Path
import os
from mission_manager.project_paths import runtime_directory
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    folder=Path(os.environ.get('RACK_SIM_CONFIG_DIR',
        str(Path(get_package_share_directory('robot_bringup'))/'config')))
    runtime=runtime_directory()
    return LaunchDescription([
        DeclareLaunchArgument('builtin_qr',default_value='false',description='Optional embedded camera decoder; external QR topic is default'),
        Node(package='rack_simulation',executable='framework_execution',name='rack_execution',output='screen',
             parameters=[{'use_sim_time':True,'interface_prefix':'/team2/sim',
                          'use_builtin_qr':ParameterValue(LaunchConfiguration('builtin_qr'),value_type=bool)}]),
        Node(package='mission_manager',executable='mission_node',namespace='team2/sim',output='screen',
             parameters=[{'use_sim_time':True,'mode':'simulation','auto_arm':True,
                 'mission_file':str(folder/'mission_rack.yaml'),'interfaces_file':str(folder/'interfaces.yaml'),
                 'targets_file':str(folder/'targets_rack.yaml'),
                 'log_dir':str(runtime/'framework')}]),
    ])
