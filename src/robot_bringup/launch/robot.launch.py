"""实机启动入口：只启动任务系统，外部模块由队友接入，不启动模拟。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    folder = Path(get_package_share_directory('robot_bringup')) / 'config'
    return LaunchDescription([
        DeclareLaunchArgument('targets_file', default_value=str(folder / 'targets_robot.yaml')),
        DeclareLaunchArgument('mission_file', default_value=str(folder / 'mission.yaml')),
        DeclareLaunchArgument('interfaces_file', default_value=str(folder / 'interfaces.yaml')),
        DeclareLaunchArgument('domain_id', default_value='42', description='整机模块必须使用相同通信域'),
        DeclareLaunchArgument('namespace', default_value='team2/robot'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        Node(package='mission_manager', executable='mission_node',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[{'mode': 'robot', 'auto_arm': False,
                          'mission_file': LaunchConfiguration('mission_file'),
                          'interfaces_file': LaunchConfiguration('interfaces_file'),
                          'targets_file': LaunchConfiguration('targets_file')}]),
    ])
