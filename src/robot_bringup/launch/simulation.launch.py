"""隔离的 ROS 模拟启动：相同接口、虚拟目标、独立通信域。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    folder = Path(get_package_share_directory('robot_bringup')) / 'config'
    common = {
        'mission_file': LaunchConfiguration('mission_file'),
        'interfaces_file': str(folder / 'interfaces.yaml'),
        'targets_file': str(folder / 'targets_sim.yaml'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='normal', description='中文说明见模拟场景配置'),
        DeclareLaunchArgument('auto_arm', default_value='true', description='模拟默认自动启用；实机必须人工启用'),
        DeclareLaunchArgument('mission_file', default_value=str(folder / 'mission.yaml')),
        DeclareLaunchArgument('domain_id', default_value='202', description='模拟专用 ROS 通信域'),
        DeclareLaunchArgument('namespace', default_value='team2/sim', description='模拟接口命名空间'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('ROS_STATIC_PEERS', ''),
        Node(package='mission_manager', executable='mock_modules',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[common, {'scenarios_file': str(folder / 'mock_scenarios.yaml'),
                                  'scenario': LaunchConfiguration('scenario')}]),
        Node(package='mission_manager', executable='mission_node',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[common, {'mode': 'simulation', 'auto_arm': ParameterValue(
                 LaunchConfiguration('auto_arm'), value_type=bool)}]),
    ])
