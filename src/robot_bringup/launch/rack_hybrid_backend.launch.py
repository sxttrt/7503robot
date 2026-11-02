"""Raspberry Pi hybrid backend: real software with simulated inputs, no Gazebo processes."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from mission_manager.project_paths import runtime_directory


def generate_launch_description():
    folder=LaunchConfiguration('config_dir')
    config=lambda name:PathJoinSubstitution([folder,name])
    common={'use_sim_time':True,'config_file':config('navigation_hybrid.yaml'),
            'interfaces_file':config('interfaces.yaml'),'points_file':config('navigation_points_rack.yaml')}
    ld=LaunchDescription([
        DeclareLaunchArgument('config_dir',default_value=os.environ.get('RACK_SIM_CONFIG_DIR',
            str(Path(get_package_share_directory('robot_bringup'))/'config'))),
        DeclareLaunchArgument('domain_id',default_value=os.environ.get('ROS_DOMAIN_ID','0')),
        SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('domain_id'))])
    for executable in ('lidar_localization','navigation_server','path_tracker'):
        ld.add_action(Node(package='robot_navigation',executable=executable,namespace='team2/sim',
            output='screen',parameters=[common]))
    ld.add_action(Node(package='robot_navigation',executable='velocity_json_bridge',namespace='team2/sim',
        output='screen',parameters=[common]))
    ld.add_action(Node(package='nav2_planner',executable='planner_server',name='planner_server',output='screen',
        parameters=[config('costmap_hybrid.yaml'),{'use_sim_time':True}]))
    ld.add_action(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_navigation',output='screen',parameters=[{'use_sim_time':True,
            'autostart':True,'node_names':['planner_server']}]))
    ld.add_action(Node(package='mission_manager',executable='mission_node',namespace='team2/sim',output='screen',
        parameters=[{'use_sim_time':True,'mode':'simulation','auto_arm':True,
            'mission_file':config('mission_rack.yaml'),'targets_file':config('targets_rack.yaml'),
            'interfaces_file':config('interfaces.yaml'),'log_dir':str(runtime_directory()/'hybrid')}]))
    return ld
