"""Gazebo sensors/actuator are started by supervisor; this launch owns the real software backend."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from mission_manager.project_paths import project_root

def generate_launch_description():
    folder=Path(get_package_share_directory('robot_bringup'))/'config'
    root=project_root(folder);namespace='team2/sim'
    common={'use_sim_time':True,'config_file':str(folder/'navigation_hybrid.yaml'),
            'interfaces_file':str(folder/'interfaces.yaml'),'points_file':str(folder/'navigation_points_rack.yaml')}
    ld=LaunchDescription()
    for executable in ('lidar_localization','navigation_server','path_tracker'):
        ld.add_action(Node(package='robot_navigation',executable=executable,namespace=namespace,
            output='screen',parameters=[common]))
    # Only global costmap host; no old controller, lidar, map alignment or SLAM.
    ld.add_action(Node(package='nav2_planner',executable='planner_server',name='planner_server',output='screen',
        parameters=[str(folder/'costmap_hybrid.yaml'),{'use_sim_time':True}]))
    ld.add_action(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_navigation',output='screen',parameters=[{'use_sim_time':True,
            'autostart':True,'node_names':['planner_server']}]))
    ld.add_action(Node(package='mission_manager',executable='hybrid_execution',output='screen',
        parameters=[{'use_sim_time':True,'interface_prefix':'/team2/sim',
                     'provide_navigation':False,'provide_docking':False,'use_builtin_qr':False}]))
    ld.add_action(Node(package='mission_manager',executable='mission_node',namespace=namespace,output='screen',
        parameters=[{'use_sim_time':True,'mode':'simulation','auto_arm':True,
            'mission_file':str(folder/'mission_rack.yaml'),'targets_file':str(folder/'targets_rack.yaml'),
            'interfaces_file':str(folder/'interfaces.yaml'),'log_dir':str(root/'runtime_logs/hybrid')}]))
    ld.add_action(Node(package='robot_navigation',executable='hybrid_evaluator',namespace=namespace,
        output='screen',parameters=[{'use_sim_time':True,'csv_file':os.environ.get('HYBRID_EVALUATION_FILE',
            str(root/'navigation/log/hybrid_metrics.csv'))}]))
    return ld
