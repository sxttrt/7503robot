"""PC side: simulated actuator/feedback and evaluation; navigation backend is separate."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from mission_manager.project_paths import runtime_directory


def generate_launch_description():
    return LaunchDescription([
        Node(package='rack_simulation',executable='hybrid_execution',output='screen',
             parameters=[{'use_sim_time':True,'interface_prefix':'/team2/sim',
                          'provide_navigation':False,'provide_docking':False,'use_builtin_qr':False}]),
        Node(package='robot_navigation',executable='hybrid_evaluator',namespace='team2/sim',output='screen',
             parameters=[{'use_sim_time':True,'csv_file':os.environ.get('HYBRID_EVALUATION_FILE',
                 str(runtime_directory()/'hybrid_metrics.csv'))}])])
