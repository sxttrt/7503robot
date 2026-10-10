"""Only Nav2 costmap hosts needed by the mission's rectilinear planner."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params=LaunchConfiguration('params_file')
    sim=LaunchConfiguration('use_sim_time')
    autostart=LaunchConfiguration('autostart')
    ld=LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('autostart',default_value='true')])
    for package,name in [('nav2_controller','controller_server'),('nav2_planner','planner_server')]:
        ld.add_action(Node(package=package,executable=name,name=name,output='screen',
                           parameters=[params,{'use_sim_time':sim}],
                           remappings=[('cmd_vel','cmd_vel_nav_unused')]))
    ld.add_action(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
                       name='lifecycle_manager_navigation',output='screen',
                       parameters=[{'use_sim_time':sim,'autostart':autostart,
                                    'node_names':['controller_server','planner_server']}]))
    return ld
