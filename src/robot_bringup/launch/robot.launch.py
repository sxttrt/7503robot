"""一个实机入口：定位、雷达 costmap、导航，按需加任务框架；无 Gazebo。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    folder=Path(get_package_share_directory('robot_bringup'))/'config'
    namespace=LaunchConfiguration('namespace')
    settings=LaunchConfiguration('navigation_file')
    interfaces=LaunchConfiguration('interfaces_file')
    costmap=RewrittenYaml(source_file=LaunchConfiguration('costmap_file'),root_key=namespace,
                         param_rewrites={},convert_types=True)
    declarations=[
        ('targets_file',str(folder/'targets_robot.yaml')),
        ('mission_file',str(folder/'mission_robot.yaml')),
        ('interfaces_file',str(folder/'interfaces.yaml')),
        ('navigation_file',str(folder/'navigation_robot.yaml')),
        ('points_file',str(folder/'navigation_points_robot.yaml')),
        ('costmap_file',str(folder/'costmap_robot.yaml')),
        ('domain_id','42'),('namespace','team2/robot'),
        # 无实物时默认只启动用户负责的定位导航，等待传感器/队友。
        # 完整目标已测量填写后，start_mission:=true 在同一入口加上状态机。
        ('start_mission','false')]
    ld=LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in declarations])
    ld.add_action(SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('domain_id')))
    for executable in ('lidar_localization','navigation_server','path_tracker'):
        ld.add_action(Node(package='robot_navigation',executable=executable,namespace=namespace,
            output='screen',parameters=[{'use_sim_time':False,'config_file':settings,
                                        'interfaces_file':interfaces,
                                        'points_file':LaunchConfiguration('points_file')}]))
    # 只借用 planner_server 托管 global_costmap，不发送其规划动作，且不启动 Nav2 控制器。
    ld.add_action(Node(package='nav2_planner',executable='planner_server',name='planner_server',
        namespace=namespace,output='screen',parameters=[costmap,{'use_sim_time':False}]))
    ld.add_action(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        namespace=namespace,name='lifecycle_manager_navigation',output='screen',
        parameters=[{'use_sim_time':False,'autostart':True,'node_names':['planner_server']}]))
    ld.add_action(Node(package='mission_manager',executable='mission_node',namespace=namespace,
        condition=IfCondition(LaunchConfiguration('start_mission')),output='screen',parameters=[{
            'use_sim_time':False,'mode':'robot','auto_arm':False,
            'mission_file':LaunchConfiguration('mission_file'),'interfaces_file':interfaces,
            'targets_file':LaunchConfiguration('targets_file')}]))
    return ld
