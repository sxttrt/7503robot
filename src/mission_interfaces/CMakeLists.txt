cmake_minimum_required(VERSION 3.8)
project(mission_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# 动作和升降服务定义在这里生成；真实与模拟服务端使用完全相同的类型。
rosidl_generate_interfaces(${PROJECT_NAME} "action/Dock.action" "srv/SetLift.srv")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
