cmake_minimum_required(VERSION 3.8)
project(robot_bringup)
find_package(ament_cmake REQUIRED)
# 统一安装启动和配置目录，程序始终从包的安装位置读取默认配置。
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
