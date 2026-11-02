# 电脑端仿真资源

本目录随 rack_simulation 安装到 ROS 包 share 中，包含场景、世界、纹理和仿真脚本。运行器会复制到可写运行目录后生成运行数据。树莓派不需要这个包。
电脑运行 ros2 run rack_simulation run_simulation --hybrid --remote-backend，树莓派运行 ros2 launch robot_bringup rack_hybrid_backend.launch.py。单机完整联合验证不加 --remote-backend；停止电脑端使用 ros2 run rack_simulation stop_simulation。
完整仓库的 scripts/run_rack_hybrid.sh 和 stop_rack_sim.sh 是便捷入口；仿真和实机定位算法在 robot_navigation.algorithms 中共享。部署细节见仓库 docs/部署与依赖.md。
scripts/scene.py 是当前仿真几何来源；实机外参、场地和目标应实测配置。
历史 diag/stage/truth 脚本与旧日志不参与运行，未并入现行导航目录；原目录保留作历史备份。
