# 定位导航与仿真源码

此目录已包含原 rack_v3 的现行运行代码、配置、世界、二维码纹理和离线测试，不依赖外部 rack_v3。
验证实机定位、导航与进退架使用项目根目录 scripts/run_rack_hybrid.sh；停止使用 scripts/stop_rack_sim.sh。
scripts/run_rack_sim.sh 是保留的普通历史仿真。当前算法、接口和分工见 [项目 README](../README.md) 与 [导航接入与部署](../docs/导航接入与部署.md)。
scripts/scene.py 是当前仿真几何来源；实机外参、场地和目标应实测配置。
历史 diag/stage/truth 脚本与旧日志不参与运行，未并入现行导航目录；原目录保留作历史备份。
