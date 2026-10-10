> 历史记录：本文件保留旧接口与旧运行方式，当前点位接口、实机进退架与联合仿真统一流程请以 [导航接入与部署](导航接入与部署.md) 和项目 README 为准。

# rack_v3 接入框架（现行定位导航入口）

完整的任务流程、代码架构、QR／升降／底盘分工与伺服接口已写入 [README 的定位导航部分](../README.md#定位与导航当前实现和职责)。

## 启动与识别结果

```bash
cd 7503robot-main
source /opt/ros/jazzy/setup.bash
bash scripts/build.sh
bash scripts/stop_rack_sim.sh
bash scripts/run_rack_sim.sh
```

当前入口自动执行 NAV_START；等待 START、当前货架码和最后 END 的窗口都走 `/team2/sim/qr/detections`，类型 `std_msgs/String`，内容使用框架 QR JSON。
已取消进架、送货和退出的手动等待；不再用 `/rack/next` 作为识别或阶段推进。
没有视觉模块时，只由操作者在对应 WAIT 状态运行 `python3 scripts/publish_qr_result.py START`、`RACKA_XXXX` 等或 `END` 来发布测试结果。
START 成功后自动回正并前往 A；货架码成功后自动进架举升、送货、放下退出；ABCD 完成后去终点前方左转 30°，END 成功后确认停车并关闭本次受管理进程。
故障停止仍需查看日志，不能靠重复识别消息跳过故障。

## 对齐后的坐标和模块边界

仿真和实机框架目标及 Path 统一采用 `map`；控制内部保持 `odom`，通过 TF 做真实转换。
仿真导航链为 `map→odom→base_link→laser`；墙匹配已输出绝对场地坐标，雷达就绪后 `map_alignment.py` 发布一次恒等 map→odom。SLAM 只建图，不发布导航 TF。地图读取是单一异步缓存，运动中不等待 GetCostmap 服务；过期地图仍停车。
实机提供真实 map→odom，不复制仿真标定结果或未实测的目标数值。
雷达定位只用 `/scan` 点云和已知固定墙面，估计 x/y/yaw 与速度；不使用 IMU／Gazebo 车辆真值。真值仅供 `/simulation/robot_pose` 的仿真执行和货架物理确认。
导航与 Dock 串行控制 `/cmd_vel`；速度为 base_link 的 vx、vy（m/s）、wz（rad/s）。当前平移上限 0.30m/s，空载扫描旋转上限 0.375rad/s，载货不旋转，避障余量 5cm。
外部视觉发布者采用框架 QR 结果即可推进；ENTER 已不再检查执行桥的私有识别缓存。内置相机解码器默认关闭，可选 builtin_qr 仅供联调。
实体举升服务及 ESP32 尚未接入；详见 README 的角色说明、Twist 字段、麦轮逆运动学和反馈建议。替换同名模块需修改 launch 及健康汇总，并关闭被替换的仿真服务端。

## 日志与验证

`navigation/log/run_*/` 保存 mission、lidar、map_alignment、driver、nav2 和 slam 日志。
`runtime_logs/framework/` 保存状态机 JSONL。ROS_DOMAIN_ID、RMW_IMPLEMENTATION 与发现设置须与启动脚本打印值一致。
代码编译和离线测试不等于 Gazebo 全流程或实机验收；C 退出到 D 的往复问题仍需实际日志确认，已发现障碍格保留策略不等于根因已完全解决。
所有仿真启动与停止由操作者执行。

本次结构调整完成离线故障注入，未替代真实 Gazebo 全流程验收。观察 robot/status 的 navigation_diagnostic 和 costmap_pipeline，结合 driver.log 判断执行请求是否超时。实物底盘看门狗、编码器和平台反馈仍需硬件模块实现。
