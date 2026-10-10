> 历史记录：本文件保留旧接口与旧运行方式，当前点位接口、实机进退架与联合仿真统一流程请以 [导航接入与部署](导航接入与部署.md) 和项目 README 为准。

# Gazebo 与实机定位导航代码联合验证

本模式让 Gazebo 提供模拟雷达扫描和执行环境，用实机包的定位、规划、路径跟踪代码驱动 Gazebo 小车。
**不同时启动两个完整项目；用一个联合入口选择唯一的定位和控制链。**当前没有实物，这能验证软件执行和误差，不包含真实轮胎、轮速伺服、ESP32 通信或实际雷达噪声。

## 1. 启动与停止

在本项目根目录执行；程序由操作者启动和停止：

```bash
bash scripts/build.sh
# 如已有普通 Gazebo 或联合实例，先停止：
bash scripts/stop_rack_sim.sh
bash scripts/run_rack_hybrid.sh
```

无 GUI：`bash scripts/run_rack_hybrid.sh --headless`。
它与原 run_rack_sim.sh 使用同一实例锁、进程登记和停止脚本，不能并行抢 /scan、/odom、TF 或 /cmd_vel。
不要再运行 run_robot.sh、mock_modules、旧 mission_v3 主循环或另一套模拟导航。

执行到最终 END 确认或仿真扫码等待超时，并完成停止握手后，结束本次受管理的仿真；提前停止使用 `bash scripts/stop_rack_sim.sh`。
初次修改包后必须重新构建，移动项目目录后也必须重新构建；不依赖外部 rack_v3 或固定用户名。

## 2. 数据与控制归属

```text
Gazebo MS200 模拟扫描 → bridge → /scan
   → robot_navigation/localization → 雷达 /odom 和唯一导航 TF
   → Nav2 global_costmap → robot_navigation/navigation_server 规划
   → TrackPath → robot_navigation/path_tracker → /cmd_vel
   → kinematic_driver（雷达门控）→ rack_simulation（平面关节／独立看门狗）→ 小车运动

框架状态机 → NavigateToPose / Dock / lift
模拟 Dock、举升和健康适配 → Gazebo 架口动作、物理平台与货架执行
二维码测试脚本 → qr/detections → 状态机继续

/simulation/robot_pose → hybrid_evaluator → 误差 JSON / CSV
```

实机定位、导航和跟踪代码没有订阅 Gazebo 真值。真值用于仿真内部平面关节执行、机构物理反馈和独立误差评估；不进入导航位姿计算。
普通运输由实机 path_tracker 发速度；进退架由模拟 Dock 串行控制，仍保留已授权的架口地图豁免。
旧仿真定位、map_alignment、SLAM 和普通路径跟踪不启动；框架仿真执行桥的 NavigateToPose 也禁用。
模拟外围适配不重复设置地图足迹；实机导航节点按新鲜模拟机构测量切换载货包络和 footprint。

各节点使用仿真时钟 `use_sim_time=true`，统一 domain=0、命名空间 /team2/sim。
TF 为 map→odom→base_link→laser，四壁定位仍保持绝对场地坐标约定。
从 scene.py 生成 navigation_hybrid.yaml 和 costmap_hybrid.yaml，确认的是模拟场地几何，**不修改 navigation_robot.yaml 的真实标定确认位**。
联合 costmap 保留未知标记，当前按实机配置允许场地内未知格通行；若出现 no route 或定位不可观测，应查看实际扫描和地图，不能用真值补位姿或盲走普通运输路段。

## 3. 脚本识别触发与超时自动推进

不发送二维码脚本也可以运行完整仿真：各扫码点等待 15 秒后自动继续。脚本可用于提前结束当前等待。

如需查看状态，另开终端，进入同一项目：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 topic echo /team2/sim/mission/status
```

在同样环境的第三个终端，根据状态选择**一条**命令执行：

| 当前 state | 当前 rack | 在项目根目录执行 |
|---|---|---|
| WAIT_START | A | `python3 scripts/publish_qr_result.py START` |
| WAIT_RACK_QR | A | `python3 scripts/publish_qr_result.py RACKA_XXXX` |
| WAIT_RACK_QR | B | `python3 scripts/publish_qr_result.py RACKB_XXXX` |
| WAIT_RACK_QR | C | `python3 scripts/publish_qr_result.py RACKC_XXXX` |
| WAIT_RACK_QR | D | `python3 scripts/publish_qr_result.py RACKD_XXXX` |
| WAIT_FINAL_QR | 看 state 即可 | `python3 scripts/publish_qr_result.py END` |

不能提前连发所有码。比如 START 已经发过，车到 A 前停住并显示 WAIT_RACK_QR/rack=A，
现在应发 RACKA_XXXX；并非再次发 START，也不是等导航继续自己走。
框架收到 A 的有效识别结果，或仿真扫码窗口超时后，进入 DOCK_ENTER，自动进架、举升和搬运。

仿真 START、货架码、最终 END 各等待 15 秒（壁钟）：在等待状态发布匹配码可立即继续，
不发布也会在超时后自动推进。mission_rack.yaml 的 qr_timeout_advance: true 开启此行为。
WAIT_START 超时后前往 A（导航自动回正）；WAIT_RACK_QR 超时后进架；WAIT_FINAL_QR 超时后执行停车握手再结束。
超时放行在日志 reason 中明确记录，不宣称实际识别成功；最终 final_confirmed 只由真实 END 结果置真。
导航、进退架、举升超时或模块健康故障仍停车处理；实机 mission_robot.yaml 保持严格识别流程。
若已经 FAULT，发布二维码不会恢复，须先查看 mission.log 的具体原因，再由操作者停止/重新启动。
到 WAIT_START 后重新发 START，之后按上表逐架确认。
每次识别成功或仿真扫码等待超时后，导航、进架、举升、送货、卸货和退出按动作完成结果自动推进；不恢复 /rack/next 的逐动作放行。

## 4. 查看定位、路径、速度与误差

按需要在不同终端执行：

```bash
ros2 topic echo /lidar/localization_status
ros2 topic echo /team2/sim/navigation/status
ros2 topic echo /team2/sim/base/tracking_status
ros2 topic echo /team2/sim/navigation/path
ros2 topic echo /cmd_vel
ros2 topic echo /team2/sim/evaluation/metrics
```

可在 RViz 中设 Fixed Frame=map，添加 /scan、/odom、/team2/sim/navigation/path 查看扫描、位姿和路径。
速度为 base_link 的 vx/vy（m/s）与 wz（rad/s），不是地图速度。车头朝 -x 时，地图向 -x 行进对应车体 vx>0，不能按地图轴误判速度符号。

评估节点按 /odom 的采样时间，在前后两帧 Gazebo 真值间插值，最大真值间隔 0.1s；不外推过期真值。
evaluation/metrics 中主要字段：

| 字段 | 含义 |
|---|---|
| valid、samples | 最近是否有有效配对、累计配对数量 |
| position_error_m、yaw_error_rad | 当前雷达估计相对真值的位置／航向误差 |
| position_rmse_m、yaw_rmse_rad | 累计均方根误差 |
| position_max_m、yaw_max_rad | 累计最大误差 |
| cross_track_error_m | 跟踪期间 Gazebo 真值到规划折线的偏差；非跟踪阶段为 null |
| cmd_vx、cmd_vy、cmd_wz | 最近发布的车体速度，不是实际编码器测量 |
| state、rack | 当前任务阶段与货架 |
| unmatched_samples、pending_samples | 无法时间配对或尚在等待真值的样本数量 |
| csv_file | 本次 CSV 保存位置 |

路径偏差来自 Gazebo 真值，不能用定位估计自己证明自己准确。速度字段用于核对发布、方向、限幅与零速度；实际跟踪效果由真值轨迹和到位误差验证。
阶段标签和速度是采集时最新状态，定位比较按时间配对。Gazebo 运动学底盘没有真实轮胎打滑和电机动力学，不据此宣称实物跟踪精度。

## 5. 日志与离线汇总

本次日志位于 navigation/log/run_*/，CSV 为 hybrid_metrics.csv。
run.log 是启动监督，mission.log 含实机定位／规划／跟踪、costmap、外围适配和框架日志；driver.log、bridge.log、gz_srv.log 是模拟执行端。
联合模式不生成旧 lidar.log/map_alignment.log/slam.log/nav2.log，避免把旧导航日志误认作当前算法。
任务状态转换 JSONL 位于 runtime_logs/hybrid/。

运行结束后，在项目根目录查看最新 CSV：

```bash
python3 tools/summarize_hybrid_run.py
# 或指定某次 CSV：
python3 tools/summarize_hybrid_run.py navigation/log/run_某次运行/hybrid_metrics.csv
```

没有配对样本时先看时间戳和 evaluation/metrics；不能把空 CSV 或零样本解释为零误差。
定位、路径、速度和四架全流程是否实际正常，需由操作者运行后依据日志和指标判断。本次接入只做了代码构建与离线验证，没有自动启动 Gazebo。

消息时间戳和控制定时器使用仿真时钟，执行超时与断流看门狗仍按壁钟计算。长时间暂停 Gazebo 可能触发停车／FAULT，不能靠放宽数据新鲜度掩盖断流；恢复后查看诊断，再由操作者停止和重新启动。


### 场地内未知格通行与固定地图（2026-10-10）

按当前任务要求，联合和实机配置默认 `allow_unknown_in_field: true`。
规划器允许配置 `field_size` 内的 255 未知格通行；254 已探测障碍仍阻挡，
车体/载货扫掠包络、5cm 物理余量和到位误差预留保持检查。
未知不等于已测量无障碍；途中扫描若发现新障碍，会取消旧轨迹、确认停车并重规划。
地图/雷达/反馈断流、位置偏差或无法确认停车仍会失败，不靠推进状态掩盖故障。

地图保留 `track_unknown_space: true`，使 255 与实际观测自由格可区分；
运动规划按上述开关处理 255，而不是把雷达测得的 254 清成零。
因此地图显示的灰色未知区域并不阻挡当前导航。
设 `allow_unknown_in_field: false` 可恢复严格已观测模式及未知边界前的分段观测停车。
此开关只改变未知格策略，不关闭定位或地图检查，不引入直线盲走控制。

global_costmap 改为固定窗口、原点 (0,0)。当前 Nav2 网格为 4×2m，
宽高按整数米向上取整覆盖 3.020×2.000m 场地；实际可行区域仍严格是
`[0,3.020]×[0,2.000]`，要求整个包络及余量在场内。
缓冲网格多出的 x=3.020～4.000m 不允许通行；地图外也不当作自由空间。
联合配置自动从场景生成，实机改场地尺寸后应同步 costmap 的整数覆盖范围。

未知目标可直接规划到达，不再仅因未知格增加中间停车点。
启动仍需有效雷达定位和一份已产生实际观测的地图；全未知/仅清除自身的初始化图
保持等待，当前包络中的未知按同一开关处理，已探测碰撞仍禁止启动。
无真实可行路线时保存 runtime_logs/navigation/blocked_*.npz，保留原始未知/障碍分类便于诊断。

### 启动时等待首份有效雷达地图

地图服务有响应、更新时间新鲜，不代表 Nav2 已执行第一次扫描更新。
导航启动就绪检查会拒绝全未知图，等待当前位置安全包络没有已探测障碍，
且包络之外也有实际观测格（排除仅清除车体自身的初始化图）。此时状态保持 IDLE，
速度不下发；首份有效地图到达后才自动进入 NAV_START，前往起点扫码位置并转向。
不靠固定延时，不关闭未知格检查，也不把暂未初始化当作“无路线”直接进入 FAULT。

首份观测确认后，合法进退架/升降足迹切换不会单独造成导航健康丢失；
运动仍实时检查完整扫掠包络，升降后仍等待足迹切换后的新地图再下发轨迹。
地图再次变成全未知或雷达/地图断流会阻止执行，不沿用旧就绪状态继续运动。


### 导航健康与升降过程

navigation/status 的 healthy 表示定位、地图、模块反馈和跟踪接口仍正常；ready 表示当前平台状态允许导航。
升降过程中正常发送 lift_is_up=null、lift_settled=false、lift_state_source=measured，
modules.lift/base 仍按通信/实测反馈是否正常填写，而非按是否到位填写。
此时导航 healthy=true、ready=false：健康汇总的 modules.navigation 取 healthy，
navigation_ready 单独显示允许运动状态，禁止把 ready=false 当成断联。
定位/地图断流、机构报告故障仍令 healthy=false；不跳过任何数据新鲜度或故障检查。

导航与跟踪接收运动仍要求新鲜、已确认的布尔 lift_is_up；机构状态未确认时不下发运动。
升降完成后仍需服务完成响应及请求之后的实测到位状态，载货足迹切换后还要等新地图。
lift_settled 为可选布尔字段；旧接口只发 lift_is_up=null 也表示未确认，不视为动作完成。


### 完整地图快照与足迹更新

实机定位导航包和联合模式读取 global_costmap/costmap_raw 的完整 nav2_msgs/Costmap，
该快照由 Nav2 加锁复制后发布。导航不再通过 GetCostmap 服务读取正在更新的缓冲区，
避免将足迹重算的中间空图误判为地图丢失；未知/障碍格及断流检查保持有效。
配置 costmap_topic 指定此输入，always_send_full_costmap=true 保证每次有完整网格。
新鲜度使用消息 header.stamp（快照发布时间），不使用本机接收时间刷新地图版本。
原有 metadata.update_time 缺省为零时内部转换成同一 header 时间，以便统一校验。
载货状态变化后仍需等新的完整快照才运输；没有快照会等待或停车，不沿用无限期旧地图。

足迹只在首次连接、实测空车/载货切换、重新连接时发布，不再不断发布相同多边形触发重膨胀。
途中“观测分段”停车是等待新扫描；直角路径换轴时还需停稳后换方向，符合轴向运动约束。
导航 status.costmap_pipeline.source=published_costmap_raw 可核对实际读图来源。

实现依据：[Nav2 Jazzy 地图发布源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/src/costmap_2d_publisher.cpp)
的 `prepareCostmap()` 在复制完整地图时持有更新锁；同文件的 `costmap_service_callback()`
未持有该锁，并把请求时刻写为地图更新时间。
[膨胀层源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/plugins/inflation_layer.cpp)
在收到足迹更新时要求重新膨胀。因此，重复发送相同足迹和轮询服务的组合存在读到重算中间数据的风险。
`run_1009_194159_6918` 记录了 A 已卸货、退出期间因 `no free observed cells` 停机，
同时雷达和底盘反馈仍新鲜；当次未保存故障地图原始数组，不能据日志断言所有格子具体是什么值。

同次运行的评估 CSV 中，A 前方的观测分段约暂停 2.3 个仿真秒，轴向拐点约暂停 0.5 秒。
这些暂停用于确认停止和等待新观测，不属于上述 `FAULT`；速度持续为零且状态进入 `STOPPING/FAULT`
则需要检查完整的 `reason` 和 `navigation_diagnostic`，不能仅看截断后的状态字符串。

当前五包离线构建成功，289 项 pytest、61 项导航 unittest 和一组 C++ 机构循环测试通过；模型 SDF 解析有效。
未由开发代理启动仿真，完整搬运流程需要操作者重新运行联合入口验证。



执行端已改为 world 支撑的 X／Y／航向关节链，平台物理举降与底盘运动解耦；不再使用逐帧 set_pose 服务。新包需要先 `bash scripts/build.sh`，联合入口和框架话题不变。树莓派实机可 `bash scripts/build.sh --robot` 跳过 Gazebo 包。详细故障来源、停止语义与验收边界见 [执行与停止契约审计](执行与停止契约审计.md)。


## D 接近路线与停顿诊断（2026-10-09）

规划现已在 5cm 物理安全间距之外预留 position_tolerance（默认 1.2cm），
当前实测包络不重复加入该误差。实际 D 故障地图的回放依据和离线回归见
[执行与停止契约审计](执行与停止契约审计.md)。

base/tracking_status 新增 phase、segment_index；评估 metrics／CSV 新增：

| 字段 | 含义 |
| --- | --- |
| tracking_phase | TRACKING、AXIS_STOP、SETTLING、COMPLETED、IDLE；区分跟踪和停稳阶段 |
| odom_receive_wall_sec | 从评估节点启动起的墙钟接收秒数 |
| odom_receive_interval_wall_sec | 两帧位姿在本评估节点的接收墙钟间隔 |
| odom_source_interval_sec | 两帧位姿源时间戳间隔 |
| odom_receive_rtf | 源时间增量／接收墙钟增量；观察慢仿真或成批消息 |

这些字段只用于诊断，不修改停止或动作成功判定。RTF 不是 GUI 帧率，也不是
端到端消息延迟；首帧没有相邻时间差，CSV 留空，metrics 为 null。
本次既有采样未发现离散真值跳变，后续运行仍须验证画面卡顿和消息延迟。


## 运行中地图更新与重新规划

雷达新地图阻断剩余名义路线时，导航器保留原 NavigateToPose 目标，
取消旧 TrackPath、确认实测停车、等待停车之后的新扫描与地图，再重新规划。
原总超时不刷新。已完成路径段和当前段车后部分不再参与未来路线检查，
当前位置包络、跟踪走廊、定位健康及停止确认仍需满足原契约。
真实无路、位置包络被占据或停车未确认仍会失败，不以推进状态代替完成。
阻断地图保存于 runtime_logs/navigation/blocked_*.npz，剩余路线和阻断原因写入导航日志。
本次 350 项 Python 离线检查与 C++ 回归通过；运行验收仍由操作者完成。


本次未知格策略修改通过 361 项 pytest＋61 项导航 unittest，共 422 项 Python 离线回归；五包构建成功。新增回归验证未知通行、254 障碍不被删除、完整包络不越场地/地图边界与固定覆盖范围。未启动仿真或硬件；更新后由操作者重新运行联合入口。
