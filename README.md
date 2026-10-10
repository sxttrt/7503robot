# DASE7503 第二队机器人项目

这是第二队共同开发物流机器人的代码仓库。机器人按 **A → B → C → D** 的固定顺序搬运货架，从确认 `START` 后开始三分钟计时。

**当前已有：主状态机、模块调用接口、模拟模块、中文配置和测试，以及接入框架的雷达定位、地图规划、进退架、树莓派轨迹跟踪与 Gazebo 联合验证。真实电机通信、二维码、升降和整机在线/停止模块由对应队友接入。**

模拟和实机使用同一个主流程，区别是执行模块：

```text
主状态机 → 接口代码 → 模拟模块 / 真实模块 → 返回结果 → 进入下一步
```

当前使用简化接口：**队友完成动作并停止后返回成功；失败时返回原因；收到停止请求就停止当前动作。主程序直接使用完成结果，不再做令牌、测量来源或二次静止确认。**

**状态机向导航发送点位名**，使用 [NavigateToPoint.action](src/mission_interfaces/action/NavigateToPoint.action)：A／B／C／D、DROP_OFF（放置区），实机与联合仿真配置还发送 START_SCAN、FINAL_SCAN。地图、坐标、朝向和路线由导航维护。进退架通过 `Dock` 调用同一导航跟踪器；升降使用 `std_srvs/srv/SetBool`，在线消息为 `{"ready": true, "fault": ""}`。每架到投放点后直接放下，全部搬完后扫描 END。上游通用 mock 的 mission.yaml 保留简化流程；实机使用 mission_robot.yaml，联合仿真使用 mission_rack.yaml。模拟验证不能代替实物搬运验收。

上游原点位接口版曾在树莓派构建并通过147项测试，历史记录见 [构建验证记录（自己看）](docs/自己的记录/构建验证记录（自己看）.md)。本次加入定位、导航与进退架后，在 WSL 完成5包构建，以及511项框架/导航离线测试、61项共享算法测试和1项执行器核心测试；尚未运行本次 Gazebo 全流程或实物验收。

## 第一次来，先看什么？

1. 想知道机器人怎么完成任务：看 [任务流程说明](docs/给队友看/任务流程说明.md)。
2. 要接自己的模块：先看 [模块接口说明](docs/给队友看/模块接口说明.md)，再找下面表格中的接口文件。
3. 想下载后跑一遍：看本页“下载并运行模拟”；详细操作在 [运行说明](docs/给队友看/运行说明.md)。
4. 完成一个模块，想先单独测试：看 [单模块测试说明](docs/给队友看/单模块测试说明.md)。

队友按自己的任务阅读上面四份文档即可；文档统一放在 `docs/给队友看/`。

## 你做哪部分，就从哪里开始

| 任务 | 先看这些文件 | 需要完成什么 |
|---|---|---|
| 主状态机与任务流程 | [state_machine.py](src/mission_manager/mission_manager/state_machine.py)、[mission_node.py](src/mission_manager/mission_manager/mission_node.py) | 维护顺序、计时、重试、失败处理和日志 |
| LiDAR、定位与导航 | [navigation.py](src/mission_manager/mission_manager/adapters/navigation.py)、[接口说明](docs/给队友看/模块接口说明.md) | 接收点位名，自己管理坐标和导航，返回成功／失败，支持取消 |
| 相机与 QR 识别 | [qr.py](src/mission_manager/mission_manager/adapters/qr.py)、[实机目标配置](src/robot_bringup/config/targets_robot.yaml) | 发布完整 QR、图像时间戳和当前识别结果 |
| 局部对准、进入与退出货架 | [Dock.action](src/mission_interfaces/action/Dock.action)、[导航进退架](src/robot_navigation/robot_navigation/docking.py) | 本包已实现；导航负责人填写实测架口几何，底盘组执行速度与反馈 |
| 底盘、ESP32 通信与升降 | [lift.py](src/mission_manager/mission_manager/adapters/lift.py)、[safety.py](src/mission_manager/mission_manager/adapters/safety.py)、[固件说明](firmware/README.md) | 实现升降完成反馈、简单在线状态和停止 |
| 机械结构与升降机构 | [接口说明](docs/给队友看/模块接口说明.md)中的对准与升降部分 | 提供安装尺寸、可托举条件、升降行程和退出条件 |
| 配置、启动与整机联调 | [config](src/robot_bringup/config)、[launch](src/robot_bringup/launch)、[运行说明](docs/给队友看/运行说明.md) | 填实测配置、接入各模块、维护统一启动与测试 |

表中的 `adapters` 是**主程序调用模块的接口代码**，真实算法写在各自的 ROS 包或 ESP32 固件中。不要把导航、图像解码或电机算法塞进主状态机。

## 用 AI 帮你开发时，给它什么？

最好让 AI 打开整个仓库，并先读本 README 和 [模块接口说明](docs/给队友看/模块接口说明.md)。再给它你负责的那一行中的接口文件，以及你已有的程序和硬件信息。

如果只能上传少量文件，就提供：**接口说明＋对应适配文件＋需要用到的动作定义／配置＋你自己的模块代码**。

## 下载并运行模拟

**不需要树莓派或机器人硬件。**完整模拟需要 Ubuntu 24.04＋ROS 2 Jazzy；Windows 上可先用 VS Code 阅读和修改，运行需准备相应 ROS 环境。

已经安装 ROS 2 Jazzy 并配置软件源后，在 Ubuntu 终端执行：

```bash
# 安装任务框架的构建和测试依赖；真实导航依赖由对应模块自行准备。
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-yaml python3-pytest ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime

# 下载到固定工作区名称，便于按说明运行。
git clone https://github.com/sxttrt/7503robot.git ~/7503robot_ws
cd ~/7503robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# 自动生成模拟 START，依次执行 A、B、C、D。
ros2 launch robot_bringup simulation.launch.py
```

完成后显示 `FINISHED`，按 `Ctrl+C` 退出。想看故障处理可以运行：

```bash
ros2 launch robot_bringup simulation.launch.py scenario:=navigation_fail_once
```

源码和配置说明使用中文，变量名和 ROS 类型保留英文。详细的启用、停止、状态查看、测试及实机启动步骤见 [运行说明](docs/给队友看/运行说明.md)。

## 实机接入前要补什么？

- 第二队四个货架的完整 QR，填写 [targets_robot.yaml](src/robot_bringup/config/targets_robot.yaml)。
- 导航模块自行准备 A/B/C/D/DROP_OFF 五个点位；坐标、地图和朝向不再交给主程序配置。
- 标定已接入的定位导航，并接入真实 QR、对准、升降、底盘通信、健康状态和停止模块。
- 模块按 [接口说明](docs/给队友看/模块接口说明.md)报告实际完成结果。升降响应只代表“收到命令”的程序，需要先改造。

`SIMTEAM2` 仅是模拟后缀，不能用于实机。当前实机 QR 配置为空，启动时会提示缺失项。

## 修改、提交与日志

各模块完成后，需要在实机上进行测试。需要取用板子、部署程序、烧录固件或现场联调时，可以在微信群里联系我（**刘承韬**）；开发或接口对接中有问题，也可以在群里交流。

建议使用 **VS Code Remote-SSH** 连接树莓派，直接编辑代码、编译和运行测试。树莓派程序部署运行，ESP32 固件需要烧录。

**支持单模块测试，不用等完整机器人做好。**具体操作见 [单模块测试说明](docs/给队友看/单模块测试说明.md)。

运行日志保存在 `~/7503robot_ws/runtime_logs/`，每次运行记录状态转换、目标货架、剩余时间、重试和故障原因。

上传源码、配置、文档和测试即可；`build/`、`install/`、`log/`、运行日志和缓存已由 `.gitignore` 排除。`firmware/README.md` 只是 ESP32 固件目录说明，项目入口是当前这份 README。

## 维护记录（自己看）

`docs/自己的记录/` 保存主状态机维护者的测试、构建和审查记录，队友接入模块时无需逐篇阅读。

- [构建验证记录（自己看）](docs/自己的记录/构建验证记录（自己看）.md)：历史验证结果、课程要求核对及主状态机复测命令。
- [代码审查与修复说明（自己看）](docs/自己的记录/代码审查与修复说明（自己看）.md)：逻辑问题、修复理由和验证范围。

代码使用 [Apache-2.0 许可证](LICENSE)。

## 定位与导航（已接入）

详细交接、接口、数据格式和部署步骤见 [导航接入与部署](docs/导航接入与部署.md)。这部分保留主框架最新的 `NavigateToPoint`、`SetBool` 与 `ready/fault` 接口，不把定位和底盘算法写进状态机。

| 内容 | 位置 |
|---|---|
| 雷达定位、导航、进退架、树莓派速度跟踪 | `src/robot_navigation/robot_navigation/` |
| 导航点位与实测参数 | `src/robot_bringup/config/navigation_points_robot.yaml`、`navigation_robot.yaml` |
| 点位动作与内部轨迹动作 | `src/mission_interfaces/action/NavigateToPoint.action`、`TrackPath.action` |
| 场景与共享规划算法 | `navigation/scripts/scene.py`、`grid_planner.py` |
| Gazebo 举降/反馈代理 | `src/mission_manager/mission_manager/hybrid_execution.py`；联合仿真的 Dock 由实机导航提供 |
| 一键启动与底盘慢速测试 | `scripts/` |

数据管线：`/scan LaserScan → 雷达位姿 /odom 与 TF → Nav2 costmap → 点位名解析 → 4 连通 Path → TrackPath → 树莓派跟踪器 → /cmd_vel Twist → 队友 ESP32 通信与麦轮伺服`。定位不使用 Gazebo 真值或 IMU；仿真真值仅用于执行反馈与误差评估。

主框架只发点位名与 `Dock ENTER/EXIT`；坐标、载货包络和放置顺序由导航维护。`navigation/path` 是观察用 Path，进退架也交给同一 `TrackPath` 跟踪器；底盘订阅 `/cmd_vel`：`linear.x/y` 是车体系前后/左右速度（m/s），`angular.z` 是逆时针转速（rad/s），默认 20 Hz。ESP32 组负责速度传输、轮速换算、编码器 PID、超时停车和实际反馈。串口协议与固件尚需队友交付。

主框架在线接口仍只需要 `robot/status` 的 `ready/fault`；导航内部另需 `base/feedback` 的真实停止与平台状态，格式见交接文档。正常举降中的导航 `ready=false` 不代表健康故障，整机汇总应读取导航 `healthy`。

在已安装 ROS 2 Jazzy 和依赖的机器上，从项目根目录构建与运行实机导航：

```bash
bash scripts/build.sh --robot
bash scripts/run_robot.sh
# 全部实物模块和实测配置就绪后，同一入口启动主框架：
bash scripts/run_robot.sh start_mission:=true
```

默认只启动定位、costmap、导航与跟踪，不启动 Gazebo 或队友模块。实机校准标记为 false、导航点位与真实 QR 留空，必须按实测填写。树莓派拉取源码后本机重新构建；不复制电脑的 build/install。项目路径可以随电脑改变。

Gazebo 联合验证由操作者执行：

```bash
bash scripts/build.sh
bash scripts/run_rack_hybrid.sh
# 查看任务时，另一终端 source ROS/install 并使用 ROS_DOMAIN_ID=0。
# 在相应等待状态用框架 QR 话题触发：
python3 scripts/publish_qr_result.py START
python3 scripts/publish_qr_result.py RACKA_XXXX
python3 scripts/publish_qr_result.py END
# 停止：
bash scripts/stop_rack_sim.sh
```

实机与联合仿真使用相同成功流程：起点前方左转30°扫描 START，识别后自动回正并前往 A，依次识别/进架/升起/送货/放下/退出 A、B、C、D；落点从左到右、从里到外；最后前往终点前方左转30°扫描 END，确认停车后结束任务。不再使用 `/rack/next`。进退架由本包导航执行，升降由机构组执行。架口内使用标定几何与雷达闭环，允许绕过本架的地图膨胀；正常运输仍检查地图。QR 仿真等待可在15秒后自动推进，实机禁止此选项；所有运动和机构完成仍依赖实际结果。

普通货架仿真为 `bash scripts/run_rack_sim.sh`，上游接口 mock 仍为 `ros2 launch robot_bringup simulation.launch.py`（通信域62，流程不变）。货架 Gazebo 使用域0，实机域42，不能混用配置或同时控制同一底盘。

地图策略是场地内未知格可通行，已知障碍仍阻挡；规划检查整个车体/载货包络、5 cm 间距及跟踪余量，不允许驶出场地或收到的地图范围。

本次接口合并已通过五个包构建及离线回归检查。未由开发助手启动仿真或实物进程；实时性能、真实雷达和搬运需现场验收。历史定位导航文档保留为技术记录，当前接口以本节和 [导航接入与部署](docs/导航接入与部署.md) 为准。
