# DASE7503 第二队机器人项目

这是第二队共同开发物流机器人的代码仓库。机器人按 **A → B → C → D** 的固定顺序搬运货架，从确认 `START` 后开始三分钟计时。

**当前已有：主状态机、模块调用接口、模拟模块、中文配置和测试，以及接入框架的雷达定位、地图规划、进退架、树莓派轨迹跟踪与 Gazebo 联合验证。真实电机通信、二维码、升降和整机在线/停止模块由对应队友接入。**

模拟和实机使用同一个主流程，区别是执行模块：

```text
主状态机 → 接口代码 → 模拟模块 / 真实模块 → 返回结果 → 进入下一步
```

当前使用简化接口：**队友完成动作并停止后返回成功；失败时返回原因；收到停止请求就停止当前动作。主程序直接使用完成结果，不再做令牌、测量来源或二次静止确认。**

**状态机向导航发送点位名**，使用 [NavigateToPoint.action](src/mission_interfaces/action/NavigateToPoint.action)：A／B／C／D、DROP_OFF（放置区），实机与联合仿真配置还发送 START_SCAN、FINAL_SCAN。地图、坐标、朝向和路线由导航维护。进退架通过 `Dock` 调用同一导航跟踪器；升降使用 `std_srvs/srv/SetBool`，在线消息为 `{"ready": true, "fault": ""}`。每架到投放点后直接放下，全部搬完后扫描 END。上游通用 mock 的 mission.yaml 保留简化流程；实机使用 mission_robot.yaml，联合仿真使用 mission_rack.yaml。模拟验证不能代替实物搬运验收。

上游原点位接口版曾在树莓派构建并通过147项测试，历史记录见 [构建验证记录（自己看）](docs/自己的记录/构建验证记录（自己看）.md)。2026-10-10 的导航接入版本在 WSL 完成5包构建，以及511项框架/导航离线测试、61项共享算法测试和1项执行器核心测试；2026-10-26 加入速度 JSON 转换后，4个实机包构建成功，557项框架/导航离线测试通过。未运行此次 Gazebo 全流程或串口/实物验收。

## 部署分工与依赖安装

**电脑运行 Gazebo，树莓派只运行实机上位机逻辑。树莓派可以只复制 src 下的 mission_interfaces、mission_manager、robot_navigation、robot_bringup 四个包并重新构建，不需要外层项目标记、scripts 或仿真包。**

完整依赖表、首次 rosdep 初始化、只复制 src 的步骤和双机联调见 [部署与依赖](docs/部署与依赖.md)。Ubuntu 24.04 + ROS 2 Jazzy 为当前支持环境；电脑仿真使用 Gazebo Harmonic（gz-sim8），树莓派不安装 Gazebo。共享算法已放入 robot_navigation/algorithms，仿真资源和执行桥位于 rack_simulation 包内。

| 环节 | 电脑 | 树莓派 |
|---|---|---|
| 第一阶段：联合验证 | Gazebo、雷达/时钟桥接、虚拟执行反馈、评估 | 定位、costmap、规划/进退架、跟踪、JSON、状态机 |
| 第二阶段：真实雷达 | 可用于观察数据 | 接真实 /scan，检查扫描方向、遮挡和定位 |
| 第三阶段：实物场景 | 可用于监控日志 | 接 ESP32、平台与 QR 模块，实际低速联调 |

已经安装 ROS Jazzy、构建工具并完成 rosdep 初始化后，树莓派在工作区执行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/mission_interfaces src/mission_manager src/robot_navigation src/robot_bringup --ignore-src --rosdistro jazzy -y
colcon build --base-paths src --symlink-install --packages-select mission_interfaces mission_manager robot_navigation robot_bringup
source install/setup.bash
ros2 run robot_navigation check_dependencies --profile robot
```

电脑构建全部五包时，使用 rosdep install --from-paths src --ignore-src --rosdistro jazzy -y，再执行 bash scripts/build.sh 或标准 colcon build。qrcode/Pillow/OpenCV、ros_gz、slam_toolbox 和 Nav2 依赖已纳入包声明，不再依赖历史文档补装。

双方 source 各自安装环境后，设 ROS_DOMAIN_ID=0、RMW_IMPLEMENTATION=rmw_fastrtps_cpp、ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET，确认网络互通，然后分别启动：

```bash
# 电脑：只运行仿真前端
ros2 run rack_simulation run_simulation --hybrid --remote-backend
# 树莓派：运行实际定位导航代码，接收电脑的仿真输入
ros2 launch robot_bringup rack_hybrid_backend.launch.py
```

需要电脑独自运行两端时保留 bash scripts/run_rack_hybrid.sh。真实雷达阶段改用 ros2 launch robot_bringup robot.launch.py，默认域42、真实时钟；完整任务加 start_mission:=true。真实 QR、点位和标定标记仍保留待实测，不能把仿真参数当作实机标定。

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

## 下载并运行框架 mock

**不需要树莓派或机器人硬件。**完整模拟需要 Ubuntu 24.04＋ROS 2 Jazzy；Windows 上可先用 VS Code 阅读和修改，运行需准备相应 ROS 环境。

已经安装 ROS 2 Jazzy 并配置软件源后，在 Ubuntu 终端执行：

```bash
# 安装构建工具；ROS Jazzy 本身须先按官方方式安装。
sudo apt update
sudo apt install -y git build-essential cmake python3-colcon-common-extensions python3-rosdep

# 下载到固定工作区名称，便于按说明运行。
git clone https://github.com/sxttrt/7503robot.git ~/7503robot_ws
cd ~/7503robot_ws
source /opt/ros/jazzy/setup.bash
# rosdep 尚未初始化时先 sudo rosdep init，然后 rosdep update --rosdistro jazzy。
rosdep install --from-paths src/mission_interfaces src/mission_manager src/robot_navigation src/robot_bringup --ignore-src --rosdistro jazzy -y
bash scripts/build.sh --robot
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
- 标定已接入的定位导航与进退架，并接入真实 QR、升降、底盘通信、健康状态和停止模块。
- 模块按 [接口说明](docs/给队友看/模块接口说明.md)报告实际完成结果。升降响应只代表“收到命令”的程序，需要先改造。

`SIMTEAM2` 仅是模拟后缀，不能用于实机。当前实机 QR 配置为空，启动时会提示缺失项。

## 修改、提交与日志

各模块完成后，需要在实机上进行测试。需要取用板子、部署程序、烧录固件或现场联调时，可以在微信群里联系我（**刘承韬**）；开发或接口对接中有问题，也可以在群里交流。

建议使用 **VS Code Remote-SSH** 连接树莓派，直接编辑代码、编译和运行测试。树莓派程序部署运行，ESP32 固件需要烧录。

**支持单模块测试，不用等完整机器人做好。**具体操作见 [单模块测试说明](docs/给队友看/单模块测试说明.md)。

直接使用 ROS 启动入口时，日志默认保存在 `~/.ros/7503robot/runtime_logs/`；完整仓库的 run_*.sh 将日志放在本项目 runtime_logs。可用 ROBOT_RUNTIME_DIR 指定路径。每次运行记录状态转换、货架、剩余时间和故障原因。

上传源码、配置、文档和测试即可；`build/`、`install/`、`log/`、运行日志和缓存已由 `.gitignore` 排除。`firmware/README.md` 只是 ESP32 固件目录说明，项目入口是当前这份 README。

整目录复制或压缩仍会带上被 Git 忽略的本机构建文件。不要在新电脑直接 source 复制来的 install/setup.bash；源码新增配置后也需要重新构建。换电脑、换路径或怀疑安装副本过旧时，先停止正在运行的程序，在新终端、项目根目录执行：

```bash
bash scripts/build.sh --clean          # 联合仿真：构建全部包
# 仅部署实机时使用：bash scripts/build.sh --clean --robot
```

`--clean` 将旧 build/install 移到 .build_cache_backups，再从 src 重建；它保留日志、源码和运行记录，不用于释放磁盘空间。脚本清除继承的旧 ROS overlay 路径并重新加载 /opt/ros/jazzy，构建成功后才写入当前路径的 .build-root。构建失败时启动脚本会继续拒绝运行，不要手工改标记或修改旧 install 来绕过检查。

优先通过 git clone 或 GitHub 的 Download ZIP 分发源码。在已提交且工作区干净的 Git 克隆里，也可以运行以下命令，生成只含已提交源码的 ZIP，默认放在项目目录旁边：

```bash
bash scripts/export_source.sh
```

源码包不含 build/install/.build-root、日志、备份或 Git 历史。解压后必须在目标电脑构建。未提交的实测配置需先妥善保存，导出脚本会拒绝遗漏这些修改。

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
| Twist 转 ESP32 速度 JSON | `src/robot_navigation/robot_navigation/velocity_json_bridge.py` |
| 导航点位与实测参数 | `src/robot_bringup/config/navigation_points_robot.yaml`、`src/robot_navigation/config/navigation_robot.yaml` |
| 点位动作与内部轨迹动作 | `src/mission_interfaces/action/NavigateToPoint.action`、`TrackPath.action` |
| 电脑仿真场景 | `src/rack_simulation/navigation/scripts/scene.py` |
| 共享定位与规划算法 | `src/robot_navigation/robot_navigation/algorithms/` |
| Gazebo 举降/反馈代理 | `src/rack_simulation/rack_simulation/hybrid_execution.py`；联合仿真的 Dock 由实机导航提供 |
| 一键启动与底盘慢速测试 | `scripts/` |

数据管线：`/scan LaserScan → 雷达位姿 /odom 与 TF → Nav2 costmap → 点位名解析 → 4 连通 Path → TrackPath → 树莓派跟踪器 → /cmd_vel Twist → velocity_json_bridge → base/cmd_vel_json → 队友 ESP32 通信与麦轮伺服`。定位不使用 Gazebo 真值或 IMU；仿真真值仅用于执行反馈与误差评估。

主框架只发点位名与 `Dock ENTER/EXIT`；坐标、载货包络和放置顺序由导航维护。`navigation/path` 是观察用 Path，进退架也交给同一 `TrackPath` 跟踪器；底盘订阅 `/cmd_vel`：`linear.x/y` 是车体系前后/左右速度（m/s），`angular.z` 是逆时针转速（rad/s），默认 20 Hz。ESP32 组负责速度传输、轮速换算、编码器 PID、超时停车和实际反馈。串口协议与固件尚需队友交付。

实机与联合仿真入口现已自动加入速度 JSON 转换节点。使用 JSON 的底盘通信模块订阅 `/team2/robot/base/cmd_vel_json`（仿真为 `/team2/sim/base/cmd_vel_json`），类型 `std_msgs/msg/String`，data 内容为：

```json
{"linear":{"x":0.10,"y":0.0,"z":0.0},"angular":{"x":0.0,"y":0.0,"z":0.0}}
```

转换节点逐条接收 `/cmd_vel`，不重发缓存的非零命令；默认断流0.3秒或输入非法即输出全零。它不打开串口，通信组将 JSON 字符串发给 ESP32；ESP32 仍需独立的断流停车和实测反馈。完整数据格式、启动和测试方法见 [导航接入与部署](docs/导航接入与部署.md#esp32-接什么数据)。

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
