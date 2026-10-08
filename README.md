# DASE7503 第二队机器人项目

这是第二队共同开发物流机器人的代码仓库。机器人按 **A → B → C → D** 的固定顺序搬运货架，从确认 `START` 后开始三分钟计时。

**当前已有：主状态机、模块调用接口、模拟模块、中文配置和测试。真实导航、电机、对准及升降程序还需要接入。**

模拟和实机使用同一个主流程，区别是执行模块：

```text
主状态机 → 接口代码 → 模拟模块 / 真实模块 → 返回结果 → 进入下一步
```

目前三个 ROS 包已在树莓派上编译，109 项测试通过；这些是任务逻辑和模拟通信测试，尚未完成实物搬运验收。

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
| LiDAR、定位与导航 | [navigation.py](src/mission_manager/mission_manager/adapters/navigation.py)、[接口说明](docs/给队友看/模块接口说明.md) | 提供真实到点导航，返回成功／失败，支持取消 |
| 相机与 QR 识别 | [qr.py](src/mission_manager/mission_manager/adapters/qr.py)、[实机目标配置](src/robot_bringup/config/targets_robot.yaml) | 发布完整 QR、图像时间戳和当前识别结果 |
| 局部对准、进入与退出货架 | [Dock.action](src/mission_interfaces/action/Dock.action)、[docking.py](src/mission_manager/mission_manager/adapters/docking.py) | 完成实际对准进入，确认可托举；放下后确认退出 |
| 底盘、ESP32 通信与升降 | [lift.py](src/mission_manager/mission_manager/adapters/lift.py)、[safety.py](src/mission_manager/mission_manager/adapters/safety.py)、[固件说明](firmware/README.md) | 实现真实执行、到位反馈、健康状态和安全停止 |
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
# 安装项目构建和测试依赖；导航消息定义不等于完整导航系统。
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-yaml python3-pytest ros-jazzy-nav2-msgs ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime

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

- 第二队四个货架的完整 QR、实际地图目标位姿，填写 [targets_robot.yaml](src/robot_bringup/config/targets_robot.yaml)。
- 接入真实导航、QR、对准、升降、健康状态和停止模块。
- 模块按 [接口说明](docs/给队友看/模块接口说明.md)报告实际完成结果。升降响应只代表“收到命令”的程序，需要先改造。

`SIMTEAM2` 仅是模拟后缀，不能用于实机。当前实机配置为空，启动时会提示缺失项。

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
