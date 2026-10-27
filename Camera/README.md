# 相机与二维码识别

本目录从 `Camera.zip` 导入，保留相机采图、二维码解码、GTK 桌面界面、ROS 2 原型、测试和树莓派部署依赖。源码按原包保留；缓存、历史识别截图及日志、诊断输出和旧的嵌套压缩包不纳入版本管理。

目前作为独立模块提供，尚未加入仓库的 ROS 包构建和统一启动文件。导入仓库不代表完成实机或整机联调。

## 文件入口

| 文件或目录 | 用途 |
|---|---|
| `pi_camera_qr.py` | Picamera2 直接采图，预览或无界面运行，逐帧输出 JSON |
| `qr_decoder.py` | CLI 和 GUI 共用的解码器，优先 zxing-cpp，缺失时回退 OpenCV |
| `qr_gui_app.py`、`qr_test_app.sh` | GTK4 实时预览、JSON 侧栏与退出按钮 |
| `qr_node.py` | ROS 2 原型，订阅相机图像，使用 OpenCV 解码并发布 QR 结果 |
| `test_*.py` | 解码、假相机流程、GUI 和实机采图检查 |
| `demo_qr/`、`testdata/qr_samples/` | 演示码及解码回归样例；这些后缀不能直接当作本组实机配置 |
| `4.+DASE7503_Appendix.Samples+of+QR+codes.pdf` | 原包附带的二维码样例资料 |
| `vendor/` | zxing-cpp 3.1.1 的 Linux ARM64、CPython 3.12+ 二进制及原始许可证 |
| `ppa_debs/` | 原包附带的 Ubuntu 24.04 ARM64 相机栈安装包，供 `step3_ppa_libcamera.sh` 使用 |

## 树莓派独立运行

在已正确配置相机的 Raspberry Pi OS 上安装 Picamera2 和 OpenCV 后，从仓库进入此目录：

```bash
cd ~/7503robot_ws/Camera
python3 pi_camera_qr.py
# SSH 或无桌面环境：
python3 pi_camera_qr.py --no-preview
# 降低采图尺寸和解码频率：
python3 pi_camera_qr.py --no-preview --width 640 --height 480 --hz 5
```

独立 CLI 依赖 `picamera2`、`libcamera`、OpenCV 和 NumPy。原包的安装方式为 `sudo apt install python3-picamera2 python3-opencv`，适用于提供这些包的 Raspberry Pi OS 软件源；Ubuntu Jazzy 环境需要先配好对应相机栈。可先用 `rpicam-hello --list-cameras` 检查枚举。

OpenCV 预览窗口中按 `q` 退出、`s` 保存截图，或按 Ctrl+C 退出。GTK 界面另需 PyGObject 和较新 GTK4（建议 GTK 4.12+），可运行：

```bash
bash qr_test_app.sh
```

`vendor/` 的原生模块不能用于 Windows 或 x86 电脑。若在其他平台测试 zxing-cpp，应在单独副本中不部署该目录，再安装与本机匹配的 `zxing-cpp==3.1.1`；原代码优先加载本地 `vendor/`，不兼容时会回退 OpenCV，可能无法通过全部模糊、小码测试。

## 输出与主框架接口

CLI 的标准输出逐帧打印 JSON，最新结果同时写入 `qr_evidence/latest.json` 和当次运行目录的 `latest.json`：

```json
{
  "image_stamp": {"sec": 1791450000, "nanosec": 100000000},
  "status": "ok",
  "detections": [{"raw": "RACKA_实际后缀"}]
}
```

没看到有效码时仍输出 `detections: []`。运行目录另存截图、对应 JSON 和识别历史 `detections.jsonl`。退出后最新文件会保留旧值，消费者必须检查时间戳。格式有效不代表属于本组，实机目标必须填写完整二维码字符串。

CLI 和 GUI 不发布 ROS 话题，也不控制电机或升降。`qr_node.py` 需要 ROS 2、`rclpy`、`cv_bridge`、`sensor_msgs`、`std_msgs` 和 OpenCV；先由相机驱动发布 `sensor_msgs/msg/Image`，再手动启动识别节点。例如，在已 source ROS 环境的终端，从仓库根目录执行：

```bash
export ROS_DOMAIN_ID=42
python3 Camera/qr_node.py --ros-args \
  -p image_topic:=/camera/image_raw \
  -r /qr/detections:=/team2/robot/qr/detections \
  -r /qr/text:=/team2/robot/qr/text
```

`image_topic` 应替换成实际相机驱动的话题。节点在图像消息中取采图时间；主框架按完整字符串匹配、过滤旧图并确认连续帧。完整约定见[模块接口说明](../docs/给队友看/模块接口说明.md#qr-输出)，实机目标配置见 [targets_robot.yaml](../src/robot_bringup/config/targets_robot.yaml)。

## 验证

在依赖齐全的目标环境中，从 `Camera/` 执行已有测试：

```bash
python3 test_qr_decode.py
python3 test_robust_decode.py
python3 test_e2e_fake_camera.py
# 需要 GTK 桌面环境，使用假相机：
python3 test_gui_fake_camera.py
# 需要实际连接的相机，会保存 test_capture.png：
python3 test_picamera2.py
```

`test_qr_decode.py` 和假相机流程测试仍会导入 `picamera2`、`libcamera`，因此不等于可直接在任意电脑上运行。强化解码测试覆盖原始样例、模糊、旋转、倾斜、压缩、低对比度、多码及 PDF 页面样例。

2026-10-27 导入时，在 Windows Python 3.12、OpenCV 5.0 和本机版 zxing-cpp 3.1.1 下，40 项强化解码场景、6 项基础解码/分类、假相机端到端流程及 10 个 Python 文件的语法检查通过。测试进程预加载 Windows 解码依赖，用导入占位替代 `libcamera`、`picamera2`，采图由原测试的假相机提供；未验证真实相机、ROS、GTK 或 ARM64 二进制。

## 原包中的相机修复脚本

`step1_overlay_and_deps.sh`、`step2_cam0.sh`、`step3_ppa_libcamera.sh` 和 `postreboot_report.sh` 是特定树莓派环境的历史修复材料，保留供维护者检查。前两份会修改启动配置并重启；第三份会移除冲突的系统相机包并安装 `ppa_debs/`；报告脚本会更新报告并删除对应 crontab 条目。它们不是通用安装入口，不应按编号无条件执行。本次导入未执行这些脚本。
