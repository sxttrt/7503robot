#!/bin/bash
# 安装 ppa:manajev/pi5-camera 的 Pi 5 (PiSP) 相机栈 — Ubuntu 24.04 arm64
# 由 Claude Code 创建 2026-10-05；需 root 运行：sudo bash step3_ppa_libcamera.sh
#
# 内容：libpisp-rpi 1.5.0 + libcamera-rpi 0.7.1（含 ipa_rpi_pisp.so、pisp 调优文件、
#        python3.12 绑定）+ rpicam-apps-rpi 1.12.0。
# 事务已用 apt-get -s 模拟通过：须先移除占用同名文件的 Ubuntu 0.2.0 包
# （libcamera-tools / gstreamer1.0-libcamera / libcamera-ipa），gnome-snapshot 连带移除。
# deb 已按 PPA Packages 索引校验过 SHA256（本地路径 ~/Desktop/Camera/ppa_debs/）。
set -euo pipefail
DEBS="$(cd "$(dirname "$0")" && pwd)/ppa_debs"

echo "===== 1/3 移除冲突的 Ubuntu 0.2.0 相机包 ====="
apt-get remove -y gnome-snapshot gstreamer1.0-libcamera libcamera-ipa libcamera-tools

echo
echo "===== 2/3 安装 PPA 包 ====="
cd "$DEBS"
apt-get install -y \
  ./libpisp-rpi_1.5.0-1~ppa202605121209~ubuntu24.04.1_arm64.deb \
  ./libcamera-rpi_0.7.1-1~ppa202604290905~ubuntu24.04.1_arm64.deb \
  ./rpicam-apps-rpi_1.12.0-1~ppa202605121213~ubuntu24.04.1_arm64.deb

echo
echo "===== 3/3 验证 ====="
echo "--- python3 的 libcamera 模块解析到哪个包（期望 dist-packages 的 0.7.1） ---"
python3 - <<'EOF' || true
import libcamera
print("module:", libcamera.__file__)
try:
    print("version:", libcamera.version())
except Exception as e:
    print("version attr:", e)
EOF
echo
echo "--- rpicam-hello --list-cameras（期望列出 imx708） ---"
rpicam-hello --list-cameras 2>&1 | head -25 || true
echo
echo "STEP3 DONE."
