#!/bin/bash
# 2026-10-05 相机修复第一步：启用 CAM/DISP 1 上的 imx708 overlay + 安装 libcamera 源码构建依赖 + 重启
# 用法：sudo bash ~/Desktop/Camera/step1_overlay_and_deps.sh   （约 1-3 分钟，最后自动重启）
set -u

CFG=/boot/firmware/config.txt

echo "===== [1/3] 相机固件配置 ====="
BK="/boot/firmware/config.txt.bak-$(date +%Y%m%d-%H%M%S)"
cp "$CFG" "$BK" && echo "[ok] 已备份: $BK"

sed -i 's/^camera_auto_detect=.*/camera_auto_detect=0/' "$CFG" && echo "[ok] camera_auto_detect=0（改为显式配置）"

if grep -q '^dtoverlay=imx708' "$CFG"; then
  echo "[ok] dtoverlay=imx708 已存在，跳过"
else
  echo 'dtoverlay=imx708' >> "$CFG" && echo "[ok] 已追加 dtoverlay=imx708（默认端口 = cam1 = CAM/DISP 1）"
fi

echo "--- config.txt 尾部（确认无误）---"
tail -6 "$CFG"
echo

echo "===== [2/3] 安装 libcamera 源码构建依赖 ====="
apt-get update
apt-get install -y \
  cmake meson ninja-build pkg-config \
  libgnutls28-dev openssl libtiff-dev libevent-dev libdrm-dev \
  libyaml-dev libudev-dev nlohmann-json3-dev \
  pybind11-dev python3-dev python3-yaml python3-ply python3-jinja2
echo "[note] 若上面 apt 有报错也不用担心，重启后 Claude 会处理缺失的包"
echo

echo "===== [3/3] 5 秒后重启 ====="
echo "重启完成后，重新打开 Claude 并输入「继续」，会自动检查相机检测结果。"
sleep 5
reboot
