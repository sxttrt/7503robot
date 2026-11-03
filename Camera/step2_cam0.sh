#!/bin/bash
# 2026-10-05 相机修复第二步：imx708 overlay 切到 CAM/DISP 0（相机已移到 0 号口）
# 用法： sudo bash ~/Desktop/Camera/step2_cam0.sh [noreboot]
set -u
CFG=/boot/firmware/config.txt

echo "===== [1/2] 修改 $CFG ====="
BK="$CFG.bak-$(date +%Y%m%d-%H%M%S)"
cp "$CFG" "$BK" && echo "[ok] 已备份: $BK"

# dtoverlay=imx708（默认=cam1）→ dtoverlay=imx708,cam0
sed -i 's/^dtoverlay=imx708\b.*$/dtoverlay=imx708,cam0/' "$CFG"

if grep -q '^dtoverlay=imx708,cam0$' "$CFG"; then
  echo "[ok] 已写入: dtoverlay=imx708,cam0"
else
  echo "[ERROR] config.txt 未按预期更新，中止（不重启）。当前相关行："
  grep -n 'imx708' "$CFG"
  exit 1
fi
echo "--- 相关配置行 ---"
grep -n 'camera_auto_detect\|dtoverlay=imx708' "$CFG"

echo
echo "===== [2/2] 重启 ====="
if [ "${1:-}" = "noreboot" ]; then
  echo "[info] 已跳过重启。稍后自行执行: sudo reboot"
  exit 0
fi
echo "15 秒后自动重启（想取消请立刻按 Ctrl-C）。"
echo "重启会结束当前 Claude 会话 —— 重启完成后重新打开 Claude 输入「继续」，"
echo "会自动读取 ~/Desktop/Camera/postreboot_report.txt 里的检测结果。"
sleep 15
reboot
