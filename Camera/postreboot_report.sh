#!/bin/bash
# 一次性「重启后相机检测」报告脚本（v2：相机已接到 CAM/DISP 0）
# 由 Claude Code 创建于 2026-10-05；运行后自动删除自己的 crontab 项
# v2 修复：/proc/device-tree 是符号链接，find/grep -r 默认不跟随 → 改用 /sys/firmware/devicetree/base
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
sleep 20
DT=/sys/firmware/devicetree/base
OUT="$(cd "$(dirname "$0")" && pwd)/postreboot_report.txt"
{
  echo "===== POST-REBOOT CAMERA REPORT (v2, expect CAM/DISP 0) ====="
  date
  echo "kernel: $(uname -r)"
  uptime
  echo
  echo "--- overlay 配置（应为 dtoverlay=imx708,cam0） ---"
  grep -n 'camera_auto_detect\|dtoverlay=imx708' /boot/firmware/config.txt
  echo
  echo "--- [cam1 总线 i2c@80000 / i2c-11] 设备树子节点（应为空） ---"
  ls "$DT/axi/pcie@120000/rp1/i2c@80000/" 2>&1
  echo
  echo "--- [cam0 总线 i2c@88000 / i2c-10] 设备树子节点（应出现 imx708@1a 和 dw9817@c） ---"
  ls "$DT/axi/pcie@120000/rp1/i2c@88000/" 2>&1
  echo "imx708@1a status: $(tr -d '\0' < "$DT/axi/pcie@120000/rp1/i2c@88000/imx708@1a/status" 2>/dev/null)"
  echo
  echo "--- sysfs i2c 设备 ---"
  ls /sys/bus/i2c/devices/ 2>/dev/null
  echo
  echo "--- ★ 内核相机日志（关键：imx708 chip id 是否读取成功、无 -5） ---"
  journalctl -k -b --no-pager 2>&1 | grep -iE "imx708|dw9817|cam[01]_reg|rp1.?cfe|csi@" | head -40
  echo
  echo "--- 相机相关模块 ---"
  lsmod | grep -iE "imx708|rp1_cfe|pisp|dw9807|v4l2_fwnode"
  echo
  echo "--- video/media 设备节点及名称 ---"
  ls /dev/media* /dev/video* 2>/dev/null
  for f in /sys/class/video4linux/video*/name; do echo "$(basename "$(dirname "$f")"): $(cat "$f")"; done 2>/dev/null
  echo
  echo "--- cam -l（libcamera 枚举；Ubuntu 0.2.0 缺 pisp 管线时预计仍为空，属已知问题） ---"
  cam -l 2>&1 | head -25
  echo
  echo "===== END ====="
} > "$OUT" 2>&1

# 一次性任务：自删 crontab 项
crontab -l 2>/dev/null | grep -v "postreboot_report" | crontab -
