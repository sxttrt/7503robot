#!/usr/bin/env bash
# Desktop launcher: open the camera preview with live QR recognition.
# Logs (stderr) show in the terminal; the machine-readable JSON stream is
# suppressed here — read qr_evidence/latest.json for the latest frame result.
cd "$(dirname "$(readlink -f "$0")")" || exit 1
printf '\033]2;QR 扫码测试\007'
echo "QR 扫码测试：把 START / END / RACK 二维码对准摄像头。"
echo "右侧边栏实时显示返回的 JSON；点「结束」按钮（或 Esc / q）退出。"
echo

python3 qr_gui_app.py --output ./qr_evidence "$@" >/dev/null
status=$?
if [ "$status" -ne 0 ] && [ "$status" -ne 130 ]; then
    echo
    echo "QR 扫码程序异常退出（code $status）"
    read -r -p "按回车关闭窗口..."
fi
