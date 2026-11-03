#!/usr/bin/env python3
"""picamera2 端到端冒烟测试：配置 -> 启动 -> 连拍 3 帧 -> 存 PNG + 打印统计。"""
import time
from pathlib import Path

import cv2
from libcamera import controls
from picamera2 import Picamera2

print("cameras:", Picamera2.global_camera_info())
cam = Picamera2(0)
print("sensor_modes:", len(cam.sensor_modes))
print("has AfMode:", "AfMode" in cam.camera_controls,
      "| has ExposureTime:", "ExposureTime" in cam.camera_controls)

cfg = cam.create_video_configuration(main={"size": (1280, 720), "format": "RGB888"})
cam.configure(cfg)
if "AfMode" in cam.camera_controls:
    cam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
cam.start()
time.sleep(2)

frames = []
for i in range(3):
    f = cam.capture_array("main")
    frames.append(f)
    time.sleep(0.15)
md = cam.capture_metadata()
cam.stop()

f = frames[-1]
print("frame:", f.shape, f.dtype, "mean:", round(float(f.mean()), 2),
      "min/max:", int(f.min()), int(f.max()))
print("metadata: ExposureTime=", md.get("ExposureTime"),
      "AnalogueGain=", md.get("AnalogueGain"),
      "LensPosition=", md.get("LensPosition"))
out = Path(__file__).resolve().parent / "test_capture.png"
ok = cv2.imwrite(str(out), f)
print("saved:", out, ok)
cam.close()
print("PICAMERA2 SMOKE TEST OK")
