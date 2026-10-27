#!/usr/bin/env python3
"""GUI test: fake camera feeds a QR frame into qr_gui_app for ~2 seconds.

Asserts the sidebar TextBuffer shows the standard JSON, the status line
names the code, the video paintable is set, and the 结束 button exists.
"""
import json
import sys
import tempfile
import time
from pathlib import Path

import cv2
from gi.repository import GLib
import numpy as np

import pi_camera_qr
import qr_gui_app


class FakeCamera:
    camera_controls = {'AfMode': object()}

    def __init__(self, frame):
        self.frame = frame

    def create_video_configuration(self, main=None):
        return {'main': main}

    def configure(self, config):
        pass

    def set_controls(self, controls):
        pass

    def start(self):
        pass

    def capture_array(self, stream='main'):
        time.sleep(0.05)
        return self.frame.copy()

    def stop(self):
        pass

    def close(self):
        pass


def make_qr_frame(text, width=1280, height=720):
    qr = cv2.QRCodeEncoder_create().encode(text)
    scale = max(1, height // max(qr.shape))
    qr = cv2.resize(qr, None, fx=scale, fy=scale,
                    interpolation=cv2.INTER_NEAREST)
    canvas = np.full((height, width), 255, np.uint8)
    y = (height - qr.shape[0]) // 2
    x = (width - qr.shape[1]) // 2
    canvas[y:y + qr.shape[0], x:x + qr.shape[1]] = qr
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def main():
    text = 'RACKA_7'
    qr_gui_app.Picamera2 = lambda index: FakeCamera(make_qr_frame(text))

    with tempfile.TemporaryDirectory() as tmp:
        # --duration is only a safety net; the 结束 button must quit earlier.
        args = qr_gui_app.parse_args(
            ['--duration', '9', '--hz', '10', '--output', tmp])
        app = qr_gui_app.QRApp(args)

        def click_quit():
            app.quit_button.emit('clicked')
            return False

        started = time.monotonic()
        GLib.timeout_add(3500, click_quit)
        app.run([sys.argv[0]])
        elapsed = time.monotonic() - started
        assert elapsed < 6.5, f'结束 button did not quit in time ({elapsed:.1f}s)'

        buffer_text = app.text_buffer.get_text(
            app.text_buffer.get_start_iter(),
            app.text_buffer.get_end_iter(), False)
        payload = json.loads(buffer_text)
        assert payload['status'] == 'ok', payload
        assert payload['detections'] == [{'raw': text}], payload
        assert set(payload) == {'image_stamp', 'status', 'detections'}, payload

        status = app.status_label.get_text()
        assert text in status, status
        assert '1 个检测' in status, status
        assert app.picture.get_paintable() is not None, 'no video frame'
        assert app.quit_button.get_label() == '结束', app.quit_button.get_label()

        print('PASS: GTK sidebar JSON + status + 结束 button + video')
        print('  sidebar:', json.dumps(payload, ensure_ascii=False))
        print('  status :', status)
        print(f'  button : {app.quit_button.get_label()} '
              f'(quit after {elapsed:.1f}s)')


if __name__ == '__main__':
    main()
