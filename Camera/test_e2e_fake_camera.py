#!/usr/bin/env python3
"""End-to-end test of pi_camera_qr.py with a synthetic camera.

Feeds a generated QR frame through the real main() loop (detection,
classification, evidence PNG/JSON/JSONL writing) without any hardware.
"""
import contextlib
import io
import json
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

import pi_camera_qr


class FakeCamera:
    """Minimal stand-in for Picamera2 that serves one QR frame."""

    def __init__(self, frame, frames=5):
        self.frame = frame
        self.frames_left = frames
        self.camera_controls = {'AfMode': object()}

    def create_video_configuration(self, main=None):
        return {'main': main}

    def configure(self, config):
        pass

    def set_controls(self, controls):
        pass

    def start(self):
        pass

    def capture_array(self, stream='main'):
        if self.frames_left <= 0:
            raise KeyboardInterrupt
        self.frames_left -= 1
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
    frame = make_qr_frame(text)
    fake = FakeCamera(frame)
    pi_camera_qr.Picamera2 = lambda index: fake

    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / 'evidence'
        sys.argv = ['pi_camera_qr.py', '--no-preview', '--hz', '20',
                    '--output', str(out)]
        stdout = io.StringIO()
        try:
            with contextlib.redirect_stdout(stdout):
                pi_camera_qr.main()
        except KeyboardInterrupt:
            pass

        sessions = [p for p in out.iterdir() if p.is_dir()]
        assert len(sessions) == 1, sessions
        session = sessions[0]

        # Standard latest.json schema.
        latest = json.loads((session / 'latest.json').read_text(encoding='utf-8'))
        assert set(latest) == {'image_stamp', 'status', 'detections'}, latest
        assert latest['status'] == 'ok', latest
        assert latest['detections'] == [{'raw': text}], latest
        stamp = latest['image_stamp']
        assert set(stamp) == {'sec', 'nanosec'}, stamp
        assert isinstance(stamp['sec'], int), stamp
        assert isinstance(stamp['nanosec'], int), stamp
        assert stamp['sec'] > 0 and 0 <= stamp['nanosec'] < 1_000_000_000, stamp

        root_latest = json.loads((out / 'latest.json').read_text(encoding='utf-8'))
        assert root_latest == latest, root_latest

        # stdout is a clean JSONL stream, one object per frame.
        stream = [json.loads(line) for line in stdout.getvalue().splitlines()
                  if line]
        assert stream, 'no JSON lines on stdout'
        assert all(obj['status'] == 'ok' for obj in stream), stream
        assert all(set(obj) == {'image_stamp', 'status', 'detections'}
                   for obj in stream), stream
        assert stream[-1]['detections'] == [{'raw': text}], stream[-1]

        pngs = list(session.glob('RACKA_7_*.png'))
        jsons = list(session.glob('RACKA_7_*.json'))
        assert len(pngs) == 1 and len(jsons) == 1, (pngs, jsons)
        log_lines = (session / 'detections.jsonl').read_text(
            encoding='utf-8').strip().splitlines()
        assert log_lines, 'detections.jsonl is empty'
        logged = json.loads(log_lines[-1])
        assert logged['detections'][0]['raw'] == text, logged

        print('PASS: fake camera -> standard JSON stream -> evidence')
        print(f'  latest.json: {json.dumps(latest, ensure_ascii=False)}')
        print(f'  stdout JSON lines: {len(stream)}')
        print(f'  evidence: {pngs[0].name}, {jsons[0].name}, '
              f'detections.jsonl ({len(log_lines)} line(s))')


if __name__ == '__main__':
    main()
