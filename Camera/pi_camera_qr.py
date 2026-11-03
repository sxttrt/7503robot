#!/usr/bin/env python3
"""Standalone Camera Module 3 QR demo for Raspberry Pi OS.

Local desktop: python3 pi_camera_qr.py   (live preview, q quits, s snapshots)
SSH:           python3 pi_camera_qr.py --no-preview

Machine-readable output: one JSON object per processed frame on stdout, and
the newest object is written to <output>/latest.json (stable path across
runs) and <output>/<session>/latest.json:

    {"image_stamp": {"sec": <int>, "nanosec": <int>},
     "status": "ok",
     "detections": [{"raw": "RACKA_7"}]}

Human-readable logs go to stderr, so stdout can be piped directly.
Only well-formed START/END/RACK[ABCD]_* codes appear in that stream; every
decoded value is still kept in the rich detections.jsonl history.
This program does not drive motors or publish ROS topics.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import sys
import time

import cv2
from libcamera import controls
from picamera2 import Picamera2

from qr_decoder import QRDecoder


def classify(raw):
    if raw == 'START':
        return 'start', None
    if raw == 'END':
        return 'end', None
    match = re.fullmatch(r'RACK([ABCD])_([A-Za-z0-9]+)', raw)
    return ('rack', match.group(1)) if match else ('unknown', None)


def decode(detector, frame):
    """Decode with QRDecoder, or legacy OpenCV detector for old callers."""
    if isinstance(detector, QRDecoder):
        return detector.decode(frame)
    found = []
    ok, strings, corners, _ = detector.detectAndDecodeMulti(frame)
    if ok and corners is not None:
        found = [(raw, points.reshape(4, 2))
                 for raw, points in zip(strings, corners) if raw]
    if not found:
        raw, points, _ = detector.detectAndDecode(frame)
        if raw and points is not None:
            found = [(raw, points.reshape(4, 2))]
    return found


def standard_result(captured_at_ns, detections, status='ok'):
    """Public per-frame result: ROS-style stamp, valid code raws only."""
    return {
        'image_stamp': {
            'sec': captured_at_ns // 1_000_000_000,
            'nanosec': captured_at_ns % 1_000_000_000,
        },
        'status': status,
        'detections': [{'raw': item['raw']} for item in detections
                       if item['valid_format']],
    }


def write_json_atomic(path, payload):
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                    encoding='utf-8')
    temp.replace(path)


def emit_standard(payload, paths):
    """Publish one frame result: atomic latest.json copies + one stdout line."""
    for path in paths:
        write_json_atomic(path, payload)
    sys.stdout.write(json.dumps(payload, ensure_ascii=False,
                                separators=(',', ':')) + '\n')
    sys.stdout.flush()


def save_evidence(out, annotated, result, item, saved):
    """Save annotated PNG + rich JSON once per valid raw. Return PNG path or None."""
    raw = item['raw']
    if not item['valid_format'] or raw in saved:
        return None
    label = re.sub(r'[^A-Za-z0-9_-]', '_', raw)[:64]
    digest = hashlib.sha256(raw.encode()).hexdigest()[:8]
    stem = out / f'{label}_{digest}'
    if not cv2.imwrite(str(stem.with_suffix('.png')), annotated):
        raise OSError('Cannot save evidence PNG')
    stem.with_suffix('.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    saved.add(raw)
    return stem.with_suffix('.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-preview', action='store_true', help='Use for SSH')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--hz', type=float, default=10)
    parser.add_argument('--output', default='./qr_evidence')
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0 or args.hz <= 0:
        parser.error('Width, height and hz must be positive')
    out_root = Path(args.output).expanduser()
    session = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out = out_root / session
    out.mkdir(parents=True, exist_ok=True)
    decoder = QRDecoder()
    camera = Picamera2(args.camera)
    started = False
    saved = set()
    last_report = {}
    try:
        # Picamera2 RGB888 arrays use B,G,R byte order, suitable for OpenCV.
        config = camera.create_video_configuration(
            main={'size': (args.width, args.height), 'format': 'RGB888'})
        camera.configure(config)
        if 'AfMode' in camera.camera_controls:
            camera.set_controls({'AfMode': controls.AfModeEnum.Continuous})
        camera.start()
        started = True
        time.sleep(2)
        print(f'Camera started. Evidence: {out.resolve()}', file=sys.stderr, flush=True)
        print('Preview: press q to quit, s to save current view. Ctrl+C also exits.',
              file=sys.stderr, flush=True)
        if not args.no_preview:
            cv2.namedWindow('Camera QR', cv2.WINDOW_NORMAL)
        with (out / 'detections.jsonl').open('a', encoding='utf-8') as log:
            while True:
                cycle = time.monotonic()
                frame = camera.capture_array('main')
                captured_at = time.time_ns()
                h, w = frame.shape[:2]
                result = {'captured_at_unix_ns': captured_at, 'width': w,
                          'height': h, 'status': 'ok', 'detections': []}
                annotated = frame.copy()
                try:
                    decoded = decode(decoder, frame)
                except Exception as exc:
                    result['status'] = 'decode_error'
                    decoded = []
                    print(f'decode failed: {exc}', file=sys.stderr, flush=True)
                for raw, points in decoded:
                    kind, rack = classify(raw)
                    center = points.mean(axis=0)
                    item = {
                        'raw': raw, 'kind': kind, 'rack': rack,
                        'valid_format': kind != 'unknown',
                        'corners_px': points.astype(float).tolist(),
                        'center_px': center.astype(float).tolist(),
                        'horizontal_error_normalized': float((center[0] - w / 2) / (w / 2)),
                    }
                    result['detections'].append(item)
                    color = (0, 255, 0) if kind != 'unknown' else (0, 0, 255)
                    cv2.polylines(annotated, [points.astype('int32')], True, color, 2)
                    x, y = points.min(axis=0).astype(int)
                    cv2.putText(annotated, raw, (max(5, x), max(25, y - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    if cycle - last_report.get(raw, -10) > 1:
                        print(f'{kind.upper()}: {raw} | center={center.tolist()}',
                              file=sys.stderr, flush=True)
                        last_report[raw] = cycle
                cv2.putText(annotated, 'q: quit | s: snapshot', (10, h - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                for item in result['detections']:
                    png = save_evidence(out, annotated, result, item, saved)
                    if png is not None:
                        print(f'SAVED: {png}', file=sys.stderr, flush=True)
                # Published even when detections is empty; consumers must
                # check image_stamp for freshness.
                standard = standard_result(captured_at, result['detections'],
                                           result['status'])
                emit_standard(standard, (out / 'latest.json',
                                         out_root / 'latest.json'))
                if result['detections']:
                    log.write(json.dumps(result, ensure_ascii=False) + '\n')
                    log.flush()
                if not args.no_preview:
                    cv2.imshow('Camera QR', annotated)
                    key = cv2.waitKey(1) & 0xff
                    if key == ord('q'):
                        break
                    if key == ord('s'):
                        name = out / f'manual_{time.time_ns()}.png'
                        if not cv2.imwrite(str(name), annotated):
                            raise OSError('Cannot save manual PNG')
                        print(f'SAVED: {name}', file=sys.stderr, flush=True)
                time.sleep(max(0, 1 / args.hz - (time.monotonic() - cycle)))
    finally:
        if started:
            camera.stop()
        camera.close()
        if not args.no_preview:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
