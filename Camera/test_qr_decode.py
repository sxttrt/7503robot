#!/usr/bin/env python3
"""Offline self-test: encode sample QR payloads, decode them back.

Does not touch the camera. Exercises the same decode()/classify()
functions used by pi_camera_qr.py.
"""
import cv2
import numpy as np

from pi_camera_qr import classify, decode


CASES = {
    'START': ('start', None),
    'END': ('end', None),
    'RACKA_7': ('rack', 'A'),
    'RACKD_abc123': ('rack', 'D'),
    'RACKE_1': ('unknown', None),   # valid QR, wrong format
    'HELLO': ('unknown', None),     # valid QR, wrong format
}


def make_frame(text, size=480):
    qr = cv2.QRCodeEncoder_create().encode(text)
    scale = max(1, size // max(qr.shape))
    qr = cv2.resize(qr, None, fx=scale, fy=scale,
                    interpolation=cv2.INTER_NEAREST)
    canvas = np.full((size, size), 255, np.uint8)
    y = (size - qr.shape[0]) // 2
    x = (size - qr.shape[1]) // 2
    canvas[y:y + qr.shape[0], x:x + qr.shape[1]] = qr
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def main():
    detector = cv2.QRCodeDetector()
    failures = 0
    for text, (kind, rack) in CASES.items():
        found = decode(detector, make_frame(text))
        raws = [raw for raw, _ in found]
        got = classify(raws[0]) if raws else None
        ok = text in raws and got == (kind, rack)
        print(f'{"PASS" if ok else "FAIL"}: {text!r} -> raws={raws} '
              f'classify={got} expected={(kind, rack)}')
        failures += not ok
    raise SystemExit(1 if failures else 0)


if __name__ == '__main__':
    main()
