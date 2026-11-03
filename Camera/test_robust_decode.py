#!/usr/bin/env python3
"""Robust-decoding regression test using the PDF appendix sample codes.

Covers clean samples, small/blurred/rotated/tilted/low-contrast/JPEG
variants, multi-code frames, and the real PDF page renders at 35% scale.
"""
import cv2
import numpy as np
from pathlib import Path

from qr_decoder import QRDecoder

DATA = Path(__file__).resolve().parent / 'testdata' / 'qr_samples'
W, H = 1280, 720
CODES = ['START', 'RACKA_4X6M', 'RACKB_AM9L', 'RACKC_K8Y3',
         'RACKD_R983', 'END']
decoder = QRDecoder()


def load(name):
    img = cv2.imread(str(DATA / f'{name}.png'))
    assert img is not None, f'missing sample {name}'
    return img


def place(qr, width, angle=0.0, tilt=0.0):
    q = cv2.resize(qr, None, fx=width / qr.shape[1],
                   fy=width / qr.shape[1], interpolation=cv2.INTER_AREA)
    h, w = q.shape[:2]
    if tilt:
        src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        dx, dy = int(w * tilt), int(h * tilt * 0.4)
        dst = np.float32([[dx, dy], [w - dx, dy],
                          [w + dx // 2, h], [-dx // 2, h]])
        m = cv2.getPerspectiveTransform(src, dst)
        pad = max(dx, dy) + 10
        q = cv2.warpPerspective(q, m, (w + 2 * pad, h + 2 * pad),
                                borderValue=(255, 255, 255))
    if angle:
        h, w = q.shape[:2]
        m = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
        cos, sin = abs(m[0, 0]), abs(m[0, 1])
        nw, nh = int(h * sin + w * cos), int(h * cos + w * sin)
        m[0, 2] += nw / 2 - w / 2
        m[1, 2] += nh / 2 - h / 2
        q = cv2.warpAffine(q, m, (nw, nh), borderValue=(255, 255, 255))
    canvas = np.full((H, W, 3), 255, np.uint8)
    y, x = (H - q.shape[0]) // 2, (W - q.shape[1]) // 2
    canvas[y:y + q.shape[0], x:x + q.shape[1]] = q
    return canvas


def check(name, frame, expected):
    found = decoder.decode(frame)
    raws = [raw for raw, _ in found]
    h, w = frame.shape[:2]
    for raw, points in found:
        assert points.shape == (4, 2), (name, points.shape)
        assert np.isfinite(points).all(), (name, points)
        assert (points[:, 0] >= -1).all() and (points[:, 0] <= w).all(), (name, points)
        assert (points[:, 1] >= -1).all() and (points[:, 1] <= h).all(), (name, points)
    missing = set(expected) - set(raws)
    assert not missing, f'{name}: missing {sorted(missing)}, got {raws}'
    print(f'PASS {name:18s} -> {sorted(raws)}')


def main():
    qrs = {code: load(code) for code in CODES}

    for code, qr in qrs.items():
        check(f'clean/{code}', place(qr, 300), {code})

    # Small + rotated + defocused: raw zxing usually misses, enhancement rescues.
    for code, qr in qrs.items():
        frame = cv2.GaussianBlur(place(qr, 80, angle=12, tilt=0.10), (5, 5), 0)
        check(f'tiny80_mix/{code}', frame, {code})

    # Moderate real-world tilt, defocus, compression, low contrast.
    for code, qr in qrs.items():
        check(f'tilt/{code}', place(qr, 220, tilt=0.20), {code})
        frame = cv2.GaussianBlur(place(qr, 260), (13, 13), 0)
        check(f'defocus/{code}', frame, {code})
        frame = place(qr, 300)
        ok, enc = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 15])
        check(f'jpeg/{code}', cv2.imdecode(enc, cv2.IMREAD_COLOR), {code})
        frame = cv2.convertScaleAbs(place(qr, 300), alpha=0.4, beta=77)
        check(f'contrast/{code}', frame, {code})

    # Six codes in one frame.
    sheet = np.full((H, W, 3), 255, np.uint8)
    for i, code in enumerate(CODES):
        q = cv2.resize(qrs[code], (340, 340), interpolation=cv2.INTER_AREA)
        row, col = divmod(i, 3)
        sheet[20 + row * 350:20 + row * 350 + 340,
              20 + col * 400:20 + col * 400 + 340] = q
    check('multi_sheet', sheet, set(CODES))

    # The actual PDF page renders, shrunk to 35% (photo-at-distance proxy).
    page1 = cv2.resize(load('pdf_page_1'), None, fx=0.35, fy=0.35,
                       interpolation=cv2.INTER_AREA)
    check('pdf_page_1@35%', page1, {'START', 'RACKA_4X6M', 'RACKB_AM9L',
                                    'RACKC_K8Y3'})
    page2 = cv2.resize(load('pdf_page_2'), None, fx=0.35, fy=0.35,
                       interpolation=cv2.INTER_AREA)
    check('pdf_page_2@35%', page2, {'RACKD_R983', 'END'})

    # No false positives on a blank frame.
    assert decoder.decode(np.full((H, W, 3), 255, np.uint8)) == []
    print('PASS blank frame      -> no detections')


if __name__ == '__main__':
    main()
