#!/usr/bin/env python3
"""Robust QR decoding shared by the CLI and GTK apps.

Pipeline (first stage that finds a code wins):
  1. zxing-cpp on the raw frame: fast, multi-code, handles rotation,
     downscaling and inverted codes by default.
  2. zxing-cpp on 2x upscaled unsharp-mask grayscale: small/blurred codes.
  3. zxing-cpp on 2x CLAHE grayscale: low contrast / defocused codes.
  4. zxing-cpp on 2x adaptive-threshold grayscale: last resort.
If zxing-cpp is unavailable (vendor/ not deployed), fall back to OpenCV
QRCodeDetector plus WeChatQRCode when available.

decode() returns [(raw, points)], points = float32 4x2 (TL, TR, BR, BL)
in original-frame pixel coordinates.
"""
from pathlib import Path
import sys

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
VENDOR = HERE / 'vendor'
if VENDOR.is_dir() and str(VENDOR) not in sys.path:
    sys.path.insert(0, str(VENDOR))

try:
    import zxingcpp
except Exception:  # keep the app usable without the vendored wheel
    zxingcpp = None

QR_FORMAT = zxingcpp.BarcodeFormat.QRCode if zxingcpp is not None else None


def _dedupe(found):
    seen = set()
    unique = []
    for raw, points in found:
        if raw in seen:
            continue
        seen.add(raw)
        unique.append((raw, points))
    return unique


class QRDecoder:
    def __init__(self):
        self._cv = cv2.QRCodeDetector()
        self._wechat = None
        if zxingcpp is not None:
            return
        # Legacy fallback only when zxing-cpp is missing.
        if hasattr(cv2, 'wechat_qrcode_WeChatQRCode'):
            try:
                self._wechat = cv2.wechat_qrcode_WeChatQRCode()
            except Exception:
                self._wechat = None

    # ---------- public ----------
    def decode(self, frame):
        if zxingcpp is None:
            return self._legacy_decode(frame)
        found = self._zxing_decode(frame, 1.0)
        if found:
            return found
        for kind in ('sharp', 'clahe', 'adaptive'):
            enhanced, scale = self._enhance(frame, kind)
            found = self._zxing_decode(enhanced, scale)
            if found:
                return found
        return []

    # ---------- zxing-cpp ----------
    @staticmethod
    def _zxing_decode(image, scale):
        try:
            results = zxingcpp.read_barcodes(image, formats=QR_FORMAT)
        except Exception:
            return []
        found = []
        for result in results:
            if not result.valid or not result.text:
                continue
            p = result.position
            points = np.array([[p.top_left.x, p.top_left.y],
                               [p.top_right.x, p.top_right.y],
                               [p.bottom_right.x, p.bottom_right.y],
                               [p.bottom_left.x, p.bottom_left.y]],
                              dtype=np.float32)
            found.append((result.text, points / scale))
        return _dedupe(found)

    @staticmethod
    def _enhance(frame, kind):
        gray = (cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if frame.ndim == 3 else frame)
        if kind == 'sharp':
            blur = cv2.GaussianBlur(gray, (0, 0), 3)
            gray = cv2.addWeighted(gray, 1.6, blur, -0.6, 0)
            interp = cv2.INTER_CUBIC
        elif kind == 'clahe':
            gray = cv2.createCLAHE(clipLimit=3.0,
                                   tileGridSize=(8, 8)).apply(gray)
            interp = cv2.INTER_CUBIC
        elif kind == 'adaptive':
            gray = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 31, 7)
            interp = cv2.INTER_NEAREST
        else:
            raise ValueError(f'unknown enhancement: {kind}')
        return cv2.resize(gray, None, fx=2, fy=2,
                          interpolation=interp), 2.0

    # ---------- legacy fallback ----------
    def _legacy_decode(self, frame):
        found = []
        ok, strings, corners, _ = self._cv.detectAndDecodeMulti(frame)
        if ok and corners is not None:
            found += [(raw, points.reshape(4, 2))
                      for raw, points in zip(strings, corners) if raw]
        if not found:
            raw, points, _ = self._cv.detectAndDecode(frame)
            if raw and points is not None:
                found.append((raw, points.reshape(4, 2)))
        if not found and self._wechat is not None:
            try:
                results, points = self._wechat.detectAndDecode(frame)
                found += [(raw, np.asarray(points[i], np.float32).reshape(4, 2))
                          for i, raw in enumerate(results) if raw]
            except Exception:
                pass
        return _dedupe(found)
