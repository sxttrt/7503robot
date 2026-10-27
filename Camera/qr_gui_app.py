#!/usr/bin/env python3
"""GTK4 desktop app: live camera preview + JSON sidebar + exit button.

Left pane: live camera with QR boxes. Right sidebar: the standard per-frame
JSON (same object printed to stdout and written to qr_evidence/latest.json).
Click 结束 or press Esc/q to quit. Evidence PNG/JSON/JSONL are written exactly
like the CLI version.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import threading
import time
import traceback

import cv2
import gi

gi.require_version('Gtk', '4.0')
gi.require_version('Gdk', '4.0')
gi.require_version('GdkPixbuf', '2.0')
from gi.repository import Gdk, GdkPixbuf, Gio, GLib, Gtk
from libcamera import controls
from picamera2 import Picamera2

import pi_camera_qr as core
from qr_decoder import QRDecoder

HERE = Path(__file__).resolve().parent


class QRApp(Gtk.Application):
    def __init__(self, args):
        super().__init__(application_id='local.camera_qr.app',
                         flags=Gio.ApplicationFlags.NON_UNIQUE)
        self.args = args
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.frame = None
        self.payload = None
        self.frame_is_new = False
        self.worker = None
        self.window = None
        self.status_label = None
        self.text_buffer = None
        self.picture = None

    # ---------- window ----------
    def do_activate(self):
        if self.window is None:
            self.build_window()
            GLib.timeout_add(50, self.pump_ui)
        self.window.present()
        if self.worker is None:
            self.worker = threading.Thread(target=self.worker_loop, daemon=True)
            self.worker.start()
        if self.args.duration > 0:
            GLib.timeout_add(int(self.args.duration * 1000), self._auto_quit)

    def build_window(self):
        self.window = Gtk.ApplicationWindow(application=self,
                                            title='QR 扫码测试')
        self.window.set_default_size(1420, 720)
        try:
            self.window.set_icon_from_file(str(HERE / 'qr_icon.png'))
        except Exception:
            pass

        css = Gtk.CssProvider()
        css.load_from_string('picture { background-color: #000000; }')
        display = Gdk.Display.get_default()
        if display is not None:
            Gtk.StyleContext.add_provider_for_display(
                display, css, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

        outer = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=0)

        self.picture = Gtk.Picture()
        self.picture.set_content_fit(Gtk.ContentFit.CONTAIN)
        self.picture.set_hexpand(True)
        self.picture.set_vexpand(True)
        outer.append(self.picture)
        outer.append(Gtk.Separator(orientation=Gtk.Orientation.VERTICAL))

        sidebar = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        sidebar.set_size_request(380, -1)
        sidebar.set_margin_top(12)
        sidebar.set_margin_bottom(12)
        sidebar.set_margin_start(12)
        sidebar.set_margin_end(12)

        title = Gtk.Label(xalign=0)
        title.set_markup('<b>实时返回 JSON</b>')
        sidebar.append(title)

        source = Gtk.Label(xalign=0, label='qr_evidence/latest.json')
        source.add_css_class('dim-label')
        sidebar.append(source)

        self.status_label = Gtk.Label(xalign=0, label='等待摄像头…')
        self.status_label.add_css_class('dim-label')
        sidebar.append(self.status_label)

        self.text_view = Gtk.TextView()
        self.text_view.set_editable(False)
        self.text_view.set_cursor_visible(False)
        self.text_view.set_monospace(True)
        self.text_view.set_wrap_mode(Gtk.WrapMode.WORD_CHAR)
        self.text_view.set_left_margin(8)
        self.text_view.set_right_margin(8)
        self.text_view.set_top_margin(8)
        self.text_view.set_bottom_margin(8)
        self.text_buffer = self.text_view.get_buffer()
        self.text_buffer.set_text('{}')

        scrolled = Gtk.ScrolledWindow()
        scrolled.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        scrolled.set_vexpand(True)
        scrolled.set_child(self.text_view)
        sidebar.append(scrolled)

        hint = Gtk.Label(xalign=0, label='按 Esc 或 q 也可结束')
        hint.add_css_class('dim-label')
        sidebar.append(hint)

        self.quit_button = Gtk.Button(label='结束')
        self.quit_button.add_css_class('destructive-action')
        self.quit_button.connect('clicked', self.on_quit)
        sidebar.append(self.quit_button)

        outer.append(sidebar)
        self.window.set_child(outer)

        keys = Gtk.EventControllerKey()
        keys.connect('key-pressed', self.on_key)
        self.window.add_controller(keys)
        self.window.connect('close-request', self.on_close)

    def on_key(self, controller, keyval, keycode, state):
        if keyval in (Gdk.KEY_q, Gdk.KEY_Escape):
            self.on_quit()
            return True
        return False

    def on_close(self, *args):
        self.on_quit()
        return False

    def on_quit(self, *args):
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        if self.worker is not None and self.worker.is_alive():
            self.worker.join(timeout=2.0)
        self.quit()

    def _auto_quit(self):
        self.on_quit()
        return False

    # ---------- sidebar / video updates ----------
    def pump_ui(self):
        with self.lock:
            is_new = self.frame_is_new
            frame = self.frame
            payload = self.payload
            self.frame_is_new = False
        if is_new and frame is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            pixbuf = GdkPixbuf.Pixbuf.new_from_bytes(
                GLib.Bytes.new(rgb.tobytes()), GdkPixbuf.Colorspace.RGB,
                False, 8, w, h, w * 3)
            self.picture.set_paintable(Gdk.Texture.new_for_pixbuf(pixbuf))
        if is_new and payload is not None:
            self.text_buffer.set_text(
                json.dumps(payload, ensure_ascii=False, indent=2))
            stamp = payload['image_stamp']
            clock = datetime.fromtimestamp(stamp['sec']).strftime('%H:%M:%S')
            millis = stamp['nanosec'] // 1_000_000
            raws = ', '.join(d['raw'] for d in payload['detections']) or '—'
            self.status_label.set_text(
                f"{payload['status']} · {clock}.{millis:03d} · "
                f"{len(payload['detections'])} 个检测 · {raws}")
        return True

    def show_error(self, message):
        print(f'camera error: {message}', file=sys.stderr, flush=True)
        dialog = Gtk.AlertDialog()
        dialog.set_message('摄像头启动或采集失败')
        dialog.set_detail(message)
        dialog.set_modal(True)
        dialog.show(self.window)
        GLib.timeout_add(3000, self._auto_quit)

    # ---------- camera worker ----------
    def worker_loop(self):
        camera = None
        started = False
        try:
            out_root = Path(self.args.output).expanduser()
            session = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            out = out_root / session
            out.mkdir(parents=True, exist_ok=True)

            camera = Picamera2(self.args.camera)
            config = camera.create_video_configuration(
                main={'size': (self.args.width, self.args.height),
                      'format': 'RGB888'})
            camera.configure(config)
            if 'AfMode' in camera.camera_controls:
                camera.set_controls({'AfMode': controls.AfModeEnum.Continuous})
            camera.start()
            started = True
            time.sleep(2)
            print(f'Camera started. Evidence: {out.resolve()}',
                  file=sys.stderr, flush=True)

            decoder = QRDecoder()
            saved = set()
            last_report = {}
            with (out / 'detections.jsonl').open('a', encoding='utf-8') as log:
                while not self.stop_event.is_set():
                    cycle = time.monotonic()
                    frame = camera.capture_array('main')
                    captured_at = time.time_ns()
                    h, w = frame.shape[:2]
                    result = {'captured_at_unix_ns': captured_at, 'width': w,
                              'height': h, 'status': 'ok', 'detections': []}
                    annotated = frame.copy()
                    try:
                        decoded = core.decode(decoder, frame)
                    except Exception as exc:
                        result['status'] = 'decode_error'
                        decoded = []
                        print(f'decode failed: {exc}', file=sys.stderr, flush=True)
                    for raw, points in decoded:
                        kind, rack = core.classify(raw)
                        center = points.mean(axis=0)
                        item = {
                            'raw': raw, 'kind': kind, 'rack': rack,
                            'valid_format': kind != 'unknown',
                            'corners_px': points.astype(float).tolist(),
                            'center_px': center.astype(float).tolist(),
                            'horizontal_error_normalized': float(
                                (center[0] - w / 2) / (w / 2)),
                        }
                        result['detections'].append(item)
                        color = (0, 255, 0) if kind != 'unknown' else (0, 0, 255)
                        cv2.polylines(annotated, [points.astype('int32')],
                                      True, color, 2)
                        x, y = points.min(axis=0).astype(int)
                        cv2.putText(annotated, raw, (max(5, x), max(25, y - 10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                        if cycle - last_report.get(raw, -10) > 1:
                            print(f'{kind.upper()}: {raw} | '
                                  f'center={center.tolist()}',
                                  file=sys.stderr, flush=True)
                            last_report[raw] = cycle

                    standard = core.standard_result(
                        captured_at, result['detections'], result['status'])
                    core.emit_standard(standard, (out / 'latest.json',
                                                  out_root / 'latest.json'))
                    for item in result['detections']:
                        png = core.save_evidence(out, annotated, result,
                                                 item, saved)
                        if png is not None:
                            print(f'SAVED: {png}', file=sys.stderr, flush=True)
                    if result['detections']:
                        log.write(json.dumps(result, ensure_ascii=False) + '\n')
                        log.flush()

                    with self.lock:
                        self.frame = annotated
                        self.payload = standard
                        self.frame_is_new = True
                    time.sleep(max(0, 1 / self.args.hz -
                                   (time.monotonic() - cycle)))
        except Exception as exc:
            traceback.print_exc()
            GLib.idle_add(self.show_error, f'{type(exc).__name__}: {exc}')
        finally:
            if camera is not None:
                try:
                    if started:
                        camera.stop()
                    camera.close()
                except Exception:
                    pass


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--hz', type=float, default=10)
    parser.add_argument('--output', default=str(HERE / 'qr_evidence'))
    parser.add_argument('--duration', type=float, default=0,
                        help='auto close after N seconds (0 = manual)')
    args = parser.parse_args(argv)
    if args.width <= 0 or args.height <= 0 or args.hz <= 0:
        parser.error('width, height and hz must be positive')
    return args


def main():
    args = parse_args(sys.argv[1:])
    app = QRApp(args)
    app.run([sys.argv[0]])


if __name__ == '__main__':
    main()
