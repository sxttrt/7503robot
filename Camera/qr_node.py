#!/usr/bin/env python3
"""ROS 2 QR prototype. Requires rclpy, cv_bridge, sensor_msgs, std_msgs, OpenCV.

Input: /camera/image_raw (sensor-data QoS, latest frame).
Outputs: /qr/detections (String containing JSON, including empty detections),
         /qr/text (String, valid project codes only, observations not triggers).
Evidence: annotated PNG + JSON, once per valid full code per process run.
The task controller must reject old timestamps and require repeated observations.
"""

import hashlib
import json
from pathlib import Path
import re
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def classify(raw):
    if raw == 'START':
        return 'start', None
    if raw == 'END':
        return 'end', None
    match = re.fullmatch(r'RACK([ABCD])_([A-Za-z0-9]+)', raw)
    if match:
        return 'rack', match.group(1)
    return 'unknown', None


class QRNode(Node):
    def __init__(self):
        super().__init__('qr_detector')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('max_decode_hz', 10.0)
        self.declare_parameter('evidence_dir', './qr_evidence')
        hz = float(self.get_parameter('max_decode_hz').value)
        if hz <= 0:
            raise ValueError('max_decode_hz must be positive')
        self.period = 1.0 / hz
        self.previous_decode = 0.0
        self.last_frame = time.monotonic()
        self.warned_stale = False
        self.saved = set()
        self.evidence = Path(self.get_parameter('evidence_dir').value).expanduser()
        self.evidence.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.result_pub = self.create_publisher(String, '/qr/detections', 10)
        self.text_pub = self.create_publisher(String, '/qr/text', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        topic = str(self.get_parameter('image_topic').value)
        self.subscription = self.create_subscription(Image, topic, self.on_image, qos)
        self.watchdog = self.create_timer(1.0, self.check_stream)
        self.get_logger().info(f'Listening on {topic}; evidence: {self.evidence.resolve()}')

    def check_stream(self):
        if time.monotonic() - self.last_frame > 2.0:
            if not self.warned_stale:
                self.get_logger().warning('No image for >2 seconds. QR results are stale.')
                self.warned_stale = True

    def decode(self, frame):
        found = []
        ok, strings, corners, _ = self.detector.detectAndDecodeMulti(frame)
        if ok and corners is not None:
            for raw, points in zip(strings, corners):
                if raw:
                    found.append((raw, points.reshape(4, 2)))
        if not found:
            raw, points, _ = self.detector.detectAndDecode(frame)
            if raw and points is not None:
                found.append((raw, points.reshape(4, 2)))
        return found

    def on_image(self, msg):
        now = time.monotonic()
        self.last_frame = now
        self.warned_stale = False
        if now - self.previous_decode < self.period:
            return
        self.previous_decode = now
        result = {
            'image_stamp': {'sec': int(msg.header.stamp.sec),
                            'nanosec': int(msg.header.stamp.nanosec)},
            'processed_at_ns': int(self.get_clock().now().nanoseconds),
            'frame_id': msg.header.frame_id,
            'width': int(msg.width), 'height': int(msg.height),
            'status': 'ok', 'detections': [],
        }
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            decoded = self.decode(frame)
        except Exception as exc:
            result['status'] = 'decode_error'
            self.get_logger().error(f'Image conversion/decode failed: {exc}')
            self.result_pub.publish(String(data=json.dumps(result)))
            return

        for raw, points in decoded:
            kind, rack = classify(raw)
            center = points.mean(axis=0)
            item = {
                'raw': raw, 'valid_format': kind != 'unknown',
                'kind': kind, 'rack': rack,
                'corners_px': points.astype(float).tolist(),
                'center_px': center.astype(float).tolist(),
                'horizontal_error_normalized': float(
                    (center[0] - msg.width / 2.0) / (msg.width / 2.0)),
            }
            result['detections'].append(item)
            if kind != 'unknown':
                self.text_pub.publish(String(data=raw))
                if raw not in self.saved:
                    self.save_evidence(frame, points, item, result)

        # Always publish a current result, including [] when no code is visible.
        self.result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))

    def save_evidence(self, frame, points, item, result):
        raw = item['raw']
        digest = hashlib.sha256(raw.encode('utf-8')).hexdigest()[:12]
        label = re.sub(r'[^A-Za-z0-9_-]', '_', raw)[:64]
        stem = self.evidence / f'{label}_{digest}'
        annotated = frame.copy()
        cv2.polylines(annotated, [points.astype('int32')], True, (0, 255, 0), 2)
        cv2.putText(annotated, raw, (10, 35), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0), 2)
        try:
            if not cv2.imwrite(str(stem.with_suffix('.png')), annotated):
                raise OSError('Could not write PNG')
            evidence = {k: v for k, v in result.items() if k != 'detections'}
            evidence['detection'] = item
            stem.with_suffix('.json').write_text(
                json.dumps(evidence, ensure_ascii=False, indent=2), encoding='utf-8')
        except OSError as exc:
            self.get_logger().error(f'Evidence save failed: {exc}')
            return
        self.saved.add(raw)
        self.get_logger().info(f'Decoded {raw}; saved {stem.with_suffix(".png")}')


def main():
    rclpy.init()
    node = None
    try:
        node = QRNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
