#!/usr/bin/python3
"""Publish color-only 2D candidates from the Gemini RGB stream.

The output has no metric coordinates and is not suitable for robot motion.
"""

from __future__ import annotations

import argparse
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


COLOR_RANGES = {
    "red": (((0, 90, 45), (10, 255, 255)), ((170, 90, 45), (180, 255, 255))),
    "yellow": (((18, 80, 55), (38, 255, 255)),),
    "blue": (((92, 75, 45), (132, 255, 255)),),
}
COLOR_BGR = {"red": (35, 55, 245), "yellow": (20, 220, 245), "blue": (245, 125, 35)}


def detect_color_blobs(frame_bgr: np.ndarray, *, min_area: int = 220) -> list[dict]:
    """Return conservative HSV blob candidates as image-space rectangles."""
    if frame_bgr is None or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise ValueError("expected a BGR image with three channels")
    height, width = frame_bgr.shape[:2]
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), np.uint8)
    detections: list[dict] = []

    for color, ranges in COLOR_RANGES.items():
        mask = np.zeros((height, width), np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_width < 8 or box_height < 8:
                continue
            ratio = box_width / box_height
            if ratio < 0.4 or ratio > 2.5:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            detections.append({
                "color": color,
                "center_px": [round(moments["m10"] / moments["m00"], 1),
                              round(moments["m01"] / moments["m00"], 1)],
                "bbox_px": [int(x), int(y), int(box_width), int(box_height)],
                "area_px": round(area, 1),
                "aspect_ratio": round(ratio, 3),
            })

    detections.sort(key=lambda item: item["area_px"], reverse=True)
    return detections[:20]


def preview_dimensions(width: int, height: int, *, max_width: int = 320,
                       max_height: int = 240) -> tuple[int, int]:
    scale = min(1.0, max_width / width, max_height / height)
    return round(width * scale), round(height * scale)


def encode_preview_jpeg(frame_bgr: np.ndarray, *, max_width: int = 320,
                        max_height: int = 240, quality: int = 60) -> tuple[int, int, bytes]:
    height, width = frame_bgr.shape[:2]
    output_width, output_height = preview_dimensions(
        width, height, max_width=max_width, max_height=max_height
    )
    preview = cv2.resize(frame_bgr, (output_width, output_height),
                         interpolation=cv2.INTER_AREA) if (output_width, output_height) != (width, height) else frame_bgr
    ok, encoded = cv2.imencode(
        ".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), max(40, min(quality, 95))]
    )
    if not ok:
        raise RuntimeError("camera preview JPEG encode failed")
    preview_height, preview_width = preview.shape[:2]
    return preview_width, preview_height, encoded.tobytes()


def detection_payload(msg: CompressedImage, detections: list[dict], width: int, height: int) -> dict:
    preview_width, preview_height = preview_dimensions(width, height)
    return {
        "schema": "rebotarm.camera_rgb_candidates.v1",
        "source": "gemini336l_color_rgb_2d",
        "frame_id": msg.header.frame_id,
        "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
        "image_width": int(width),
        "image_height": int(height),
        "preview_width": preview_width,
        "preview_height": preview_height,
        "calibration_verified": False,
        "base_frame": None,
        "count": len(detections),
        "detections": detections,
    }


class OrbbecColorDetector(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("orbbec_color_detector")
        self.detections_topic = args.detections_topic
        self.min_area = max(1, args.min_area)
        self.min_interval = 1.0 / max(1.0, args.max_fps)
        self.preview_interval = 1.0 / max(1.0, args.preview_fps)
        self.preview_quality = args.preview_quality
        self.last_processed = 0.0
        self.last_preview = 0.0
        self.publisher = self.create_publisher(String, self.detections_topic, 10)
        self.preview_publisher = self.create_publisher(
            CompressedImage, args.preview_topic, qos_profile_sensor_data
        ) if args.preview_topic else None
        self.subscription = self.create_subscription(
            CompressedImage, args.image_topic, self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"RGB-only detector ready: {args.image_topic} -> {self.detections_topic}; "
            f"preview={args.preview_topic or 'disabled'} at {args.preview_fps:g} Hz; "
            "pixel candidates only, calibration_verified=false"
        )

    def on_image(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        if now - self.last_processed < self.min_interval:
            return
        self.last_processed = now
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("compressed camera frame could not be decoded")
            return
        height, width = frame.shape[:2]
        if self.preview_publisher and now - self.last_preview >= self.preview_interval:
            preview_width, preview_height, preview_bytes = encode_preview_jpeg(
                frame, quality=self.preview_quality
            )
            preview = CompressedImage()
            preview.header = msg.header
            preview.format = "jpeg"
            preview.data = preview_bytes
            self.preview_publisher.publish(preview)
            self.last_preview = now
        payload = detection_payload(msg, detect_color_blobs(frame, min_area=self.min_area), width, height)
        output = String()
        output.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(output)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", default="/camera/color/image_raw/compressed")
    parser.add_argument("--detections-topic", default="/camera/color/detections")
    parser.add_argument("--preview-topic", default="/camera/color/preview/compressed")
    parser.add_argument("--min-area", type=int, default=220)
    parser.add_argument("--max-fps", type=float, default=10.0)
    parser.add_argument("--preview-fps", type=float, default=4.0)
    parser.add_argument("--preview-quality", type=int, default=60)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init(args=[])
    node = None
    try:
        node = OrbbecColorDetector(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
