#!/usr/bin/python3
"""Publish an Orbbec Gemini UVC color stream as ROS CompressedImage.

The Gemini 336L exposes a color UVC endpoint even when the Orbbec ROS SDK is
not installed. This bridge intentionally publishes color only; it does not
pretend that a UVC monochrome/Bayer endpoint is calibrated depth.
"""

from __future__ import annotations

import argparse
import fcntl
from pathlib import Path
import struct
import sys
import time

# The main RS workspace activates a NumPy 2 venv; system OpenCV here is built
# against the Ubuntu NumPy ABI, so keep that venv off this sensor process.
sys.path[:] = [entry for entry in sys.path if "/.venv/" not in entry]

import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage


DEFAULT_DEVICE = "auto"


def supports_color_capture(device: str | Path) -> bool:
    try:
        with open(device, "rb", buffering=0) as capture:
            for index in range(64):
                # Linux v4l2_fmtdesc: VIDEO_CAPTURE, 64 bytes; query only.
                descriptor = bytearray(64)
                struct.pack_into("=II", descriptor, 0, index, 1)
                fcntl.ioctl(capture.fileno(), 0xC0405602, descriptor, True)
                if descriptor[44:48] in (b"MJPG", b"YUYV"):
                    return True
    except OSError:
        pass
    return False


def resolve_camera_device(requested: str, by_id_dir: str | Path = "/dev/v4l/by-id",
                          video_dir: str | Path = "/sys/class/video4linux") -> str:
    if requested != "auto":
        return requested
    candidates = sorted(Path(by_id_dir).glob("usb-Orbbec_R__Orbbec_Gemini_336L_*-video-index*"))
    # UVC interfaces can share a by-id index. The color node may therefore
    # have no by-id alias, while index0 points to Z16 depth after a reconnect.
    for node in sorted(Path(video_dir).glob("video*")):
        try:
            if node.joinpath("name").read_text().startswith("Orbbec Gemini 336L"):
                candidates.append(Path("/dev") / node.name)
        except OSError:
            continue
    seen = set()
    for device in candidates:
        resolved = device.resolve()
        if resolved not in seen and supports_color_capture(device):
            return str(device)
        seen.add(resolved)
    raise RuntimeError("no Gemini 336L MJPG/YUYV color capture device found")


class OrbbecColorBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("orbbec_gemini_color_bridge")
        self.device = resolve_camera_device(args.device)
        self.topic = args.topic
        self.info_topic = args.info_topic
        self.width = args.width
        self.height = args.height
        self.fps = max(args.fps, 1.0)
        self.jpeg_quality = max(40, min(args.jpeg_quality, 95))
        self.frame_id = args.frame_id
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open color camera: {self.device}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or self.width)
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or self.height)
        self.publisher = self.create_publisher(
            CompressedImage, self.topic, qos_profile_sensor_data
        )
        self.preview_publisher = self.create_publisher(
            CompressedImage, args.preview_topic, qos_profile_sensor_data
        )
        self.info_publisher = self.create_publisher(
            CameraInfo, self.info_topic, qos_profile_sensor_data
        )
        self.frames = 0
        self.last_log = time.monotonic()
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        self.get_logger().info(
            "Gemini color bridge ready: "
            f"device={self.device}, topic={self.topic}, "
            f"size={self.width}x{self.height}, fps={self.fps:g}"
        )

    def publish_frame(self) -> None:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warning("Gemini color frame read failed")
            return
        ok, encoded = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            self.get_logger().warning("Gemini color JPEG encode failed")
            return
        stamp = self.get_clock().now().to_msg()
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.publisher.publish(msg)
        height, width = frame.shape[:2]
        scale = min(1.0, 320 / width, 240 / height)
        preview = cv2.resize(frame, (round(width * scale), round(height * scale)),
                             interpolation=cv2.INTER_AREA) if scale < 1.0 else frame
        ok, preview_jpeg = cv2.imencode('.jpg', preview, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        if ok:
            preview_msg = CompressedImage()
            preview_msg.header = msg.header
            preview_msg.format = 'jpeg'
            preview_msg.data = preview_jpeg.tobytes()
            self.preview_publisher.publish(preview_msg)
        info = CameraInfo()
        info.header = msg.header
        info.width = self.width
        info.height = self.height
        self.info_publisher.publish(info)
        self.frames += 1
        now = time.monotonic()
        if now - self.last_log >= 5.0:
            self.get_logger().info("Gemini color frames published: %d" % self.frames)
            self.last_log = now

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--topic", default="/camera/color/image_raw/compressed")
    parser.add_argument("--preview-topic", default="/camera/color/preview/compressed")
    parser.add_argument("--info-topic", default="/camera/color/camera_info")
    parser.add_argument("--frame-id", default="camera_color_optical_frame")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init(args=[])
    node = None
    try:
        node = OrbbecColorBridge(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
