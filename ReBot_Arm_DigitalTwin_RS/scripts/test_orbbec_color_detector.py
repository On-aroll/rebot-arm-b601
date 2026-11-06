#!/usr/bin/python3
"""Offline checks for RGB candidate detection and calibration provenance."""

from argparse import Namespace

import cv2
import numpy as np

from orbbec_color_detector import detection_payload, detect_color_blobs, encode_preview_jpeg


def test_detects_primary_color_blobs_in_pixel_coordinates():
    frame = np.zeros((240, 320, 3), np.uint8)
    cv2.rectangle(frame, (30, 50), (90, 110), (0, 0, 255), -1)
    cv2.rectangle(frame, (130, 50), (190, 110), (0, 255, 255), -1)
    cv2.rectangle(frame, (230, 50), (290, 110), (255, 0, 0), -1)

    found = detect_color_blobs(frame)

    assert {item["color"] for item in found} == {"red", "yellow", "blue"}
    assert all("bbox_px" in item and "center_px" in item for item in found)
    assert all(not any(key in item for key in ("x", "y", "z")) for item in found)


def test_rejects_small_blobs_and_limits_candidates():
    frame = np.zeros((120, 160, 3), np.uint8)
    cv2.rectangle(frame, (4, 4), (9, 9), (0, 0, 255), -1)
    assert len(detect_color_blobs(frame, min_area=220)) == 0

    crowded = np.zeros((360, 480, 3), np.uint8)
    for row in range(6):
        for column in range(8):
            x, y = 5 + column * 58, 5 + row * 58
            cv2.rectangle(crowded, (x, y), (x + 24, y + 24), (0, 0, 255), -1)
    assert len(detect_color_blobs(crowded, min_area=10)) == 20


def test_payload_keeps_rgb_candidates_uncalibrated():
    msg = Namespace(header=Namespace(frame_id="camera_color_optical_frame",
                                     stamp=Namespace(sec=123, nanosec=456)))
    payload = detection_payload(msg, [{"color": "red", "center_px": [50, 60]}], 640, 480)

    assert payload["source"] == "gemini336l_color_rgb_2d"
    assert payload["calibration_verified"] is False
    assert payload["base_frame"] is None
    assert payload["preview_width"] == 320
    assert payload["preview_height"] == 240
    assert payload["stamp"] == {"sec": 123, "nanosec": 456}
    assert not any(key in payload["detections"][0] for key in ("x", "y", "z"))


def test_preview_is_downscaled_and_remains_decodable():
    frame = np.full((480, 640, 3), (20, 120, 220), np.uint8)
    width, height, encoded = encode_preview_jpeg(frame)
    decoded = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR)

    assert (width, height) == (320, 240)
    assert decoded.shape == (240, 320, 3)
