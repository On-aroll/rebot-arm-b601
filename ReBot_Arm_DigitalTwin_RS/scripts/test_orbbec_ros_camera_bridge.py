#!/usr/bin/python3
"""Small offline/device smoke checks for the Gemini color bridge."""

from pathlib import Path

import orbbec_ros_camera_bridge as bridge
from orbbec_ros_camera_bridge import DEFAULT_DEVICE, resolve_camera_device


def test_auto_selects_stable_gemini_color_interface(tmp_path, monkeypatch):
    first = tmp_path / "usb-Orbbec_R__Orbbec_Gemini_336L_AAA-video-index0"
    second = tmp_path / "usb-Orbbec_R__Orbbec_Gemini_336L_ZZZ-video-index0"
    first.touch()
    second.touch()
    monkeypatch.setattr(bridge, "supports_color_capture", lambda p: True)
    assert DEFAULT_DEVICE == "auto"
    assert resolve_camera_device("auto", tmp_path, tmp_path / "sys") == str(first)


def test_explicit_device_path_is_preserved(tmp_path):
    device = tmp_path / "camera-node"
    assert resolve_camera_device(str(device), tmp_path) == str(device)


def test_auto_does_not_fall_back_to_an_unrelated_camera(tmp_path):
    unrelated = tmp_path / "usb-Other_Camera-video-index0"
    unrelated.touch()
    try:
        resolve_camera_device("auto", tmp_path, tmp_path / "sys")
    except RuntimeError as exc:
        assert "Gemini 336L" in str(exc)
    else:
        raise AssertionError("auto must not select an unrelated camera")


def test_auto_skips_depth_index_zero_for_a_color_endpoint(tmp_path, monkeypatch):
    depth = tmp_path / "usb-Orbbec_R__Orbbec_Gemini_336L_AAA-video-index0"
    color = tmp_path / "usb-Orbbec_R__Orbbec_Gemini_336L_AAA-video-index6"
    depth.touch()
    color.touch()
    monkeypatch.setattr(bridge, "supports_color_capture", lambda p: Path(p) == color,
                        raising=False)
    assert resolve_camera_device("auto", tmp_path, tmp_path / "sys") == str(color)


def test_auto_finds_color_without_a_by_id_alias(tmp_path, monkeypatch):
    node = tmp_path / "sys" / "video6"
    node.mkdir(parents=True)
    node.joinpath("name").write_text("Orbbec Gemini 336L: Orbbec Gemini Color")
    monkeypatch.setattr(bridge, "supports_color_capture", lambda p: str(p) == "/dev/video6")
    assert resolve_camera_device("auto", tmp_path / "by-id", tmp_path / "sys") == "/dev/video6"


def test_auto_does_not_probe_an_unrelated_sysfs_camera(tmp_path, monkeypatch):
    node = tmp_path / "sys" / "video6"
    node.mkdir(parents=True)
    node.joinpath("name").write_text("Other Webcam")
    def reject_probe(device):
        raise AssertionError("unrelated camera must not be opened")
    monkeypatch.setattr(bridge, "supports_color_capture", reject_probe)
    try:
        resolve_camera_device("auto", tmp_path / "by-id", tmp_path / "sys")
    except RuntimeError:
        pass
    else:
        raise AssertionError("unrelated camera must not be selected")


def test_color_detection_rejects_depth_and_accepts_mjpg(tmp_path, monkeypatch):
    device = tmp_path / "capture"
    device.touch()
    formats = [b"Z16 ", b"GREY", b"BA81", b"MJPG"]
    def enumerate_format(fd, request, descriptor, mutate):
        index = int.from_bytes(descriptor[:4], "little")
        if index >= len(formats):
            raise OSError("end of formats")
        descriptor[44:48] = formats[index]
    monkeypatch.setattr(bridge.fcntl, "ioctl", enumerate_format)
    assert bridge.supports_color_capture(device)
    formats.pop()
    assert not bridge.supports_color_capture(device)
