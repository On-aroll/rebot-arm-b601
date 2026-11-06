#!/usr/bin/env python3
"""Generate inert gripper-frame BIT103 previews; never opens CAN.

This geometric preview is not a pen program. Actual writing needs the tool
shaft direction, the pen-tip offset and a collision model including the tool.
"""
import argparse
import json
import math
from pathlib import Path
from writing_plane import tool_plane, plane_point

TEXT = "BIT103"
# Single-line block lettering, normalized coordinates (right, up).
STROKES = {
    "B": [
        [(0, 0), (0, 1)],
        [(0, 1), (.7, 1), (1, .85), (1, .65), (.7, .5), (0, .5)],
        [(0, .5), (.7, .5), (1, .35), (1, .15), (.7, 0), (0, 0)],
    ],
    "I": [[(0, 1), (1, 1)], [(.5, 1), (.5, 0)], [(0, 0), (1, 0)]],
    "T": [[(0, 1), (1, 1)], [(.5, 1), (.5, 0)]],
    "1": [[(.1, .8), (.5, 1), (.5, 0)], [(.15, 0), (.9, 0)]],
    "0": [[(.2, 0), (.8, 0), (1, .2), (1, .8), (.8, 1), (.2, 1), (0, .8), (0, .2), (.2, 0)]],
    "3": [[(0, 1), (.75, 1), (1, .8), (.75, .5), (.25, .5)],
          [(.75, .5), (1, .3), (.75, 0), (0, 0)]],
}


def make_program(pose, *, width=.09, height=.02, lift=.005, spacing=.003,
                 speed=.05):
    plane = tool_plane(pose)
    if not (0 < width <= .1 and 0 < height <= .03 and 0 < lift <= .01 and
            .002 <= spacing <= .005 and .005 <= speed <= .1):
        raise ValueError("书写范围/速度无效；总宽度必须不超过 0.1 m")
    char_width, gap = width / 7.5, width / 7.5 * .3
    steps, strokes = [], []
    for ci, char in enumerate(TEXT):
        for si, normalized in enumerate(STROKES[char], 1):
            points = [plane_point(plane, ci * (char_width + gap) + x * char_width, y * height)
                      for x, y in normalized]
            strokes.append(dict(char=char, stroke=si, points=points))

            def add(point, action):
                steps.append(dict(type="pose", pose=[*point, *pose[3:]], orientation=True,
                                  speed=speed, label=f"{char}·{si:02d}·{action}"))

            add([points[0][i] + lift * plane["normal"][i] for i in range(3)], "移笔")
            add(points[0], "落笔")
            for a, b in zip(points, points[1:]):
                count = max(1, math.ceil(math.dist(a, b) / spacing))
                for j in range(1, count + 1):
                    add([a[k] + (b[k] - a[k]) * j / count for k in range(3)], "书写")
            add([points[-1][i] + lift * plane["normal"][i] for i in range(3)], "抬笔")
    if len(steps) > 250:
        raise ValueError("书写路径超过现有程序 250 步限制")
    program = dict(schema_version="1.0", robot_model="b601_rs", base_frame="base_link",
                   calibration_id="rs-identity-v1", units=dict(position="m", angle="rad", time="s"),
                   name=f"BIT103 · 夹爪坐标几何预览（未标定工具） · {width * 1000:g}mm", writing_plane=plane, steps=steps)
    geometry = dict(text=TEXT, frame_id="base_link", tcp="gripper_end", plane=plane,
                    width_m=width, height_m=height, lift_m=lift,
                    origin_pose=list(pose), strokes=strokes, contact_calibrated=False)
    return program, geometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gripper-frame-preview", action="store_true",
                        help="explicitly generate an uncalibrated gripper-frame geometric preview")
    args = parser.parse_args()
    if not args.gripper_frame_preview:
        parser.error("实际书写须先标定笔杆方向、笔尖偏移及工具碰撞体；未自动套用夹爪轴。仅几何演示可显式使用 --gripper-frame-preview")
    snapshot = json.loads(args.snapshot.read_text())
    state = snapshot["state"]
    if not state["feedback_valid"] or state["owned"] or state["recovery_required"]:
        raise ValueError("记录的起点须反馈有效、无控制者且无恢复锁")
    program, geometry = make_program(state["pose"])
    geometry["captured_at"] = snapshot["captured_at"]
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "bit103.json").write_text(json.dumps(program, ensure_ascii=False, indent=2) + "\n")
    (args.output / "bit103.path.json").write_text(json.dumps(geometry, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(dict(steps=len(program["steps"]), strokes=len(geometry["strokes"]),
                         plane=geometry["plane"], width_mm=geometry["width_m"] * 1000)))


if __name__ == "__main__":
    main()
