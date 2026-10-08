"""Re-apply the local quoting fix to reBotArmController_ROS2 launch files.

Why this exists
---------------
Upstream `rebotarm_bringup/launch/{fake_rs_bringup,bringup}.launch.py` build the
robot_description with:

    Command(["python3 ", rviz_urdf_compat, " ", urdf_file])

The substituted paths are NOT quoted, so any workspace path containing a space
breaks it. This project lives in "/home/bonvoyage/Desktop/seeed Robotics", so
python3 receives "/home/bonvoyage/Desktop/seeed" and dies with FileNotFoundError.
robot_state_publisher then never gets a robot_description and the TF tree is
empty (observed 2026-10-06).

The fix wraps the substituted paths in double quotes.

The upstream tree here is NOT a git checkout (it was fetched file-by-file via
jsDelivr), so there is no `git apply`. This script is idempotent: run it after
any re-fetch, or at any time -- it reports whether a change was needed.

Usage:
    python3 scripts/patch_rebotarm_launch_quoting.py
"""
import sys
from pathlib import Path

ROOT = Path('/home/bonvoyage/Desktop/seeed Robotics')
PKG = ROOT / 'external/reBotArmController_ROS2/src/rebotarm_bringup/launch'
FILES = ['fake_rs_bringup.launch.py', 'bringup.launch.py']

OLD = 'Command(["python3 ", rviz_urdf_compat, " ", urdf_file])'
NEW = """Command(['python3 "', rviz_urdf_compat, '" "', urdf_file, '"'])"""

changed, already, missing = [], [], []

for name in FILES:
    p = PKG / name
    if not p.exists():
        missing.append(name)
        continue
    text = p.read_text()
    if NEW in text:
        already.append(name)
    elif OLD in text:
        p.write_text(text.replace(OLD, NEW))
        changed.append(name)
    else:
        print('[!!] %s: 既无原式也无修复式，需人工检查' % name)
        sys.exit(1)

for n in changed:
    print('已修复  :', n)
for n in already:
    print('已是最新:', n)
for n in missing:
    print('[!!] 文件不存在:', n)

if missing:
    sys.exit(1)
print('OK')
