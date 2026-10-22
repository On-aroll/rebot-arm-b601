#!/usr/bin/env python3
"""Offline qualification CLI. No service connection, hardware import or actuation."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'rebotarm_ros2_RS/src/rebotarmcontroller'))

from qualification.common import read_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=['j7-width', 'handeye', 'path', 'grasp'])
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, help='new review JSON file; existing files are not overwritten')
    parser.add_argument('--direction', choices=['opening', 'closing'])
    query = parser.add_mutually_exclusive_group()
    query.add_argument('--width-mm', type=float)
    query.add_argument('--position-rad', type=float)
    args = parser.parse_args()
    try:
        if args.kind != 'j7-width' and (args.direction or args.width_mm is not None or args.position_rad is not None):
            raise ValueError('jaw-width queries are only available for j7-width')
        data = read_json(args.input)
        if args.kind == 'j7-width':
            from qualification.j7_width import analyze, query as lookup
            result = analyze(data)
            if args.width_mm is not None or args.position_rad is not None:
                result['query'] = lookup(result, args.direction, width_mm=args.width_mm, position_rad=args.position_rad)
        elif args.kind == 'handeye':
            from qualification.handeye import analyze
            result = analyze(data)
        elif args.kind == 'path':
            from qualification.path_review import analyze
            result = analyze(data)
        else:
            from qualification.grasp_review import analyze
            result = analyze(data)
        result['input_sha256'] = hashlib.sha256(args.input.read_bytes()).hexdigest()
        payload = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
        if args.output:
            with args.output.open('x') as stream:stream.write(payload)
        else:print(payload, end='')
        return 2 if result.get('validation_passed') is False else 0
    except (ValueError, KeyError, TypeError, OSError, ImportError, RuntimeError) as error:
        print(f'offline review failed: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':sys.exit(main())
