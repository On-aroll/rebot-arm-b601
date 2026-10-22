#!/usr/bin/env python3
"""Local feedback-controller adapter. No ROS/CAN ownership or motor writes.

    python3 controller.py                 # hold output; protocol test
    python3 controller.py --demo-servo    # simple P velocity example, for Fake first

Replace control_step() with your own controller/model. MPC, a ROS process,
or C++ can implement the same HTTP JSON contract without this Python file.
"""
import argparse
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SCHEMA = 'b601-controller/1.0'


def control_step(request, demo_servo=False):
    """Return six joint position (rad) OR velocity (rad/s) references."""
    if request.get('schema') != SCHEMA:
        raise ValueError('unsupported schema')
    q = request['state']['position']
    goal = request['reference']['position']
    if len(q) != 6 or len(goal) != 6 or not all(type(v) in (float, int) and math.isfinite(v) for v in q+goal):
        raise ValueError('six finite positions required')
    output = request['output']
    speed = float(request['limits']['velocity'])
    kp = float(request.get('parameters', {}).get('kp', .6))
    if not math.isfinite(kp) or not 0 <= kp <= 10:
        raise ValueError('demo kp must be finite within 0..10')
    velocity = [max(-speed, min(speed, kp*(g-a))) if demo_servo else 0. for a, g in zip(q, goal)]
    if output == 'joint_velocity':
        values = velocity
    elif output == 'joint_position':
        dt = request['period_s']
        values = [max(lo, min(hi, a+v*dt)) for a, v, (lo, hi) in zip(q, velocity, request['limits']['position'])]
    else:
        raise ValueError('torque/PID outputs are not supported by the CSP adapter')
    return dict(schema=SCHEMA, request_id=request['request_id'], output=output, values=values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8089)
    parser.add_argument('--demo-servo', action='store_true')
    args = parser.parse_args()
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            try:
                if self.path != '/step':
                    raise ValueError('use POST /step')
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 32768:
                    raise ValueError('request size invalid')
                result = control_step(json.loads(self.rfile.read(size)), args.demo_servo)
                code = 200
            except (ValueError, KeyError, TypeError) as exc:
                result, code = dict(error=str(exc)), 400
            body = json.dumps(result, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        def log_message(self, *_):
            pass
    print(f'Controller on http://127.0.0.1:{args.port}/step · {"P servo demo" if args.demo_servo else "hold-only template"}', flush=True)
    ThreadingHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
