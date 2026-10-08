#!/usr/bin/env python3
"""Read-only ROS 2 feedback bridge for the exact simulator behind the dashboard.

No command subscription, motor transport, or second physics engine is created.
"""
import argparse
import json
import math
import signal
import time
import urllib.request
from urllib.parse import urlparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINTS = [f'joint{i}' for i in range(1, 7)] + ['gripper_joint1', 'gripper_joint2']


def finite_vector(value, size):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError('Invalid joint feedback dimensions')
    out = [float(x) for x in value]
    if not all(math.isfinite(x) for x in out):
        raise ValueError('Non-finite joint feedback')
    return out


class FeedbackBridge(Node):
    def __init__(self, url, prefix='/rebot/sim', stale_timeout=.5):
        super().__init__('rebot_sim_feedback_bridge')
        parsed = urlparse(url)
        if parsed.scheme != 'http' or parsed.hostname not in ('localhost', '127.0.0.1', '::1'):
            raise ValueError('This bridge only reads the local simulation server')
        self.url = url.rstrip('/') + '/state'
        self.timeout = stale_timeout
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.actual_pub = self.create_publisher(JointState, prefix+'/joint_states', qos_profile_sensor_data)
        self.target_pub = self.create_publisher(JointState, prefix+'/target_joint_states', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, prefix+'/status', 10)
        self.last_key = None
        self.last_status = 0.
        self.was_healthy = None
        self.latest = None
        self.create_timer(.04, self.poll)
        self.get_logger().info(f'Simulation feedback: {self.url} -> {prefix}; no hardware or command output')

    def health(self, healthy, reason='', source=None):
        now = time.monotonic()
        if now-self.last_status < .2 and healthy == self.was_healthy:
            return
        self.last_status = now
        self.was_healthy = healthy
        state = source or self.latest or {}
        stamp = state.get('feedback_time_unix_s')
        msg = dict(connected=healthy, reason=reason, feedback_source='simulation',
                   hardware_connected=False, stream_id=state.get('stream_id'),
                   sequence=state.get('sequence'), sim_time_s=state.get('sim_time'),
                   source_time_unix_s=stamp,
                   source_age_s=max(0.,time.time()-stamp) if isinstance(stamp,(int,float)) and math.isfinite(stamp) else None,
                   job=state.get('job'))
        if healthy:
            msg['max_tracking_error_rad'] = max(abs(a-b) for a,b in zip(state['q_rad'],state['target_rad']))
        self.status_pub.publish(String(data=json.dumps(msg, ensure_ascii=False, allow_nan=False)))

    def poll(self):
        try:
            with self.opener.open(self.url, timeout=.25) as response:
                state = json.load(response)
            if state.get('backend') != 'mujoco' or state.get('feedback_source') != 'simulation':
                raise ValueError('Unexpected feedback source')
            if state.get('error'):
                raise ValueError('Simulator error: '+str(state['error']))
            stamp = float(state['feedback_time_unix_s'])
            age = time.time()-stamp
            if not math.isfinite(stamp) or not -.1 <= age <= self.timeout:
                raise ValueError('Simulation feedback timestamp is stale')
            actual = finite_vector(state['q_rad'],6)
            target = finite_vector(state['target_rad'],6)
            velocity = finite_vector(state['velocity_rad_s'],6)
            fingers = finite_vector(state['finger_positions_m'],2)
            finger_target = finite_vector([state['finger_target_m']],1)[0]
            key = (state['stream_id'], int(state['sequence']))
            if self.last_key and key[0] == self.last_key[0] and key[1] < self.last_key[1]:
                raise ValueError('Simulation sample sequence moved backwards')
            self.latest = state
            if key != self.last_key:
                # Both streams carry the SAME source timestamp. Do not publish a
                # new timestamp on an old/frozen measurement to fake liveness.
                msg = JointState()
                stamp_ns = int(stamp*1_000_000_000)
                msg.header.stamp.sec = stamp_ns//1_000_000_000
                msg.header.stamp.nanosec = stamp_ns%1_000_000_000
                msg.header.frame_id = 'rebot_sim/base_link'
                msg.name = JOINTS
                msg.position = actual+fingers
                # Finger velocities are not supplied by this API; omit the
                # whole velocity field rather than inventing measured zeros.
                self.actual_pub.publish(msg)
                desired = JointState()
                desired.header = msg.header
                desired.name = JOINTS
                desired.position = target+[finger_target,finger_target]
                self.target_pub.publish(desired)
                self.last_key = key
            self.health(True, source=state)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.health(False, str(exc))
            # No joint-state publications while disconnected or stale.


def main():
    p = argparse.ArgumentParser(description='把网页仿真实际反馈与目标分别发布到 ROS 2')
    p.add_argument('--url', default='http://127.0.0.1:8765')
    p.add_argument('--prefix', default='/rebot/sim')
    args, ros_args = p.parse_known_args()
    rclpy.init(args=ros_args)
    node = FeedbackBridge(args.url, args.prefix)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch may forward a second SIGINT during orderly teardown.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
