#!/usr/bin/env python3
"""Build an OFFLINE fixed-pick proposal. No ROS, CAN, SDK, or execution API.

The proposal deliberately uses a different schema from executable programs.
TCP samples describe desired geometry; they do not certify a Cartesian robot
path, collision clearance, calibrated tool position, or successful grasp.
"""
import argparse
import json
import math
from pathlib import Path

REQUEST_SCHEMA = 'b601-fixed-pick-request/1.0'
PROPOSAL_SCHEMA = 'b601-fixed-pick-proposal/1.0'
UNITS = dict(position='m', angle='rad', time='s')


def number(value, label, low=None, high=None):
    if type(value) not in (int, float):
        raise ValueError(f'{label}: finite number required')
    try:
        value = float(value)
    except OverflowError as error:
        raise ValueError(f'{label}: finite number required') from error
    if not math.isfinite(value):
        raise ValueError(f'{label}: finite number required')
    if (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(f'{label}: out of range')
    return float(value)


def vector(value, size, label):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f'{label}: {size} numbers required')
    return [number(x, label) for x in value]


def fields(value, keys, label):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError(f'{label}: required fields are {sorted(keys)}')


def build_proposal(request):
    fields(request, {'schema_version', 'robot_model', 'base_frame', 'tcp_frame',
                     'units', 'calibration_id', 'pick_pose', 'place_pose',
                     'workspace', 'approach_m', 'clearance_z_m', 'sample_step_m',
                     'joint_speed_rad_s', 'gripper'}, 'request')
    if (request['schema_version'] != REQUEST_SCHEMA or
            request['robot_model'] != 'b601_rs' or
            request['base_frame'] != 'base_link' or
            request['tcp_frame'] != 'gripper_end' or request['units'] != UNITS):
        raise ValueError('use fixed-pick request schema, b601_rs/base_link/gripper_end, m/rad/s')
    calibration = request['calibration_id']
    if not isinstance(calibration, str) or not calibration.strip() or len(calibration) > 100:
        raise ValueError('calibration_id: nonempty version required')
    pick = vector(request['pick_pose'], 6, 'pick_pose XYZ/RPY')
    place = vector(request['place_pose'], 6, 'place_pose XYZ/RPY')
    fields(request['workspace'], {'min', 'max'}, 'workspace')
    lower = vector(request['workspace']['min'], 3, 'workspace.min')
    upper = vector(request['workspace']['max'], 3, 'workspace.max')
    if any(lo >= hi for lo, hi in zip(lower, upper)):
        raise ValueError('workspace: min must be smaller than max')
    approach = number(request['approach_m'], 'approach_m', .001, .3)
    clearance = number(request['clearance_z_m'], 'clearance_z_m')
    step = number(request['sample_step_m'], 'sample_step_m', .001, .02)
    speed = number(request['joint_speed_rad_s'], 'joint_speed_rad_s', .005, .1)
    if clearance < max(pick[2], place[2]) + approach:
        raise ValueError('clearance_z_m must cover both approach poses')
    fields(request['gripper'], {'open_rad', 'close_rad', 'speed_rad_s'}, 'gripper')
    opening = number(request['gripper']['open_rad'], 'open_rad', 0, 5)
    closing = number(request['gripper']['close_rad'], 'close_rad', 0, 5)
    grip_speed = number(request['gripper']['speed_rad_s'], 'gripper speed', .005, 1)
    if closing >= opening:
        raise ValueError('close_rad must be below open_rad (motor radians, not jaw width)')

    stages = []
    waypoint_count = 0

    def barrier(name, requirements):
        stages.append(dict(type='verification_required', name=name,
                           requirements=requirements))

    def checked(pose):
        if any(not lo <= x <= hi for lo, x, hi in zip(lower, pose[:3], upper)):
            raise ValueError('candidate TCP waypoint outside requested workspace')
        return pose

    def move(name, start, goal):
        nonlocal waypoint_count
        checked(goal)
        if start is None or start[3:] != goal[3:]:
            samples = [goal[:]]
            path_kind = 'endpoint_requires_path_planning'
        else:
            checked(start)
            distance = math.dist(start[:3], goal[:3])
            intervals = distance / step
            if not math.isfinite(intervals) or intervals > 240:
                raise ValueError('proposal exceeds 240 TCP samples; split the task')
            count = max(1, math.ceil(intervals))
            if waypoint_count + count + 1 > 240:
                raise ValueError('proposal exceeds 240 TCP samples; split the task')
            samples = [[*(a+(b-a)*i/count for a, b in zip(start[:3], goal[:3])),
                        *goal[3:]] for i in range(count+1)]
            path_kind = 'sampled_tcp_line_requires_path_planning'
        waypoint_count += len(samples)
        if waypoint_count > 240:
            raise ValueError('proposal exceeds 240 TCP samples; split the task')
        stages.append(dict(type='move_candidate', name=name, path_kind=path_kind,
                           joint_speed_rad_s=speed, tcp_waypoints=samples))

    def grip(name, position):
        stages.append(dict(type='gripper_candidate', name=name,
                           motor_position_rad=position, speed_rad_s=grip_speed))

    pick_above = [*pick[:2], pick[2]+approach, *pick[3:]]
    pick_high = [*pick[:2], clearance, *pick[3:]]
    place_high_old_rpy = [*place[:2], clearance, *pick[3:]]
    place_high = [*place[:2], clearance, *place[3:]]
    place_above = [*place[:2], place[2]+approach, *place[3:]]
    barrier('qualify_start_and_scene', [
        'fresh actual joints and recovery cleared by existing recovery procedure',
        'measured TCP, work frame and jaw calibration for this installation',
        'whole robot, tool and payload collision planning, including initial travel',
        'current limits, speed envelope and explicit execution authorization'])
    grip('open_before_pick', opening)
    move('travel_to_pick_clearance', None, pick_high)
    move('approach_pick', pick_high, pick_above)
    move('descend_to_pick', pick_above, pick)
    grip('close_at_pick', closing)
    barrier('confirm_grasp_and_attach_payload', [
        'independent grasp evidence; motor arrival or waiting is insufficient',
        'payload mass, geometry, tool attachment and new path validation'])
    move('lift_from_pick', pick, pick_above)
    move('raise_to_transfer_clearance', pick_above, pick_high)
    move('transfer_at_clearance', pick_high, place_high_old_rpy)
    if place_high_old_rpy[3:] != place_high[3:]:
        move('reorient_above_place', place_high_old_rpy, place_high)
    move('approach_place', place_high, place_above)
    move('descend_to_place', place_above, place)
    grip('open_at_place', opening)
    barrier('confirm_release_and_detach_payload', [
        'object supported and released, with independent evidence',
        'update collision scene and validate retreat'])
    move('retreat_from_place', place, place_above)
    move('return_to_clearance', place_above, place_high)

    return dict(schema_version=PROPOSAL_SCHEMA, robot_model='b601_rs',
                base_frame='base_link', tcp_frame='gripper_end', units=UNITS.copy(),
                calibration_id=calibration, hardware_execution_allowed=False,
                status='offline_geometry_proposal_only', waypoint_count=waypoint_count,
                limitations=[
                    'No IK, self/environment/payload collision, dynamics or timing certification.',
                    'TCP bounds do not bound the robot links, jaws, tool or payload.',
                    'Cartesian samples do not guarantee Cartesian motion between joint targets.',
                    'No grasp/release sensor, force calibration, or verified jaw-width mapping.',
                    'Calibration ID is a reference, not evidence of measured accuracy.',
                    'This schema is intentionally rejected by the existing motion-program importer.'
                ], stages=stages)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('request', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        proposal = build_proposal(json.loads(args.request.read_text()))
    except (ValueError, TypeError) as error:
        parser.error(str(error))
    # Never overwrite an existing result or a recovery/configuration file.
    with args.output.open('x', encoding='utf-8') as handle:
        json.dump(proposal, handle, ensure_ascii=False, allow_nan=False, indent=2)
        handle.write('\n')
    print(f'{args.output}: {proposal["waypoint_count"]} TCP candidates; OFFLINE ONLY')


if __name__ == '__main__':
    main()
