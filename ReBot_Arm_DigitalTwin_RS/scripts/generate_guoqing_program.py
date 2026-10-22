#!/usr/bin/env python3
"""Generate inert B601-RS writing data; no ROS node or actuator is constructed."""
import json
import math
from pathlib import Path

import numpy as np

from rebotarmcontroller.csp_twin_driver import CheckedKinematics
from rebotarmcontroller.csp_runtime.rebot_native_xyz import gravity_limits_for_mode
from rebotarmcontroller.twin_program import validate_program

ROOT = Path(__file__).resolve().parents[1]
# Hand drawn single-line strokes, x right / y down; no font outlines or raster tracing.
STROKES = {
    '国': [
        [[.10,.08],[.10,.92]], [[.10,.08],[.90,.08],[.90,.92]],
        [[.28,.30],[.72,.30]], [[.30,.52],[.70,.52]],
        [[.50,.30],[.50,.76]], [[.25,.76],[.75,.76]],
        [[.64,.60],[.72,.69]], [[.10,.92],[.90,.92]],
    ],
    '庆': [
        [[.48,.06],[.57,.14]], [[.16,.22],[.90,.22]],
        [[.22,.22],[.21,.57],[.12,.92]], [[.32,.49],[.84,.49]],
        [[.60,.30],[.58,.62],[.48,.79],[.29,.93]],
        [[.59,.59],[.72,.80],[.91,.93]],
    ],
    '快': [
        [[.10,.34],[.06,.56]], [[.25,.31],[.31,.49]],
        [[.20,.10],[.20,.92]], [[.40,.29],[.81,.29],[.81,.58]],
        [[.34,.58],[.92,.58]], [[.60,.10],[.60,.54],[.52,.76],[.36,.93]],
        [[.61,.59],[.75,.78],[.93,.93]],
    ],
    '乐': [
        [[.78,.10],[.28,.23]], [[.28,.23],[.21,.55],[.88,.55]],
        [[.55,.29],[.55,.88],[.44,.93]], [[.36,.70],[.15,.89]],
        [[.71,.70],[.90,.90]],
    ],
}
COLORS = ['#ffc857', '#50e3c2', '#70b7ff', '#ff8fbd']


def make_program(rpy, size=.05, gap=.012, origin=(.082,-.295,.25), lift=.012, spacing=.009):
    steps = []
    for ci, (char, strokes) in enumerate(STROKES.items()):
        def point(p):
            return [round(origin[0]+ci*(size+gap)+p[0]*size, 9),
                    round(origin[1]+(1-p[1])*size, 9), origin[2]]
        for si, stroke in enumerate(strokes, 1):
            def add(p, action):
                steps.append(dict(type='pose', pose=[*p, *rpy], orientation=True,
                                  speed=.1, label=f'{char}·{si:02d}·{action}'))
            start = point(stroke[0])
            add([*start[:2],start[2]+lift], '移笔')
            add(start, '落笔')
            for a, b in zip(stroke, stroke[1:]):
                a, b = np.array(point(a)), np.array(point(b))
                count = math.ceil(float(np.linalg.norm(b-a))/spacing)
                for j in range(1,count+1):
                    add(np.round(a+(b-a)*j/count,9).tolist(), '书写')
            end = point(stroke[-1])
            add([*end[:2],end[2]+lift], '抬笔')
    return validate_program(dict(schema_version='1.0',robot_model='b601_rs',base_frame='base_link',
        calibration_id='rs-identity-v1',units=dict(position='m',angle='rad',time='s'),
        name='国庆快乐 · 26笔空中书写',steps=steps))


def main():
    kin = CheckedKinematics()
    limits = gravity_limits_for_mode('native-csp-twin-full-v3-official')
    check = lambda a,b: kin.check_path(a,b,gravity_limits=limits)
    center, _ = kin.solve([.2,-.27,.25,0,0,0], [-.8,1.,1.4,0.,.4,0.], False, path_check=check)
    program = make_program(kin.pose(center)[3:])
    q, previous = [0.]*6, None
    strokes, current = [], None
    max_endpoint, max_line, max_plane, max_rotation = 0., 0., 0., 0.
    min_transfer_z = math.inf
    samples = []
    for i, step in enumerate(program['steps']):
        target = step['pose']
        try:
            goal, error = kin.solve(target,q,True,path_check=check)
        except ValueError as exc:
            raise ValueError(f'{i+1}: {step["label"]} {target}: {exc}') from exc
        max_endpoint = max(max_endpoint,float(np.linalg.norm(error[:3])))
        max_rotation = max(max_rotation,float(np.linalg.norm(error[3:])))
        char, stroke_id, action = step['label'].split('·')
        if action == '移笔' and previous is not None:
            for blend in np.linspace(0,1,21):
                min_transfer_z = min(min_transfer_z,kin.pose(np.array(q)+(np.array(goal)-q)*blend)[2])
        if action == '落笔':
            current = dict(char=char,stroke=int(stroke_id),color=COLORS[list(STROKES).index(char)],
                           points=[target[:3]],actual_points=[kin.pose(goal)[:3]])
            strokes.append(current)
        if action == '书写':
            current['points'].append(target[:3])
            a,b = np.array(previous[:3]),np.array(target[:3])
            for blend in np.linspace(0,1,9)[1:]:
                p = np.array(kin.pose(np.array(q)+(np.array(goal)-q)*blend)[:3])
                closest = a+np.clip(np.dot(p-a,b-a)/np.dot(b-a,b-a),0,1)*(b-a)
                max_line = max(max_line,float(np.linalg.norm(p-closest)))
                max_plane = max(max_plane,abs(float(p[2]-.25)))
                current['actual_points'].append(p.tolist())
        samples.append(dict(index=i,joints=goal,pose=target,pen_down=action=='书写'))
        q, previous = goal, target
    if max_line > .0005:
        raise ValueError(f'Interpolated TCP error {max_line*1000:.3f} mm exceeds 0.5 mm')
    if min_transfer_z < .258:
        raise ValueError('Lifted transfer has less than 8 mm model clearance above writing plane')
    report = dict(steps=len(program['steps']),strokes=len(strokes),text='国庆快乐',
        frame_id='base_link',plane='XY',z_m=.25,lift_m=.012,
        character_size_m=.05,character_gap_m=.012,speed_rad_s=.1,
        orientation_rpy=program['steps'][0]['pose'][3:],
        max_endpoint_error_mm=max_endpoint*1000,max_line_error_mm=max_line*1000,
        max_plane_error_mm=max_plane*1000,max_rotation_error_deg=math.degrees(max_rotation),
        min_transfer_clearance_mm=(min_transfer_z-.25)*1000,
        verification='Offline URDF IK, official continuous gravity envelope, sampled self collision and TCP interpolation; no physical motion')
    path = dict(schema_version='1.0',frame_id='base_link',text='国庆快乐',strokes=strokes,samples=samples,report=report)
    for folder in [ROOT/'examples/writing',ROOT/'reBotArm_simulator-RS/public/programs']:
        folder.mkdir(parents=True,exist_ok=True)
        for name, value in [('guoqing_kuaile.json',program),('guoqing_kuaile.path.json',path)]:
            (folder/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    out=ROOT/'outputs/writing';out.mkdir(parents=True,exist_ok=True)
    (out/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
