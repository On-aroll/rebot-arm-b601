"""Replay logged commands and check grasp/lift evidence independently of the actor."""
import json
from pathlib import Path
import shutil

from gate import HERE, EPISODE, RealisticSimulation, GOALS, independent_score, mj, write, digest, verify_v3
import numpy as np


if __name__ == '__main__':
    verify_v3()
    output = HERE / 'outputs/audit-r2'
    output.mkdir(exist_ok=False)
    shutil.copy2(__file__, output / 'audit.py')
    original = json.loads((EPISODE / 'result.json').read_text())
    results = []
    for path in sorted((HERE / 'outputs').glob('*/episode-*.json')):
        record = json.loads(path.read_text())
        observes = json.loads((path.parent / 'run.json').read_text())['mode'] == 'evaluate'
        sim = RealisticSimulation()
        sim.reset(original['reset']['seed'], force_pulse=None)
        sim.data.qpos[sim.oq:sim.oq + 2] += record.get('initial_offset_xy_m', [0., 0.])
        mj.mj_forward(sim.model, sim.data)
        initial_z = float(sim.data.xpos[sim.obj, 2])
        peak_z, held_seconds, longest, forbidden = initial_z, 0., 0., 0
        traces = []
        ticks = 0
        try:
            for row in record['trajectory']:
                # observe() calls mj_forward, which updates solver state. Match
                # the original observation schedule during physical replay.
                if observes:
                    sim.observe(labels=False)
                sim.target_q = np.asarray(row['command'][:6])
                sim.grip = row['command'][6]
                for _ in range(round(.2 / sim.model.opt.timestep)):
                    sim.step()
                    contacts = sim.contacts()
                    z = float(sim.data.xpos[sim.obj, 2])
                    peak_z = max(peak_z, z)
                    held_seconds = (held_seconds + sim.model.opt.timestep if
                        contacts['left'] and contacts['right'] and z > initial_z + .03 else 0.)
                    longest = max(longest, held_seconds)
                    forbidden += bool(contacts['forbidden'])
                    if ticks % 25 == 0:
                        traces.append({'time_s': float(sim.data.time),
                            'object_xyz_m': sim.data.xpos[sim.obj].tolist(), 'contacts': contacts})
                    ticks += 1
            scores = []
            for _ in range(round(1. / sim.model.opt.timestep)):
                sim.step()
                forbidden += bool(sim.contacts()['forbidden'])
                scores.append(independent_score(sim, GOALS[original['task']['destination']]))
            error = float(np.max(np.abs(np.asarray(scores[-1]['object_xyz_m']) - record['final_score']['object_xyz_m'])))
            assert error < 1e-7, (path, error)
            placement = all(s['success'] for s in scores) and not forbidden
            assert placement == record['success'], path
            item = {'run': path.parent.name, 'episode': path.name,
                'source_record_sha256': digest(path), 'replay_final_position_error_m': error,
                'peak_lift_m': peak_z - initial_z, 'bilateral_lift_continuous_s': longest,
                'forbidden_steps': forbidden, 'placement_success': placement,
                'grasp_lift_place_success': bool(placement and longest >= .1)}
            results.append(item)
            trace_file = output / f'{path.parent.name}-{path.stem}.jsonl'
            trace_file.write_text(''.join(json.dumps(r) + '\n' for r in traces), encoding='utf-8')
            write(output / 'summary.json', {'scope': 'independent replay of saved commands, no learned-policy inputs',
                'lift_requirement': 'both fingers contact object continuously >=0.1s while object rises >0.03m',
                'source_sha256': digest(Path(__file__)), 'results': results})
            print(json.dumps(item), flush=True)
        finally:
            sim.close()
