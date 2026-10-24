"""Check whether recorded teacher commands survive the deployment sampling rate."""
import json
from pathlib import Path

import numpy as np
from runtime import ROOT, RealisticSimulation, digest, verify_v3, write
from task import GOALS, independent_score


def replay(directory, rate):
    result = json.loads((directory / 'result.json').read_text())
    sim = RealisticSimulation()
    pulse = result['environment_for_scoring_only']['config']
    metadata = sim.reset(result['reset']['seed'], force_pulse=pulse)
    assert metadata['realism'] == result['reset']['realism']
    rows = [json.loads(s) for s in (directory / 'trajectory.jsonl').read_text().splitlines()]
    if rate == 20:
        times = [r['t_sim_s'] for r in rows]
        actions = np.array([r['q_command_rad'] + [r['gripper_command_m']] for r in rows])
    else:
        with np.load(directory / 'imitation.npz', allow_pickle=False) as data:
            times, actions = data['timestamp_s'], data['action']
    forbidden = 0
    try:
        for i, action in enumerate(actions):
            sim.target_q, sim.grip = action[:6].astype(float), float(action[6])
            end = times[i + 1] if i + 1 < len(times) else result['sim_seconds']
            while sim.data.time < end - 1e-9:
                sim.step()
                forbidden += bool(sim.contacts()['forbidden'])
        scores = []
        for _ in range(round(1. / sim.model.opt.timestep)):
            sim.step()
            forbidden += bool(sim.contacts()['forbidden'])
            scores.append(independent_score(sim, GOALS[result['task']['destination']]))
        return {'seed': result['reset']['seed'], 'rate_hz': rate,
            'teacher_success': result['success'], 'command_samples': len(actions),
            'physical_success': all(s['success'] for s in scores) and not forbidden,
            'forbidden_steps': forbidden, 'final_score': scores[-1]}
    finally:
        sim.close()


if __name__ == '__main__':
    verify_v3()
    output = ROOT / 'outputs/teacher-replay-probe-01'
    output.mkdir(exist_ok=False)
    results = []
    for episode in range(3):
        directory = ROOT / 'outputs/demos-r1' / f'episode-{episode:03d}'
        for rate in (20, 5):
            row = replay(directory, rate)
            results.append(row)
            write(output / 'summary.json', {'source_sha256': digest(Path(__file__)),
                'scope': 'command transport diagnostic, not autonomous policy evaluation', 'results': results})
            print(json.dumps(row), flush=True)
