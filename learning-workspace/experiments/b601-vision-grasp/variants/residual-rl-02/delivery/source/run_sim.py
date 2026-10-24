"""RGB-D + own feedback + requested A/B/C: run the complete simulated task."""
import argparse
import json
from pathlib import Path
from curriculum import HERE, CurriculumEnv
from task_r2 import hybrid_task
from infer_numpy import predict
import numpy as np


class NumpyActor:
    def __init__(self, directory):
        with np.load(directory / 'numpy_actor.npz') as archive:
            self.weights = {name: archive[name] for name in archive.files}

    def predict(self, observation, deterministic=True):
        return predict(observation, self.weights), None


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', choices=list('ABC'), required=True)
    parser.add_argument('--seed', type=int, default=55000000)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--delivery', type=Path, default=HERE / 'delivery')
    args = parser.parse_args()
    env = CurriculumEnv([args.seed], args.output.parent / 'unused_resets', log_episodes=False)
    try:
        result = hybrid_task(env, NumpyActor(args.delivery),
            {'seed': args.seed, 'goal': args.goal, 'kind': 'user_requested_simulation',
             'force_pulse': None, 'first_grasp_offset_m': 0.}, args.output)
        print(json.dumps({name: result[name] for name in ['success', 'retries', 'sim_seconds', 'false_success']}))
    finally:
        env.close()
