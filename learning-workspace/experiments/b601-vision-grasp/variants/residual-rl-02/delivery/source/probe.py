"""Find recoverable alignment difficulties before spending a long training budget."""
import argparse
import json
from curriculum import HERE, PREVIOUS, CurriculumEnv, base, write
from stable_baselines3 import SAC
import numpy as np


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='probe-01')
    args = parser.parse_args()
    output = HERE / 'outputs' / args.output
    model = SAC.load(PREVIOUS / 'delivery/policy.zip', device='cpu')
    rows = []
    for bias in [0., .02, -.02, .03, -.03, .04, -.04]:
        for controller in ['zero', 'previous', 'visual_feedback']:
            env = CurriculumEnv(range(52000000, 52000003), output / f'{bias:+.3f}-{controller}',
                                validation=True, fixed_bias=[0., bias], fixed_level=0)
            try:
                for i in range(3):
                    obs, _ = env.reset()
                    done = False
                    while not done:
                        action = np.zeros(4)
                        if controller == 'previous':
                            action = model.predict(obs[:36], deterministic=True)[0]
                        elif controller == 'visual_feedback':
                            action[:2] = np.clip((obs[32:34] * base.BOUNDS[:2] + obs[36:38] * .04) / base.BOUNDS[:2], -1., 1.)
                        obs, reward, done, _, info = env.step(action)
                    row = {'bias_y_m': bias, 'controller': controller, 'seed': 52000000 + i, **info}
                    rows.append(row)
                    write(output / 'summary.json', rows)
                    print(json.dumps(row), flush=True)
            finally:
                env.close()
