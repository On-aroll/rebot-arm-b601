"""Development feasibility only: distinguish solvable offsets from the hard limit."""
import argparse
import json
from curriculum import HERE, CurriculumEnv, base, write
import numpy as np


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='reachability-reproduce')
    args = parser.parse_args()
    output = HERE / 'outputs' / args.output
    rows = []
    for bias in [.027, -.027, .028, -.028, .029, -.029]:
        for controller in ['zero', 'bound_feasibility_only', 'camera_y_feedback']:
            env = CurriculumEnv(range(52000100, 52000103), output / f'{bias:+.3f}-{controller}',
                                validation=True, fixed_bias=[0., bias], fixed_level=0)
            try:
                for index in range(3):
                    obs, _ = env.reset()
                    done = False
                    while not done:
                        action = np.zeros(4)
                        if controller == 'bound_feasibility_only':
                            # Knows the injected sign ONLY for actuator feasibility.
                            # Never supplied to SAC, never a deployable controller.
                            action[1] = -np.sign(bias)
                        elif controller == 'camera_y_feedback':
                            action[1] = np.clip((obs[33] * base.BOUNDS[1] + obs[37] * .04) / base.BOUNDS[1], -1., 1.)
                        obs, _, done, _, info = env.step(action)
                    row = {'bias_y_m': bias, 'seed': 52000100 + index, 'controller': controller, **info}
                    rows.append(row)
                    write(output / 'summary.json', rows)
                    print(json.dumps(row), flush=True)
            finally:
                env.close()
