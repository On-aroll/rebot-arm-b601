"""Reuse R1 task logic, restoring V3 retries after rejected reobservation."""
import time
import numpy as np
from curriculum import STEPS, base
from runtime import write
from task import Recorder, independent_score, visual_target, GOALS
from sim import HOME
from episode import DOWN
prepare = base.prepare


def hybrid_task(env, model, scene, directory, zero=False):
    sim, perception, residual = env.sim, env.perception, env.residual
    directory.mkdir(parents=True, exist_ok=False)
    reset = sim.reset(scene['seed'], force_pulse=scene['force_pulse'])
    recorder = Recorder(sim, directory, False)
    result = {'controller': 'zero_skill' if zero else 'sac_residual_skill', 'scene': scene,
        'reset': reset, 'success': False, 'controller_success': False,
        'physical_success': False, 'false_success': False, 'attempts': [],
        'hardware_executable': False, 'skill_decisions': 0}
    started = time.monotonic()
    try:
        for attempt in range(3):
            row = {'attempt': attempt + 1}
            result['attempts'].append(row)
            try:
                center, grasp = prepare(sim, perception, residual, recorder,
                    scene['first_grasp_offset_m'] if attempt == 0 else 0.)
            except ValueError as exc:
                if str(exc) != 'reobservation_rejected':
                    raise
                row['outcome'] = 'reobservation_rejected'
                result['reason'] = 'retries_exhausted_after_reobservation'
                recorder.move(HOME, .046, 2., 'return_for_reobserve')
                continue
            env.current_observation = env.begin(recorder, scene['goal'], grasp)
            while True:
                action = np.zeros(4) if zero else model.predict(env.current_observation, deterministic=True)[0]
                _, _, _, _, info = env.step(action)
                result['skill_decisions'] += 1
                # The training environment may terminate on scorer contact truth.
                # Deployment does not use that reward/terminal signal for control.
                if env.index >= STEPS or env.failed is not None:
                    break
            row['skill_actions'] = env.steps
            row['scoring_only_skill_result'] = info
            # Deployment recovery uses jaw feedback, not the reward scorer's success.
            jaw = float(sim.data.qpos[sim.gq])
            row['jaw_feedback_after_lift_m'] = jaw
            if env.failed:
                raise ValueError(env.failed)
            if not .008 < jaw < .038:
                row['outcome'] = 'empty_grasp_or_slip'
                recorder.move(sim.target_q, .046, .8, 'reopen')
                recorder.move(HOME, .046, 2., 'return_for_reobserve')
                continue
            destination = grasp.copy()
            destination[:2] += np.asarray(GOALS[scene['goal']]) - center[:2]
            destination[2] += .002
            qt = sim.ik(destination + [0., 0., .11], DOWN, sim.target_q)
            qp = sim.ik(destination, DOWN, qt)
            recorder.move(qt, 0., 2., 'transport')
            recorder.move(qp, 0., 1.8, 'place')
            recorder.move(qp, .046, 1.2, 'release')
            recorder.move(qt, .046, 1.8, 'retreat')
            recorder.move(HOME, .046, 2., 'return_for_verification')
            recorder.move(HOME, .046, .5, 'settle')
            frame, _ = recorder.capture_decision('verify')
            prediction = visual_target(perception, frame)
            row['verification_prediction'] = prediction
            verified = prediction is not None and np.max(np.abs(np.asarray(prediction['top_xyz_m'])[:2] - GOALS[scene['goal']])) < .032
            verified = bool(verified and .13 <= prediction['top_xyz_m'][2] <= .155 and sim.data.qpos[sim.gq] > .04)
            row['outcome'] = 'visually_verified' if verified else 'placement_not_verified'
            if verified:
                result['controller_success'] = True
                result.pop('reason', None)
                break
    except (ValueError, RuntimeError) as exc:
        result['reason'] = str(exc)
    recorder.stage = 'independent_score'
    for _ in range(round(1. / sim.model.opt.timestep)):
        recorder.tick()
        recorder.final_scores.append(independent_score(sim, GOALS[scene['goal']]))
    result['physical_success'] = all(s['success'] for s in recorder.final_scores) and not recorder.physics_audit['forbidden_steps']
    result['success'] = result['controller_success'] and result['physical_success']
    result['false_success'] = result['controller_success'] and not result['physical_success']
    result.update(retries=max(0, len(result['attempts']) - 1), sim_seconds=float(sim.data.time),
        wall_seconds=time.monotonic() - started, physics_tick_audit=recorder.physics_audit,
        final_score=recorder.final_scores[-1])
    recorder.save()
    write(directory / 'result.json', result)
    return result

