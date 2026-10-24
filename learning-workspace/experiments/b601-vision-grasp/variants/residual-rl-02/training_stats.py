"""Summarize actual completed and partial episodes without counting partials as trials."""
import collections
import json
from curriculum import HERE, write


if __name__ == '__main__':
    root = HERE / 'outputs/long-r2-hour'
    levels = collections.defaultdict(lambda: collections.Counter())
    failures, seeds = collections.Counter(), set()
    totals = collections.Counter()
    for path in root.glob('env-*/episode-*/result.json'):
        result = json.loads(path.read_text())
        reset = json.loads((path.parent / 'reset.json').read_text())
        seeds.add(reset['reset']['seed'])
        level = reset['curriculum_level']
        for counter in [totals, levels[level]]:
            counter['completed_episodes'] += 1
            counter['successes'] += bool(result['skill_success'])
            counter['decisions'] += result['decisions']
            counter['forbidden_steps'] += result['forbidden_steps']
            counter['rehearsal_episodes'] += bool(reset['rehearsal'])
        if not result['skill_success']:
            failures[result.get('failure_reason') or 'grasp_or_lift_failed'] += 1
    partial = [json.loads(p.read_text()) for p in root.glob('env-*/episode-*/partial_result.json')]
    training = json.loads((root / 'training.json').read_text())
    assert totals['decisions'] + sum(r['decisions'] for r in partial) == training['timesteps']
    summary = {'completed': dict(totals), 'by_level': {str(k): dict(v) for k, v in levels.items()},
        'failure_reasons': dict(failures), 'distinct_training_seeds': len(seeds),
        'partial_episodes': len(partial), 'partial_decisions': sum(r['decisions'] for r in partial),
        'timesteps': training['timesteps'], 'learn_wall_seconds_including_final_validation': training['learn_wall_seconds'],
        'total_wall_seconds_including_initial_validation': training['including_initial_validation_wall_seconds'],
        'scope': 'training grasp/lift skill, not complete placement'}
    write(root / 'statistics.json', summary)
    write(HERE / 'delivery/TRAINING_SUMMARY.json', summary)
    print(json.dumps(summary, indent=2))
