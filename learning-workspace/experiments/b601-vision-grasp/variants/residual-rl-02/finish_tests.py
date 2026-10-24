"""After training: export, paired tasks, independent audits and CPU inference."""
from concurrent.futures import ThreadPoolExecutor
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUTPUT = HERE / 'outputs'


def run(key, script, *arguments):
    with (OUTPUT / f'pipeline-{key}.stdout.log').open('w') as out, (OUTPUT / f'pipeline-{key}.stderr.log').open('w') as err:
        subprocess.run([sys.executable, str(HERE / script), *map(str, arguments)],
                       cwd=HERE, stdout=out, stderr=err, check=True)
    print(json.dumps({'completed': key}), flush=True)


def task(job):
    key, suite, controllers, start, count = job
    directory = OUTPUT / key
    expected = count * len(controllers)
    if directory.exists():
        actual = len(list(directory.glob('episode-*/result.json')))
        if actual != expected:
            raise ValueError(f'{directory}: {actual}/{expected}; preserve incomplete run before reproducing')
        print(json.dumps({'reused_complete': key, 'results': actual}), flush=True)
        return
    run(key, 'evaluate.py', '--suite', suite, '--controllers', *controllers,
        '--start', start, '--count', count, '--output', key)


if __name__ == '__main__':
    if not (OUTPUT / 'long-r2-hour/training.json').exists():
        raise ValueError('one-hour training has not completed')
    run('export', 'export.py')
    jobs = []
    for suite, prefix, count in [('broad', 'formal-r2', 12), ('boundary', 'boundary-r2', 10)]:
        for label, controllers in [('base', ['v3', 'zero', 'previous']), ('sac', ['sac'])]:
            for batch in range(3):
                jobs.append((f'{prefix}-{label}-b{batch}', suite, controllers, count * batch, count))
    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(task, jobs))
    run('audit-broad', 'audit.py')
    run('audit-boundary', 'audit.py', '--suite', 'boundary')
    run('benchmark-torch', 'infer.py', '--directory', HERE / 'delivery', '--benchmark')
    run('benchmark-numpy', 'infer_numpy.py', '--directory', HERE / 'delivery', '--benchmark')
    for goal in 'ABC':
        run(f'demo-{goal}', 'run_sim.py', '--goal', goal, '--seed', 55000010 + 'ABC'.index(goal),
            '--output', OUTPUT / f'final-numpy-demo-{goal}')
    (OUTPUT / 'TESTS_FINISHED.json').write_text(json.dumps({'completed': True,
        'broad_cases': 36, 'boundary_cases': 30, 'controllers': 4, 'numpy_demos': 3}, indent=2) + '\n')
