"""Replay a preserved run with its exact source and data hashes."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

from runtime import ROOT, digest


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', required=True, help='Existing directory below outputs')
    p.add_argument('--name', required=True, help='New sibling variant directory name')
    a = p.parse_args()
    run = ROOT / 'outputs' / a.run
    is_training = (run / 'frozen_training.json').exists()
    provenance = json.loads((run / ('frozen_training.json' if is_training else 'provenance.json')).read_text())
    destination = ROOT.parent / a.name
    if Path(a.name).name != a.name or destination.exists():
        raise ValueError('Choose a new sibling directory name')
    for name, sha in provenance['source_sha256'].items():
        assert digest(run / 'source' / name) == sha
    shutil.copytree(run / 'source', destination)
    if is_training:
        datasets = {}
        for group in ('train', 'validation'):
            folders = set()
            for original, sha in provenance[f'{group}_data_sha256'].items():
                suffix = original.replace('\\', '/').split('/outputs/', 1)[1]
                path = ROOT / 'outputs' / suffix
                assert digest(path) == sha
                folders.add(str(path.parent.parent))
            datasets[group] = sorted(folders)
        call = {'directories': datasets['train'], 'validation': datasets['validation'][0],
                'output': str(destination / 'outputs/reproduced'),
                'steps': provenance['steps'], 'seed': provenance['seed']}
        program = 'import sys,json;sys.path.insert(0,sys.argv[1]);from chunk_policy import train;train(**json.loads(sys.argv[2]))'
        subprocess.run([sys.executable, '-c', program, str(destination), json.dumps(call)], check=True)
        sys.exit(0)
    config = json.loads((run / 'config.json').read_text())
    command = [sys.executable, str(destination / 'workflow.py'), config['command']]
    config['name'] = 'reproduced'
    for key, value in config.items():
        if key == 'command' or value is None or value is False:
            continue
        flag = '--' + key.replace('_', '-')
        command.append(flag)
        if value is not True:
            command.extend(str(x) for x in value) if isinstance(value, list) else command.append(str(value))
    subprocess.run(command, check=True)
