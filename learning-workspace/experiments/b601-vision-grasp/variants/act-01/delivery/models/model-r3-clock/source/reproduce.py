"""Replay a preserved collection/evaluation with its exact source snapshot."""
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
    config = json.loads((run / 'config.json').read_text())
    provenance = json.loads((run / 'provenance.json').read_text())
    destination = ROOT.parent / a.name
    if Path(a.name).name != a.name or destination.exists():
        raise ValueError('Choose a new sibling directory name')
    for name, sha in provenance['source_sha256'].items():
        assert digest(run / 'source' / name) == sha
    shutil.copytree(run / 'source', destination)
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
