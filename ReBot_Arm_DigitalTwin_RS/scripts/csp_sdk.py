#!/usr/bin/env python3
"""Check/reproduce the pinned checkout without importing its hardware code."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def git(directory, *args):
    return subprocess.check_output(
        ['git', '--no-optional-locks', '-C', str(directory), *args],
        text=True, stderr=subprocess.PIPE, env={**os.environ, 'GIT_TERMINAL_PROMPT': '0'}).strip()


def spec(root):
    lock = json.loads((root / 'configs/csp-sdk-lock.json').read_text())
    directory = root / lock['path']
    if (lock['schema_version'] != 1 or not re.fullmatch(r'[0-9a-f]{40}', lock['commit'])
            or directory.resolve().parent != (root / 'rebotarm_ros2_RS/third_party').resolve()
            or directory.is_symlink()):
        raise ValueError('CSP SDK lock path/version/commit is invalid')
    return lock, directory


def check(root=ROOT):
    lock, directory = spec(root)
    if not (directory / '.git').exists():
        raise ValueError('CSP SDK missing; run bash scripts/setup_csp_sdk.sh')
    if Path(git(directory, 'rev-parse', '--show-toplevel')).resolve() != directory.resolve():
        raise ValueError('CSP SDK must have its own checkout or submodule metadata')
    if git(directory, 'rev-parse', 'HEAD') != lock['commit']:
        raise ValueError('CSP SDK commit differs from configs/csp-sdk-lock.json; not overwritten')
    # url is the canonical bootstrap source. An offline mirror may be origin;
    # the immutable commit and clean tree, rather than its transport URL, decide.
    if git(directory, 'status', '--porcelain', '--untracked-files=all'):
        raise ValueError('CSP SDK has modified or untracked source; not overwritten')
    # A known pinned Git checkout (or .git file for a submodule) is intentional.
    # Other nested repositories still require an explicit dependency decision.
    unexpected = [p.name for p in directory.parent.iterdir()
                  if p != directory and (p / '.git').exists()]
    if unexpected:
        raise ValueError('Unregistered nested Git repositories: ' + ', '.join(sorted(unexpected)))
    return lock


def prepare(root=ROOT):
    lock, directory = spec(root)
    if not directory.exists():
        directory.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(['git', 'clone', '--no-checkout', lock['url'], str(directory)], check=True)
        subprocess.run(['git', '-C', str(directory), 'checkout', '--detach', lock['commit']], check=True)
    return check(root)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='read only; no clone, fetch or checkout')
    args = parser.parse_args()
    try:
        lock = check() if args.check else prepare()
        print(f"CSP SDK verified: {lock['commit']} ({lock['path']})")
    except (ValueError, OSError, KeyError, subprocess.CalledProcessError) as error:
        print(f'CSP SDK check failed: {error}', file=sys.stderr)
        sys.exit(1)
