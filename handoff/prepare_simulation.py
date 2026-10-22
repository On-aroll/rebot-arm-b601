#!/usr/bin/env python3
"""Restore portable simulation links and verify frozen releases; no simulation runs."""
import argparse
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'learning-workspace/experiments/b601-vision-grasp'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    links = {}
    for group in ('versions', 'variants'):
        for directory in sorted((SIM / group).iterdir()):
            if not directory.is_dir():
                continue
            links[directory / '.venv'] = SIM / '.venv'
            links[directory / 'outputs'] = SIM / 'outputs'
    assets = {'home-first-01': SIM / 'assets',
              'initial-observation-01': SIM / 'versions/V2/assets'}
    for directory in sorted((SIM / 'variants').iterdir()):
        if directory.is_dir():
            links[directory / 'assets'] = assets.get(directory.name, SIM / 'versions/V3/assets')
    if not args.verify_only:
        (SIM / 'outputs').mkdir(exist_ok=True)
        for link, target in links.items():
            relative = os.path.relpath(target, link.parent)
            if link.is_symlink() and os.readlink(link) != relative:
                link.unlink()
            if not link.is_symlink():
                if link.exists():
                    raise ValueError(f'Refusing to replace existing directory: {link}')
                link.symlink_to(relative, target_is_directory=True)
    for link, target in links.items():
        if not link.is_symlink() or link.resolve() != target.resolve():
            raise ValueError(f'Missing or incorrect link: {link}; run without --verify-only')
    latest = json.loads((SIM / 'LATEST_VERIFIED.json').read_text())
    frozen = SIM / latest['runtime_manifest']
    assert hashlib.sha256(frozen.read_bytes()).hexdigest() == latest['runtime_manifest_sha256']
    for version in ('V2', 'V3'):
        directory = SIM / 'versions' / version
        manifest = json.loads((directory / 'release_manifest.json').read_text())
        assert manifest['hardware_executable'] is False
        for name, expected in manifest['files_sha256'].items():
            assert hashlib.sha256((directory / name).read_bytes()).hexdigest() == expected, (version, name)
        print(f'{version}: verified {len(manifest["files_sha256"])} frozen files')
    print(f'Default: {latest["version"]}; no simulation or hardware operation executed')


if __name__ == '__main__':
    main()
