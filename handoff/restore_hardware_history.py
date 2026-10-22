#!/usr/bin/env python3
"""Restore byte-identical history for the SAME arm before starting any services."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--same-arm', action='store_true', required=True,
                        help='Confirm this is the original B601-RS, not a different arm')
    args = parser.parse_args()
    archive = ROOT / 'handoff/evidence/hardware-history.tar.zst'
    manifest = json.loads((ROOT / 'handoff/evidence/hardware-history.manifest.json').read_text())
    assert hashlib.file_digest(archive.open('rb'), 'sha256').hexdigest() == manifest['archive_sha256']
    target = ROOT / 'ReBot_Arm_DigitalTwin_RS/outputs/twin-console/hardware'
    if target.exists() and any(target.iterdir()):
        raise ValueError('Existing hardware history found; refusing to overwrite it')
    # No driver, SDK, ROS or CAN is imported. Restored history can retain a recovery lock.
    target.mkdir(parents=True, exist_ok=True)
    proc = subprocess.Popen(['zstd', '-dc', str(archive.resolve())], stdout=subprocess.PIPE)
    try:
        with tarfile.open(fileobj=proc.stdout, mode='r|') as source:
            for entry in source:
                if not entry.isfile() or '/' in entry.name or entry.name not in manifest['files']:
                    raise ValueError(f'Unexpected archive entry: {entry.name}')
                data = source.extractfile(entry).read()
                if hashlib.sha256(data).hexdigest() != manifest['files'][entry.name]['sha256']:
                    raise ValueError(f'History hash mismatch: {entry.name}')
                (target / entry.name).write_bytes(data)
    finally:
        proc.stdout.close()
        status = proc.wait()
    assert status == 0
    assert {p.name for p in target.iterdir()} == set(manifest['files'])
    print(f'Restored {len(manifest["files"])} original files. No services started; no device writes.')
    print('Read HANDOFF.md and check fresh readonly feedback before any new hardware action.')


if __name__ == '__main__':
    main()
