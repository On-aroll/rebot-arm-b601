#!/usr/bin/env python3
"""Package the paused workspace's outputs and index every retained source file."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[1]
SKIP = {'.git', '__pycache__', '.venv', 'node_modules', '.certs', '.pytest_cache'}
SECRET_SUFFIX = ('.pem', '.key', '.token')
MEDIA = {'.npz', '.gif', '.png', '.jpg', '.jpeg', '.mp4', '.pdf', '.docx'}


def files(directory):
    for base, dirs, names in os.walk(directory, followlinks=False):
        dirs[:] = sorted(d for d in dirs if d not in SKIP)
        for name in sorted(names):
            path = Path(base) / name
            if name.startswith('.env') or name.endswith(SECRET_SUFFIX) or name.endswith('.pyc'):
                continue
            if path.is_file() and not path.is_symlink():
                yield path


def digest(path):
    with path.open('rb') as source:
        return hashlib.file_digest(source, 'sha256').hexdigest()


def package(destination, base, selected, prefix=''):
    destination.parent.mkdir(parents=True, exist_ok=True)
    selected = sorted(selected)
    proc = subprocess.Popen(['zstd', '-T2', '-6', '-q', '-o', str(destination)], stdin=subprocess.PIPE)
    indexed = {}
    try:
        with tarfile.open(fileobj=proc.stdin, mode='w|') as archive:
            for path in selected:
                relative = path.relative_to(base).as_posix()
                expected = digest(path)
                name = f'{prefix}/{relative}' if prefix else relative
                archive.add(path, arcname=name, recursive=False)
                if digest(path) != expected:
                    raise RuntimeError(f'Source changed during snapshot: {path}')
                indexed[name] = dict(sha256=expected, size=path.stat().st_size)
    finally:
        proc.stdin.close()
        status = proc.wait()
    if status:
        raise RuntimeError(f'zstd exited {status}: {destination}')
    result = dict(archive=destination.name, archive_sha256=digest(destination),
                  archive_size=destination.stat().st_size, files=indexed)
    destination.with_suffix('').with_suffix('.manifest.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(f'{destination.name}: {len(indexed)} files, {destination.stat().st_size} bytes', flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--main-source', type=Path, required=True)
    parser.add_argument('--learning-source', type=Path, required=True)
    args = parser.parse_args()
    sim = args.learning_source / 'experiments/b601-vision-grasp/outputs'
    evidence = ROOT / 'handoff/evidence'
    release = ROOT / 'handoff/release-assets'
    release.mkdir(parents=True, exist_ok=True)
    groups = []
    hardware = args.main_source / 'outputs/twin-console/hardware'
    record = package(evidence / 'hardware-history.tar.zst', hardware, files(hardware))
    groups.append(dict(name='hardware-history', location='git', **{k:v for k,v in record.items() if k!='files'}))
    record = package(evidence / 'simulation-records.tar.zst', sim,
                     (p for p in files(sim) if p.suffix not in MEDIA | {'.jsonl', '.ndjson'}), prefix='outputs')
    groups.append(dict(name='simulation-records', location='git', **{k:v for k,v in record.items() if k!='files'}))
    record = package(release / 'main-outputs.tar.zst', args.main_source,
                     files(args.main_source / 'outputs'))
    groups.append(dict(name='main-outputs', location='release', **{k:v for k,v in record.items() if k!='files'}))
    for directory in sorted(sim.iterdir()):
        if directory.is_dir() and not directory.is_symlink():
            record = package(release / f'sim-{directory.name}.tar.zst', sim,
                             files(directory), prefix='outputs')
            groups.append(dict(name=directory.name, location='release', **{k:v for k,v in record.items() if k!='files'}))
    for path in sorted((args.learning_source / 'dist').iterdir()):
        if path.is_file():
            shutil.copy2(path, release / path.name)
    # Historical installer validation logs; installed environments are reproducible caches.
    validation = args.learning_source / '.state/rebotarm-validation'
    selected = [p for p in validation.iterdir() if p.is_file() and p.suffix in {'.json','.log','.txt'}]
    package(evidence / 'deployment-validation.tar.zst', validation, selected)
    sums = ''.join(f'{digest(p)}  {p.name}\n' for p in sorted(release.iterdir()) if p.is_file())
    (release / 'SHA256SUMS').write_text(sums)
    (ROOT / 'handoff/ARTIFACTS.json').write_text(json.dumps(
        dict(schema='b601-paused-handoff-artifacts-v1', raw_output_files=sum(len(list(files(p))) for p in [sim,args.main_source/'outputs']),
             release_tag='handoff-2026-10-22', release_status='prepared_pending_upload', groups=groups,
             excluded=['virtual environments, build/install/log caches, node_modules, process PID/lock state',
                       'API keys, real .env/config credentials, private keys',
                       'validation installed copies; standalone original validation logs retained']),
        ensure_ascii=False, indent=2)+'\n')
    print('Release assets prepared; upload status must be verified separately.', flush=True)


if __name__ == '__main__':
    main()
