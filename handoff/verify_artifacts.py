#!/usr/bin/env python3
"""Verify archived bytes against every source file digest without extracting or running code."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile


def verify(archive, manifest):
    with archive.open('rb') as source:
        assert hashlib.file_digest(source, 'sha256').hexdigest() == manifest['archive_sha256'], archive
    expected = manifest['files']
    seen = set()
    proc = subprocess.Popen(['zstd', '-dc', str(archive.resolve())], stdout=subprocess.PIPE)
    try:
        with tarfile.open(fileobj=proc.stdout, mode='r|') as source:
            for entry in source:
                assert entry.isfile() and entry.name in expected and entry.name not in seen, entry.name
                item = expected[entry.name]
                assert entry.size == item['size'], entry.name
                stream = source.extractfile(entry)
                sha = hashlib.sha256()
                while chunk := stream.read(1024 * 1024):
                    sha.update(chunk)
                assert sha.hexdigest() == item['sha256'], entry.name
                seen.add(entry.name)
    finally:
        proc.stdout.close()
        status = proc.wait()
    assert status == 0 and seen == set(expected), archive
    return len(seen)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, action='append', required=True)
    args = parser.parse_args()
    result = []
    for directory in args.directory:
        for archive in sorted(directory.glob('*.tar.zst')):
            manifest = json.loads(archive.with_suffix('').with_suffix('.manifest.json').read_text())
            count = verify(archive, manifest)
            result.append(dict(archive=str(archive), files=count, sha256=manifest['archive_sha256']))
            print(f'Verified {archive.name}: {count} files', flush=True)
    Path('handoff/ARCHIVES_VERIFIED.json').write_text(json.dumps(
        dict(all_archives_verified=True, archive_count=len(result), archives=result),indent=2)+'\n')


if __name__ == '__main__':
    main()
