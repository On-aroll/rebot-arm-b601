#!/usr/bin/env python3
"""Check the portable source snapshot without importing robot software."""
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    manifest = json.loads((ROOT / 'handoff/SOURCE_FILES_SHA256.json').read_text())
    for name, item in manifest['files'].items():
        path = ROOT / name
        if item['kind'] == 'symlink':
            assert path.is_symlink() and os.readlink(path) == item['target'], name
        else:
            with path.open('rb') as source:
                assert hashlib.file_digest(source, 'sha256').hexdigest() == item['sha256'], name
    print(f'Verified {len(manifest["files"])} published source/evidence files; no robot software imported')


if __name__ == '__main__':
    main()
