#!/usr/bin/env python3
"""Resume private Release uploads, checking remote SHA256 digests when available."""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
REPO = 'On-aroll/rebot-arm-b601'
TAG = 'handoff-2026-10-22'


def gh(*args):
    return subprocess.check_output(['gh', *args], text=True)


def assets(release_id):
    result = {}
    page = 1
    while True:
        entries = json.loads(gh('api', f'repos/{REPO}/releases/{release_id}/assets?per_page=100&page={page}'))
        result.update({entry['name']: entry for entry in entries})
        if len(entries) < 100:
            return result
        page += 1


def main():
    directory = ROOT / 'handoff/release-assets'
    release = json.loads(gh('api', f'repos/{REPO}/releases/tags/{TAG}'))
    remote = assets(release['id'])
    local = sorted(p for p in directory.iterdir() if p.is_file())
    if any(not path.name.isascii() for path in local):
        raise ValueError('Use ASCII asset names to avoid server renaming; preserve original names in ARTIFACTS.json')
    for path in local:
        expected = hashlib.file_digest(path.open('rb'), 'sha256').hexdigest()
        previous = remote.get(path.name)
        if previous:
            if previous['size'] != path.stat().st_size or previous.get('state') != 'uploaded':
                raise ValueError(f'Conflicting/incomplete remote asset: {path.name}; no overwrite attempted')
            if previous.get('digest') and previous['digest'] != f'sha256:{expected}':
                raise ValueError(f'Remote digest mismatch: {path.name}')
            print(f'Present: {path.name}', flush=True)
            continue
        if path.stat().st_size >= 2_000_000_000:
            raise ValueError(f'Asset exceeds conservative 2 GB upload boundary: {path.name}')
        print(f'Uploading: {path.name}', flush=True)
        subprocess.run(['gh', 'release', 'upload', TAG, '--repo', REPO, str(path)], check=True)
    remote = assets(release['id'])
    verified = []
    for path in local:
        item = remote[path.name]
        expected = hashlib.file_digest(path.open('rb'), 'sha256').hexdigest()
        assert item['state'] == 'uploaded' and item['size'] == path.stat().st_size
        assert item.get('digest') == f'sha256:{expected}', f'Server SHA256 unavailable/mismatch: {path.name}'
        verified.append(dict(name=path.name, size=item['size'], sha256=expected, url=item['browser_download_url']))
    record = dict(schema='b601-release-verification-v1', repo=REPO, tag=TAG,
                  release_url=release['html_url'], all_assets_verified=True, assets=verified)
    (ROOT / 'handoff/RELEASE_VERIFIED.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    print(f'Verified {len(verified)} remote assets against server SHA256.', flush=True)


if __name__ == '__main__':
    main()
