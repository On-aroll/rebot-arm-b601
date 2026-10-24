#!/usr/bin/env python3
"""Resume private Release uploads, checking remote SHA256 digests when available."""
import hashlib
import json
import os
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
REPO = 'On-aroll/rebot-arm-b601'
TAG = 'handoff-2026-10-22'


def gh(*args):
    for attempt in range(5):
        env = dict(os.environ)
        if attempt % 2:
            env['NO_PROXY'] = '' if env.get('NO_PROXY') else 'api.github.com,uploads.github.com'
        try:
            return subprocess.check_output(['gh', *args], encoding='utf-8', env=env, timeout=45)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            if attempt == 4:
                raise
            time.sleep(2 ** attempt)


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
    def upload(path):
        expected = hashlib.file_digest(path.open('rb'), 'sha256').hexdigest()
        previous = remote.get(path.name)
        if previous:
            if previous['size'] != path.stat().st_size or previous.get('state') != 'uploaded':
                raise ValueError(f'Conflicting/incomplete remote asset: {path.name}; no overwrite attempted')
            if previous.get('digest') and previous['digest'] != f'sha256:{expected}':
                raise ValueError(f'Remote digest mismatch: {path.name}')
            print(f'Present: {path.name}', flush=True)
            return
        if path.stat().st_size >= 2_000_000_000:
            raise ValueError(f'Asset exceeds conservative 2 GB upload boundary: {path.name}')
        for attempt in range(5):
            print(f'Uploading: {path.name}', flush=True)
            command = ['gh', 'release', 'upload', TAG, '--repo', REPO, str(path)]
            try:
                result = subprocess.run(command, timeout=180)
            except subprocess.TimeoutExpired:
                print(f'Upload timed out: {path.name}; checking server before retry.', flush=True)
                result = subprocess.CompletedProcess(command, 124)
            if result.returncode == 0:
                print(f'Uploaded: {path.name}', flush=True)
                return
            # The request can fail after the server has already stored the asset.
            latest = assets(release['id']).get(path.name)
            if latest and latest['state'] == 'uploaded':
                assert latest['size'] == path.stat().st_size and latest.get('digest') == f'sha256:{expected}'
                print(f'Uploaded: {path.name} (server verified after lost response)', flush=True)
                return
            if latest:
                # Only this invocation's failed upload may be removed. Conflicts
                # present before the request are still refused above.
                assert latest['state'] == 'starter' and latest.get('digest') is None
                assert latest['name'] == path.name and latest['size'] == path.stat().st_size
                subprocess.run(['gh', 'api', '-X', 'DELETE',
                    f"repos/{REPO}/releases/assets/{latest['id']}"], check=True)
            if attempt == 4:
                raise subprocess.CalledProcessError(result.returncode, result.args)
            print(f'Retrying: {path.name} (attempt {attempt + 2}/5)', flush=True)
            time.sleep(2 ** attempt)

    with ThreadPoolExecutor(max_workers=3) as executor:
        for _ in executor.map(upload, local):
            pass
    remote = assets(release['id'])
    verified = []
    for path in local:
        item = remote[path.name]
        expected = hashlib.file_digest(path.open('rb'), 'sha256').hexdigest()
        assert item['state'] == 'uploaded' and item['size'] == path.stat().st_size
        assert item.get('digest') == f'sha256:{expected}', f'Server SHA256 unavailable/mismatch: {path.name}'
        verified.append(dict(name=path.name, size=item['size'], sha256=expected, url=item['browser_download_url']))
    record = dict(schema='b601-release-verification-v1', repo=REPO, tag=TAG,
                  release_url=release['html_url'], all_assets_verified=True,
                  verified_at_utc=datetime.now(timezone.utc).isoformat(),
                  asset_count=len(verified), total_bytes=sum(item['size'] for item in verified),
                  assets=verified)
    (ROOT / 'handoff/RELEASE_VERIFIED.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    print(f'Verified {len(verified)} remote assets against server SHA256.', flush=True)


if __name__ == '__main__':
    main()
