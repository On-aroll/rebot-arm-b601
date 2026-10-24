"""Preserve all new RL records, with independent decompression and remote-ready parts."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import zipfile

import zstandard as zstd
from environment import HERE
from runtime import digest, write, verify_v3


if __name__ == '__main__':
    verify_v3()
    exports, delivery = HERE / 'outputs/exports', HERE / 'delivery'
    stage = exports / 'handoff/release-assets'
    stage.mkdir(parents=True, exist_ok=False)
    files = sorted(p for p in (HERE / 'outputs').rglob('*') if p.is_file() and exports not in p.parents)
    records = {p.relative_to(HERE).as_posix(): {'bytes': p.stat().st_size, 'sha256': digest(p)} for p in files}
    manifest = delivery / 'RECORDS_SHA256.json'
    write(manifest, records)
    archive = exports / 'b601-sac-r1-records.tar.zst'
    print(json.dumps({'record_files': len(files), 'raw_bytes': sum(r['bytes'] for r in records.values())}), flush=True)
    with archive.open('wb') as raw, zstd.ZstdCompressor(level=1, threads=4, write_checksum=True).stream_writer(raw) as stream:
        with tarfile.open(fileobj=stream, mode='w|', bufsize=1024 * 1024) as tar:
            tar.add(manifest, arcname='RECORDS_SHA256.json', recursive=False)
            for path in files:
                tar.add(path, arcname=path.relative_to(HERE).as_posix(), recursive=False)
    verified = 0
    with archive.open('rb') as raw, zstd.ZstdDecompressor().stream_reader(raw) as stream:
        with tarfile.open(fileobj=stream, mode='r|') as tar:
            for member in tar:
                if member.name == 'RECORDS_SHA256.json':
                    continue
                expected = records[member.name]
                assert member.size == expected['bytes']
                with tar.extractfile(member) as data:
                    assert hashlib.file_digest(data, 'sha256').hexdigest() == expected['sha256']
                verified += 1
    assert verified == len(records)
    archive_sha = digest(archive)
    write(delivery / 'RECORD_ARCHIVE.json', {'name': archive.name, 'bytes': archive.stat().st_size,
        'sha256': archive_sha, 'verified_files': verified,
        'scope': 'new residual-rl-01 outputs only; previous ACT records not uploaded again'})
    source = delivery / 'source'
    source.mkdir(exist_ok=True)
    for path in HERE.glob('*.py'):
        shutil.copy2(path, source / path.name)
    for name in ['README.md', 'REPORT.md', 'PROTOCOL.md', 'requirements-windows.lock']:
        shutil.copy2(HERE / name, delivery / name)
    package_files = sorted(p for p in delivery.rglob('*') if p.is_file() and p.name != 'SHA256SUMS')
    (delivery / 'SHA256SUMS').write_text(''.join(f'{digest(p)}  {p.relative_to(delivery).as_posix()}\n' for p in package_files), encoding='utf-8')
    package = stage / 'b601-sac-r1-delivery.zip'
    with zipfile.ZipFile(package, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zip:
        for path in package_files + [delivery / 'SHA256SUMS']:
            zip.write(path, arcname=path.relative_to(delivery).as_posix())
    with zipfile.ZipFile(package) as zip:
        assert zip.testzip() is None
    parts, joined = [], hashlib.sha256()
    with archive.open('rb') as stream:
        while block := stream.read(64 * 1024 * 1024):
            path = stage / f'{archive.name}.part{len(parts) + 1:04d}'
            path.write_bytes(block)
            joined.update(block)
            parts.append({'name': path.name, 'bytes': len(block), 'sha256': digest(path)})
    assert joined.hexdigest() == archive_sha
    write(stage / 'BACKUP_MANIFEST.json', {'schema': 'b601-sac-r1-backup-v1',
        'source_commit': subprocess.check_output(['git', '-C', str(HERE), 'rev-parse', 'HEAD'], text=True).strip(),
        'adopted': False, 'default': 'V3', 'hardware_executable': False,
        'archive': {'name': archive.name, 'bytes': archive.stat().st_size, 'sha256': archive_sha},
        'parts_in_join_order': parts, 'delivery': {'name': package.name, 'bytes': package.stat().st_size, 'sha256': digest(package)}})
    (stage / 'RESTORE.txt').write_text(
        'Simulation-only SAC residual skill, not a replacement for V3.\n'
        '1. Verify downloads with SHA256SUMS.\n'
        '2. Concatenate records parts as raw bytes in BACKUP_MANIFEST.json order; verify joined archive SHA256.\n'
        '3. Decompress and extract into variants/residual-rl-01. Archive paths begin with outputs/.\n'
        '4. RECORDS_SHA256.json verifies every saved record. Raw partial-tail limitations are documented in REPORT.md.\n'
        '5. The small delivery ZIP contains selected policy, actor, inference/configuration, reports and evidence.\n'
        '6. Full-system reproduction also needs the repository frozen V3 and act-01/runtime.py; follow README.md.\n'
        'Only new RL records are included; prior ACT archives are not uploaded again.\n', encoding='utf-8')
    (stage / 'SHA256SUMS').write_text(''.join(f'{digest(p)}  {p.name}\n' for p in sorted(stage.iterdir()) if p.is_file()), encoding='utf-8')
    print(json.dumps({'verified_records': verified, 'archive_bytes': archive.stat().st_size,
        'archive_sha256': archive_sha, 'assets': len(list(stage.iterdir())), 'parts': len(parts),
        'delivery_zip_bytes': package.stat().st_size}), flush=True)
