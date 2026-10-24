"""Package inference artifacts and all local experiment evidence, with hashes."""
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import zipfile

import zstandard as zstd
from runtime import ROOT, digest, verify_v3, write


if __name__ == '__main__':
    verify_v3()
    outputs, delivery = ROOT / 'outputs', ROOT / 'delivery'
    exports = outputs / 'exports'
    exports.mkdir(exist_ok=True)
    archive = exports / 'b601-act-20261024-records.tar.zst'
    package = exports / 'b601-act-20261024-delivery.zip'
    if archive.exists() or package.exists():
        raise FileExistsError('Preserve existing archives; choose a fresh run')
    paths = sorted(p for p in outputs.rglob('*') if p.is_file()
                   and exports not in p.parents and '__pycache__' not in p.parts)
    records = {}
    for i, path in enumerate(paths):
        records[path.relative_to(ROOT).as_posix()] = {'size': path.stat().st_size, 'sha256': digest(path)}
        if i % 400 == 0:
            print(json.dumps({'hashed_records': i, 'total': len(paths)}), flush=True)
    manifest = delivery / 'evidence/RECORDS_SHA256.json'
    write(manifest, records)
    print(json.dumps({'compressing_bytes': sum(r['size'] for r in records.values())}), flush=True)
    with archive.open('wb') as raw, zstd.ZstdCompressor(level=1, threads=4, write_checksum=True).stream_writer(raw) as compressed:
        with tarfile.open(fileobj=compressed, mode='w|', bufsize=1024*1024) as tar:
            tar.add(manifest, arcname='RECORDS_SHA256.json', recursive=False)
            for path in paths:
                tar.add(path, arcname=path.relative_to(ROOT).as_posix(), recursive=False)
    print('Verifying every archived file', flush=True)
    verified = 0
    with archive.open('rb') as raw, zstd.ZstdDecompressor().stream_reader(raw) as decompressed:
        with tarfile.open(fileobj=decompressed, mode='r|', bufsize=1024*1024) as tar:
            for member in tar:
                if member.name == 'RECORDS_SHA256.json':
                    continue
                expected = records[member.name]
                assert member.size == expected['size']
                with tar.extractfile(member) as stream:
                    assert hashlib.file_digest(stream, 'sha256').hexdigest() == expected['sha256']
                verified += 1
    assert verified == len(records)
    for name in ('README.md', 'REPORT.md', 'REAL_DATA.md', 'PROTOCOL.md'):
        shutil.copy2(ROOT / name, delivery / name)
    write(delivery / 'RECORD_ARCHIVE.json', {'filename': archive.name,
        'sha256': digest(archive), 'bytes': archive.stat().st_size, 'verified_files': verified,
        'restore': 'Decompress and extract into variants/act-01; member paths begin with outputs/'})
    files = sorted(p for p in delivery.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.name != 'SHA256SUMS')
    (delivery / 'SHA256SUMS').write_text(''.join(f'{digest(p)}  {p.relative_to(delivery).as_posix()}\n' for p in files))
    with zipfile.ZipFile(package, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zip:
        for path in files + [delivery / 'SHA256SUMS']:
            zip.write(path, arcname=path.relative_to(delivery).as_posix())
    with zipfile.ZipFile(package) as zip:
        assert zip.testzip() is None
    result = {p.name: {'path': str(p), 'bytes': p.stat().st_size, 'sha256': digest(p)} for p in (package, archive)}
    write(exports / 'ARTIFACTS.json', result)
    print(json.dumps(result), flush=True)
