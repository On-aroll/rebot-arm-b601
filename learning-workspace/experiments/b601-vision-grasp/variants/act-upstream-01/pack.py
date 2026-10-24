"""Incremental backup: one new checkpoint and the small diagnostic records only."""
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

from gate import HERE, REPO, REVISION, digest, write


if __name__ == '__main__':
    stage = HERE / 'outputs/exports/handoff/release-assets'
    stage.mkdir(parents=True, exist_ok=False)
    model = HERE / 'outputs/model-r2/checkpoint/model.safetensors'
    parts = []
    with model.open('rb') as stream:
        while block := stream.read(64 * 1024 * 1024):
            part = stage / f'b601-upstream-act-known-model.safetensors.part{len(parts) + 1:04d}'
            part.write_bytes(block)
            parts.append({'name': part.name, 'bytes': len(block), 'sha256': digest(part)})
    # Verify joining in memory-size-bounded streams, without creating a second model.
    joined = hashlib.sha256()
    for part in parts:
        with (stage / part['name']).open('rb') as stream:
            while block := stream.read(1024 * 1024):
                joined.update(block)
    assert joined.hexdigest() == digest(model)
    files = sorted(p for p in (HERE / 'outputs').rglob('*') if p.is_file()
        and 'exports' not in p.relative_to(HERE / 'outputs').parts and p != model)
    files += sorted(p for p in HERE.iterdir() if p.is_file())
    files += sorted(p for p in (HERE / 'evidence').iterdir() if p.is_file())
    records = {p.relative_to(HERE).as_posix(): {'bytes': p.stat().st_size, 'sha256': digest(p)} for p in files}
    package = stage / 'b601-upstream-act-known-evidence.zip'
    with zipfile.ZipFile(package, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in files:
            archive.write(path, arcname=path.relative_to(HERE).as_posix())
        archive.writestr('RECORDS_SHA256.json', json.dumps(records, indent=2) + '\n')
    with zipfile.ZipFile(package) as archive:
        assert archive.testzip() is None
        for name, expected in records.items():
            with archive.open(name) as stream:
                assert hashlib.file_digest(stream, 'sha256').hexdigest() == expected['sha256']
    write(stage / 'BACKUP_MANIFEST.json', {'schema': 'b601-upstream-act-gate-backup-v1',
        'source_commit': subprocess.check_output(['git', '-C', str(REPO), 'rev-parse', 'HEAD'], text=True).strip(),
        'upstream_revision': REVISION, 'default': 'V3', 'adopted': False, 'hardware_executable': False,
        'scope': 'one known A-goal demonstration; not a new verified grasp version',
        'model': {'name': model.name, 'bytes': model.stat().st_size, 'sha256': digest(model)},
        'parts_in_join_order': parts, 'evidence_zip': {'name': package.name, 'bytes': package.stat().st_size, 'sha256': digest(package)},
        'original_input_records': 'https://github.com/On-aroll/rebot-arm-b601/releases/tag/sim-act-2026-10-24'})
    (stage / 'RESTORE.txt').write_text(
        'This is an incremental simulation diagnostic, not V4 or a hardware-ready model.\n'
        '1. Verify all assets against SHA256SUMS.\n'
        '2. Extract the evidence ZIP into variants/act-upstream-01. It includes source, config, records and hashes.\n'
        '3. Concatenate model parts as raw bytes in BACKUP_MANIFEST.json order into outputs/model-r2/checkpoint/model.safetensors.\n'
        '4. Verify the joined model SHA256 against BACKUP_MANIFEST.json.\n'
        '5. Original demonstrations remain in the preceding sim-act-2026-10-24 release; they are not uploaded again here.\n'
        '6. Follow README.md to fetch pinned LeRobot source and run teacher/train/evaluate.\n', encoding='utf-8')
    (stage / 'SHA256SUMS').write_text(''.join(f'{digest(p)}  {p.name}\n' for p in sorted(stage.iterdir()) if p.is_file()), encoding='utf-8')
    print(json.dumps({'assets': len(list(stage.iterdir())), 'bytes': sum(p.stat().st_size for p in stage.iterdir()),
                      'model_sha256': digest(model)}))
