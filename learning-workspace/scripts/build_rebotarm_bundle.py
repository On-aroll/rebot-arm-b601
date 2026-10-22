#!/usr/bin/env python3
"""Snapshot both ROS chats into a relocatable, checksum verified Linux installer."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]
OVERLAY = ROOT / 'deployment/rebotarm'
SDK_COMMIT = '6415d43130d1e143c70dc106096a857ac5556f81'
VERSION = '1.0.0'
NAME = f'rebotarm-b601-rs-{VERSION}-ubuntu24.04-x86_64'
EXCLUDED = {'.git', '.venv', '__pycache__', 'node_modules', '.certs', 'outputs',
            'build', 'install', 'log', '.pytest_cache', '.mypy_cache'}


def digest(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def copy_tree(source, destination):
    def ignore(directory, names):
        return [n for n in names if n in EXCLUDED or n == '.env' or n.endswith(('.pyc', '.pem', '.key'))]
    shutil.copytree(source, destination, ignore=ignore)


def replace(path, before, after):
    value = path.read_text()
    if before not in value:
        raise RuntimeError(f'源码已变化，请重新核对部署适配：{path.name}')
    path.write_text(value.replace(before, after))


def stage_bundle(source, stage):
    lock = json.loads((source / 'configs/csp-sdk-lock.json').read_text())
    if lock.get('schema_version') != 1 or lock.get('commit') != SDK_COMMIT or lock.get('path') != 'rebotarm_ros2_RS/third_party/reBotArm_control_py_csp':
        raise RuntimeError('CSP SDK清单与打包器审核版本不一致，未打包。')
    sdk = source / 'rebotarm_ros2_RS/third_party/reBotArm_control_py_csp'
    head = subprocess.check_output(['git', '-C', str(sdk), 'rev-parse', 'HEAD'], text=True).strip()
    changes = subprocess.check_output(['git', '--no-optional-locks', '-C', str(sdk), 'status', '--porcelain', '--untracked-files=all'], text=True)
    if head != SDK_COMMIT or changes.strip():
        raise RuntimeError('CSP SDK 与已审核提交不符，未打包。')
    (stage / 'configs').mkdir()
    shutil.copy2(source / 'configs/csp-sdk-lock.json', stage / 'configs/csp-sdk-lock.json')
    for name in ['scripts', 'examples', 'reBotArm_simulator-RS']:
        copy_tree(source / name, stage / name)
    for name in ['src', 'third_party/reBotArm_control_py']:
        copy_tree(source / 'rebotarm_ros2_RS' / name, stage / 'rebotarm_ros2_RS' / name)
    shutil.copy2(source / 'rebotarm_ros2_RS/third_party/README.md', stage / 'rebotarm_ros2_RS/third_party/README.md')
    docs = stage / 'docs'
    docs.mkdir()
    for file in (source / 'docs').rglob('*.md'):
        relative = file.relative_to(source / 'docs')
        if any(part in EXCLUDED for part in relative.parts):continue
        target_doc = docs / relative
        target_doc.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(file, target_doc)
    for file in source.glob('requirements-rs-*.txt'):
        shutil.copy2(file, stage / file.name)
    shutil.copy2(source / 'requirements-vision-offline.txt', stage / 'requirements-vision-offline.txt')
    for file in source.iterdir():
        if file.is_file() and (file.name.startswith(('LICENSE', 'NOTICE')) or file.name.endswith('_ZH.md')):
            shutil.copy2(file, stage / file.name)
    # The controller intentionally verifies both the upstream commit and a
    # clean source tree before CAN use. Preserve that check with a fresh,
    # one-commit repository containing no development remotes or local history.
    target = stage / 'rebotarm_ros2_RS/third_party/reBotArm_control_py_csp'
    subprocess.run(['git', 'init', '-q', str(target)], check=True)
    subprocess.run(['git', '-C', str(target), '-c', 'protocol.file.allow=always',
                    'fetch', '-q', '--depth=1', sdk.as_uri(), SDK_COMMIT], check=True)
    subprocess.run(['git', '-C', str(target), 'checkout', '-q', '--detach', 'FETCH_HEAD'], check=True)
    # FETCH_HEAD records the development path and is not needed at runtime.
    (target / '.git/FETCH_HEAD').unlink()
    shutil.rmtree(target / '.git/hooks')
    shutil.rmtree(target / '.git/logs', ignore_errors=True)

    deployment = stage / 'deployment'
    deployment.mkdir()
    for file in OVERLAY.iterdir():
        if file.is_file() and file.name not in {'self-extract.sh', 'ros.key'}:
            shutil.copy2(file, deployment / file.name)
    shutil.copy2(OVERLAY / 'rebotarm-app', stage / 'rebotarm-app')
    (stage / 'rebotarm-app').chmod(0o755)
    shutil.copy2(OVERLAY / 'README.md', stage / 'README.md')
    (stage / 'install.sh').write_text('#!/usr/bin/env bash\nset -Eeuo pipefail\nexec /usr/bin/python3 "$(dirname "$(readlink -f "$0")")/deployment/install.py" "$@"\n')
    (stage / 'install.sh').chmod(0o755)
    # The runtime's ABI and safeguards stay unchanged. Only locate the project
    # from its installed path and choose the bundled Node over system Node 18.
    replace(stage / 'scripts/rs_env.sh',
            'REBOTARM_RS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"',
            'REBOTARM_RS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"\n'
            'export PATH="${REBOTARM_RS_ROOT}/.tools/node/bin:${PATH}"\n'
            'export REBOTARM_GATEWAY_ROOT="${REBOTARM_GATEWAY_ROOT:-${REBOTARM_RS_ROOT}}"')
    replace(stage / 'scripts/quick_start.sh',
            'ENV_SCRIPT="${ROOT_DIR}/scripts/rs_env.sh"',
            'export PATH="${ROOT_DIR}/.tools/node/bin:${PATH}"\nENV_SCRIPT="${ROOT_DIR}/scripts/rs_env.sh"')
    replace(stage / 'rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/csp_twin_driver.py',
            '"/home/bonvoyage/Desktop/seeed_Robotics"',
            'str(next(parent for parent in Path(__file__).resolve().parents if (parent / "rebotarm_ros2_RS").is_dir()))')
    replace(stage / 'rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/csp_runtime/rebot_seeed_transport.py',
            "OFFICIAL = Path(__file__).resolve().parents[4] / 'third_party/reBotArm_control_py_csp'",
            "OFFICIAL = next(parent / 'third_party/reBotArm_control_py_csp' for parent in Path(__file__).resolve().parents "
            "if (parent / 'third_party/reBotArm_control_py_csp').is_dir())")
    # Generate launchers for the target user; never ship the development path.
    shutil.copy2(OVERLAY / 'install_desktop_launcher.sh', stage / 'scripts/install_desktop_launcher.sh')
    # Git may refresh its index stat cache during the runtime provenance check.
    # Hash the SDK sources; the original commit/tree check remains in place.
    files = {p.relative_to(stage).as_posix(): digest(p) for p in sorted(stage.rglob('*'))
             if p.is_file() and '.git' not in p.relative_to(stage).parts}
    source_id = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()
    manifest = dict(version=VERSION, source_id=source_id, sdk_commit=SDK_COMMIT,
                    source_threads=['01a144e1-f673-79c2-9d4c-539ea322ca55', '01a13e02-997e-7540-9723-8838e8bec1a4'],
                    files=files)
    (stage / 'BUNDLE_MANIFEST.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True, help='两个对话成果所在的 ReBot_Arm_DigitalTwin_RS 目录')
    parser.add_argument('--output', type=Path, default=ROOT / 'dist')
    parser.add_argument('--stage', type=Path, help='保留构建暂存目录，便于验证')
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rebotarm-build-') as temporary:
        stage = args.stage.resolve() if args.stage else Path(temporary) / 'source'
        stage.mkdir(parents=True, exist_ok=False)
        manifest = stage_bundle(source, stage)
        archive = output / f'{NAME}.tar.gz'
        with tarfile.open(archive, 'w:gz', compresslevel=6) as handle:
            for file in sorted(stage.iterdir()):
                handle.add(file, arcname=file.name)
        payload_sha = digest(archive)
        header = (OVERLAY / 'self-extract.sh').read_text().replace('@PAYLOAD_SHA@', payload_sha).encode()
        executable = output / f'{NAME}.run'
        with executable.open('wb') as handle:
            handle.write(header)
            with archive.open('rb') as payload:
                shutil.copyfileobj(payload, handle)
        executable.chmod(0o755)
        sums = output / 'REBOTARM_SHA256SUMS'
        sums.write_text(''.join(f'{digest(p)}  {p.name}\n' for p in [executable, archive]))
        (output / f'{NAME}.manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
        shutil.copy2(OVERLAY / 'README.md', output / 'ReBotArm-部署说明.md')
        print(json.dumps(dict(installer=str(executable), archive=str(archive),
                             installer_bytes=executable.stat().st_size, source_id=manifest['source_id'],
                             files=len(manifest['files'])), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
