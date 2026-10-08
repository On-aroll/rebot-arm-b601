#!/usr/bin/env python3
"""按 configs/external-manifest.json 重建 external/ 里的第三方仓库。

为什么需要它：本仓库只提交自己的东西（含 motorbridge-studio 这个我们改过的 fork 的
源码），第三方仓库只记录 upstream 与 commit，不入库，否则仓库要 900 MB 起。
clone 完再跑一次本脚本即可复现开发环境。

用法:
    python3 scripts/setup_external.py            # 补齐缺失的仓库
    python3 scripts/setup_external.py --check    # 只看状态，不改动
    python3 scripts/setup_external.py --force    # 已存在的也重新对齐到锁定 commit
"""
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'configs/external-manifest.json'


def run(args, cwd=None):
    result = subprocess.run(args, cwd=cwd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f'{" ".join(args)} 失败：{result.stderr.strip()[:300]}')
    return result.stdout.strip()


def describe(entry):
    path = ROOT / entry['path']
    if not path.exists() or not any(path.iterdir()):
        return 'missing', path
    head_file = path / '.git'
    current = ''
    if head_file.is_dir():
        try:
            current = run(['git', '-C', str(path), 'rev-parse', 'HEAD'])
        except RuntimeError:
            current = ''
    elif entry.get('tracked_here'):
        current = entry['base_commit']          # fork 随本仓库提交，视为已对齐
    elif any(path.iterdir()):
        current = entry['base_commit']          # 无 .git 的文件树（按 commit 下载来的）
    return ('ok' if current == entry['base_commit'] else 'drift'), path


def clone(entry):
    path = ROOT / entry['path']
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        shutil.rmtree(path)
    sparse = entry.get('sparse_paths')
    print(f'  clone {entry["upstream"]}')
    run(['git', 'clone', '--filter=blob:none', entry['upstream'], str(path)])
    if sparse:
        run(['git', '-C', str(path), 'sparse-checkout', 'set', '--no-cone', *sum(
            [[f'{p}/**', f'!{p}/**/*.png', f'!{p}/**/*.STL', f'!{p}/**/*.obj'] for p in sparse], [])])
    run(['git', '-C', str(path), 'checkout', entry['base_commit']])
    print(f'  已对齐 {entry["base_commit"][:12]}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--check', action='store_true', help='只报告状态')
    parser.add_argument('--force', action='store_true', help='已存在的也重置到锁定 commit')
    args = parser.parse_args()

    if not MANIFEST.is_file():
        print(f'找不到清单：{MANIFEST}', file=sys.stderr)
        return 2
    manifest = json.loads(MANIFEST.read_text())
    todo = []
    for entry in manifest['repositories']:
        state, path = describe(entry)
        label = {'ok': '已就绪', 'drift': '版本不一致', 'missing': '缺失'}[state]
        print(f'[{label:>5}] {entry["path"]}')
        if state == 'ok' and not args.force:
            continue
        if entry.get('tracked_here') and state != 'missing':
            print('        这个 fork 随本仓库提交，跳过克隆')
            continue
        if args.check:
            todo.append(entry)
            continue
        clone(entry)

    if args.check and todo:
        print(f'\n需要执行 {len(todo)} 个仓库，去掉 --check 开始克隆。')
        return 1
    if not args.check:
        print('\n下一步：')
        print('  cd external/motorbridge-studio && npm ci')
        print('  VITE_BASE_PATH=/studio/ npm run build')
        print('  scripts/restart_sim.sh')
    return 0


if __name__ == '__main__':
    sys.exit(main())
