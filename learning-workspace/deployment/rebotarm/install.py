#!/usr/bin/env python3
"""Install the bundled ROS workbench as the calling desktop user."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

SOURCE = Path(__file__).resolve().parents[1]
NODE = 'node-v22.23.2-linux-x64.tar.xz'
NODE_SHA = 'd60acfe00a2932254bb0ad20e01b0d74397a0875595de719654b214f4b03f307'
ROS_DEB = 'ros2-apt-source_1.3.0.noble_all.deb'
ROS_SHA = 'f31d84adf5054c7d60ded0e82c0f776a77ea33af53d08f40b9ad7c94cca55296'
PACKAGES = [
    'ca-certificates', 'curl', 'git', 'build-essential', 'cmake', 'ninja-build',
    'pkg-config', 'python3-dev', 'python3-venv', 'python3-pip', 'python3-tk',
    'python3-colcon-common-extensions', 'python3-rosdep', 'iproute2', 'can-utils',
    'util-linux', 'xz-utils', 'xdg-utils', 'pkexec', 'libgl1', 'libegl1',
    'libgl1-mesa-dri', 'libosmesa6', 'libx11-6', 'libxrandr2', 'libxinerama1',
    'libxcursor1', 'libxi6', 'ros-jazzy-desktop', 'ros-jazzy-rosbridge-suite',
    'ros-jazzy-moveit', 'ros-jazzy-tf-transformations', 'ros-jazzy-ros2-control',
    'ros-jazzy-ros2-controllers',
]


def run(*args, **kwargs):
    print('+', ' '.join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), check=True, **kwargs)


def digest(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def verify(root):
    manifest = json.loads((root / 'BUNDLE_MANIFEST.json').read_text())
    for name, expected in manifest['files'].items():
        path = root / name
        if not path.is_file() or path.is_symlink() or digest(path) != expected:
            raise RuntimeError(f'安装文件校验失败：{name}')
    for name, expected in [(NODE, NODE_SHA), (ROS_DEB, ROS_SHA)]:
        if digest(root / 'deployment' / name) != expected:
            raise RuntimeError(f'依赖校验失败：{name}')
    return manifest


def check_platform():
    release = {}
    for line in Path('/etc/os-release').read_text().splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            release[key] = value.strip('"')
    if release.get('ID') != 'ubuntu' or release.get('VERSION_ID') != '24.04':
        raise RuntimeError('此安装包要求 Ubuntu / Kubuntu 24.04；其他系统尚未验证。')
    if os.uname().machine != 'x86_64' or sys.version_info[:2] != (3, 12):
        raise RuntimeError('此安装包要求 x86_64 与系统 Python 3.12。')
    if os.geteuid() == 0:
        raise RuntimeError('请用普通桌面用户运行，不要 sudo bash；系统依赖步骤会单独请求 sudo。')


def desktop_entry(prefix):
    # Use the stable current link so upgrades do not invalidate shortcuts.
    current = prefix / 'current'
    entry = '\n'.join([
        '[Desktop Entry]', 'Type=Application', 'Name=ReBot Arm B601-RS 控制工作台',
        'Comment=ROS 2 Jazzy · 仿真 · 反馈孪生 · 程序与算法',
        'Exec=' + json.dumps(str(current / 'rebotarm-app'), ensure_ascii=False) + ' start',
        f'Icon={current}/reBotArm_simulator-RS/public/favicon.png',
        'Terminal=false', 'Categories=Development;Science;', '',
    ])
    applications = Path(os.environ.get('XDG_DATA_HOME', str(Path.home() / '.local/share'))) / 'applications'
    applications.mkdir(parents=True, exist_ok=True)
    (applications / 'rebotarm-b601-rs.desktop').write_text(entry)
    try:
        directory = Path(subprocess.check_output(['xdg-user-dir', 'DESKTOP'], text=True).strip())
    except (OSError, subprocess.SubprocessError):
        directory = Path.home() / 'Desktop'
    if directory != Path.home():
        directory.mkdir(parents=True, exist_ok=True)
        shortcut = directory / 'rebotarm-b601-rs.desktop'
        shortcut.write_text(entry)
        shortcut.chmod(0o755)


def main():
    parser = argparse.ArgumentParser(description='B601-RS Ubuntu 24.04 / Jazzy 一键部署')
    parser.add_argument('--prefix', type=Path, default=Path.home() / '.local/share/rebotarm-b601-rs')
    parser.add_argument('--dry-run', action='store_true', help='校验文件并显示计划，不安装系统依赖')
    parser.add_argument('--skip-system', action='store_true', help='管理员已预装全部系统依赖时使用')
    parser.add_argument('--no-launch', action='store_true', help='安装后不打开网页')
    args = parser.parse_args()
    check_platform()
    manifest = verify(SOURCE)
    prefix = args.prefix.expanduser().resolve()
    # colcon-generated scripts and several upstream build tools reject spaces.
    if any(char.isspace() for char in str(prefix)) or any(c in str(prefix) for c in '%$`"\\'):
        raise RuntimeError('安装路径不能包含空白或 % $ ` " \\；请用 --prefix 指定简单路径。')
    identity = manifest['version'] + '-' + manifest['source_id'][:12]
    destination = prefix / 'versions' / identity
    print(f'部署版本：{identity}\n安装目录：{destination}\n持久数据：{prefix}/shared/outputs', flush=True)
    if args.dry_run:
        print('系统包：' + ' '.join(PACKAGES))
        print('流程：官方 ROS 软件源 → 系统包 → 独立 Node 22 → 哈希锁定 Python 依赖 → colcon → 无硬件自检 → 桌面入口')
        print('默认仅启动网页；实机反馈与电机使能由工作台显式操作。')
        return
    if (prefix / 'current').exists() and not (prefix / 'current').is_symlink():
        raise RuntimeError('current 不是部署器管理的软链接，未覆盖。')
    # Updating code underneath live CAN workers would invalidate their recovery
    # provenance. Require the existing explicit project shutdown first.
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            cwd = (proc / 'cwd').resolve(strict=True)
            command = (proc / 'cmdline').read_bytes().decode(errors='replace').split('\0')
            entries = {Path(arg).name for arg in command}
            service = bool(entries & {'server.js', 'FakeRsDriver', 'mujoco_rs_sync',
                                      'start_rs_sim.sh', 'rosbridge_websocket', 'rebotarm_mcp_server'})
            service = service or any(arg in command for arg in ['rebotarmcontroller.csp_twin_driver',
                                                               'rebotarm_agent.rebotarm_text_agent'])
            if service and proc.stat().st_uid == os.getuid() and cwd.is_relative_to(prefix / 'versions'):
                raise RuntimeError('旧版本仍有运行进程，请先在网页点击“结束项目”后重新安装。')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    prefix.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(prefix).free < 6 * 1024**3:
        raise RuntimeError('安装目录至少需要 6 GiB 可用空间。')
    if not args.skip_system:
        run('sudo', '-v')
        run('sudo', 'apt-get', 'update')
        run('sudo', 'apt-get', 'install', '-y', 'software-properties-common', 'ca-certificates')
        run('sudo', 'add-apt-repository', '-y', 'universe')
        available = subprocess.run(['apt-cache', 'show', 'ros-jazzy-desktop'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
        if not available:
            run('sudo', 'dpkg', '-i', SOURCE / 'deployment' / ROS_DEB)
        run('sudo', 'apt-get', 'update')
        run('sudo', 'apt-get', 'install', '-y', *PACKAGES)
    if not Path('/opt/ros/jazzy/setup.bash').is_file() or not shutil.which('colcon'):
        raise RuntimeError('缺少 ROS 2 Jazzy / colcon，请安装系统依赖后重试。')
    if destination.exists():
        verify(destination)
    else:
        shutil.copytree(SOURCE, destination)
    shared = prefix / 'shared/outputs'
    shared.mkdir(parents=True, exist_ok=True)
    shared_env = prefix / 'shared/web.env'
    if not shared_env.exists():
        shutil.copy2(destination / 'reBotArm_simulator-RS/.env.example', shared_env)
        shared_env.chmod(0o600)
    web_env = destination / 'reBotArm_simulator-RS/.env'
    if not web_env.is_symlink():
        if web_env.exists():
            raise RuntimeError('发现独立 .env 文件，未覆盖；请先核查配置。')
        web_env.symlink_to(shared_env)
    outputs = destination / 'outputs'
    if not outputs.is_symlink():
        if outputs.exists():
            raise RuntimeError('发现已有独立 outputs 目录，未覆盖数据。')
        outputs.symlink_to(shared, target_is_directory=True)
    tools = destination / '.tools'
    tools.mkdir(exist_ok=True)
    node = tools / 'node'
    if not node.exists():
        with tarfile.open(destination / 'deployment' / NODE) as archive:
            archive.extractall(tools, filter='data')
        (tools / 'node-v22.23.2-linux-x64').rename(node)
    env = dict(os.environ, PATH=str(node / 'bin') + ':' + os.environ['PATH'])
    env.pop('ROS_DISTRO', None)
    env.pop('VIRTUAL_ENV', None)
    # All packages build at their final absolute path; venv/build products are
    # never copied from the development computer or moved after generation.
    run('/bin/bash', destination / 'deployment/build_workspace.sh', cwd=destination, env=env)
    run('/bin/bash', destination / 'deployment/doctor.sh', cwd=destination, env=env)
    switch = prefix / f'.current-{os.getpid()}'
    switch.symlink_to(destination, target_is_directory=True)
    switch.replace(prefix / 'current')
    desktop_entry(prefix)
    print('\n部署完成。应用菜单：ReBot Arm B601-RS 控制工作台')
    print(f'命令启动：{prefix}/current/rebotarm-app start')
    print(f'诊断：{prefix}/current/rebotarm-app doctor')
    if not args.no_launch:
        run(prefix / 'current/rebotarm-app', 'start', env=env)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(f'\n部署未完成：{error}\n修复后可重新运行，已有配置与运行记录会保留。', file=sys.stderr)
        raise SystemExit(1)
