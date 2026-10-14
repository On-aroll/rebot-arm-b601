#!/usr/bin/env python3
"""Start local services without motor commands or automatic task replay."""
import argparse
from contextlib import nullcontext
from dataclasses import dataclass, field
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
URL = 'http://127.0.0.1:8765/studio/console'
HTTP = urllib.request.build_opener(urllib.request.ProxyHandler({}))


@dataclass
class Service:
    name: str
    port: int
    command: list
    identity: Path
    log: Path
    pidfile: Path
    timeout: float = 30
    env: dict = field(default_factory=dict)


def services(root=ROOT, sim_only=False, no_ros=False):
    output = root / 'outputs'
    python = Path(os.environ.get('REBOT_PYTHON', str(Path.home() / '.local/share/robotics/envs/rebot-sim/bin/python'))).expanduser()
    if not python.is_file() and (root / '.venv-sim/bin/python').is_file():
        python = root / '.venv-sim/bin/python'
    result = [Service('MuJoCo / 网页', 8765,
        [str(python), '-u', str(root / 'scripts/rebot_sim.py'), 'serve', '--host', '127.0.0.1', '--port', '8765'],
        root / 'scripts/rebot_sim.py', output / 'dashboard.log', output / 'dashboard.pid', 60,
        {'MUJOCO_GL': 'egl', 'PYOPENGL_PLATFORM': 'egl', 'REBOT_OUTPUT_DIR': str(output)})]
    if not sim_only:
        binaries = sorted(root.glob('.venv/lib/python*/site-packages/motorbridge/bin/ws_gateway'))
        binary = binaries[0] if len(binaries) == 1 else root / '.venv/missing-ws_gateway'
        result.append(Service('实机网关（仅监听）', 9002,
            [str(binary), '--bind', '127.0.0.1:9002'], binary,
            output / 'gateway.log', output / 'gateway.pid', 15))
        if not no_ros:
            result.append(Service('ROS 虚拟验证', 8767,
                ['bash', str(root / 'scripts/start_ros_validation.sh')], root / 'scripts/rebot_ros_validation.py',
                output / 'startup/ros.log', output / 'startup/ros.pid'))
    return result


def listener(port):
    result = subprocess.run(['ss', '-H', '-ltnp', f'sport = :{port}'],
                            capture_output=True, text=True, check=True, timeout=5)
    if not result.stdout.strip():
        return None
    pids = set(re.findall(r'pid=(\d+)', result.stdout))
    if len(pids) != 1:
        raise RuntimeError(f'端口 {port} 已占用，无法唯一确认进程；请人工检查，未结束任何进程')
    return int(pids.pop())


def owns(service, pid):
    try:
        proc = Path('/proc') / str(pid)
        if proc.stat().st_uid != os.getuid():
            return False
        if service.identity.name == 'ws_gateway':
            return (proc / 'exe').resolve() == service.identity.resolve()
        argv = (proc / 'cmdline').read_bytes().decode().split('\0')
        cwd = (proc / 'cwd').resolve()
        return any((Path(arg) if Path(arg).is_absolute() else cwd / arg).resolve() == service.identity.resolve()
                   for arg in argv if arg.endswith('.py'))
    except (OSError, UnicodeError):
        return False


def get_state(port):
    with HTTP.open(f'http://127.0.0.1:{port}/state', timeout=3) as response:
        return json.load(response)


def healthy(service):
    if service.port == 9002:
        with socket.create_connection(('127.0.0.1', service.port), timeout=3):
            return
    state = get_state(service.port)
    if service.port == 8765:
        if state.get('backend') != 'mujoco' or state.get('error'):
            raise RuntimeError('仿真后端或错误状态异常')
        time.sleep(.4)
        next_state = get_state(service.port)
        if next_state.get('error') or next_state.get('stream_id') != state.get('stream_id') or next_state.get('sequence', 0) <= state.get('sequence', 0):
            raise RuntimeError('仿真状态未持续更新')
        with HTTP.open(URL, timeout=3) as response:
            if b'<div id="root"' not in response.read():
                raise RuntimeError('Studio 页面缺失或不是构建后的前端，请按重启指南构建')
    elif state.get('backend') != 'ros2-fake' or state.get('namespace') != 'rebotarm_validation' or state.get('hardware') is not False or state.get('domain') != 43 or state.get('fresh') is not True:
        raise RuntimeError('ROS 不是预期的隔离虚拟服务，或反馈已过期')


def ensure_service(service, root=ROOT, status_only=False):
    pid = listener(service.port)
    if pid is not None:
        if not owns(service, pid):
            raise RuntimeError(f'端口 {service.port} 被其他进程占用（PID {pid}）；未结束它')
        healthy(service)
        return {'status': 'running', 'pid': pid, 'reused': True}
    if status_only:
        return {'status': 'not-running', 'pid': None}
    if not shutil.which(service.command[0]):
        raise RuntimeError(f'缺少可执行文件：{service.command[0]}')
    if not service.identity.is_file():
        raise RuntimeError(f'缺少服务文件：{service.identity}')
    service.log.parent.mkdir(parents=True, exist_ok=True)
    service.pidfile.parent.mkdir(parents=True, exist_ok=True)
    with service.log.open('ab') as log:
        child = subprocess.Popen(service.command, cwd=root, env=dict(os.environ, **service.env),
                                 stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
    service.pidfile.write_text(f'{child.pid}\n')
    deadline = time.monotonic() + service.timeout
    error = '尚未监听'
    while time.monotonic() < deadline:
        code = child.poll()
        if code is not None:
            raise RuntimeError(f'启动进程退出（{code}），查看 {service.log}')
        try:
            pid = listener(service.port)
            if pid is not None:
                if not owns(service, pid):
                    raise RuntimeError(f'启动期间端口被其他进程占用：{pid}')
                healthy(service)
                service.pidfile.write_text(f'{pid}\n')
                return {'status': 'running', 'pid': pid, 'reused': False}
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
            error = str(exc)
        time.sleep(.5)
    raise RuntimeError(f'启动等待超时：{error}；查看 {service.log}，未强制结束进程')


def can_state():
    result = subprocess.run(['ip', '-json', '-details', 'link', 'show', 'dev', 'can0'],
                            capture_output=True, text=True, timeout=5)
    if result.returncode:
        return {'ready': False, 'message': '未发现 can0；仿真仍可使用'}
    link = json.loads(result.stdout)[0]
    data = link.get('linkinfo', {}).get('info_data', {})
    bitrate = data.get('bittiming', {}).get('bitrate')
    up = 'UP' in link.get('flags', [])
    modes = data.get('ctrlmode', [])
    state = data.get('state', '未知')
    ready = up and bitrate == 1000000 and state == 'ERROR-ACTIVE' and not any(mode in modes for mode in ('LISTEN-ONLY', 'LOOPBACK'))
    return {'ready': ready, 'up': up, 'bitrate': bitrate, 'state': state,
            'message': f"can0 {'UP' if up else 'DOWN'} / {bitrate or '未知'} bps / {state}"}


def prepare_can():
    state = can_state()
    if state.get('ready'):
        return
    if 'up' not in state:
        raise RuntimeError(state['message'])
    if state['up']:
        raise RuntimeError('can0 已 UP 但配置或总线状态异常；未中断现有通信，请按重启指南检查')
    for command in (['ip', 'link', 'set', 'can0', 'type', 'can', 'bitrate', '1000000', 'listen-only', 'off', 'loopback', 'off'],
                    ['ip', 'link', 'set', 'can0', 'up']):
        subprocess.run(command if os.geteuid() == 0 else ['sudo', *command], check=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description='启动 reBot 本地项目服务；不使能电机、不恢复动作任务')
    parser.add_argument('--status', action='store_true', help='只读查看服务和 CAN，不启动、不写报告')
    parser.add_argument('--no-browser', action='store_true', help='不打开浏览器')
    parser.add_argument('--sim-only', action='store_true', help='仅启动 MuJoCo 与网页')
    parser.add_argument('--no-ros', action='store_true', help='跳过 ROS 虚拟验证服务')
    parser.add_argument('--can-up', action='store_true', help='显式配置 DOWN 的 can0 为 1 Mbps 并 UP，需要 sudo')
    args = parser.parse_args(argv)
    if args.can_up and (args.status or args.sim_only):
        parser.error('--can-up 不能与 --status 或 --sim-only 同用')
    for command in ('ss', 'ip'):
        if not shutil.which(command):
            parser.error(f'缺少系统命令：{command}')
    output = ROOT / 'outputs/startup'
    if not args.status:
        output.mkdir(parents=True, exist_ok=True)
    report = {'time': time.strftime('%Y-%m-%d %H:%M:%S %z'), 'url': URL, 'services': {}, 'motor_commands_sent': False}
    with (nullcontext() if args.status else (output / 'launcher.lock').open('a')) as lock:
        if lock is not None:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                print('另一个启动程序正在运行；请等待它完成。', file=sys.stderr)
                return 1
        if args.can_up:
            try:
                prepare_can()
            except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
                report['can_error'] = str(exc)
                print(f'CAN 准备失败：{exc}', flush=True)
        for service in services(sim_only=args.sim_only, no_ros=args.no_ros):
            print(f'检查 {service.name}（{service.port}）...', flush=True)
            try:
                result = ensure_service(service, status_only=args.status)
            except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
                result = {'status': 'error', 'error': str(exc), 'log': str(service.log)}
            report['services'][str(service.port)] = result
            print('  ' + (f"正常 · PID {result['pid']}" if result['status'] == 'running' else result.get('error', '未运行')), flush=True)
        report['can'] = can_state()
        print('CAN：' + report['can']['message'], flush=True)
        if not report['can']['ready']:
            print('实机通信尚未就绪；接口 DOWN 时可运行：python3 scripts/start_project.py --can-up')
        if not args.status:
            (output / 'last-start.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
            print(f'启动报告：{output / "last-start.json"}')
        sim_ready = report['services']['8765']['status'] == 'running'
        if sim_ready:
            print(f'控制台：{URL}')
            if not args.status and not args.no_browser:
                opener = shutil.which('xdg-open')
                if opener:
                    subprocess.Popen([opener, URL], stdin=subprocess.DEVNULL,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
                else:
                    print('未找到 xdg-open，请在浏览器打开上面的地址。')
        print('启动不使能电机、不写设备参数、不自动执行或续跑任务。')
        return int(any(item['status'] != 'running' for item in report['services'].values()) or 'can_error' in report)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f'启动检查失败：{exc}', file=sys.stderr)
        sys.exit(1)
