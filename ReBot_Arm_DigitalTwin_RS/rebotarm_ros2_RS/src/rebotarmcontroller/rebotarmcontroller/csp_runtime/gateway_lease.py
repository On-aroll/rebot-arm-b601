"""Project gateway lease, extracted without the old HTTP/UI layer."""
import fcntl, os, re, shutil, signal, socket, subprocess, time
from pathlib import Path

class GatewayLease:
    """Temporarily stop the project's read-only gateway around a CAN trial.

    The Rust gateway has no project-level lease API. Closing one browser socket
    therefore cannot prevent another tab from reopening a CAN receive filter
    while the SDK worker is starting. This lease only acts on a gateway whose
    executable and process identity are verified; an unknown listener is never
    terminated. A missing gateway is allowed for offline workers.
    """

    def __init__(self, root, *, clock=time.monotonic, sleeper=time.sleep,
                 kill=os.kill, popen=subprocess.Popen):
        self.root = Path(root)
        self.clock = clock
        self.sleeper = sleeper
        self.kill = kill
        self.popen = popen
        self.pidfile = self.root / 'outputs/gateway.pid'
        self.logfile = self.root / 'outputs/gateway.log'
        self.lockfile = self.root / 'outputs/hardware-can-session.lock'
        self.lock_handle = None
        self.pid = None
        self.argv = None
        self.cwd = self.root
        self.proc = None
        self.active = False
        self.present = False
        self.kill_sent = False
        self.stopped = False

    def _binaries(self):
        return sorted(self.root.glob('.venv/lib/python*/site-packages/motorbridge/bin/ws_gateway'))

    def _pid_from_file(self):
        try:
            value = int(self.pidfile.read_text().strip())
        except (OSError, ValueError):
            return None
        return value if value > 0 else None

    def _listener_pid(self):
        if not shutil.which('ss'):
            raise RuntimeError('缺少 ss，无法确认 9002 是否已释放，拒绝实机接管')
        try:
            result = subprocess.run(
                ['ss', '-H', '-ltnp', 'sport = :9002'],
                capture_output=True, text=True, check=False, timeout=3,
            )
        except (OSError, subprocess.SubprocessError):
            raise RuntimeError('无法检查 9002 监听进程，拒绝实机接管')
        if result.returncode != 0:
            raise RuntimeError('ss 无法确认 9002 监听状态，拒绝实机接管')
        lines = result.stdout.splitlines()
        if not lines:
            return None
        pids = set(re.findall(r'pid=(\d+)', result.stdout))
        if len(pids) != 1:
            raise RuntimeError('9002 监听进程无法唯一识别，拒绝实机接管')
        return int(pids.pop())

    def _read_process(self, pid, expected):
        proc = Path('/proc') / str(pid)
        try:
            if proc.stat().st_uid != os.getuid():
                raise RuntimeError('9002 网关属于其他用户，拒绝接管 CAN')
            if (proc / 'exe').resolve() != expected.resolve():
                raise RuntimeError('9002 端口进程不是项目锁定的 ws_gateway，拒绝结束它')
            argv = [item for item in (proc / 'cmdline').read_bytes().decode().split('\0') if item]
            if not argv or Path(argv[0]).resolve() != expected.resolve():
                raise RuntimeError('ws_gateway 启动命令来源不符，拒绝接管 CAN')
            cwd = (proc / 'cwd').resolve()
        except (OSError, UnicodeError) as exc:
            raise RuntimeError(f'无法核验 9002 网关进程：{exc}') from exc
        return argv, cwd

    def _alive(self, pid):
        try:
            stat = (Path('/proc') / str(pid) / 'stat').read_text()
        except OSError:
            return False
        # /proc/<pid>/stat has a comm field that may contain ')'; use the last
        # closing parenthesis before the one-letter process state.
        state_start = stat.rfind(')') + 2
        return state_start < len(stat) and stat[state_start] != 'Z'

    def acquire(self):
        if self.active:
            raise RuntimeError('实机任务已持有网关租约')
        self.lockfile.parent.mkdir(parents=True, exist_ok=True)
        self.lock_handle = self.lockfile.open('a+')
        try:
            fcntl.flock(self.lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self.lock_handle.close()
            self.lock_handle = None
            if not isinstance(exc, BlockingIOError):
                raise
            raise RuntimeError('本项目已有 CAN/网关操作正在进行，拒绝并发实机任务') from exc
        try:
            binaries = self._binaries()
            if len(binaries) > 1:
                raise RuntimeError('发现多个未锁定版本的 ws_gateway，拒绝启动实机任务')
            expected = binaries[0] if binaries else None
            # Offline test workers and deliberately gateway-free deployments
            # have no project binary to manage. An occupied 9002 is still
            # rejected rather than silently sharing an unknown CAN listener.
            if expected is None:
                if self._listener_pid() is not None:
                    raise RuntimeError('9002 被占用但项目 ws_gateway 无法核验，拒绝启动实机任务')
                self.active = True
                self.present = False
                return {'present': False, 'stopped': False, 'restarted': False}
            listener_pid = self._listener_pid()
            pidfile_pid = self._pid_from_file()
            if listener_pid is not None and pidfile_pid and pidfile_pid != listener_pid:
                raise RuntimeError(f'9002 pid 文件 PID {pidfile_pid} 与监听进程 PID {listener_pid} 不一致，拒绝接管 CAN')
            pid = listener_pid or (pidfile_pid if pidfile_pid and self._alive(pidfile_pid) else None)
            if pid is None:
                self.active = True
                self.present = False
                return {'present': False, 'stopped': False, 'restarted': False}
            self.argv, self.cwd = self._read_process(pid, expected)
            self.pid = pid
            self.kill(pid, signal.SIGTERM)
            self.kill_sent = True
            deadline = self.clock() + 5.
            while self.clock() < deadline:
                listener = self._listener_pid()
                if listener is not None and listener != pid:
                    raise RuntimeError(f'9002 释放时出现未知监听进程 PID {listener}，拒绝实机任务')
                if not self._alive(pid) and listener is None:
                    break
                self.sleeper(.05)
            if self._alive(pid) or self._listener_pid() is not None:
                raise RuntimeError('9002 网关未在 5 秒内退出，未启动实机任务')
            self.stopped = True
        except Exception as acquire_error:
            restore_error = None
            if self.kill_sent and self.argv is not None:
                # If shutdown raced with our timeout, restore only when the
                # original process is gone and 9002 is confirmed free. Never
                # launch a second gateway beside a process that may still own
                # CAN or beside an unknown listener.
                try:
                    if not self._alive(self.pid) and self._listener_pid() is None:
                        self.stopped = True
                        self._restart({'restarted': False})
                except Exception as exc:
                    restore_error = str(exc)
                    self._cleanup_started_gateway()
            self.active = False
            self._unlock()
            self.pid = None
            self.argv = None
            self.cwd = self.root
            self.kill_sent = False
            self.stopped = False
            if restore_error:
                raise RuntimeError(f'{acquire_error}；原网关恢复失败：{restore_error}') from acquire_error
            raise
        self.active = True
        self.present = True
        return {'present': True, 'stopped': True, 'restarted': False, 'pid': pid}

    def _unlock(self):
        if self.lock_handle is not None:
            try:
                fcntl.flock(self.lock_handle, fcntl.LOCK_UN)
            finally:
                self.lock_handle.close()
                self.lock_handle = None

    def _cleanup_started_gateway(self):
        """Best-effort cleanup of a gateway process that failed to become ready."""
        process = self.proc
        if process is None:
            return
        try:
            if process.poll() is None:
                self.kill(process.pid, signal.SIGTERM)
                deadline = self.clock() + 2.
                while self.clock() < deadline and process.poll() is None:
                    self.sleeper(.05)
        except (OSError, AttributeError, TypeError):
            pass

    def _restart(self, result):
        if self._listener_pid() is not None:
            raise RuntimeError('9002 已被占用，拒绝启动第二个网关')
        self.logfile.parent.mkdir(parents=True, exist_ok=True)
        with self.logfile.open('ab') as log:
            self.proc = self.popen(
                self.argv, cwd=str(self.cwd), stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
        pid = getattr(self.proc, 'pid', None)
        if not isinstance(pid, int) or pid <= 0:
            raise RuntimeError('9002 网关重启未返回有效 PID')
        self.pidfile.write_text(f'{pid}\n')
        deadline = self.clock() + 5.
        while self.clock() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(f'9002 网关重启后退出（{self.proc.returncode}）')
            listener = self._listener_pid()
            if listener is not None:
                if listener != pid:
                    raise RuntimeError(f'9002 已被未知进程 PID {listener} 占用，未确认网关恢复')
                try:
                    socket.create_connection(('127.0.0.1', 9002), timeout=.2).close()
                except OSError:
                    self.sleeper(.05)
                    continue
                result['restarted'] = True
                return
            self.sleeper(.05)
        raise RuntimeError('实机任务结束后 9002 网关未恢复监听')

    def release(self):
        if not self.active:
            self._unlock()
            return {'present': False, 'stopped': False, 'restarted': False}
        if not self.present:
            self.active = False
            self._unlock()
            return {'present': False, 'stopped': False, 'restarted': False}
        result = {'present': True, 'stopped': True, 'restarted': False, 'pid': self.pid}
        if not self.stopped:
            self.active = False
            self._unlock()
            return {'present': True, 'stopped': False, 'restarted': False, 'pid': self.pid}
        try:
            self._restart(result)
            return result
        except Exception:
            self._cleanup_started_gateway()
            raise
        finally:
            self.active = False
            self.stopped = False
            self._unlock()
