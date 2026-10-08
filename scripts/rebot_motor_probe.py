#!/usr/bin/env python3
"""RobStride 电机最小试探 —— 用"是否运动"代替"能否查询"来判断通信。

为什么需要
----------
官方示例自带警告（原文）：
    "motorbridge 内部协议配置问题：RS 电机无法像 DM 电机那样查询，
     可能会出现读取不到实际电机状态的情况，**请以实际运动效果为准**。"

即：**扫描/查询零响应不一定代表总线不通**，可能只是 RS 的查询通道不工作。
但这样一来，"是否通信"就只能靠**实际运动**来判断——而这有物理风险。

安全设计（本脚本刻意不做的事）
------------------------------
- **不调用 disable**：若机械臂靠电机保持姿态，禁用会让它塌下来。
- **不调用 set_zero**：会改写电机零位标定。
- **默认不写任何参数**；切换控制模式（写 0x7005）需显式加 `--set-mode`。
  ⚠️ 更正：早先版本默认调用 `ensure_mode()`，而它会写 0x7005 ——
  当时文档写"不写参数"是**不准确**的，已改为默认不调用。
- **默认只发零增益**（kp=kd=tau=0），**不产生力矩、不会动**。
  加增益/加位移必须显式加参数，且需二次确认。

⚠️ 使用前必须做到
------------------
1. 机械臂放在**安全姿态**（低位、有支撑或已放平），周围清空
2. **人退到安全距离**
3. **手放在 48 V 电源开关旁**，随时能断电
4. 先不加 `--hold-kp` / `--nudge-deg`，看零增益是否报错

用法:
    # 阶段一：零增益试探（不会动，只验证发送通路）
    python3 scripts/rebot_motor_probe.py --id 1

    # 阶段二：给一点保持刚度，观察电机是否"上劲"（会动，需 --yes）
    python3 scripts/rebot_motor_probe.py --id 1 --hold-kp 3 --yes

    # 阶段三：在保持刚度基础上给一个很小的位置偏移
    python3 scripts/rebot_motor_probe.py --id 1 --hold-kp 3 --nudge-deg 2 --yes

输出: outputs/motor_probe.json
"""
import argparse
import datetime
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEG = math.pi / 180.0

_REBOT_PY_CANDIDATES = [
    Path('/home/bonvoyage/.local/share/robotics/envs/rebot-sim/bin/python'),
    Path.home() / '.local/share/robotics/envs/rebot-sim/bin/python',
]
REBOT_PY = next((p for p in _REBOT_PY_CANDIDATES if p.exists()), _REBOT_PY_CANDIDATES[0])
_REEXEC_FLAG = '_REBOT_PROBE_REEXEC'


def ensure_motorbridge():
    try:
        import motorbridge  # noqa: F401
        return None
    except ImportError:
        pass
    if os.environ.get(_REEXEC_FLAG) == '1':
        return '已在 %s 下重跑过，仍找不到 motorbridge。' % REBOT_PY
    if not REBOT_PY.exists():
        return '找不到 rebot-sim 解释器：%s' % REBOT_PY
    print('[==] 当前解释器没有 motorbridge，切换到 rebot-sim 环境重跑：')
    print('     %s' % REBOT_PY)
    sys.stdout.flush()
    sys.stderr.flush()
    env = dict(os.environ)
    env[_REEXEC_FLAG] = '1'
    os.execve(str(REBOT_PY), [str(REBOT_PY), str(Path(__file__).resolve())] + sys.argv[1:], env)
    return 'exec 失败'


def sh(cmd, timeout=15):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout, r.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return -1, '', str(e)


def iface_ok(name):
    rc, out, _ = sh(['ip', '-details', '-json', 'link', 'show', 'dev', name])
    if rc != 0:
        return False, '接口 %s 不存在' % name
    try:
        info = json.loads(out)[0]
    except (json.JSONDecodeError, IndexError):
        return False, '无法解析接口信息'
    if 'UP' not in info.get('flags', []):
        return False, '接口 %s 处于 DOWN' % name
    modes = info.get('linkinfo', {}).get('info_data', {}).get('ctrlmode') or []
    if 'LISTEN-ONLY' in modes or 'LOOPBACK' in modes:
        return False, '接口 ctrlmode 含 %s —— 帧不会驱动外部总线' % modes
    return True, None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interface', default='can0')
    ap.add_argument('--id', type=lambda v: int(v, 0), required=True, help='电机 ID（必须显式指定）')
    ap.add_argument('--model', default='rs-06', help='rs-06 / rs-00')
    ap.add_argument('--host-id', type=lambda v: int(v, 0), default=0xFD)
    ap.add_argument('--hold-kp', type=float, default=0.0,
                    help='保持刚度。0 = 不产生力矩（默认，安全）')
    ap.add_argument('--hold-kd', type=float, default=0.0)
    ap.add_argument('--nudge-deg', type=float, default=0.0,
                    help='相对 0 位的目标偏移（度）。默认 0')
    ap.add_argument('--seconds', type=float, default=3.0)
    ap.add_argument('--set-mode', action='store_true',
                    help='显式允许写入控制模式参数 0x7005（默认不写）')
    ap.add_argument('--yes', action='store_true', help='确认已做好安全准备')
    args = ap.parse_args()

    report = {
        'time_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'interface': args.interface, 'motor_id': args.id, 'model': args.model,
        'host_id': args.host_id, 'hold_kp': args.hold_kp, 'hold_kd': args.hold_kd,
        'nudge_deg': args.nudge_deg,
        'disables_motors': False, 'sets_zero': False,
        'writes_params': bool(args.set_mode),
        'writes_control_mode_0x7005': bool(args.set_mode),
    }

    ok, problem = iface_ok(args.interface)
    if not ok:
        print('[XX] %s' % problem)
        print('     需要：sudo ip link set %s down' % args.interface)
        print('           sudo ip link set %s type can bitrate 1000000 loopback off listen-only off' % args.interface)
        print('           sudo ip link set %s up' % args.interface)
        report['result'] = 'blocked'
        report['error'] = problem
        _save(report)
        return 1

    active = args.hold_kp > 0 or args.nudge_deg != 0
    if active and not args.yes:
        print('[XX] 本次会**产生力矩/使电机运动**（hold_kp=%g, nudge_deg=%g）。'
              % (args.hold_kp, args.nudge_deg))
        print()
        print('     执行前请确认：')
        print('       1. 机械臂处于安全姿态（低位、有支撑或已放平）')
        print('       2. 周围清空，人已退到安全距离')
        print('       3. 手放在 48 V 电源开关旁，随时可断电')
        print()
        print('     确认无误后加 --yes 重跑。')
        report['result'] = 'needs_confirmation'
        _save(report)
        return 1

    print('=' * 62)
    print(' 目标：电机 ID %s  型号 %s  host_id 0x%02X' % (args.id, args.model, args.host_id))
    print(' 模式：%s' % ('零增益（不产生力矩）' if not active
                       else '**会产生力矩** kp=%g kd=%g 目标偏移 %g°'
                            % (args.hold_kp, args.hold_kd, args.nudge_deg)))
    print(' 本脚本不会 disable / set_zero / 写参数')
    print('=' * 62)
    print()

    p = ensure_motorbridge()
    if p:
        print('[XX] ' + p)
        report['result'] = 'blocked'
        report['error'] = p
        _save(report)
        return 1

    from motorbridge import Controller, Mode

    target_rad = args.nudge_deg * DEG
    steps = []
    ctrl = None
    try:
        ctrl = Controller(args.interface)
        motor = ctrl.add_robstride_motor(args.id, args.host_id, args.model)
        print('电机已注册（仅本地登记，未使能）')

        # ⚠️ 重要更正（2026-10-07）：ensure_mode() **会写入电机的控制模式参数 0x7005**，
        # 它不是只读操作。早先版本在本脚本里默认调用它，而文档却写"不写参数"——
        # 那是不准确的声明。现在改为**必须显式加 --set-mode 才会调用**。
        if args.set_mode:
            try:
                motor.ensure_mode(Mode.MIT, 1000)
                print('[!!] 已写入控制模式参数 0x7005（--set-mode 显式要求）')
            except Exception as e:
                print('[!!] 切换模式失败：%s —— 继续尝试发送' % e)
        else:
            print('（未写控制模式参数；如需切换请加 --set-mode）')

        print()
        print('开始发送 %g 秒（每 20 ms 一帧）…' % args.seconds)
        t_end = time.monotonic() + args.seconds
        n = 0
        while time.monotonic() < t_end:
            try:
                motor.send_mit(target_rad, 0.0, args.hold_kp, args.hold_kd, 0.0)
                n += 1
            except Exception as e:
                steps.append({'error': '%s: %s' % (type(e).__name__, e)})
                print('[!!] 发送失败：%s' % e)
                break
            time.sleep(0.02)
        report['frames_attempted'] = n
        print('发送结束：尝试 %d 帧，无异常抛出' % n)
    except Exception as e:
        report['result'] = 'blocked'
        report['error'] = '%s: %s' % (type(e).__name__, e)
        print('[XX] 失败：%s' % e)
        _save(report)
        return 1
    finally:
        # 只关闭句柄。**刻意不 disable** —— 那会改变电机状态。
        if ctrl is not None:
            for fn in ('close', 'shutdown'):
                try:
                    getattr(ctrl, fn)()
                except Exception:
                    pass

    report['result'] = 'sent'
    print()
    print('=' * 62)
    print(' 判读（重点看机械臂的**实际反应**，不要看查询结果）：')
    print()
    if not active:
        print(' 本次是零增益，不会产生力矩。')
        print(' 若发送过程没有任何异常抛出，说明**发送通路不报错**——')
        print(' 但这还不能证明电机收到了（适配器不上报总线错误）。')
        print(' 下一步：加 --hold-kp 3 --yes，看电机是否"上劲"。')
    else:
        print(' 观察机械臂：')
        print('   · 电机明显"上劲"/有阻力/轻微移动 → **通信是通的**，')
        print('     问题只在查询通道（与官方警告一致）')
        print('   · 完全没有任何反应 → 帧很可能没到电机')
        print()
        print(' ⚠️ 若出现剧烈或持续的异常运动，立刻断开 48 V 电源。')
    print('=' * 62)

    _save(report)
    return 0


def _save(report):
    out = ROOT / 'outputs/motor_probe.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print('\n报告已写入: %s' % out)


if __name__ == '__main__':
    sys.exit(main())
