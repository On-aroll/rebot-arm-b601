#!/usr/bin/env python3
"""RobStride 电机握手探测 —— 只 ping，不使能、不运动、不禁用。

为什么不直接用官方 example/0x01rs06_test.py
-------------------------------------------
那个示例是交互式的，而且：
  · finally 块里调用 ctrl.disable_all() —— 若机械臂正靠电机保持姿态，
    一旦禁用就可能塌下来；
  · 命令表里带 mit / posvel / vel 运动指令，误输入就会让机械臂动。
本脚本只做一件事：对指定 ID 发 ping，看谁应答。不改变任何电机状态。

⚠️ 会发送 CAN 帧（ping 是查询，不是运动指令），因此：
  1. 必须先退出只监听模式（只监听下无法发送）；
  2. 确认机械臂处于安全姿态、周围无人；
  3. 这是显式选择，不是默认动作。

用法:
    python3 scripts/rebot_can_ping.py                     # ping ID 1..7
    python3 scripts/rebot_can_ping.py --ids 1             # 只 ping ID 1
    python3 scripts/rebot_can_ping.py --model rs-00 --ids 4,5,6,7
    python3 scripts/rebot_can_ping.py --check-only        # 只检查环境，不发任何帧

输出: outputs/can_ping.json
"""
import argparse
import datetime
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# motorbridge 装在 rebot-sim 的 Python 3.10 venv 里，系统 python3 没有。
# 直接跑本脚本（或用 ~/.local/bin 里的软链）会拿到系统解释器 → ImportError。
# 与其让用户手动指定解释器，不如自己换过去重跑一次。
#
# ⚠️ 不能用 Path.home() 定位：sudo 会把 HOME 设成 /root，
# 于是变成 /root/.local/... 而找不到（2026-10-07 在别的脚本上踩到同一类坑）。
# 改为按候选列表探测。
_REBOT_PY_CANDIDATES = [
    Path('/home/bonvoyage/.local/share/robotics/envs/rebot-sim/bin/python'),
    Path.home() / '.local/share/robotics/envs/rebot-sim/bin/python',
]
REBOT_PY = next((p for p in _REBOT_PY_CANDIDATES if p.exists()), _REBOT_PY_CANDIDATES[0])
_REEXEC_FLAG = '_REBOT_PING_REEXEC'


def ensure_motorbridge():
    """若当前解释器没有 motorbridge，就用 rebot-sim 的解释器重启自己。

    返回 None 表示当前解释器可用；否则返回错误说明字符串。
    """
    try:
        import motorbridge  # noqa: F401
        return None
    except ImportError:
        pass

    if os.environ.get(_REEXEC_FLAG) == '1':
        return ('已在 %s 下重跑过，仍然找不到 motorbridge。\n'
                '     请确认该环境已安装：%s -c "import motorbridge"'
                % (REBOT_PY, REBOT_PY))

    if not REBOT_PY.exists():
        return ('系统 python3 没有 motorbridge，且找不到 rebot-sim 解释器：\n'
                '     %s\n'
                '     请用完整路径运行：\n'
                '     ~/.local/share/robotics/envs/rebot-sim/bin/python %s'
                % (REBOT_PY, Path(__file__).resolve()))

    print('[==] 当前解释器没有 motorbridge，切换到 rebot-sim 环境重跑：')
    print('     %s' % REBOT_PY)
    # os.execve 会直接替换进程映像，**不会 flush Python 的缓冲区**。
    # 输出被重定向时 stdout 是块缓冲，上面两句会就此丢失（2026-10-07 实测踩到）。
    sys.stdout.flush()
    sys.stderr.flush()

    env = dict(os.environ)
    env[_REEXEC_FLAG] = '1'
    os.execve(str(REBOT_PY), [str(REBOT_PY), str(Path(__file__).resolve())] + sys.argv[1:], env)
    return 'exec 失败'  # execve 成功则不会走到这里

# 官方 RS 参考配置：motor ID 1–7，rs-06 ×3 + rs-00 ×4。
# 注意这是参考配置，尚未与本台设备核对。
DEFAULT_IDS = '1,2,3,4,5,6,7'
DEFAULT_MODEL = 'rs-06'
DEFAULT_HOST_ID = 0xFD


def sh(cmd, timeout=20):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout, r.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return -1, '', str(e)


def interface_state(name):
    rc, out, _ = sh(['ip', '-details', '-json', 'link', 'show', 'dev', name])
    if rc != 0:
        return None, '接口 %s 不存在' % name
    try:
        info = json.loads(out)[0]
    except (json.JSONDecodeError, IndexError):
        return None, '无法解析接口信息'
    data = info.get('linkinfo', {}).get('info_data', {})
    if 'UP' not in info.get('flags', []):
        return info, ('接口 %s 处于 DOWN，无法发送。请先启动（需要终端密码）：\n'
                      '      sudo ip link set %s down\n'
                      '      sudo ip link set %s type can bitrate 1000000 listen-only off\n'
                      '      sudo ip link set %s up\n'
                      '  注意：只监听模式同样发不出去，所以这里不带 listen-only on。'
                      % (name, name, name, name))
    modes = data.get('ctrlmode', []) or []
    if 'LISTEN-ONLY' in modes:
        # ⚠️ 必须显式写 listen-only off。
        # `ip link set can0 type can bitrate 1000000` 只改波特率，
        # **不会清除 LISTEN-ONLY** —— ctrlmode 只有显式指定才会变。
        # 2026-10-07 实测：只改波特率后 ctrlmode 仍是 ['LISTEN-ONLY']，发送被拒。
        return info, ('接口处于 LISTEN-ONLY，无法发送。请先切回普通模式：\n'
                      '      sudo ip link set %s down\n'
                      '      sudo ip link set %s type can bitrate 1000000 listen-only off\n'
                      '      sudo ip link set %s up\n'
                      '  注意：必须显式加 `listen-only off`。只写 bitrate 不会清除该标志，\n'
                      '  且改 ctrlmode 前接口必须是 down 状态。' % (name, name, name))
    if data.get('state') in ('STOPPED', 'BUS-OFF'):
        return None, '接口状态为 %s，无法发送' % data.get('state')
    return info, None


def bus_counters(name):
    """取 CAN 控制器的收发与错误计数。用于判断"帧有没有被应答"。

    CAN 协议要求：发送方在 ACK 位期待至少一个接收方拉低电平。若总线上没有
    任何节点应答，发送方会报 ACK 错误，TEC 计数上涨并自动重发，最终进
    error-passive / bus-off。**所以"发了帧但错误计数为零"等价于"被应答了"**，
    这与"收到响应"是两件事：应答发生在数据链路层，与帧 ID 是否匹配无关。
    """
    rc, out, _ = sh(['ip', '-details', '-statistics', '-json', 'link', 'show', 'dev', name])
    if rc != 0:
        return None
    try:
        info = json.loads(out)[0]
    except (json.JSONDecodeError, IndexError):
        return None
    dd = info.get('linkinfo', {}).get('info_data', {})
    st = info.get('stats64', {})
    berr = dd.get('berr_counter', {}) or {}
    xst = info.get('linkinfo', {}).get('info_xstats', {}) or {}
    return {
        'state': dd.get('state'),
        'tx_packets': (st.get('tx') or {}).get('packets'),
        'tx_errors': (st.get('tx') or {}).get('errors'),
        'rx_packets': (st.get('rx') or {}).get('packets'),
        'rx_dropped': (st.get('rx') or {}).get('dropped'),
        'berr_tx': berr.get('tx'),
        'berr_rx': berr.get('rx'),
        'bus_error': xst.get('bus_error'),
        'bus_off': xst.get('bus_off'),
        'error_passive': xst.get('error_passive'),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interface', default='can0')
    ap.add_argument('--ids', default=DEFAULT_IDS, help='逗号分隔的电机 ID，默认 1,2,3,4,5,6,7')
    ap.add_argument('--model', default=DEFAULT_MODEL, help='电机型号，默认 rs-06')
    ap.add_argument('--host-id', type=lambda v: int(v, 0), default=DEFAULT_HOST_ID,
                    help='RobStride host/feedback ID，默认 0xFD')
    ap.add_argument('--check-only', action='store_true',
                    help='只检查接口是否具备发送条件，不发送任何帧')
    args = ap.parse_args()

    ids = []
    for tok in args.ids.split(','):
        tok = tok.strip()
        if tok:
            ids.append(int(tok, 0))

    report = {
        'time_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'interface': args.interface,
        'model': args.model,
        'host_id': args.host_id,
        'target_ids': ids,
        'transmits': not args.check_only,
        'enables_motors': False,
        'sends_motion': False,
        'disables_motors': False,
    }

    info, problem = interface_state(args.interface)
    if problem:
        # --check-only 是诊断用途：查出"不能发送"本身就是成功的检查结果，不算失败。
        if args.check_only:
            report['result'] = 'check_only_not_ready'
            report['error'] = problem
            print('[==] --check-only 检查结果：当前不具备发送条件。')
            print('     ' + problem)
            _save(report)
            return 0
        report['result'] = 'blocked'
        report['error'] = problem
        print('[XX] ' + problem)
        _save(report)
        return 1

    data = info.get('linkinfo', {}).get('info_data', {})
    bitrate = data.get('bittiming', {}).get('bitrate')
    report['bitrate'] = bitrate
    print('接口 %s 已就绪：%s  bitrate=%s  ctrlmode=%s'
          % (args.interface, data.get('state'), bitrate, data.get('ctrlmode')))
    if bitrate and bitrate != 1000000:
        print('[!!] 波特率为 %s，官方 RS 参考配置为 1000000。请确认与本台电机一致。' % bitrate)

    if args.check_only:
        report['result'] = 'check_only_ok'
        print('[==] --check-only：具备发送条件，但本次未发送任何帧。')
        _save(report)
        return 0

    problem = ensure_motorbridge()   # 必要时换解释器重启，成功则不会返回
    if problem:
        report['result'] = 'blocked'
        report['error'] = problem
        print('[XX] ' + problem)
        _save(report)
        return 1
    from motorbridge import Controller

    print('目标电机 ID: %s   型号: %s   host_id: 0x%02X'
          % (ids, args.model, args.host_id))
    print('[!!] 即将发送 ping 帧（仅查询，不使能、不运动）\n')

    before = bus_counters(args.interface)
    results = []
    ctrl = None
    try:
        ctrl = Controller(args.interface)
        for mid in ids:
            entry = {'motor_id': mid}
            try:
                motor = ctrl.add_robstride_motor(mid, args.host_id, args.model)
                device_id, responder_id = motor.robstride_ping()
                entry.update(responded=True, device_id=int(device_id),
                             responder_id=int(responder_id))
                print('  ID %-3d 应答  device_id=%s  responder_id=0x%02X'
                      % (mid, device_id, responder_id))
            except Exception as e:
                entry.update(responded=False, error='%s: %s' % (type(e).__name__, e))
                print('  ID %-3d 无应答  (%s)' % (mid, e))
            results.append(entry)
    except Exception as e:
        report['result'] = 'blocked'
        report['error'] = '%s: %s' % (type(e).__name__, e)
        print('[XX] 打开 CAN 接口失败: %s' % e)
        _save(report)
        return 1
    finally:
        # 只关闭句柄。**刻意不调用 disable_all()** —— 那会改变电机状态。
        if ctrl is not None:
            for fn in ('close', 'shutdown'):
                try:
                    getattr(ctrl, fn)()
                except Exception:
                    pass

    after = bus_counters(args.interface)
    report['bus_before'] = before
    report['bus_after'] = after
    if before and after:
        # 只对数值字段求差。state 是字符串（如 'ERROR-ACTIVE'），不能相减。
        numeric = ('tx_packets', 'tx_errors', 'rx_packets', 'rx_dropped',
                   'berr_tx', 'berr_rx', 'bus_error', 'bus_off', 'error_passive')
        delta = {k: (after.get(k) or 0) - (before.get(k) or 0) for k in numeric}
        report['bus_delta'] = delta
        # 关键判据：发出去了帧，却没有任何总线错误 → 帧被应答 → 总线上有活节点。
        sent = delta.get('tx_packets') or 0
        bus_errors = sum(delta.get(k) or 0 for k in
                         ('bus_error', 'bus_off', 'error_passive', 'berr_tx'))
        report['frames_sent'] = sent
        report['acked'] = bool(sent > 0 and bus_errors == 0)
        print('\n--- 总线计数变化 ---')
        print('  发送帧数 %s   接收帧数 %s   接收丢弃 %s' % (
            delta.get('tx_packets'), delta.get('rx_packets'), delta.get('rx_dropped')))
        print('  TEC %s → %s   bus_error %s → %s' % (
            before.get('berr_tx'), after.get('berr_tx'),
            before.get('bus_error'), after.get('bus_error')))
        if sent == 0:
            print('  本次未发送任何帧，无法据此判断总线状况。')
        elif bus_errors == 0:
            print('  [!!] 发出 %d 帧且零总线错误 → 帧**被应答**，总线上有活节点。' % sent)
            print('       应答发生在数据链路层，与帧 ID 是否匹配无关：')
            print('       即便 ID 不对，对方也会 ACK。所以电气层是通的。')
        else:
            print('  [!!] 出现总线错误 → 发送未被应答，总线上可能没有节点接收。')
            print('       重点查：机械臂是否上电、CAN_H/CAN_L、终端电阻、波特率。')

    report['results'] = results
    answered = [r for r in results if r.get('responded')]
    report['responded_count'] = len(answered)
    report['responded_ids'] = [r['motor_id'] for r in answered]

    if answered:
        report['result'] = 'handshake_ok'
        print('\n[==] 握手成功：%d/%d 个 ID 应答 —— %s'
              % (len(answered), len(results), report['responded_ids']))
        print('     下一步：逐个核对电机型号与轴映射，再谈零位与正方向。')
    else:
        report['result'] = 'no_response'
        print('\n[==] 全部无应答。')
        if report.get('frames_sent') == 0:
            print('     （本次未发送任何帧，无结论。）')
        print('     本脚本只试了 host_id=0x%02X 一个主机 ID。' % args.host_id)
        print('     ⚠️ 官方 scan 会依次尝试 host_id 0xFD / 0xFF / 0xFE / 0x00 / 0xAA，')
        print('        本脚本没有这个覆盖 —— 若电机期望的不是 0x%02X，ping 永远不会被响应。'
              % args.host_id)
        print('     → 建议改用官方 CLI（同样只查询，不使能、不运动）：')
        print('         rebot-can-scan')
        print('       或手动：')
        print('         ~/.local/share/robotics/envs/rebot-sim/bin/motorbridge-cli scan \\')
        print('           --vendor robstride --channel %s --start-id 1 --end-id 16 \\' % args.interface)
        print('           --feedback-ids 0xFD,0xFF,0xFE,0x00,0xAA')
        print('     其它可能：电机 ID 不在所试范围；电机未使能或处于不响应查询的状态。')

    _save(report)
    return 0


def _save(report):
    out = ROOT / 'outputs/can_ping.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print('\n报告已写入: %s' % out)


if __name__ == '__main__':
    sys.exit(main())
