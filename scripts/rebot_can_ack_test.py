#!/usr/bin/env python3
"""用 ENOBUFS 判断总线上有没有节点应答 —— 本适配器唯一可用的判据。

为什么需要它
------------
本机的 UTC-101 是克隆适配器，**不上报 CAN 总线错误**：
拔掉总线后发帧，`berr_counter` 与 `bus_error` 仍为 0（2026-10-07 对照实验证实）。
所以"错误计数器"这条路走不通。

但 **SocketCAN 的写路径会如实报错**：

  - 帧被对端 ACK → 传输很快完成 → 发送队列排空 → 连续发送不会失败
  - 帧**没有被 ACK** → 控制器不断重传 → 发送队列填满 → `ENOBUFS (errno 105)`

**实测（2026-10-07）**：1 Hz 慢发 5/5 成功；500 Hz 快发**第 16 帧**即报 ENOBUFS
（0.03 秒内）。500 Hz 远低于全速 USB 的 CAN 带宽上限，因此这个失败
**不是速率问题，是没有 ACK**。

判据
----
| 结果 | 含义 |
| --- | --- |
| 连续 N 帧全部成功 | 帧被应答 → **总线上有活着的节点** |
| 很快出现 ENOBUFS（几十帧内） | 帧未被应答 → **总线上没有节点在接收** |

用法:
    # 需要接口处于普通模式（非 listen-only / 非 loopback）
    python3 scripts/rebot_can_ack_test.py
    python3 scripts/rebot_can_ack_test.py --count 200 --interval-ms 2

    # 对照实验：把 CAN 线从机械臂拔掉再跑一次，比较两者的失败位置
    #   两者相同  → 说明机械臂本来就没在应答
    #   明显不同  → 说明机械臂确实在应答（那问题在协议层）

⚠️ 会向总线发送保留 ID（0x7FE）的帧，电机不会识别，但请确保机械臂处于安全状态。

输出: outputs/can_ack_test.json
"""
import argparse
import datetime
import json
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROBE_ID = 0x7FE


def sh(cmd, timeout=15):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout, r.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return -1, '', str(e)


def check_iface(name):
    rc, out, _ = sh(['ip', '-details', '-json', 'link', 'show', 'dev', name])
    if rc != 0:
        return None, '接口 %s 不存在' % name
    try:
        info = json.loads(out)[0]
    except (json.JSONDecodeError, IndexError):
        return None, '无法解析接口信息'
    if 'UP' not in info.get('flags', []):
        return None, '接口 %s 处于 DOWN' % name
    modes = info.get('linkinfo', {}).get('info_data', {}).get('ctrlmode') or []
    if 'LISTEN-ONLY' in modes or 'LOOPBACK' in modes:
        return None, ('接口 ctrlmode 含 %s —— 帧不会驱动外部总线，测了没意义。\n'
                      '     请先切普通模式：\n'
                      '       sudo ip link set %s down\n'
                      '       sudo ip link set %s type can bitrate 1000000 loopback off listen-only off\n'
                      '       sudo ip link set %s up' % (modes, name, name, name))
    return info, None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interface', default='can0')
    ap.add_argument('--count', type=int, default=200, help='最多尝试发送多少帧')
    ap.add_argument('--interval-ms', type=float, default=2.0, help='帧间隔（毫秒）')
    ap.add_argument('--label', default='', help='本次测量的标签，写进报告便于对比')
    args = ap.parse_args()

    report = {
        'time_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'interface': args.interface,
        'probe_id': '0x%X' % PROBE_ID,
        'count_target': args.count,
        'interval_ms': args.interval_ms,
        'label': args.label,
    }

    info, problem = check_iface(args.interface)
    if problem:
        print('[XX] ' + problem)
        report['result'] = 'blocked'
        report['error'] = problem
        _save(report)
        return 1

    dd = info.get('linkinfo', {}).get('info_data', {})
    print('接口 %s：state=%s  ctrlmode=%s  bitrate=%s'
          % (args.interface, dd.get('state'), dd.get('ctrlmode') or '(无)',
             dd.get('bittiming', {}).get('bitrate')))
    if args.label:
        print('标签：%s' % args.label)
    print()
    print('连续发送最多 %d 帧，间隔 %g ms …' % (args.count, args.interval_ms))
    print('（帧被 ACK 则队列排空、可持续发送；未被 ACK 则很快报 ENOBUFS）')
    print()

    frame = struct.pack('=IB3x8s', PROBE_ID, 8, bytes([0x5A] * 8))
    sent = 0
    err = None
    t0 = time.monotonic()
    try:
        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as s:
            s.bind((args.interface,))
            for i in range(args.count):
                try:
                    s.send(frame)
                except OSError as e:
                    err = {'errno': e.errno, 'message': str(e), 'at_frame': i + 1}
                    break
                sent += 1
                if args.interval_ms > 0:
                    time.sleep(args.interval_ms / 1000.0)
    except OSError as e:
        print('[XX] 打开接口失败：%s' % e)
        report['result'] = 'blocked'
        report['error'] = str(e)
        _save(report)
        return 1

    elapsed = time.monotonic() - t0
    report.update(frames_sent=sent, elapsed_s=round(elapsed, 4),
                  rate_hz=round(sent / elapsed, 1) if elapsed > 0 else None,
                  error=err)

    print('=' * 62)
    if err:
        print(' 结果：发送 %d 帧后出现 ENOBUFS' % sent)
        print('       errno=%s  %s' % (err['errno'], err['message']))
        print('       耗时 %.3f 秒，约 %.0f 帧/秒' % (elapsed, sent / elapsed if elapsed else 0))
        print()
        if sent <= 40:
            print(' 判读：**很快失败 ⇒ 帧没有被应答**')
            print('       总线上很可能没有节点在接收。这不是速率问题 ——')
            print('       全速 USB 的 CAN 适配器远不止这个吞吐。')
            report['verdict'] = 'no_ack_suspected'
        else:
            print(' 判读：发送了 %d 帧才失败，处在中间地带。' % sent)
            print('       建议降低 --interval-ms 再试一次，或与"拔掉总线"的对照结果比较。')
            report['verdict'] = 'inconclusive'
    else:
        print(' 结果：连续 %d 帧全部发送成功，无 ENOBUFS' % sent)
        print('       耗时 %.3f 秒，约 %.0f 帧/秒' % (elapsed, sent / elapsed if elapsed else 0))
        print()
        print(' 判读：**帧被应答了** ⇒ 总线上有活着的节点。')
        print('       那问题就不在物理层，而在电机 ID / 协议 / 电机状态。')
        report['verdict'] = 'ack_ok'

    print()
    print(' 对照实验建议：把 CAN 线从机械臂拔掉，用同样的参数再跑一次，')
    print(' 比较两者的失败位置 —— 相同则说明机械臂本来就没在应答。')
    print('=' * 62)

    _save(report)
    return 0


def _save(report):
    out = ROOT / 'outputs/can_ack_test.json'
    out.parent.mkdir(exist_ok=True)
    # 保留历史，便于对照（追加到列表）
    hist = []
    if out.exists():
        try:
            old = json.loads(out.read_text())
            hist = old if isinstance(old, list) else [old]
        except (json.JSONDecodeError, OSError):
            hist = []
    hist.append(report)
    out.write_text(json.dumps(hist, ensure_ascii=False, indent=2))
    print('\n报告已写入（累计 %d 次）: %s' % (len(hist), out))


if __name__ == '__main__':
    sys.exit(main())
