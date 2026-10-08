#!/usr/bin/env python3
"""CAN 适配器自环自测 —— 判断适配器自身的收发通道是否可用。

为什么需要它
------------
本机的 USB-CAN 适配器是克隆品（VID/PID 0c72:000c 冒用 PEAK PCAN-USB，
实际设备串 XCAN-USB），且**不上报总线错误**——拔掉总线后发帧，
TEC 与 bus_error 仍为零（2026-10-07 对照实验证实）。
因此"发了帧但没有任何响应"这一现象，无法区分下面两种可能：

  (a) 帧确实送上总线了，只是没有电机应答；
  (b) 帧根本没出去，适配器的发送通道是坏的。

自环模式（controller loopback）把发送路径内部接回接收路径，
**不需要总线上有任何其他节点**，因此能把 (b) 单独测出来。

⚠️ 会发送 CAN 帧。**请把 CAN 线从机械臂上拔掉再做**，
这样总线上没有其他节点，自测帧不可能影响电机。

用法:
    python3 scripts/rebot_can_loopback.py            # 先检查环境
    python3 scripts/rebot_can_loopback.py --run      # 实际执行自环测试

输出: outputs/can_loopback.json
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
TEST_ID = 0x7FF          # 标准帧里最高优先级最低的 ID，避免与常见电机 ID 冲突
TEST_DATA = bytes([0xDE, 0xAD, 0xBE, 0xEF])


def sh(cmd, timeout=20):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout, r.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return -1, '', str(e)


def iface_info(name):
    rc, out, _ = sh(['ip', '-details', '-json', 'link', 'show', 'dev', name])
    if rc != 0:
        return None
    try:
        return json.loads(out)[0]
    except (json.JSONDecodeError, IndexError):
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interface', default='can0')
    ap.add_argument('--run', action='store_true', help='实际执行自环测试（会发送帧）')
    args = ap.parse_args()

    report = {
        'time_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'interface': args.interface,
        'test_id': '0x%X' % TEST_ID,
        'transmits': bool(args.run),
    }

    info = iface_info(args.interface)
    if info is None:
        print('[XX] 接口 %s 不存在。适配器插好了吗？' % args.interface)
        return 1
    data = info.get('linkinfo', {}).get('info_data', {})
    modes = data.get('ctrlmode', []) or []
    up = 'UP' in info.get('flags', [])
    report['state'] = data.get('state')
    report['ctrlmode'] = modes
    report['up'] = up
    report['bitrate'] = data.get('bittiming', {}).get('bitrate')

    print('接口 %s：operstate=%s  state=%s  ctrlmode=%s  bitrate=%s'
          % (args.interface, info.get('operstate'), data.get('state'),
             modes or '(无)', data.get('bittiming', {}).get('bitrate')))

    if 'LOOPBACK' not in modes:
        cat = '''[XX] 接口未处于自环（LOOPBACK）模式，现在无法自测。

请先切到自环模式（需要终端密码）：

      sudo ip link set %s down
      sudo ip link set %s type can bitrate 1000000 loopback on
      sudo ip link set %s up
      python3 scripts/rebot_can_loopback.py --run

测完恢复成只监听（观察用）：

      sudo ip link set %s down
      sudo ip link set %s type can bitrate 1000000 listen-only on
      sudo ip link set %s up

⚠️ 做自环测试时请把 CAN 线从机械臂上拔掉。''' % ((args.interface,) * 6)
        print(cat)
        report['result'] = 'not_in_loopback'
        _save(report)
        return 0 if not args.run else 1

    if not up:
        print('[XX] 接口处于 DOWN，无法发送。')
        report['result'] = 'down'
        _save(report)
        return 1

    # ---- 实际自测 ---------------------------------------------------------
    print('\n发送测试帧 id=%s data=%s，等待回环…' % ('0x%X' % TEST_ID, TEST_DATA.hex()))
    got = None
    try:
        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as s:
            s.bind((args.interface,))
            s.settimeout(2.0)
            frame = struct.pack('=IB3x8s', TEST_ID, len(TEST_DATA),
                                TEST_DATA + bytes(8 - len(TEST_DATA)))
            s.send(frame)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                try:
                    raw = s.recv(16)
                except socket.timeout:
                    break
                if len(raw) != 16:
                    continue
                rid, length, payload = struct.unpack('=IB3x8s', raw)
                got = {'id': '0x%X' % (rid & 0x7FF), 'dlc': length,
                       'data': payload[:min(length, 8)].hex()}
                break
    except OSError as e:
        report['result'] = 'send_failed'
        report['error'] = '%s: %s' % (type(e).__name__, e)
        print('[XX] 发送失败：%s' % e)
        _save(report)
        return 1

    report['loopback_frame'] = got
    if got:
        report['result'] = 'loopback_ok'
        print('[==] 收到回环帧：id=%s dlc=%s data=%s' % (got['id'], got['dlc'], got['data']))
        print('     → 适配器自身的发送/接收通道可用。')
        print('       注意：自环通常在控制器内部完成，**不经过外部收发器与线缆**，')
        print('       所以这不能证明"帧能送上总线"，只能排除适配器内部通道故障。')
    else:
        report['result'] = 'loopback_failed'
        print('[XX] 未收到回环帧。')
        print('     → 适配器自身的收发通道存在问题，与机械臂无关。')
        print('       这解释了为什么扫描始终零响应。')

    _save(report)
    return 0


def _save(report):
    out = ROOT / 'outputs/can_loopback.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print('\n报告已写入: %s' % out)


if __name__ == '__main__':
    sys.exit(main())
