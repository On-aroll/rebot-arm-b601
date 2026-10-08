#!/usr/bin/env python3
"""持续发送 CAN 帧，供万用表观察总线电压是否被驱动。

要解决的问题
------------
自环测试通过只证明适配器**内部**通道可用（自环在控制器内部完成，
不经过外部收发器与线缆）。所以下面两种可能始终无法区分：

  (a) 帧确实送上了总线，只是没有电机应答；
  (b) 适配器的**外部收发器没有驱动总线**（或线没真正连上）。

本工具用"发送期间量电压"来区分：持续发送时总线大量处于**显性**电平，

| 状态 | CAN_H 对地 | CAN_L 对地 |
| --- | --- | --- |
| 空闲（隐性） | 约 2.5 V | 约 2.5 V |
| 持续发送 | **升高**（趋向 3.5 V） | **降低**（趋向 1.5 V） |

万用表读的是平均值，发送时能看到明显的偏移。**读数纹丝不动 ⇒ (b)。**

用法:
    # 1) 切到普通模式（需要终端密码）
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000 loopback off listen-only off
    sudo ip link set can0 up

    # 2) 启动持续发送，然后在发送期间用万用表量 CAN_H / CAN_L 对地
    python3 scripts/rebot_can_tx_probe.py --seconds 30

    # 3) 测完恢复只监听
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000 listen-only on
    sudo ip link set can0 up

⚠️ 会持续向总线发送帧。这些帧使用保留 ID，电机不会识别，
   但**请确保机械臂处于安全状态**再做。
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
PROBE_ID = 0x7FE          # 保留 ID，避免与常见电机 ID 冲突


def sh(cmd, timeout=15):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout, r.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return -1, '', str(e)


def iface(name):
    rc, out, _ = sh(['ip', '-details', '-statistics', '-json', 'link', 'show', 'dev', name])
    if rc != 0:
        return None
    try:
        return json.loads(out)[0]
    except (json.JSONDecodeError, IndexError):
        return None


def snapshot(info):
    if not info:
        return {}
    dd = info.get('linkinfo', {}).get('info_data', {})
    st = info.get('stats64', {})
    return {
        'state': dd.get('state'),
        'ctrlmode': dd.get('ctrlmode'),
        'bitrate': dd.get('bittiming', {}).get('bitrate'),
        'tx_packets': (st.get('tx') or {}).get('packets'),
        'rx_packets': (st.get('rx') or {}).get('packets'),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interface', default='can0')
    ap.add_argument('--seconds', type=float, default=30.0, help='持续发送时长（默认 30 秒）')
    args = ap.parse_args()

    report = {
        'time_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'interface': args.interface,
        'probe_id': '0x%X' % PROBE_ID,
        'seconds': args.seconds,
    }

    info = iface(args.interface)
    if info is None:
        print('[XX] 接口 %s 不存在。' % args.interface)
        return 1
    before = snapshot(info)
    report['before'] = before
    print('接口 %s：state=%s  ctrlmode=%s  bitrate=%s'
          % (args.interface, before.get('state'),
             before.get('ctrlmode') or '(无)', before.get('bitrate')))

    modes = before.get('ctrlmode') or []
    if 'LOOPBACK' in modes or 'LISTEN-ONLY' in modes:
        print('[XX] 当前 ctrlmode 含 %s —— 帧不会驱动外部总线，测了也没意义。' % modes)
        print('     请先切到普通模式：')
        print('       sudo ip link set %s down' % args.interface)
        print('       sudo ip link set %s type can bitrate 1000000 loopback off listen-only off' % args.interface)
        print('       sudo ip link set %s up' % args.interface)
        return 1

    print()
    print('=' * 62)
    print('  现在开始持续发送 %g 秒。请立刻用万用表量：' % args.seconds)
    print('     CAN_H ↔ GND   与   CAN_L ↔ GND')
    print()
    print('  空闲时两者都应约 2.5 V。')
    print('  发送期间：CAN_H 应升高、CAN_L 应降低。')
    print('  **读数纹丝不动 ⇒ 适配器没有驱动总线**（问题在适配器或接线）。')
    print('=' * 62)
    print()

    sent = 0
    t_end = time.monotonic() + args.seconds
    try:
        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as s:
            s.bind((args.interface,))
            payload = bytes([0xAA, 0x55, 0xAA, 0x55, 0x01, 0x02, 0x03, 0x04])
            frame = struct.pack('=IB3x8s', PROBE_ID, 8, payload)
            last = 0.0
            while time.monotonic() < t_end:
                try:
                    s.send(frame)
                    sent += 1
                except OSError as e:
                    print('[!!] 发送失败：%s' % e)
                    break
                now = time.monotonic()
                if now - last >= 2.0:
                    left = max(0, t_end - now)
                    print('  已发送 %-7d 帧   剩余 %.0f 秒' % (sent, left), flush=True)
                    last = now
                time.sleep(0.002)      # 约 500 帧/秒，足够让万用表看到平均电平偏移
    except OSError as e:
        report['result'] = 'blocked'
        report['error'] = str(e)
        print('[XX] 打开接口失败：%s' % e)
        _save(report)
        return 1

    after = snapshot(iface(args.interface))
    report['after'] = after
    report['frames_sent'] = sent
    report['tx_delta'] = (after.get('tx_packets') or 0) - (before.get('tx_packets') or 0)

    print()
    print('发送结束：本进程发出 %d 帧；接口 tx 计数 %s → %s（+%s）'
          % (sent, before.get('tx_packets'), after.get('tx_packets'), report['tx_delta']))
    print('接口 state：%s → %s' % (before.get('state'), after.get('state')))

    if after.get('state') == 'BUS-OFF':
        print()
        print('[!!] 接口进入了 BUS-OFF —— 这说明控制器**确实检测到了发送失败**，')
        print('     即总线上没有节点应答。这本身是一个有用的结论：')
        print('     收发器在驱动总线，但对方没接上或没在工作。')
    else:
        print()
        print('判读（结合你刚才量到的电压）：')
        print('  · 电压有明显偏移 → 帧确实上了总线 → 问题在机械臂侧')
        print('  · 电压纹丝不动   → 适配器外部收发器没驱动总线 → 问题在适配器/接线')

    _save(report)
    return 0


def _save(report):
    out = ROOT / 'outputs/can_tx_probe.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print('\n报告已写入: %s' % out)


if __name__ == '__main__':
    sys.exit(main())
