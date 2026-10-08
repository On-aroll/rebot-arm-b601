#!/usr/bin/env python3
"""只读诊断 USB-CAN 链路：区分「USB 层故障」与「CAN 总线无数据」。

设计意图
--------
实机接不上时，最容易走错的路是把 CAN 侧（接线、终端电阻、波特率）翻来覆去地查。
但如果内核里持续出现 `Rx urb aborted (-71)`，问题在 USB 层：驱动在接口 UP 时就会
提交接收 URB，**与总线上有没有数据无关**。适配器不响应 IN 请求，CAN 侧怎么调都收不到帧。

本脚本不发送任何数据、不需要 sudo、不导入电机 SDK，只读取：
  - /sys/bus/usb/devices  适配器是否在总线上、走的是直连还是 Hub、协商速率
  - ip -details -json      can0 状态、控制器状态、总线错误计数、丢包计数
  - dmesg / journalctl     USB 与 CAN 相关内核消息，按关键字分类计数

用法:
    python3 scripts/diagnose_can_usb.py            # 观察最近 10 分钟内核日志
    python3 scripts/diagnose_can_usb.py --minutes 30
    python3 scripts/diagnose_can_usb.py --no-log   # 不读内核日志（无权限时）

输出: outputs/can_usb_diagnose.json
"""
import argparse
import collections
import datetime
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# 只把真正的 CAN 适配器算作适配器。0c72:000c 是 PEAK PCAN-USB 的 ID，
# 本机设备字符串却是 XCAN-USB —— 克隆品冒用该 ID。
CAN_ADAPTER_IDS = {
    '0c72': 'PEAK / XCAN-USB（克隆冒用）',
    '1d50': 'CANable / candleLight',
    '1209': 'gs_usb',
    '04d8': 'Microchip CAN',
}
# Hub 单独标注。早先版本把 Hub 厂商混进适配器候选，导致把 Genesys Hub 误报成
# CAN 适配器（2026-10-07 实测踩到），务必分开。
HUB_IDS = {
    '1a86': 'QinHeng Hub',
    '05e3': 'Genesys Logic Hub',
    '2109': 'VIA Hub',
    '0424': 'Microchip Hub',
}

USB_ERROR_PATTERNS = {
    'rx_urb_aborted': re.compile(r'Rx urb aborted', re.I),
    'tx_urb_error': re.compile(r'Tx urb', re.I),
    'set_btr_failed': re.compile(r'f=0x1 n=0x2', re.I),
    'device_start_failed': re.compile(r'f=0x0 n=0x[0-9a-f]+', re.I),
    'hub_port_status': re.compile(r'hub_ext_port_status failed', re.I),
    'clear_tt_error': re.compile(r'clear tt', re.I),
    'disconnect': re.compile(r'USB disconnect', re.I),
    'new_device': re.compile(r'new (full|high|low)-speed USB device', re.I),
    'can_link': re.compile(r'\bcan0\b', re.I),
}


def sh(cmd, timeout=20):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout, r.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        return -1, '', '%s: %s' % (type(e).__name__, e)


def usb_inventory():
    """从 sysfs 读全部 USB 设备，标出直连 / Hub 下游。"""
    out = []
    base = Path('/sys/bus/usb/devices')
    if not base.is_dir():
        return out
    for d in sorted(base.iterdir()):
        if not (d / 'idVendor').is_file():
            continue
        def rd(name):
            p = d / name
            try:
                return p.read_text().strip()
            except OSError:
                return ''
        vid, pid = rd('idVendor'), rd('idProduct')
        drv = ''
        link = d / 'driver'
        if link.exists():
            drv = Path(link.resolve()).name
        is_can = vid in CAN_ADAPTER_IDS or drv == 'peak_usb'
        out.append({
            'sysfs_name': d.name,
            'vid_pid': '%s:%s' % (vid, pid),
            'known_as': CAN_ADAPTER_IDS.get(vid, '') or HUB_IDS.get(vid, ''),
            'is_can_adapter': is_can,
            'is_hub': vid in HUB_IDS,
            'manufacturer': rd('manufacturer'),
            'product': rd('product'),
            'serial': rd('serial'),
            'bcdDevice': rd('bcdDevice'),
            'speed_mbps': rd('speed'),
            'driver': drv,
            # sysfs 名规则：3-3 = 总线3的3号根口（直连）；3-3.4 = 3-3 这个 Hub 的 4 号口。
            # 判据是名字里有没有点，不是减号个数 —— 3-3.4 只有一个减号。
            'is_hub_downstream': '.' in d.name,
        })
    return out


def can_interface(name='can0'):
    rc, out, err = sh(['ip', '-details', '-statistics', '-json', 'link', 'show', 'dev', name])
    if rc != 0:
        return None
    try:
        return json.loads(out)[0]
    except (json.JSONDecodeError, IndexError):
        return None


# 这些输出看起来"成功"，其实一条内核日志都没有。journalctl 在无日志时
# 仍返回退出码 0，只看退出码会被骗（2026-10-07 实测：日志仅内存、/var/log/journal
# 不存在，于是 journalctl 输出 "No journal files were found." 却 rc=0）。
LOG_FAILURE_MARKERS = (
    'no journal files were found',
    'no entries',
    'read kernel buffer failed',
    'operation not permitted',
    'permission denied',
    'you must be root',
)


def kernel_log(minutes):
    """返回 (行列表, 是否真的读到了内核日志)。

    必须把「读到但没有错误」和「根本没读到」分开。否则无权限时会错误地
    报出「USB 通道未报错」——这正是 2026-10-07 踩到的坑。
    """
    for cmd in (['journalctl', '-k', '--since', '-%dmin' % minutes, '--no-pager'],
                ['dmesg', '--time-format', 'iso']):
        rc, out, err = sh(cmd, timeout=30)
        blob = (out + '\n' + err).lower()
        if rc != 0 or not out.strip():
            continue
        if any(m in blob for m in LOG_FAILURE_MARKERS):
            continue
        lines = [ln for ln in out.splitlines() if ln.strip()]
        if lines:
            return lines, True
    return [], False


def classify(lines):
    hits = collections.Counter()
    examples = {}
    for ln in lines:
        low = ln.lower()
        if 'usb' not in low and 'can' not in low and 'peak' not in low and 'hub' not in low:
            continue
        for key, pat in USB_ERROR_PATTERNS.items():
            if pat.search(ln):
                hits[key] += 1
                examples.setdefault(key, ln.strip()[:190])
    return hits, examples


def verdict(usb, iface, hits, log_ok=True):
    """按证据链给结论。返回 (等级, 标题, 说明列表)。"""
    reasons = []
    can_like = [d for d in usb if d['is_can_adapter']]

    if not can_like:
        hubs = [d['sysfs_name'] for d in usb if d['is_hub']]
        extra = []
        if hubs:
            extra.append('当前在线的 Hub：%s。Hub 在线不代表其下游端口正常。' % ', '.join(hubs))
        return ('未连接', '未在 USB 总线上发现 CAN 适配器', [
            '适配器现在没有枚举。请重新插好（建议直连电脑，不经 Hub），再跑本脚本。',
            'sysfs 与 lsusb 都看不到设备时，先解决物理连接，其他分析都没有意义。',
        ] + extra)

    ad = can_like[0]
    behind_hub = ad['is_hub_downstream']
    reasons.append('适配器：%s  路径 %s  %s  %s Mbps  驱动 %s' % (
        ad['product'] or ad['vid_pid'], ad['sysfs_name'],
        '（★ Hub 下游，非直连）' if behind_hub else '（直连）',
        ad['speed_mbps'] or '?', ad['driver'] or '未绑定'))
    # 身份线索：只陈述观察到的事实，不据此单独断定是克隆品。
    ident = []
    if ad['product'] and 'pcan' not in ad['product'].lower() and 'peak' not in (ad['manufacturer'] or '').lower():
        ident.append('设备串为 %r，而 VID/PID %s 对应 PEAK PCAN-USB' % (ad['product'], ad['vid_pid']))
    if ad['serial'] in ('FFFFFFFF', '00000000'):
        ident.append('序列号为 %s（占位值）' % ad['serial'])
    if ident:
        reasons.append('身份线索（不足单独定性，但值得记录）：' + '；'.join(ident))

    if iface is None:
        return ('接口缺失', '适配器在总线上，但 can0 未创建', reasons + [
            '驱动未成功 probe。查看内核日志里 peak_usb 的报错。',
        ])

    data = iface.get('linkinfo', {}).get('info_data', {})
    state = data.get('state')
    berr = data.get('berr_counter', {})
    xst = iface.get('linkinfo', {}).get('info_xstats', {})
    st = iface.get('stats64', {})
    rx, tx = st.get('rx', {}), st.get('tx', {})

    reasons.append('can0 状态 %s  控制器 %s  波特率 %s' % (
        iface.get('operstate'), state,
        data.get('bittiming', {}).get('bitrate')))
    reasons.append('总线错误计数 tx=%s rx=%s  bus_error=%s  bus_off=%s' % (
        berr.get('tx'), berr.get('rx'), xst.get('bus_error'), xst.get('bus_off')))
    reasons.append('收包 %s  发包 %s  接收丢弃 %s' % (
        rx.get('packets'), tx.get('packets'), rx.get('dropped')))

    bus_clean = (not any([berr.get('tx'), berr.get('rx'), xst.get('bus_error'),
                          xst.get('bus_off'), xst.get('error_passive')]))
    usb_broken = hits.get('rx_urb_aborted', 0) > 0

    if usb_broken:
        reasons.append('内核日志出现 %d 次 Rx urb aborted (-71 / EPROTO)' % hits['rx_urb_aborted'])
    if hits.get('set_btr_failed'):
        reasons.append('内核日志出现 %d 次设置波特率命令失败（-32 / EPIPE）' % hits['set_btr_failed'])
    if hits.get('clear_tt_error') or hits.get('hub_port_status'):
        reasons.append('内核日志出现 Hub 事务翻译器错误（clear tt / hub_ext_port_status）')
    if ad['is_hub_downstream']:
        reasons.append('适配器挂在 Hub 下游，而不是直连电脑')

    if usb_broken:
        return ('USB 层故障', 'USB 接收通道异常 —— 问题不在 CAN 侧，也不在机械臂', reasons + [
            '判据：驱动在接口 UP 时就提交接收 URB，与总线上有无数据无关。'
            '出现 Rx urb aborted 说明适配器没有响应 USB IN 请求。',
            '只要这一条不消除，无论 CAN 接线、终端电阻、波特率怎么调，都不可能收到帧。',
            'CAN 侧零错误（上面那行计数器全为 0）反而说明接线和波特率很可能是对的。',
        ])

    if rx.get('packets', 0) == 0:
        if not log_ok:
            return ('无法判定', '接口在，但内核日志读不到 —— USB 层是否正常无从判断', reasons + [
                '本次没有读到内核日志，所以不能下"USB 通道正常"的结论。',
                '请在有权限的终端重跑（需要能读 journalctl 或 dmesg）。',
                '若确认只是权限问题，可用 --no-log 明确跳过，但结论会停留在"无法判定"。',
            ] + (['★ 适配器挂在 Hub 下游，不是直连 —— 这是当前最值得先排除的一项。']
                 if behind_hub else []))
        return ('无数据', 'USB 层未见错误，但总线上没有收到任何帧', reasons + [
            'USB 层看起来正常了，可以开始查 CAN 侧。',
            '依次确认：适配器与机械臂是否共地、CAN_H/CAN_L 是否接反、'
            '两端终端电阻（通常各 120Ω，合计约 60Ω）、波特率是否与电机一致。',
            '注意：只监听模式不会主动请求，电机在未收到读取请求时可能完全不回话，'
            '「零帧」不等于「断线」。',
        ] + (['★ 适配器仍挂在 Hub 下游，不是直连 —— 建议先直连再判断其他项。']
             if behind_hub else []))

    return ('收到数据', '已在总线上收到 CAN 帧', reasons + [
        '下一步：统计帧 ID 分布，与官方参考的 motor ID 1–7 对照，确认轴映射。',
    ])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interface', default='can0')
    ap.add_argument('--minutes', type=int, default=10, help='回溯内核日志的分钟数')
    ap.add_argument('--no-log', action='store_true', help='跳过内核日志（无读权限时）')
    args = ap.parse_args()

    report = {
        'time_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'interface': args.interface,
        'transmit_calls': 0,
        'sudo_required': False,
    }

    usb = usb_inventory()
    report['usb_devices'] = usb
    report['can_devices_found'] = [d['vid_pid'] for d in usb if d['is_can_adapter']]
    report['hubs_online'] = [d['sysfs_name'] for d in usb if d['is_hub']]

    iface = can_interface(args.interface)
    report['interface'] = iface

    hits, examples = collections.Counter(), {}
    log_ok = False
    if not args.no_log:
        lines, log_ok = kernel_log(args.minutes)
        report['kernel_log_readable'] = log_ok
        report['kernel_lines_scanned'] = len(lines)
        if log_ok:
            hits, examples = classify(lines)
            report['usb_error_counts'] = dict(hits)
            report['usb_error_examples'] = examples
        else:
            report['kernel_log_note'] = (
                '未能读取内核日志（journalctl 与 dmesg 都不可用）。'
                '这不等于没有 USB 错误 —— 本次无法判断。'
                '请在有权限的终端重跑，或显式加 --no-log 表示已知跳过。')

    level, title, reasons = verdict(usb, iface, hits, log_ok)
    report['level'] = level
    report['verdict'] = title
    report['reasons'] = reasons

    out = ROOT / 'outputs/can_usb_diagnose.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2))

    print('=' * 66)
    print('结论: %s —— %s' % (level, title))
    print('=' * 66)
    for r in reasons:
        print('  · %s' % r)
    print()
    print('报告已写入: %s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
