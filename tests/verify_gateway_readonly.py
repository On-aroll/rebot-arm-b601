#!/usr/bin/env python3
"""只读核实 motorbridge ws_gateway 能否承担控制台的实机通路。

结论写进 outputs/gateway_readonly_verification.json。

为什么值得单独一个脚本：控制台要显示七个电机模块，就必须确认「看见电机」这件事
不会顺手改电机。官方前端在选中电机时走的 setTargetFor 会写 0x7026（主动上报周期，
存在电机里）并打开 set_active_report —— 见 src/hooks/useGatewayBridge.js:239-285。
本脚本绕开它，只用三条不改电机的 op：set_target / robstride_read_param / close_bus。

用法:
    python3 tests/verify_gateway_readonly.py
    python3 tests/verify_gateway_readonly.py --ids 1,2,3
"""
import argparse
import asyncio
import datetime
import json
import math
import sys
import urllib.request
from pathlib import Path

import websockets

ROOT = Path(__file__).resolve().parents[1]
DEG = 180.0 / math.pi
GATEWAY = 'ws://127.0.0.1:9002'
FEEDBACK_ID = 0xFD          # 本机实测：host_id/feedback_id = 0xFD
MECH_POS = 0x7019           # 未归一化的多圈机械位置(rad)
LIMIT_SPD = 0x7017          # 位置模式速度上限(rad/s) —— 限速兵要看的就是它
# 官方 config/rebotarm_rs.yaml：motor_id N ↔ joint N，1–3 为 rs-06，4–7 为 rs-00
MODELS = {1: 'rs-06', 2: 'rs-06', 3: 'rs-06', 4: 'rs-00', 5: 'rs-00', 6: 'rs-00', 7: 'rs-00'}
ALLOWED_OPS = {'set_target', 'robstride_read_param', 'close_bus'}


def wrap_deg(angle):
    """mechPos 是多圈值，归一化到 (-180, 180] 才能和模型限位比较。"""
    a = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if abs(a + 180.0) < 1e-9 else a


async def probe(ids, gateway, with_modes=False):
    """逐轴读 mechPos 与 limit_spd。只发 ALLOWED_OPS，任何写/使能/运动 op 都不在列表里。"""
    sent = []
    readings = {}
    hello = None
    async with websockets.connect(gateway, open_timeout=8, max_size=2 ** 22) as ws:
        async def ask(op, payload=None, timeout=6.0):
            req_id = len(sent) + 1
            sent.append(op)
            await ws.send(json.dumps({'op': op, 'req_id': req_id, **(payload or {})}))
            deadline = asyncio.get_event_loop().time() + timeout
            while True:
                left = deadline - asyncio.get_event_loop().time()
                if left <= 0:
                    return {'ok': False, 'error': 'timeout', 'op': op}
                try:
                    msg = json.loads(await asyncio.wait_for(ws.recv(), left))
                except asyncio.TimeoutError:
                    return {'ok': False, 'error': 'timeout', 'op': op}
                if msg.get('type') == 'event' and hello is None:
                    hello.update(msg.get('data') or {})
                if msg.get('req_id') == req_id:
                    return msg

        # 连接本身应当不占用总线；router_mode standby 时 connected_bus 为 false。
        first = json.loads(await asyncio.wait_for(ws.recv(), 5))
        hello = dict(first.get('data') or {})
        assert first.get('event') == 'connected', f'网关首帧不是 connected：{first}'

        for mid in ids:
            target = await ask('set_target', {
                'vendor': 'robstride', 'transport': 'socketcan', 'channel': 'can0',
                'model': MODELS[mid], 'motor_id': mid, 'feedback_id': FEEDBACK_ID,
            })
            if not target.get('ok'):
                readings[mid] = {'error': target.get('error', 'set_target failed')}
                continue
            pos = await ask('robstride_read_param', {'param_id': MECH_POS, 'type': 'f32', 'timeout_ms': 800})
            spd = await ask('robstride_read_param', {'param_id': LIMIT_SPD, 'type': 'f32', 'timeout_ms': 800})
            readings[mid] = {
                'model': MODELS[mid],
                'mech_pos_rad': (pos.get('data') or {}).get('value') if pos.get('ok') else None,
                'mech_pos_error': None if pos.get('ok') else pos.get('error'),
                'limit_spd_rad_s': (spd.get('data') or {}).get('value') if spd.get('ok') else None,
                'limit_spd_error': None if spd.get('ok') else spd.get('error'),
            }
            if with_modes:
                mode = await ask('robstride_read_param', {'param_id': 0x7005, 'type': 'i8', 'timeout_ms': 800})
                pp = await ask('robstride_read_param', {'param_id': 0x7024, 'type': 'f32', 'timeout_ms': 800})
                readings[mid]['run_mode'] = (mode.get('data') or {}).get('value') if mode.get('ok') else None
                readings[mid]['vel_max_rad_s'] = (pp.get('data') or {}).get('value') if pp.get('ok') else None
                readings[mid]['mode_error'] = None if mode.get('ok') else mode.get('error')
                readings[mid]['vel_max_error'] = None if pp.get('ok') else pp.get('error')
            if isinstance(readings[mid]['mech_pos_rad'], (int, float)):
                readings[mid]['mech_pos_deg'] = round(readings[mid]['mech_pos_rad'] * DEG, 3)
                readings[mid]['mech_pos_deg_wrapped'] = round(wrap_deg(readings[mid]['mech_pos_rad'] * DEG), 3)

        await ask('close_bus')

    assert not (set(sent) - ALLOWED_OPS), f'发出了允许范围之外的 op：{set(sent) - ALLOWED_OPS}'
    return hello, readings, sent


def twin_sync_reference():
    """和 rebot-twin-sync 走 python-can 的读数交叉核对：两条独立通路应当一致。"""
    live = ROOT / 'outputs/machine_live.json'
    if not live.is_file():
        return None
    try:
        data = json.loads(live.read_text())
    except (json.JSONDecodeError, OSError):
        return None
    age = (datetime.datetime.now(datetime.timezone.utc)
           - datetime.datetime.fromisoformat(data['time_utc'])).total_seconds()
    return {'q_deg': data.get('q_deg'), 'age_s': round(age, 1), 'round': data.get('round')}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ids', default='1,2,3,4,5,6,7')
    ap.add_argument('--gateway', default=GATEWAY)
    ap.add_argument('--position-modes', action='store_true', help='同时只读当前模式与 PP 速度上限')
    ap.add_argument('--tolerance-deg', type=float, default=0.5,
                    help='与 twin-sync 参考读数的最大允许差值')
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(',') if x.strip()]
    hello, readings, sent = asyncio.run(probe(ids, args.gateway, args.position_modes))

    got = [m for m, v in readings.items() if v.get('mech_pos_rad') is not None]
    assert len(got) == len(ids), f'只读到 {len(got)}/{len(ids)} 个轴：{got}'
    assert all(readings[m].get('limit_spd_rad_s') is not None for m in got), '有限速参数读不到'
    assert hello.get('connected_bus') is False or hello.get('router_mode'), '连接首帧异常'

    if args.position_modes:
        for mid in ids:
            assert readings[mid].get('run_mode') is not None, f'ID {mid} 模式未读到'
            assert readings[mid].get('vel_max_rad_s') is not None, f'ID {mid} PP 速度上限未读到'
    reference = twin_sync_reference()
    cross = None
    if reference and reference['q_deg'] and reference['age_s'] < 3600:
        diffs = []
        for i, mid in enumerate(got):
            if mid > 6 or reference['q_deg'][i] is None:
                continue
            diffs.append(round(abs(readings[mid]['mech_pos_deg_wrapped'] - reference['q_deg'][i]), 3))
        if diffs:
            worst = max(diffs)
            cross = {'compared_axes': len(diffs), 'max_abs_diff_deg': worst,
                     'tolerance_deg': args.tolerance_deg, 'match': worst <= args.tolerance_deg}
            assert worst <= args.tolerance_deg, f'两条通路读数不一致：最大差 {worst}°'

    report = {
        'time_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'gateway': args.gateway, 'hello': hello, 'ops_sent': sorted(set(sent)),
        'ids': ids, 'readings': {str(k): v for k, v in readings.items()},
        'twin_sync_reference': reference, 'cross_check': cross,
        'read_only': True, 'enables_motors': False, 'writes_params': False,
        'sends_motion_to_hardware': False,
        'note': ('只用 set_target / robstride_read_param / close_bus。'
                 '官方前端选中电机时会写 0x7026 并开主动上报，本脚本不走那条路。'),
        'result': 'ok',
    }
    out = ROOT / 'outputs/gateway_readonly_verification.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({k: report[k] for k in ('result', 'ops_sent', 'cross_check')}, ensure_ascii=False))
    for mid in ids:
        v = readings[mid]
        print('  ID %d %-6s mechPos=%+9.3f°  limit_spd=%s rad/s'
              % (mid, v.get('model'), v.get('mech_pos_deg_wrapped', float('nan')),
                 v.get('limit_spd_rad_s')))
    print('报告：', out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
