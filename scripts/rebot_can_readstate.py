#!/usr/bin/env python3
"""只读实机七轴状态 —— 通过参数读取，绕开失效的反馈路径。

背景（2026-10-07 实测）
----------------------
本机 7 个 RobStride 电机（ID 1–7，host_id 0xFD）**ping 全部应答**，
但 `motor.get_state()` **全部返回空** —— 印证官方示例的警告：

    "motorbridge 内部协议配置问题：RS 电机无法像 DM 电机那样查询，
     可能会出现读取不到实际电机状态的情况，请以实际运动效果为准。"

**但参数读取是可用的。** 实测能读到的：

| 参数 | 含义 | 实测值示例（ID 1） |
| --- | --- | --- |
| `0x7019` | 疑似**机械位置**（本脚本主用） | +0.0963 rad |
| `0x7016` | `loc_ref` 位置参考 | +0.0961 rad |
| `0x7017` | `limit_spd` 速度限幅 | 1.0 |
| `0x701E` | 位置环 Kp | 30.0 |

**所以"实机反馈驱动数字孪生"这条路是可行的** —— 位置读得到，
只是不能走 `get_state()` 那条路。

⚠️ `0x7019` 的含义尚未与官方参数表逐字核对，目前依据是"数值合理、
各轴不同、符合零位附近姿态"。**在用于标定之前应先确认参数定义。**

安全
----
**只读**：只调用 `robstride_get_param_f32()`。
**不使能、不发运动指令、不写任何参数、不 disable、不 set_zero。**

用法:
    python3 scripts/rebot_can_readstate.py
    python3 scripts/rebot_can_readstate.py --ids 1,2,3 --param 0x7019
    python3 scripts/rebot_can_readstate.py --watch 5      # 每 0.5 s 打印一次，共 5 轮

输出: outputs/machine_state.json
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
DEG = 180.0 / math.pi

_REBOT_PY_CANDIDATES = [
    Path('/home/bonvoyage/.local/share/robotics/envs/rebot-sim/bin/python'),
    Path.home() / '.local/share/robotics/envs/rebot-sim/bin/python',
]
REBOT_PY = next((p for p in _REBOT_PY_CANDIDATES if p.exists()), _REBOT_PY_CANDIDATES[0])
_REEXEC_FLAG = '_REBOT_READSTATE_REEXEC'

# 只读参数表。0x7019 为主用（疑似机械位置），其余用于交叉核对。
PARAMS = [
    (0x7019, 'mech_pos_rad', '疑似机械位置'),
    (0x7016, 'loc_ref_rad', '位置参考 loc_ref'),
    (0x7017, 'limit_spd', '速度限幅 limit_spd'),
    (0x701E, 'loc_kp', '位置环 Kp'),
]


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


def iface_ok(name):
    try:
        r = subprocess.run(['ip', '-details', '-json', 'link', 'show', 'dev', name],
                           capture_output=True, text=True, timeout=15)
        if r.returncode != 0:
            return False, '接口 %s 不存在' % name
        info = json.loads(r.stdout)[0]
    except Exception as e:
        return False, '无法读取接口信息：%s' % e
    if 'UP' not in info.get('flags', []):
        return False, '接口 %s 处于 DOWN' % name
    modes = info.get('linkinfo', {}).get('info_data', {}).get('ctrlmode') or []
    if 'LISTEN-ONLY' in modes or 'LOOPBACK' in modes:
        return False, '接口 ctrlmode 含 %s' % modes
    return True, None


def read_once(ctrl, ids, param, host_id, model, timeout_ms):
    out = {}
    for mid in ids:
        try:
            motor = ctrl.add_robstride_motor(mid, host_id, model)
        except Exception as e:
            out[mid] = {'error': '注册失败: %s' % str(e)[:60]}
            continue
        try:
            out[mid] = {'value': motor.robstride_get_param_f32(param, timeout_ms)}
        except Exception as e:
            out[mid] = {'error': str(e)[:70]}
        try:
            motor.close()
        except Exception:
            pass
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interface', default='can0')
    ap.add_argument('--ids', default='1,2,3,4,5,6,7')
    ap.add_argument('--host-id', type=lambda v: int(v, 0), default=0xFD)
    ap.add_argument('--model', default='rs-06', help='rs-06 / rs-00')
    ap.add_argument('--param', type=lambda v: int(v, 0), default=0x7019,
                    help='要读的参数，默认 0x7019（疑似机械位置）')
    ap.add_argument('--timeout-ms', type=int, default=800)
    ap.add_argument('--watch', type=int, default=0, help='连续采样轮数（0 = 只读一次）')
    ap.add_argument('--interval', type=float, default=0.5, help='采样间隔（秒）')
    args = ap.parse_args()

    ids = [int(x, 0) for x in args.ids.split(',') if x.strip()]

    ok, problem = iface_ok(args.interface)
    if not ok:
        print('[XX] %s' % problem)
        print('     sudo ip link set %s down' % args.interface)
        print('     sudo ip link set %s type can bitrate 1000000 loopback off listen-only off' % args.interface)
        print('     sudo ip link set %s up' % args.interface)
        return 1

    p = ensure_motorbridge()
    if p:
        print('[XX] ' + p)
        return 1

    from motorbridge import Controller

    report = {
        'time_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'interface': args.interface, 'motor_ids': ids,
        'host_id': args.host_id, 'model': args.model,
        'param': '0x%04X' % args.param,
        'read_only': True, 'enables_motors': False,
        'sends_motion': False, 'writes_params': False,
        'note': '通过 robstride_get_param_f32 读取；get_state() 反馈路径在本机不工作。',
    }

    rounds = args.watch if args.watch > 0 else 1
    samples = []
    ctrl = None
    try:
        ctrl = Controller(args.interface)
        for i in range(rounds):
            t = time.monotonic()
            vals = read_once(ctrl, ids, args.param, args.host_id, args.model, args.timeout_ms)
            samples.append({'t': round(t, 3), 'values': vals})
            if rounds > 1:
                line = ' '.join(
                    ('%s=%.4f' % (k, v['value']) if 'value' in v else '%s=--' % k)
                    for k, v in sorted(vals.items()))
                print('  [%2d/%d] %s' % (i + 1, rounds, line))
            if i < rounds - 1:
                time.sleep(args.interval)
    except Exception as e:
        report['result'] = 'blocked'
        report['error'] = '%s: %s' % (type(e).__name__, e)
        print('[XX] %s' % e)
        _save(report)
        return 1
    finally:
        if ctrl is not None:
            for fn in ('close_bus', 'close', 'shutdown'):
                try:
                    getattr(ctrl, fn)()
                except Exception:
                    pass

    report['samples'] = samples
    report['result'] = 'ok'

    if rounds == 1:
        vals = samples[0]['values']
        print('%-5s %14s %14s  %s' % ('ID', '值', '换算(deg)', '说明'))
        print('-' * 56)
        okn = 0
        for mid in ids:
            v = vals.get(mid, {})
            if 'value' in v:
                okn += 1
                print('%-5d %+14.6f %+14.3f  0x%04X' % (mid, v['value'], v['value'] * DEG, args.param))
            else:
                print('%-5d %14s %14s  %s' % (mid, '--', '--', v.get('error', '无数据')[:40]))
        print('-' * 56)
        print('读到 %d/%d 个轴' % (okn, len(ids)))
        if args.param == 0x7019 and okn:
            print()
            print('⚠️ 0x7019 的含义尚未与官方参数表逐字核对。当前依据是')
            print('   "数值合理、各轴不同、符合零位附近姿态"。用于标定前应先确认。')
    else:
        # 判断是否在变化
        changed = []
        for mid in ids:
            vs = [s['values'].get(mid, {}).get('value') for s in samples]
            vs = [v for v in vs if v is not None]
            if len(vs) >= 2 and max(vs) - min(vs) > 1e-6:
                changed.append(mid)
        print()
        print('采样 %d 轮，期间数值有变化的轴：%s' % (rounds, changed or '无'))

    _save(report)
    return 0


def _save(report):
    out = ROOT / 'outputs/machine_state.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print('\n报告已写入: %s' % out)


if __name__ == '__main__':
    sys.exit(main())
