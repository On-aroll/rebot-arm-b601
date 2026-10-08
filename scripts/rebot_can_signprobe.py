#!/usr/bin/env python3
"""零力矩符号标定 —— 你用手拨关节，我只读 mechPos。

来源
----
官方 `tools/gravity_calibration/phase_a_sign_probe.py` 的做法（原文）：

    NO-torque sign check: streams mechPos while you move joints by hand —
    anchors the motor→URDF sign convention **before any torque is applied**.

**这是确定符号约定最安全的方法：电机全程保持未使能，不产生任何力矩。**

为什么需要它
------------
官方 `config/rebotarm_rs.yaml` 里有电机 ID ↔ 关节 的映射（ID N = joint N），
**但没有符号/方向定义**。而项目文档明确提醒：
"当前六轴 URDF/Menagerie 的反号映射不能直接当作实机映射"。

另外实测发现：实机各轴读数与模型限位对不上（J2 读到负值，而模型要求 0~180°），
说明**实机零位与模型零位之间还有偏移**，不只是符号问题。

做法
----
1. 运行本脚本（电机保持未使能，**不产生力矩**）
2. **用手缓慢拨动一根关节**，观察终端里哪一行在变
3. 记录：拨动的是哪根轴、往哪个方向拨、mechPos 变大还是变小
4. 对 6 根轴依次做

⚠️ 机械臂未使能时是"软"的，重关节可能因重力下落——**先托住再拨**。

安全
----
**只读**：只调用 `robstride_get_param_f32(0x7019)`。
**不使能、不发运动指令、不写参数、不 disable。**

用法:
    python3 scripts/rebot_can_signprobe.py                  # 默认 120 秒
    python3 scripts/rebot_can_signprobe.py 60               # 60 秒
    python3 scripts/rebot_can_signprobe.py --threshold 0.02 # 更灵敏

输出: 终端实时打印 + outputs/signprobe_trace.jsonl
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

# 官方 config/rebotarm_rs.yaml：motor_id N ↔ joint N
MODELS = ['rs-06', 'rs-06', 'rs-06', 'rs-00', 'rs-00', 'rs-00', 'rs-00']
MECH_POS = 0x7019

_REBOT_PY_CANDIDATES = [
    Path('/home/bonvoyage/.local/share/robotics/envs/rebot-sim/bin/python'),
    Path.home() / '.local/share/robotics/envs/rebot-sim/bin/python',
]
REBOT_PY = next((p for p in _REBOT_PY_CANDIDATES if p.exists()), _REBOT_PY_CANDIDATES[0])
_REEXEC_FLAG = '_REBOT_SIGN_REEXEC'


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
    print('[==] 切换到 rebot-sim 环境重跑：%s' % REBOT_PY)
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
        return False, '无法读取接口：%s' % e
    if 'UP' not in info.get('flags', []):
        return False, '接口 %s 处于 DOWN' % name
    modes = info.get('linkinfo', {}).get('info_data', {}).get('ctrlmode') or []
    if 'LISTEN-ONLY' in modes or 'LOOPBACK' in modes:
        return False, '接口 ctrlmode 含 %s' % modes
    return True, None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('duration', nargs='?', type=float, default=120.0, help='持续秒数，默认 120')
    ap.add_argument('--interface', default='can0')
    ap.add_argument('--ids', default='1,2,3,4,5,6')
    ap.add_argument('--threshold', type=float, default=0.03,
                    help='变化超过该值（弧度）才打印，默认 0.03')
    ap.add_argument('--hz', type=float, default=5.0, help='采样频率，默认 5 Hz')
    args = ap.parse_args()

    ids = [int(x, 0) for x in args.ids.split(',') if x.strip()]

    ok, problem = iface_ok(args.interface)
    if not ok:
        print('[XX] %s' % problem)
        return 1
    p = ensure_motorbridge()
    if p:
        print('[XX] ' + p)
        return 1

    from motorbridge import Controller

    ctrl = Controller(args.interface)
    motors = {}
    for mid in ids:
        model = MODELS[mid - 1] if 1 <= mid <= len(MODELS) else 'rs-06'
        try:
            motors[mid] = ctrl.add_robstride_motor(mid, 0xFD, model)
        except Exception as e:
            print('  [!!] ID %d 注册失败: %s' % (mid, str(e)[:60]))

    def read_all():
        out = {}
        for mid, m in motors.items():
            try:
                out[mid] = m.robstride_get_param_f32(MECH_POS, 600)
            except Exception:
                out[mid] = None
        return out

    print('=' * 66)
    print(' 零力矩符号标定 —— 电机保持未使能，不产生任何力矩')
    print('=' * 66)
    print(' 操作：用手**缓慢**拨动一根关节，观察下面哪一行在变。')
    print(' ⚠️ 未使能时机械臂是"软"的，重关节可能因重力下落 —— 先托住再拨。')
    print(' 依次对 6 根轴做一遍，记录：哪根轴、往哪拨、数值变大还是变小。')
    print(' 按 Ctrl+C 结束。')
    print('=' * 66)
    print()

    start = read_all()
    print('起始姿态 (deg): ' + '  '.join(
        'J%d=%s' % (k, '--' if v is None else '%+.2f' % (v * DEG))
        for k, v in sorted(start.items())))
    print()

    trace_path = ROOT / 'outputs/signprobe_trace.jsonl'
    trace_path.parent.mkdir(exist_ok=True)
    last = dict(start)
    t0 = time.time()
    events = []
    end = None
    extremes = {}      # 全程极值，供标定（如夹爪开合两端）使用
    try:
        with trace_path.open('w') as trace:
            while time.time() - t0 < args.duration:
                q = read_all()
                trace.write(json.dumps({
                    't': round(time.time() - t0, 3),
                    'q': {k: (None if v is None else round(v, 6)) for k, v in q.items()},
                }) + '\n')
                trace.flush()
                for mid, v in q.items():
                    # 全程极值统计：标定（如夹爪开合两端值）需要它
                    if v is not None:
                        lo, hi = extremes.get(mid, (v, v))
                        extremes[mid] = (min(lo, v), max(hi, v))
                    lv = last.get(mid)
                    if v is None or lv is None:
                        continue
                    if abs(v - lv) > args.threshold:
                        ev = {'t': round(time.time() - t0, 2), 'motor': mid,
                              'now_deg': round(v * DEG, 3),
                              'delta_deg': round((v - lv) * DEG, 3),
                              'vs_start_deg': round((v - start[mid]) * DEG, 3)
                              if start.get(mid) is not None else None}
                        events.append(ev)
                        print('  ▶ 电机 %d 动了：%+.3f°  （相对起点 %+.3f°）'
                              % (mid, ev['delta_deg'], ev['vs_start_deg'] or 0), flush=True)
                        last[mid] = v
                time.sleep(max(0.02, 1.0 / args.hz))
            # 必须在 finally 关闭控制器之前读结束姿态（否则全部读到 None）
            end = read_all()
    except KeyboardInterrupt:
        print('\n(手动结束)')
        try:
            end = read_all()
        except Exception:
            end = dict(start)
    finally:
        for fn in ('close_bus', 'close', 'shutdown'):
            try:
                getattr(ctrl, fn)()
            except Exception:
                pass

    if end is None:
        end = dict(start)
    print()
    print('结束姿态 (deg): ' + '  '.join(
        'J%d=%s' % (k, '--' if v is None else '%+.2f' % (v * DEG))
        for k, v in sorted(end.items())))
    print()
    print('全程极值 (deg) —— 标定用，如夹爪开合的两端值:')
    for mid in sorted(extremes):
        lo, hi = extremes[mid]
        print('  ID %d: 最小 %+9.3f   最大 %+9.3f   行程 %+8.3f' % (mid, lo*DEG, hi*DEG, (hi-lo)*DEG))
    print()
    print('相对起点的总变化 (deg):')
    for mid in sorted(ids):
        if start.get(mid) is None or end.get(mid) is None:
            print('  J%d: --' % mid)
            continue
        print('  J%d: %+8.3f' % (mid, (end[mid] - start[mid]) * DEG))

    report = {
        'time_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'duration_s': args.duration, 'threshold_rad': args.threshold,
        'read_only': True, 'enables_motors': False,
        'sends_motion': False, 'writes_params': False,
        'start_rad': start, 'end_rad': end, 'events': events,
        'extremes_rad': {str(k): v for k, v in extremes.items()},
        'trace': str(trace_path),
    }
    out = ROOT / 'outputs/signprobe.json'
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print()
    print('报告: %s' % out)
    print('逐帧轨迹: %s' % trace_path)
    print()
    print('把「哪根轴、往哪拨、数值变大还是变小」告诉助手，即可定出符号约定。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
