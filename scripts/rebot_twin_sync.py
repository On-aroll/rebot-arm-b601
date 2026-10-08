#!/usr/bin/env python3
"""把实机七轴状态镜像到网页仿真里 —— 零物理风险。

为什么这是当前最合适的一步
--------------------------
用户担心实机动作会碰到笔记本。而**镜像**这个动作：

  - 实机侧：**只读** mechPos(0x7019)，不发任何指令、不使能、不产生力矩
  - 仿真侧：只改 MuJoCo 模型里的关节角，**对实体机械臂没有任何影响**

所以它**零物理风险**，而且正好是项目文档里"数字孪生"的第一形态
（第 2 步：由实机反馈驱动显示）。

它同时还能回答一个关键问题
--------------------------
**符号约定未知**（官方配置里没有定义，官方专门写了个"手拨动探针"来确定）。
镜像到仿真后肉眼对比：

  - 仿真里的姿态与实机**一致** → 符号约定就是 `+1`
  - 仿真里的姿态与实机**镜像相反** → 需要取反

**这比发运动指令去试探安全得多。**

用法:
    python3 scripts/rebot_twin_sync.py                 # 读一次并镜像
    python3 scripts/rebot_twin_sync.py --dry-run       # 只读，不推送到仿真
    python3 scripts/rebot_twin_sync.py --sign -1,-1,-1,-1,-1,-1
    python3 scripts/rebot_twin_sync.py --watch 30      # 每 0.5 s 同步一次，共 30 次
    python3 scripts/rebot_twin_sync.py --follow        # 持续同步，直到 Ctrl+C

每一轮都会把实机读数原子写入 `outputs/machine_live.json`，仿真服务通过
只读端点 `/machine-state` 转发它，供「虚实联调控制台」显示实机与仿真的差值。

⚠️ 只读实机；不使能、不发运动指令、不写参数。
"""
import argparse
import datetime
import itertools
import json
import math
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEG = 180.0 / math.pi
LIVE = ROOT / 'outputs/machine_live.json'
HISTORY_CAP = 500

# 官方 config/rebotarm_rs.yaml 的映射：motor_id N ↔ joint N
MODELS = ['rs-06', 'rs-06', 'rs-06', 'rs-00', 'rs-00', 'rs-00', 'rs-00']
MECH_POS = 0x7019

_REBOT_PY_CANDIDATES = [
    Path('/home/bonvoyage/.local/share/robotics/envs/rebot-sim/bin/python'),
    Path.home() / '.local/share/robotics/envs/rebot-sim/bin/python',
]
REBOT_PY = next((p for p in _REBOT_PY_CANDIDATES if p.exists()), _REBOT_PY_CANDIDATES[0])
_REEXEC_FLAG = '_REBOT_TWIN_REEXEC'


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


def build_motors(ctrl, ids):
    """电机对象**只注册一次**，循环里复用。

    ⚠️ 早先版本每轮都调 add_robstride_motor，第二轮起就报
    "device with motor_id N already exists"，导致读不到任何轴（实测踩到）。
    """
    motors = {}
    for mid in ids:
        model = MODELS[mid - 1] if 1 <= mid <= len(MODELS) else 'rs-06'
        try:
            motors[mid] = ctrl.add_robstride_motor(mid, 0xFD, model)
        except Exception as e:
            print('  [!!] ID %d 注册失败: %s' % (mid, str(e)[:60]))
    return motors


def read_mech_pos(motors, ids):
    out = {}
    for mid, m in motors.items():
        try:
            out[mid] = m.robstride_get_param_f32(MECH_POS, 800)
        except Exception:
            out[mid] = None
    return out


def publish_live(payload):
    """原子写出实机最新读数，供仿真服务只读转发（scripts/rebot_sim.py /machine-state）。

    先写 .tmp 再 replace，避免读方拿到半截 JSON。这里只写本地文件，
    不产生任何实机流量。
    """
    LIVE.parent.mkdir(exist_ok=True)
    tmp = LIVE.with_suffix('.json.tmp')
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2))
    tmp.replace(LIVE)


def sim_limits(base='http://127.0.0.1:8765'):
    """从仿真服务读当前模型的可达关节范围（度）。用于显示钳位。"""
    try:
        with urllib.request.urlopen(base + '/state', timeout=8) as r:
            d = json.loads(r.read())
        return d.get('lower_deg'), d.get('upper_deg')
    except Exception:
        return None, None


def post_sim(payload, base='http://127.0.0.1:8765'):
    # 服务端只接受 /program 与 /command 两个 POST 端点（见 rebot_sim.py do_POST）。
    req = urllib.request.Request(
        base + '/command', data=json.dumps(payload).encode(),
        headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return {'error': e.read().decode('utf-8', 'replace')[:200]}
    except Exception as e:
        return {'error': '%s: %s' % (type(e).__name__, e)}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interface', default='can0')
    ap.add_argument('--ids', default='1,2,3,4,5,6,7')
    ap.add_argument('--sim', default='http://127.0.0.1:8765')
    ap.add_argument('--sign', default='1,1,1,1,1,1',
                    help='六轴符号，逗号分隔，默认全 +1')
    ap.add_argument('--dry-run', action='store_true', help='只读实机，不推送仿真')
    ap.add_argument('--gripper-closed', type=float, default=-3.986,
                    help='实机夹爪(ID7) mechPos 在「完全闭合」时的角度(deg)。'
                         '默认来自 2026-10-07 标定探针实测极值 min=-3.986°')
    ap.add_argument('--gripper-open', type=float, default=339.060,
                    help='实机夹爪(ID7) mechPos 在「完全张开」时的角度(deg)。'
                         '默认来自标定探针实测极值 max=+339.060°（先开后合→张开使 mechPos 增大）')
    ap.add_argument('--gripper-stroke', type=float, default=0.05,
                    help='仿真夹爪单指行程(m)，对应模型 joint range 0–0.05')
    ap.add_argument('--gripper-invert', action='store_true',
                    help='翻转夹爪方向（若发现实机张开时仿真反而闭合，加此开关）')
    ap.add_argument('--no-wrap', dest='wrap', action='store_false',
                    help='不做多圈归一化（默认会做）')
    ap.set_defaults(wrap=True)
    # 默认开启的依据（2026-10-07 实测）：J6 的 mechPos 读到 -196.02°，
    # 归一化到 +163.98° 后正好落在模型限位 ±180° 内，镜像与实机完全一致。
    # -196° 与 +164° 物理上是同一角度（差 360°）。
    ap.add_argument('--no-clamp', dest='clamp', action='store_false',
                    help='不钳位（默认会钳位，并打印钳位量）')
    ap.set_defaults(clamp=True)
    # 默认开启的依据：实机零位与模型零位有几分之一的度级偏差（实测 J2 差 0.05°），
    # 不钳位则整条命令被拒，镜像无法进行。钳位量会被打印，不会静默隐藏。
    ap.add_argument('--watch', type=int, default=0, help='同步轮数（0=一次）')
    ap.add_argument('--follow', action='store_true',
                    help='持续同步直到 Ctrl+C（无限 --watch），并实时发布实机读数')
    ap.add_argument('--no-live', dest='live', action='store_false',
                    help='不写 outputs/machine_live.json（默认每轮都写）')
    ap.set_defaults(live=True)
    ap.add_argument('--interval', type=float, default=0.5)
    args = ap.parse_args()

    ids = [int(x, 0) for x in args.ids.split(',') if x.strip()]
    signs = [float(x) for x in args.sign.split(',') if x.strip()]
    if len(signs) < 6:
        signs += [1.0] * (6 - len(signs))

    ok, problem = iface_ok(args.interface)
    if not ok:
        print('[XX] %s' % problem)
        return 1
    p = ensure_motorbridge()
    if p:
        print('[XX] ' + p)
        return 1

    from motorbridge import Controller

    report = {
        'time_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'interface': args.interface, 'ids': ids, 'signs': signs,
        'sim': args.sim, 'dry_run': args.dry_run,
        'read_only_on_hardware': True,
        'enables_motors': False, 'sends_motion_to_hardware': False,
        'note': '只读实机 mechPos(0x7019)；仅修改仿真模型关节角。',
        'gripper_calib': {
            'closed_deg': args.gripper_closed, 'open_deg': args.gripper_open,
            'stroke_m': args.gripper_stroke, 'invert': args.gripper_invert,
            'source': '2026-10-07 标定探针 signprobe --ids 7（先开后合，张开使 mechPos 增大）',
        },
    }

    ctrl = None
    rounds = float('inf') if args.follow else (args.watch if args.watch > 0 else 1)
    history = []
    try:
        ctrl = Controller(args.interface)
        motors = build_motors(ctrl, ids)
        if not motors:
            print('[XX] 没有任何电机注册成功。检查 CAN 线是否接好。')
            report['result'] = 'no_motors'
            _save(report)
            return 1
        for i in itertools.count():
            if i >= rounds:
                break
            raw = read_mech_pos(motors, ids)
            got = {k: v for k, v in raw.items() if v is not None}
            if not got:
                print('[XX] 一个轴都没读到。检查 CAN 线是否接好。')
                report['result'] = 'no_data'
                _save(report)
                return 1

            # 关节角 = sign * mechPos；夹爪（ID7）单独处理
            q_deg = []
            for j in range(6):
                mid = j + 1
                v = raw.get(mid)
                if v is None:
                    q_deg.append(None)
                    continue
                a = signs[j] * v * DEG
                # mechPos 可能是未归一化的多圈位置（实测 J6 = -196°，真值可能 +164°）
                if args.wrap:
                    a = (a + 180.0) % 360.0 - 180.0
                    if abs(a + 180.0) < 1e-9:
                        a = 180.0
                q_deg.append(a)
            finger_m = None
            if 7 in raw and raw[7] is not None:
                # 夹爪(ID7)：mechPos 是**角度(rad)**，仿真夹爪是**移动副(m)**。
                # 标定（2026-10-07 探针：先开后合 → 张开使 mechPos 增大）：
                #   实机完全闭合 ≈ -3.986°  → 仿真 finger_m = 0（手指并拢）
                #   实机完全张开 ≈ +339.06° → 仿真 finger_m = 0.05（最大张开）
                # 与官方 ROS2 桥 real_to_sim.py 的约定一致（mechPos 增→disp 增→张开）。
                # 本机实测闭合角 ≈ -0.0696 rad，官方默认假设 0 rad，属个体零位偏差。
                closed_r = args.gripper_closed / DEG
                open_r = args.gripper_open / DEG
                span = open_r - closed_r
                frac = 0.0 if abs(span) < 1e-12 else (raw[7] - closed_r) / span
                frac = min(1.0, max(0.0, frac))          # 夹紧到 [0,1]，超出行程的部分不显示
                fm = frac * args.gripper_stroke
                if args.gripper_invert:
                    fm = args.gripper_stroke - fm
                finger_m = fm

            # 钳位之前的真实读数：虚实对比要拿实机原值比，不能拿喂给仿真的钳位值。
            real_deg = list(q_deg)
            line = ' '.join('%s' % ('--' if v is None else '%+.2f' % v) for v in q_deg)
            print('  [%2d] 实机(deg): %s   夹爪≈%.4f m' % (i + 1, line, finger_m or 0), flush=True)
            history.append({'t': round(time.monotonic(), 3), 'q_deg': q_deg,
                            'mech_pos_rad': raw, 'finger_m': finger_m})
            # --follow 可以跑很久，历史只留最近若干轮，避免内存无界增长。
            if len(history) > HISTORY_CAP:
                del history[:len(history) - HISTORY_CAP]

            if args.live:
                publish_live({
                    'time_unix_s': time.time(),
                    'time_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    'round': i + 1, 'interface': args.interface, 'ids': ids,
                    'signs': signs, 'wrap': args.wrap,
                    'q_deg': real_deg, 'mech_pos_rad': raw, 'finger_m': finger_m,
                    'read_only_on_hardware': True,
                    'enables_motors': False, 'sends_motion_to_hardware': False,
                    'enable_state': 'unknown',
                    'note': '参数读取有应答不等于已使能；本文件只记录 mechPos(0x7019) 只读结果。',
                })

            if not args.dry_run:
                valid = [v for v in q_deg if v is not None]
                if len(valid) == 6:
                    if args.clamp:
                        lo, hi = sim_limits(args.sim)
                        if lo and hi:
                            before = list(q_deg)
                            q_deg = [min(max(v, lo[i]), hi[i]) for i, v in enumerate(q_deg)]
                            moved = ['J%d %.2f→%.2f' % (i + 1, before[i], q_deg[i])
                                     for i in range(6) if abs(before[i] - q_deg[i]) > 1e-6]
                            if moved:
                                print('  [钳位] %s' % '  '.join(moved))
                    payload = {'action': 'joints', 'q_deg': q_deg}
                    if finger_m is not None:
                        payload['finger_m'] = finger_m
                    res = post_sim(payload, args.sim)
                    if i == 0:
                        if 'error' in res:
                            print('  [!!] 推送仿真失败：%s' % res['error'])
                            print('       仿真服务在跑吗？ MUJOCO_GL=egl ~/.local/bin/rebot-sim serve')
                        else:
                            print('  [==] 已推送到仿真 —— 请对照屏幕上的姿态与实机是否一致')
                else:
                    print('  [!!] 有轴读不到，本轮不推送')
            if i < rounds - 1:
                time.sleep(args.interval)
    except KeyboardInterrupt:
        report['result'] = 'interrupted'
        report['history'] = history
        print('\n[ok] 已停止跟随（Ctrl+C）。实机侧全程只读。')
        _save(report)
        return 0
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

    report['result'] = 'ok'
    report['history'] = history
    _save(report)

    if not args.dry_run:
        print()
        print('=' * 62)
        print(' 请对照屏幕：仿真里的机械臂姿态与实机是否一致？')
        print()
        print('   · 一致          → 符号约定就是当前设置（--sign %s）' % ','.join('%g' % s for s in signs))
        print('   · 明显镜像相反  → 改用 --sign %s 重试' % ','.join('%g' % -s for s in signs))
        print('   · 只有个别轴不对 → 单独把那几个轴取反')
        print()
        print(' 这一步**不涉及任何实机动作**，只改仿真模型。')
        print('=' * 62)
    return 0


def _save(report):
    out = ROOT / 'outputs/twin_sync.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print('\n报告已写入: %s' % out)


if __name__ == '__main__':
    sys.exit(main())
