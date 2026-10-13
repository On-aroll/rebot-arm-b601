"""Manual J1 trial using Seeed SDK and the existing P4 executor.

--describe is offline. --prepare is read-only. --execute requires a fresh
prepared start and explicit onsite evidence; never invoked by project startup.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import sys
import time

from rebot_mit_hardware import Envelope, HardwareExecutor, MotionIntent, OnsitePermit
from rebot_seeed_transport import SeeedTransport


def prepare(config, bus):
    checker = HardwareExecutor(config, bus, clock=bus.clock)
    readings = []
    try:
        bus.open_readonly()
        for _ in range(3):
            samples = bus.read(range(1, 7), config.policy)
            readings.append({str(i): asdict(s) for i, s in samples.items()})
            checker.validate_feedback(samples, (1,))
            if abs(samples[1].qd) > config.policy.start_speed:
                raise ValueError('实机速度读数超过本次静止阈值；不自动提高阈值')
            if any(abs(s.q-readings[0][str(i)]['q']) > config.policy.arrival_q for i,s in samples.items()):
                raise ValueError('只读采样期间机械臂姿态变化；需停稳后重新准备')
        return dict(ok=True, hardware_writes=0, readings=readings,
                    controlled_axes=[1],
                    uncommanded_model_warnings=[f'J{i} 原始位置超出模型范围；本次只读、不下达目标'
                                               for i in range(2,7) if not config.axes[i-1].q_min <= samples[i].q <= config.axes[i-1].q_max],
                    expected_start=[samples[i].q for i in range(1, 7)])
    except Exception as error:
        return dict(ok=False, hardware_writes=0, reason=str(error), readings=readings)
    finally:
        bus.close()


def execute(config, prepared, evidence):
    if prepared.get('ok') is not True or prepared.get('hardware_writes') != 0 or prepared.get('controlled_axes') != [1]:
        raise ValueError('需要通过的只读起点检查')
    if prepared.get('config_hash') != config.fingerprint or not 0 <= time.time()-prepared.get('prepared_at', 0) < 60:
        raise ValueError('检查已过期或方案变化，请重新准备')
    start = prepared['expected_start']
    target = list(start)
    target[0] += math.radians(.5)
    permit = OnsitePermit(config.fingerprint, (1,), time.monotonic()+15, evidence, (1,), True, True)
    intent = MotionIntent([target], 'onsite-single-axis', f'onsite-j1-{time.time_ns()}', expected_start=start)
    executor = HardwareExecutor(config, SeeedTransport())
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: executor.cancel())
    executor.heartbeat(executor.session, 0, True)
    executor.start(intent, permit)
    seq = 0
    try:
        # Foreground operator session; terminating it cancels this one task.
        # This is a software heartbeat, never a substitute for physical E-stop.
        while executor.worker.is_alive():
            seq += 1
            executor.heartbeat(executor.session, seq, True)
            executor.worker.join(.1)
    finally:
        executor.cancel()
        executor.worker.join()
    result = executor.status()
    result['backend'] = 'seeed-sdk-p4-guarded'
    result['goal_deg'] = math.degrees(target[0])
    result['parameter_source'] = 'explicit onsite trial proposal; not a device backup'
    result['command_log'] = executor.frames
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proposal', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument('--describe', action='store_true')
    actions.add_argument('--prepare', action='store_true')
    actions.add_argument('--execute', action='store_true')
    parser.add_argument('--prepared', type=Path)
    parser.add_argument('--onsite-evidence')
    parser.add_argument('--stop-tested', '--power-tested', dest='stop_tested', action='store_true', help='现场已核实 J1 失能效果；不据此宣称已验收 MIT 保持力矩')
    parser.add_argument('--operator-present', action='store_true')
    parser.add_argument('--power-cut-ready', action='store_true')
    parser.add_argument('--supported', action='store_true', help='已确认本轮去使能不会导致机械臂下落')
    args = parser.parse_args()
    raw = json.loads(args.proposal.read_text())
    if raw.get('operation') != 'j1-positive-half-degree' or raw.get('approval') != 'pending-human-onsite-confirmation':
        raise ValueError('需要本轮待确认 J1 +0.5° 方案')
    config = Envelope.parse(raw['envelope'])
    if any((a.v_max, a.a_max, a.jerk_max, a.error_max, a.torque_max) !=
           (.02, .04, .2, math.radians(1), .2) for a in config.axes):
        raise ValueError('本轮仅允许明确列出的保守试验包络')
    if args.execute and not all((args.prepared, args.onsite_evidence and args.onsite_evidence.strip(), args.stop_tested,
                                 args.operator_present, args.power_cut_ready, args.supported)):
        parser.error('执行需新鲜起点、当次现场证据及人员/断电/支撑/电源测试确认；默认不执行')
    if args.describe:
        result = dict(config_hash=config.fingerprint, envelope=asdict(config), hardware_writes=0, approval=raw['approval'])
    elif args.prepare:
        result = prepare(config, SeeedTransport())
        result.update(config_hash=config.fingerprint, prepared_at=time.time())
    else:
        result = execute(config, json.loads(args.prepared.read_text()), args.onsite_evidence)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False))
    return int(result.get('ok') is False or result.get('state') in ('FAULT', 'CANCELLED'))


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(1)
