"""Private pipe worker for explicit J1 or XYZ trials with the audited SDK.

No socket server; importing this module does not connect. Only a parent-issued
start message after the operator confirms a prepared plan can enable its axes.
"""
from dataclasses import asdict
import json
import signal
import sys
import threading
import time
from rebot_mit_hardware import Envelope, HardwareExecutor, HoldIntent, OnsitePermit, SdkTransport

PREFIX = 'REBOT_POWER_JSON '


def emit(value):
    print(PREFIX + json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)


def probe(config, bus):
    executor = HardwareExecutor(config, bus)
    readings = []
    try:
        bus.open_readonly()
        for _ in range(3):
            samples = bus.read(tuple(range(1, 7)), config.policy)
            readings.append({str(i): asdict(s) for i, s in samples.items()})
            # Preserve the raw readings even when they prevent the trial.
        reasons = []
        for reading in readings:
            from rebot_mit_hardware import Sample
            samples = {int(i): Sample(**s) for i, s in reading.items()}
            # Completed earlier sweeps are historical evidence, not start authority.
            for i, s in samples.items():
                axis = config.axes[i-1]
                if s.mode != 0: reasons.append(f'J{i} 实际模式不是 MIT；未自动切换')
                if not axis.q_min <= s.q <= axis.q_max:
                    import math
                    reasons.append(f'J{i} 原始角度 {math.degrees(s.q):.3f}°，模型范围 {math.degrees(axis.q_min)+0.:.3f}°～{math.degrees(axis.q_max):.3f}°；请现场核对姿态后重新检查，未修改零位或裁剪角度')
                if abs(s.qd) > config.policy.start_speed:
                    reasons.append(f'J{i} 速度读数 {s.qd:.4f} rad/s 超过当次静止阈值 {config.policy.start_speed:g}；请核对实物是否静止')
        return dict(ok=not reasons, reason='；'.join(dict.fromkeys(reasons)), feedback=readings[-1], readings=readings,
                    expected_start=[readings[-1][str(i)]['q'] for i in range(1,7)], hardware_writes=0)
    except Exception as e:
        return dict(ok=False, reason=str(e), feedback=readings[-1] if readings else {}, readings=readings, hardware_writes=0)
    finally:
        bus.close()


def run_trial(message, bus=None):
    gripper=message['operation']=='csp-j7-trial'
    if gripper:
        from rebot_gripper_csp import GripperEnvelope
        config=GripperEnvelope.parse(message['config'])
    else: config = Envelope.parse(message['config'])
    native_xyz=message['operation']=='native-xyz'
    point=message['operation'] in ('move-xyz','probe-startup') or native_xyz
    native=message['operation'] in ('csp-j3-trial','csp-j6-trial','csp-j7-trial')
    if not point and not native and (message['operation'] != 'hold-j1' or message.get('seconds') != 3):
        raise ValueError('仅支持 J1 当次 3 秒保持试验')
    if native_xyz:
        from rebot_native_xyz import NativeXYZExecutor,NativeXYZTransport
        from rebot_mit_hardware import MotionIntent
        axes=tuple(range(1,7))
        intent=MotionIntent([message['target']],'native-csp-xyz',message['request_id'],expected_start=message['expected_start'])
        executor=NativeXYZExecutor(config,bus or NativeXYZTransport())
    elif native:
        from rebot_native_csp import CspJ3Trial,CspJ6Trial,CspTransport,runtime_for_axis
        from rebot_mit_hardware import MotionIntent
        axis={'csp-j3-trial':3,'csp-j6-trial':6,'csp-j7-trial':7}[message['operation']]
        axes=(axis,)
        if gripper:
            from rebot_gripper_csp import GripperIntent,GripperTrial,GripperTransport
            intent=GripperIntent([message['target']],'native-csp-j7',message['request_id'],expected_start=message['expected_start'])
            executor=GripperTrial(config,bus or GripperTransport())
        else:
            intent=MotionIntent([message['target']],f'native-csp-j{axis}',message['request_id'],expected_start=message['expected_start'])
            cls=CspJ3Trial if axis==3 else CspJ6Trial
            executor=cls(config,bus or CspTransport(axis=axis,runtime=runtime_for_axis(axis)))
    elif point:
        from rebot_mit_hardware import MotionIntent, StartupObservationIntent
        from rebot_seeed_transport import SeeedTransport
        axes=tuple(range(1,7))
        intent_class=StartupObservationIntent if message['operation']=='probe-startup' else MotionIntent
        intent=intent_class([message['target']],'xyz',message['request_id'],expected_start=message['expected_start'])
        executor=HardwareExecutor(config,bus or SeeedTransport(position_window=True,diagnostic_axes=(3,) if config.gain_review else ()))
    else:
        axes=(1,)
        intent=HoldIntent(message['request_id'],message['expected_start'],seconds=3)
        executor=HardwareExecutor(config,bus or SdkTransport())
    permit_seconds=config.policy.task_timeout+10 if point or native else 10
    permit_class=OnsitePermit
    if gripper:
        from rebot_gripper_csp import GripperPermit
        permit_class=GripperPermit
    permit = permit_class(config.fingerprint, axes, time.monotonic()+permit_seconds,
        message['evidence'], axes, True, True,coordinated_start_checked=point)
    executor.heartbeat(executor.session, 0, True)
    for name in (signal.SIGTERM, signal.SIGINT):
        signal.signal(name, lambda *_: executor.cancel())

    def commands():
        try:
            for line in sys.stdin:
                body = json.loads(line)
                if body.get('action') == 'stop': executor.cancel()
                elif body.get('action') == 'heartbeat':
                    executor.heartbeat(executor.session, body['sequence'], body.get('focused') is True)
                else: raise ValueError('未知会话命令')
        except Exception:
            executor.cancel()
        finally:
            # Parent exit / simulator restart closes the pipe; never keep a target.
            executor.cancel()
    threading.Thread(target=commands, daemon=True, name='power-session-input').start()
    executor.start(intent, permit)
    while executor.worker.is_alive():
        emit(dict(kind='status', **executor.status()))
        executor.worker.join(.1)
    # Emit the trace once, after cleanup. Polling carries only the small status;
    # requests are host-side evidence, not per-frame acknowledgements by drives.
    emit(dict(kind='result', **executor.status(), command_log=dict(frames=executor.frames,registers=executor.bus.register_log) if native or native_xyz else executor.frames,
              expected_start=message['expected_start'], target=message.get('target'),
              drive_feedback_evidence=dict(axis=3,iq_register='0x701a',torque_register='0x302c',
                  calibrated=False,scope='raw drive reports, not independent torque measurement') if point and config.gain_review else None,
              command_evidence='host CSP register requests; explicit readbacks separately recorded' if native or native_xyz else 'host MIT requests; no per-frame drive ACK'))


def main():
    message = json.loads(sys.stdin.readline())
    if message.get('operation') == 'probe':
        emit(dict(kind='probe', **probe(Envelope.parse(message['config']), SdkTransport())))
    elif message.get('operation') == 'probe-csp-j7':
        from rebot_gripper_csp import GripperEnvelope,GripperTransport
        from rebot_native_csp import probe as csp_probe
        emit(dict(kind='probe', **csp_probe(GripperEnvelope.parse(message['config']),GripperTransport())))
    elif message.get('operation') in ('probe-csp-j3','probe-csp-j6'):
        from rebot_native_csp import CspTransport,probe as csp_probe,runtime_for_axis
        axis=3 if message['operation']=='probe-csp-j3' else 6
        emit(dict(kind='probe', **csp_probe(Envelope.parse(message['config']),CspTransport(axis=axis,runtime=runtime_for_axis(axis)))))
    else:
        run_trial(message)


if __name__ == '__main__':
    try: main()
    except Exception as error:
        emit(dict(kind='result', state='FAULT', reason=str(error), physical_acceptance=False))
        raise SystemExit(1)
