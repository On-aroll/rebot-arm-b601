"""Restore-only reset, executed by the existing CAN owner. Never enables a drive."""
import hashlib
import json
import math
from pathlib import Path

from .csp_runtime.rebot_native_xyz import AXES, MODE, FULL_MODE, SMOOTH_FULL_MODE, OFFICIAL_FULL_MODE, LIVE_FULL_MODE, runtime_for_mode, proposal
from .csp_runtime.rebot_native_csp import PROTECTED
from .csp_runtime.rebot_mit_hardware import Envelope

RUNTIME_KEYS = (0x7005, 0x7017, 0x7018, 0x700B)


def reset_source(hw):
    """Follow our own recovery journals back to the actual pre-session snapshot."""
    path = (hw.recovery_required or {}).get('report')
    source = (getattr(hw,'_reset_original',None) or hw.last_report) if hw.recovery_required else None
    seen = set()
    while path:
        candidate = Path(path).resolve()
        if (not hw.output or candidate.parent != hw.output.resolve()
                or not candidate.name.startswith('csp-session-') or candidate in seen):
            raise ValueError('恢复记录来源无效；保留锁定')
        seen.add(candidate)
        source = json.loads(candidate.read_text())
        if source.get('backend') != 'csp-control-reset-v1':
            return source, str(candidate), hashlib.sha256(candidate.read_bytes()).hexdigest()
        path = source.get('source_report')
        if not path:
            # This earlier reset had no runtime parameters to restore.
            if source.get('snapshot_required'):
                raise ValueError('缺少事故前完整参数快照；不能用默认值覆盖')
            return None, None, None
    return source, None, None


def validate_snapshot(source, limits):
    if not source or source.get('backend') != 'csp-twin-v1':
        raise ValueError('缺少事故前完整参数快照；不能用默认值覆盖')
    mode = (source.get('control_settings') or {}).get('mode')
    if mode not in ('small', 'full'):
        raise ValueError('原控制方案不可识别')
    # Match the exact historical envelope. A v2 speed extension must never
    # reinterpret a v1 accident snapshot or widen its allowed restore values.
    versions = (FULL_MODE,SMOOTH_FULL_MODE,OFFICIAL_FULL_MODE,LIVE_FULL_MODE) if mode == 'full' else (MODE,)
    version = next((v for v in versions if source.get('config_fingerprint') ==
        Envelope.parse(proposal(limits,mode=v)[0]).fingerprint),None)
    if version is None:
        raise ValueError('原参数快照与当前机械臂控制方案不符')
    snapshot = source.get('register_snapshot') or {}
    if set(snapshot) != {str(i) for i in AXES}:
        raise ValueError('事故前六轴参数快照不完整')
    originals = {}
    for i in AXES:
        values = snapshot[str(i)]
        original = {key: values[hex(key)] for key in (*RUNTIME_KEYS, *PROTECTED) if hex(key) in values}
        if len(original) != len(RUNTIME_KEYS)+len(PROTECTED) or any(
                isinstance(v, bool) or not isinstance(v, (float,int)) or not math.isfinite(v) for v in original.values()):
            raise ValueError(f'J{i} 原参数快照无效')
        runtime = runtime_for_mode(version,original=original)[i]
        if original[0x7017] <= 0: raise ValueError(f'J{i} 原始限速无效')
        if original[0x7005] != 0 or any(original[k] < runtime[k] for k in RUNTIME_KEYS if k != 0x7005):
            raise ValueError(f'J{i} 原参数不符合会话前检查')
        originals[i] = original
    return originals, version


def check_current_registers(bus, originals, check):
    """Check every axis before writing any: never overwrite foreign settings/PID."""
    observed = {}
    for i, drive in bus.drives.items():
        observed[str(i)] = {}
        for key in (*RUNTIME_KEYS, *PROTECTED):
            check(); bus.ownership.check()
            value = drive.read_register(key)
            observed[str(i)][hex(key)] = value
            if key in PROTECTED:
                matches = value == originals[i][key]
            else:
                matches = any(math.isclose(value, allowed, rel_tol=1e-6, abs_tol=1e-6)
                    for allowed in (originals[i][key], drive.runtime[key]))
            if not matches:
                raise ValueError(f'J{i} {key:#x} 与原快照/本会话临时值不符；未覆盖未知参数')
    return observed
