"""Explicit historical-baseline review for mode-only maintenance. Import is inert."""
import hashlib
import json
import math
from pathlib import Path
import time

from .csp_reset import validate_snapshot, RUNTIME_KEYS, PROTECTED
from .csp_runtime.rebot_native_xyz import AXES

KEYS = (*RUNTIME_KEYS, *PROTECTED)
CONFIRMATIONS = {'same_device_no_external_changes', 'supported_disabled', 'restore_six_modes_to_zero'}


def pinned_json(root, filename, digest, prefix):
    path = Path(filename).resolve()
    if (path.parent != root.resolve() or not path.name.startswith(prefix)
            or path.suffix != '.json' or not isinstance(digest,str) or len(digest) != 64):
        raise ValueError('模式恢复来源必须是本硬件目录内明确指定且带哈希的记录')
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != digest:
        raise ValueError('模式恢复来源哈希已改变；未采纳该来源')
    return json.loads(raw), str(path)


def validate_observed(originals, observed, *, require_zero=False):
    if not isinstance(observed,dict) or set(observed) != {str(i) for i in AXES}:
        raise ValueError('模式恢复需要完整六轴读数')
    for axis in AXES:
        row = observed[str(axis)]
        if not isinstance(row,dict) or set(row) != {hex(k) for k in KEYS}:
            raise ValueError(f'J{axis} 模式恢复读数不是完整八项')
        for index in KEYS:
            value = row[hex(index)]
            if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value):
                raise ValueError(f'J{axis} {index:#x} 模式恢复读数无效')
            if index == 0x7005:
                if type(value) is not int or value not in ((0,) if require_zero else (0,5)):
                    raise ValueError(f'J{axis} 模式不是本次允许的0/5；不覆盖未知模式')
            elif value != originals[axis][index]:
                raise ValueError(f'J{axis} {index:#x} 与已评审原值不一致；仅模式恢复不修改此项')


def validate_review(hw, request):
    expected = {'op','id','stamp_ms','reference_report','reference_sha256','audit_report',
                'audit_sha256','recovery_report','recovery_sha256','confirmations'}
    if set(request) != expected or request['op'] != 'reviewed_mode_restore':
        raise ValueError('模式恢复需要明确的版本化来源、哈希及确认字段')
    confirms = request['confirmations']
    if (not isinstance(confirms,dict) or set(confirms) != CONFIRMATIONS
            or any(value is not True for value in confirms.values())):
        raise ValueError('须明确确认设备未更换/无外部改动、已支撑失能及本次六轴模式写入')
    stamp = request['stamp_ms']
    if (isinstance(stamp,bool) or not isinstance(stamp,(int,float)) or not math.isfinite(stamp)
            or not 0 <= time.time()*1000-stamp <= 3000):
        raise ValueError('模式恢复请求已过期；须重新提交当次确认')
    if hw.output is None or not hw.recovery_required:
        raise ValueError('模式恢复只用于有持久记录的当前恢复锁')
    source, reference_path = pinned_json(hw.output,request['reference_report'],request['reference_sha256'],'csp-session-')
    if not source.get('cleanup_ok') or not (source.get('restoration') or {}).get('ok'):
        raise ValueError('所选历史基准必须有已完成恢复的记录')
    originals, version = validate_snapshot(source,hw.teleop_limits)
    recovery, recovery_path = pinned_json(hw.output,request['recovery_report'],request['recovery_sha256'],'csp-session-')
    if (recovery_path != str(Path(hw.recovery_required.get('report','')).resolve())
            or recovery.get('cleanup_ok')):
        raise ValueError('当前恢复锁记录已经改变，请重新评审')
    audit, audit_path = pinned_json(hw.output,request['audit_report'],request['audit_sha256'],'register-audit-')
    if (audit.get('schema') != 'rebot-register-audit-v1' or audit.get('complete') is not True
            or audit.get('error') is not None or audit.get('hardware_writes') != 0
            or audit.get('pre_session_snapshot') is not False
            or str(Path((audit.get('recovery_required') or {}).get('report','')).resolve()) != recovery_path):
        raise ValueError('需要对应当前恢复锁的完整只读诊断，不能伪装成事前快照')
    validate_observed(originals,audit.get('observed_registers'))
    evidence = dict(reference_report=reference_path,reference_sha256=request['reference_sha256'],
        audit_report=audit_path,audit_sha256=request['audit_sha256'],
        previous_recovery_report=recovery_path,previous_recovery_sha256=request['recovery_sha256'],
        confirmations=dict(confirms),source_kind='explicitly reviewed historical baseline',
        original_incident_source_known=False,device_identity_evidence='human attestation; no serial verification')
    return source, originals, version, evidence


def read_reviewed_registers(bus, originals, check, *, require_zero=False):
    observed = {}
    for axis in AXES:
        observed[str(axis)] = {}
        for index in KEYS:
            check();bus.ownership.check()
            observed[str(axis)][hex(index)] = bus.drives[axis].read_register(index)
            check()
    validate_observed(originals,observed,require_zero=require_zero)
    return observed
