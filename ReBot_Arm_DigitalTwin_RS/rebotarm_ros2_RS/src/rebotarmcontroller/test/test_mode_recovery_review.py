"""Reviewed mode-only recovery against the in-memory SDK; never physical CAN."""
import hashlib
import json
import os
import stat
import time
from types import SimpleNamespace as NS

import pytest

from test_csp_twin import fixture, wait_until, Controller, Owner
from test_twin_program import console
from rebotarmcontroller.csp_hardware import CspHardware
from rebotarmcontroller.csp_recovery_review import KEYS, CONFIRMATIONS
from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZTransport
from rebotarmcontroller.twin_control import TwinControl


def setup_review(hw, sdk, root):
    hw.output = root
    original = {str(i):{hex(k):m.regs[k] for k in KEYS} for i,m in sdk.motors.items()}
    reference = root/'csp-session-001.json'
    reference.write_text(json.dumps(dict(backend='csp-twin-v1',cleanup_ok=True,
        restoration=dict(ok=True),control_settings=dict(mode='small'),
        config_fingerprint=hw.config.fingerprint,register_snapshot=original)))
    recovery = root/'csp-session-002.json'
    recovery.write_text(json.dumps(dict(backend='csp-control-reset-v1',cleanup_ok=False,
        source_report=None,snapshot_required=False,stop=dict(attempted_axes=list(range(1,7))))))
    with hw.lock:
        hw.recovery_required = dict(report=str(recovery),reason='missing original lineage')
        hw.phase = 'RECOVERY'
        for m in sdk.motors.values():m.regs[0x7005] = 5
    audit = root/'register-audit-001.json'
    audit.write_text(json.dumps(dict(schema='rebot-register-audit-v1',complete=True,error=None,
        hardware_writes=0,pre_session_snapshot=False,recovery_required=dict(report=str(recovery)),
        observed_registers={str(i):{hex(k):m.regs[k] for k in KEYS} for i,m in sdk.motors.items()})))
    request = dict(op='reviewed_mode_restore',id='review',stamp_ms=time.time()*1000,
        confirmations={k:True for k in CONFIRMATIONS})
    for key,path in [('reference',reference),('recovery',recovery),('audit',audit)]:
        request[key+'_report'] = str(path)
        request[key+'_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    return request


def finish(hw, request):
    hw.request_mode_restore(request)
    wait_until(lambda:not hw.reset_pending,timeout=6)
    return hw.last_report


def test_only_six_modes_written_history_preserved_and_restart_readonly(fixture,tmp_path):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path)
    history = {p:p.read_bytes() for p in tmp_path.glob('*.json')}
    before = {i:dict(m.regs) for i,m in sdk.motors.items()}
    report = finish(hw,request)
    assert report['cleanup_ok'],report
    assert sdk.calls == [('disable',i) for i in range(1,7)]+[('write',i,0x7005,0) for i in range(1,7)]
    assert report['backend']=='csp-reviewed-mode-restore-v1'
    assert not report['original_incident_source_known'] and 'register_snapshot' not in report
    assert report['reviewed_source']['confirmations']==request['confirmations']
    assert report['restore_scope']==['0x7005']
    assert len(report['verified_registers'])==6
    for i,m in sdk.motors.items():
        assert m.regs=={**before[i],0x7005:0}
        assert report['verified_registers'][str(i)]=={hex(k):m.regs[k] for k in KEYS}
    assert all(p.read_bytes()==raw for p,raw in history.items())
    assert hw.phase=='READONLY' and not hw.recovery_required and not hw.armed and not hw.session
    assert hw.target is None and not hw.bus.active
    hw.close()
    restarted = CspHardware([(-1,1)]*6,output=tmp_path,
        transport=NativeXYZTransport(factory=lambda **_:Controller(),ownership=Owner()))
    restarted.connect()
    try:assert restarted.phase=='READONLY' and not restarted.recovery_required
    finally:restarted.close()


@pytest.mark.parametrize('change',[*CONFIRMATIONS,'hash','path','stale','audit_binding','source','extra'])
def test_invalid_review_refused_before_any_hardware_call(fixture,tmp_path,change):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path)
    if change in CONFIRMATIONS:request['confirmations'][change]=False
    elif change=='hash':request['reference_sha256']='0'*64
    elif change=='path':request['reference_report']=str(tmp_path.parent/'csp-session-foreign.json')
    elif change=='stale':request['stamp_ms']-=4000
    elif change=='extra':request['write_other_parameters']=True
    else:
        key = 'audit' if change=='audit_binding' else 'reference'
        path = tmp_path/('register-audit-001.json' if key=='audit' else 'csp-session-001.json')
        data = json.loads(path.read_text())
        if key=='audit':data['recovery_required']['report']=str(tmp_path/'other.json')
        else:data['cleanup_ok']=False
        path.write_text(json.dumps(data));request[key+'_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    with pytest.raises(ValueError):hw.request_mode_restore(request)
    assert sdk.calls==[] and hw.recovery_required and not hw.reset_pending


@pytest.mark.parametrize('index,value',[(0x701e,31.),(0x7017,.2),(0x7005,3)])
def test_fresh_register_disagreement_blocks_all_mode_writes(fixture,tmp_path,index,value):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path)
    sdk.motors[6].regs[index]=value
    report = finish(hw,request)
    assert not report['cleanup_ok'] and hw.phase=='RECOVERY' and hw.recovery_required
    assert not any(c[0]=='write' for c in sdk.calls)
    assert sdk.motors[6].regs[index]==value


def test_failed_disable_confirmation_blocks_writes(fixture,tmp_path):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path);sdk.bad_stop=True
    report = finish(hw,request)
    assert not report['cleanup_ok'] and hw.recovery_required
    assert not any(c[0]=='write' for c in sdk.calls)


def test_continuing_motion_blocks_writes(fixture,tmp_path):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path)
    read = sdk.motors[1].robstride_get_param_f32_host_id
    def moving(index,*args):
        if index==0x7019:sdk.motors[1].q+=.01
        return read(index,*args)
    sdk.motors[1].robstride_get_param_f32_host_id=moving
    report = finish(hw,request)
    assert not report['cleanup_ok'] and hw.recovery_required
    assert '停稳' in report['cleanup_error']
    assert not any(c[0]=='write' for c in sdk.calls)


def test_journal_fsync_failure_prevents_even_disable(fixture,tmp_path,monkeypatch):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path)
    def fail(_):raise OSError('durability unavailable')
    monkeypatch.setattr('rebotarmcontroller.csp_hardware.os.fsync',fail)
    report = finish(hw,request)
    assert not report['cleanup_ok'] and hw.recovery_required and sdk.calls==[]


def test_stop_after_queue_cancels_mode_writes(fixture,tmp_path):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path)
    with hw.lock:
        hw.request_mode_restore(request);hw.disable()
    wait_until(lambda:not hw.reset_pending)
    assert hw.recovery_required and not hw.last_report['cleanup_ok']
    assert not any(c[0]=='write' for c in sdk.calls)


def test_changed_reference_after_queue_is_rejected_on_can_owner(fixture,tmp_path):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path)
    with hw.lock:
        hw.request_mode_restore(request)
        (tmp_path/'csp-session-001.json').write_text('{}')
    wait_until(lambda:not hw.reset_pending)
    assert hw.recovery_required and not hw.last_report['cleanup_ok'] and sdk.calls==[]


def test_partial_write_failure_keeps_lock_and_does_not_authorize_generic_reset(fixture,tmp_path):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path)
    def fail(*_):raise OSError('mode write unavailable')
    sdk.motors[3].robstride_write_param_i8=fail
    report = finish(hw,request)
    assert not report['cleanup_ok'] and hw.recovery_required
    assert sdk.motors[3].regs[0x7005]==5
    count = len([c for c in sdk.calls if c[0]=='write'])
    hw.request_reset();wait_until(lambda:not hw.reset_pending)
    assert not hw.last_report['cleanup_ok'] and hw.recovery_required
    assert len([c for c in sdk.calls if c[0]=='write'])==count


def test_mode_write_readback_failure_keeps_lock(fixture,tmp_path):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path)
    sdk.motors[6].robstride_write_param_i8=lambda *_:None
    report = finish(hw,request)
    assert not report['cleanup_ok'] and hw.recovery_required
    assert sdk.motors[6].regs[0x7005]==5 and hw.phase=='RECOVERY'


def test_final_directory_sync_failure_withdraws_success_on_disk(fixture,tmp_path,monkeypatch):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path);sync=os.fsync
    directory_calls=[]
    def fail_final_directory(fd):
        if stat.S_ISDIR(os.fstat(fd).st_mode):
            directory_calls.append(fd)
            if len(directory_calls)>=2:raise OSError('directory durability unavailable')
        return sync(fd)
    monkeypatch.setattr('rebotarmcontroller.csp_hardware.os.fsync',fail_final_directory)
    report = finish(hw,request)
    assert not report['cleanup_ok'] and hw.recovery_required and hw.phase=='RECOVERY'
    saved=json.loads(open(hw.recovery_required['report']).read())
    assert not saved['cleanup_ok'] and hw._unrestored_session()


@pytest.mark.parametrize('state',['prepared','owned','active','j7','audit'])
def test_other_ownership_refuses_maintenance(fixture,tmp_path,state):
    hw,sdk = fixture;request = setup_review(hw,sdk,tmp_path)
    with hw.lock:
        if state=='prepared':hw.armed=True
        elif state=='owned':hw.session='owner'
        elif state=='active':hw.phase='ACTIVE'
        elif state=='j7':hw.gripper.authorized=True
        else:hw.request_register_audit()
        try:
            with pytest.raises(RuntimeError):hw.request_mode_restore(request)
            assert sdk.calls==[] and not hw.reset_pending
        finally:
            hw.armed=False;hw.session='';hw.phase='RECOVERY';hw.gripper.authorized=False
            hw._cancel_register_audit('inert cleanup')


def test_command_preserves_exact_review_and_refuses_an_owner(tmp_path):
    t,hw,q,g,events,writes,*_ = console(tmp_path);queued=[]
    hw.request_mode_restore=lambda req:queued.append(req) or dict(status='queued',message='reviewed')
    request=dict(op='reviewed_mode_restore',id='test',stamp_ms=time.time()*1000,confirmations={})
    t.session='owner';TwinControl.command(t,NS(data=json.dumps(request)))
    assert not queued and not events[-1]['ok'] and not writes
    t.session='';TwinControl.command(t,NS(data=json.dumps(request)))
    assert queued==[request] and events[-1]['ok'] and not writes
    assert t.mode=='IDLE' and t.reference is None and t.goal is None
