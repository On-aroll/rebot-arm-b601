"""Same CAN owner, read-only diagnostics; in-memory SDK, no physical CAN."""
import copy
import json
import threading
from types import SimpleNamespace as NS

import pytest

from test_csp_twin import fixture, wait_until
from test_twin_program import console
from rebotarmcontroller.csp_reset import RUNTIME_KEYS, PROTECTED
from rebotarmcontroller.twin_control import TwinControl


def test_recovery_audit_reads_all_registers_on_owner_without_replacing_history(fixture,tmp_path):
    hw,sdk=fixture;hw.output=tmp_path
    source=tmp_path/'csp-session-original.json';source.write_text('{"cleanup_ok":false}')
    before=source.read_bytes();hw.recovery_required={'report':str(source),'reason':'unverified'}
    hw.phase='RECOVERY'
    for motor in sdk.motors.values():motor.regs[0x7005]=5
    sdk.motors[3].regs[0x701E]=123.  # Unknown finite PID is evidence, never overwritten.
    recovery=copy.deepcopy(hw.recovery_required);drives=hw.bus.drives
    registers={i:dict(m.regs) for i,m in sdk.motors.items()};reads=[]
    for axis,motor in sdk.motors.items():
        f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
        def read_f32(index,*args,read=f32,axis=axis):
            reads.append((threading.current_thread().name,axis,index));return read(index,*args)
        def read_i8(index,*args,read=i8,axis=axis):
            reads.append((threading.current_thread().name,axis,index));return read(index,*args)
        motor.robstride_get_param_f32_host_id=read_f32;motor.robstride_get_param_i8=read_i8
    result=hw.request_register_audit()
    assert hw.request_register_audit()['id']==result['id']
    with pytest.raises(RuntimeError,match='只读核查'):hw.prepare()
    with pytest.raises(RuntimeError,match='只读核查'):hw.teleop_acquire()
    wait_until(lambda:hw.register_audit_status['status']=='succeeded',timeout=8)
    report=json.loads(open(hw.register_audit_status['report']).read())
    assert report['complete'] and len(report['samples'])==48
    for axis in range(1,7):
        assert report['observed_registers'][str(axis)]=={hex(k):registers[axis][k] for k in (*RUNTIME_KEYS,*PROTECTED)}
    assert {r[0] for r in reads}=={'csp-can-owner'}
    assert not report['pre_session_snapshot'] and not report['identity_verified'] and not report['recovery_authorized']
    assert report['hardware_writes']==0 and sdk.calls==[]
    assert hw.phase=='RECOVERY' and hw.recovery_required==recovery and hw.bus.drives is drives
    assert {i:m.regs for i,m in sdk.motors.items()}==registers
    assert source.read_bytes()==before and len(list(tmp_path.glob('csp-session-*')))==1


@pytest.mark.parametrize('state',['prepared','owned','active','holding','reset','j7','stale','sealed'])
def test_conflicting_state_refuses_audit_without_writes(fixture,state):
    hw,sdk=fixture
    if state=='prepared':hw.armed=True
    elif state=='owned':hw.session='owner'
    elif state in ('active','holding'):hw.phase=state.upper()
    elif state=='reset':hw.reset_pending=True
    elif state=='j7':hw.gripper.authorized=True
    elif state=='stale':hw._feedback_refreshed_at=hw.clock()+10
    elif state=='sealed':hw.shutdown_sealed=True
    with hw.lock:
        try:
            with pytest.raises(RuntimeError):hw.request_register_audit()
            assert hw._register_audit is None and sdk.calls==[]
        finally:
            hw.armed=False;hw.session='';hw.phase='READONLY';hw.reset_pending=False
            hw.gripper.authorized=False;hw.shutdown_sealed=False


@pytest.mark.parametrize('value',[float('nan'),True])
def test_invalid_register_stops_with_partial_diagnostic_and_keeps_recovery(fixture,tmp_path,value):
    hw,sdk=fixture;hw.output=tmp_path;hw.phase='RECOVERY';hw.recovery_required={'reason':'keep'}
    sdk.motors[1].regs[0x701E]=value
    hw.request_register_audit()
    wait_until(lambda:hw.register_audit_status['status']=='failed')
    report=json.loads(open(hw.register_audit_status['report']).read())
    assert not report['complete'] and len(report['samples'])==4
    assert hw.phase=='RECOVERY' and hw.recovery_required=={'reason':'keep'} and sdk.calls==[]
    assert hw.worker.is_alive() and hw._register_audit is None


def test_stop_cancels_queued_audit_without_waiting_or_resuming_it(fixture,tmp_path):
    hw,sdk=fixture;hw.output=tmp_path
    with hw.lock:
        hw.request_register_audit();hw.disable()
        assert hw._register_audit is None and hw.register_audit_status['status']=='cancelled'
        assert not list(tmp_path.glob('register-audit-*')) and sdk.calls==[]


def test_feedback_loss_cancels_audit_and_recovery_does_not_restart_it(fixture,tmp_path):
    hw,sdk=fixture;hw.output=tmp_path
    original=sdk.motors[4].robstride_get_param_f32_host_id
    with hw.lock:
        hw.request_register_audit()
        sdk.motors[4].robstride_get_param_f32_host_id=lambda index,*args:float('nan') if index==0x7019 else original(index,*args)
    wait_until(lambda:hw.phase=='FEEDBACK_WAIT')
    assert hw._register_audit is None and hw.register_audit_status['status']=='cancelled'
    sdk.motors[4].robstride_get_param_f32_host_id=original
    wait_until(lambda:hw.phase=='READONLY')
    assert hw.register_audit_status['status']=='cancelled' and not hw.armed and not hw.session and sdk.calls==[]


def test_deadline_failure_keeps_originals_and_does_not_reset(fixture,tmp_path):
    hw,sdk=fixture;hw.output=tmp_path
    with hw.lock:
        hw.request_register_audit();hw._register_audit['deadline']=hw.clock()-1
    wait_until(lambda:hw.register_audit_status['status']=='failed')
    assert '15秒' in hw.register_audit_status['message'] and sdk.calls==[]
    assert not hw.reset_pending and not hw._register_audit


def test_ros_service_queues_only_and_does_not_release_an_existing_owner(tmp_path):
    t,hw,*_=console(tmp_path);queued=[]
    hw.request_register_audit=lambda:queued.append(True) or {'status':'queued','id':'audit'}
    t.session='owner';response=NS()
    TwinControl.audit_registers(t,None,response)
    assert not response.success and not queued and t.session=='owner'
    t.session='';TwinControl.audit_registers(t,None,response)
    assert response.success and json.loads(response.message)['id']=='audit' and queued==[True]


def test_report_save_failure_is_visible_and_does_not_change_hardware(fixture,tmp_path):
    hw,sdk=fixture;hw.output=tmp_path/'missing'
    with hw.lock:
        hw.request_register_audit();hw._register_audit['deadline']=hw.clock()-1
    wait_until(lambda:hw.register_audit_status['status']=='failed')
    assert '保存失败' in hw.register_audit_status['message']
    assert hw.register_audit_status['report'] is None and sdk.calls==[] and hw.worker.is_alive()
