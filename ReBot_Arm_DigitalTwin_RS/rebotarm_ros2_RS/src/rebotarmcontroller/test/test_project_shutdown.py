"""No physical SDK: atomic readonly exit gate and racing reacquire prevention."""
import threading

import pytest
from test_csp_twin import fixture, wait_until

@pytest.mark.parametrize('phase',['ACTIVE','HOLDING','STARTING','STOPPING','RECOVERY','FEEDBACK_WAIT'])
def test_shutdown_refuses_nonreadonly_without_disable_or_cancel(fixture,phase):
 hw,sdk=fixture;hw.phase=phase;before=list(sdk.calls)
 with pytest.raises(RuntimeError):hw.seal_readonly_shutdown()
 assert not hw.shutdown_sealed and sdk.calls==before and not hw.closing.is_set()
 hw.phase='READONLY'

def test_shutdown_seals_prepare_and_acquire_atomically_and_never_writes(fixture):
 hw,sdk=fixture;before=list(sdk.calls)
 hw.seal_readonly_shutdown()
 assert hw.shutdown_sealed and sdk.calls==before and not hw.closing.is_set()
 with pytest.raises(RuntimeError,match='正在结束'):hw.prepare()
 with pytest.raises(RuntimeError,match='正在结束'):hw.teleop_acquire()

def test_prepared_lease_stale_feedback_or_failed_restore_cannot_exit(fixture):
 hw,sdk=fixture;hw.prepare()
 with pytest.raises(RuntimeError):hw.seal_readonly_shutdown()
 hw.disable();old=hw._feedback_refreshed_at;hw._feedback_refreshed_at=-100
 with pytest.raises(RuntimeError):hw.seal_readonly_shutdown()
 hw._feedback_refreshed_at=old;hw.last_report={'cleanup_ok':False}
 with pytest.raises(RuntimeError):hw.seal_readonly_shutdown()
 assert not hw.shutdown_sealed
 hw.last_report=None


def test_supported_shutdown_preempts_motion_and_disables_without_restoring(fixture):
 hw,sdk=fixture
 hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
 wait_until(lambda:hw.motion_ready)
 gripper=hw.gripper._motor()
 gripper.enabled=True
 result=hw.request_shutdown_stop().result(timeout=4)
 assert result['ok'] and set(result['states'])=={str(i) for i in range(1,8)}
 assert all(not motor.enabled for motor in sdk.motors.values())
 first_disable=next(i for i,c in enumerate(sdk.calls) if c[0]=='disable' and i>max(
     k for k,v in enumerate(sdk.calls) if v[0]=='enable'))
 assert all(c[0]=='disable' for c in sdk.calls[first_disable:])
 assert hw.shutdown_sealed and not hw.armed and not hw.session and hw.recovery_required
 with pytest.raises(RuntimeError,match='正在结束'):hw.request_reset()


def test_supported_shutdown_allows_mode_zero_faults_and_preserves_recovery(fixture):
 hw,sdk=fixture
 existing={'reason':'未完成参数恢复'}
 hw.recovery_required=existing
 sdk.faults[2]=0x12
 before=len(sdk.calls)
 request=hw.request_shutdown_stop()
 result=request.result(timeout=3)
 assert result['ok'] and result['states']['2']['fault_bits']==0x12
 assert result['states']['2']['mode_state']==0
 assert hw.recovery_required is existing
 assert hw.request_shutdown_stop() is request
 assert sdk.calls[before:]==[('disable',i) for i in range(1,7)]


def test_supported_shutdown_keeps_driver_if_any_axis_still_active(fixture):
 hw,sdk=fixture
 sdk.bad_stop=True
 result=hw.request_shutdown_stop().result(timeout=3)
 assert not result['ok'] and set(result['errors'])=={str(i) for i in range(1,7)}
 assert hw.worker.is_alive() and hw.shutdown_sealed
 sdk.bad_stop=False
 assert hw.request_shutdown_stop().result(timeout=3)['ok']


def test_supported_shutdown_interrupts_a_reset_before_parameter_restore(fixture):
 hw,sdk=fixture
 reading,resume=threading.Event(),threading.Event()
 read=hw._read
 def pause_reset(*args,**kwargs):
  if hw.phase=='RESETTING':
   reading.set()
   assert resume.wait(3)
  return read(*args,**kwargs)
 hw._read=pause_reset
 hw.request_reset()
 try:
  assert reading.wait(3)
  stop=hw.request_shutdown_stop()
 finally:
  resume.set()
 assert stop.result(timeout=3)['ok']
 assert hw.reset_status['status']=='failed' and '结束项目' in hw.reset_status['message']
 assert all(c[0]=='disable' for c in sdk.calls)
