"""No physical SDK: atomic readonly exit gate and racing reacquire prevention."""
import pytest
from test_csp_twin import fixture

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
