"""Reset completion must replace old recovery text, without clearing faults."""
from test_twin_jog import console


def test_completed_reset_replaces_stale_recovery_message_once():
    twin, hw, clock, writes, pauses, stops, _ = console()
    twin.session = ''
    twin.mode = 'IDLE'
    twin.message = '上次停止/参数恢复未确认，已锁定实机使能'
    hw.error = ''
    hw.reset_status = dict(id='reset-1', status='succeeded', message='控制已重置：已失能，参数核对完成')
    twin.tick()
    assert twin.message == hw.reset_status['message']
    twin.message = '用户刚刚切换控制范围'
    clock[0] += .05
    twin.tick()
    assert twin.message == '用户刚刚切换控制范围'
    assert writes == pauses == stops == []


def test_new_fault_after_a_completed_reset_is_not_hidden_by_old_success():
    twin, hw, clock, writes, _, _, _ = console()
    twin.session = ''
    hw.reset_status = dict(id='reset-1', status='succeeded', message='已重置')
    twin.tick()
    hw.error = '当前 CAN 反馈异常'
    clock[0] += .05
    twin.tick()
    assert twin.message == hw.error
    assert not writes
