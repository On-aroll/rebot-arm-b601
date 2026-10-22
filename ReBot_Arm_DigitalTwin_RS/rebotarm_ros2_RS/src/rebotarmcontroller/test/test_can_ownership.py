"""Cold-kernel ownership checks with an unbound fake socket; no real CAN."""
from pathlib import Path

import pytest

from rebotarmcontroller.csp_runtime import rebot_mit_hardware as module


@pytest.mark.parametrize('foreign',[False,True])
def test_cold_monitor_is_initialized_then_foreign_receivers_are_still_rejected(monkeypatch,tmp_path,foreign):
    opened=[];ready=[False];closed=[False];inspected=[]
    read_text=Path.read_text
    def read(path,*args,**kwargs):
        if path.parent!=Path('/proc/net/can'):return read_text(path,*args,**kwargs)
        if not ready[0]:raise FileNotFoundError(str(path))
        assert closed[0], 'The initializer must be closed before inspecting receivers.'
        inspected.append(path.name)
        return 'can0 0 0 0 0 0 foreign' if foreign and path.name=='rcvlist_all' else ''
    class Socket:
        def __enter__(self):ready[0]=True;return self
        def __exit__(self,*args):closed[0]=True
        # Deliberately no bind/send API: initialization cannot use a CAN device.
    def socket(*args):opened.append(args);return Socket()
    monkeypatch.setattr(Path,'read_text',read)
    monkeypatch.setattr(module.socket,'socket',socket)
    owner=module.CanOwnership(directory=tmp_path)
    try:
        if foreign:
            with pytest.raises(RuntimeError,match='其他接收者'):owner.acquire()
            assert owner.handle is None
        else:
            owner.acquire()
            assert owner.handle is not None and owner.owned is None
        assert opened==[(module.socket.AF_CAN,module.socket.SOCK_RAW,module.socket.CAN_RAW)]
        assert len(inspected)==6
    finally:owner.close()


def test_monitor_still_missing_after_initialization_fails_closed(monkeypatch,tmp_path):
    read_text=Path.read_text
    def read(path,*args,**kwargs):
        if path.parent==Path('/proc/net/can'):raise FileNotFoundError(str(path))
        return read_text(path,*args,**kwargs)
    class Socket:
        def __enter__(self):return self
        def __exit__(self,*args):pass
    monkeypatch.setattr(Path,'read_text',read)
    monkeypatch.setattr(module.socket,'socket',lambda *args:Socket())
    owner=module.CanOwnership(directory=tmp_path)
    with pytest.raises(RuntimeError,match='无法初始化.*所有权监视'):owner.acquire()
    assert owner.handle is None


def test_warm_monitor_does_not_open_a_socket(monkeypatch,tmp_path):
    read_text=Path.read_text
    def read(path,*args,**kwargs):
        if path.parent==Path('/proc/net/can'):return ''
        return read_text(path,*args,**kwargs)
    def unexpected(*args):raise AssertionError('Warm monitor does not need initialization.')
    monkeypatch.setattr(Path,'read_text',read)
    monkeypatch.setattr(module.socket,'socket',unexpected)
    owner=module.CanOwnership(directory=tmp_path)
    try:owner.acquire();assert owner.handle is not None
    finally:owner.close()
