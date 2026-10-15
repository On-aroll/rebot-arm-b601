"""Private-worker integration fixture: virtual motors only, no SocketCAN.

The parent test substitutes this file through MotorPowerAPI's existing Popen
seam. Production worker code and real Pinocchio FK still run unchanged.
"""
import json
import time
from unittest.mock import patch

from test_native_csp_offline import NativeController
from test_mit_hardware_offline import Owner
import rebot_native_xyz as native
from rebot_native_csp import CspTransport,probe,runtime_for_axis
import rebot_power_worker as worker


class WallClock:
    def __call__(self): return time.monotonic()

    @property
    def t(self): return time.monotonic()

    @t.setter
    def t(self,value): time.sleep(max(0.,value-time.monotonic()))


def main():
    message=json.loads(worker.sys.stdin.readline())
    operation=message.get('operation')
    if operation not in ('native-xyz','probe-csp-j6','csp-j6-trial'):
        raise ValueError('fixture permits only existing native XYZ or fixed J6')
    sdk=NativeController(WallClock())
    register=sdk.add_robstride_motor
    def initialized_motor(i,host,model):
        motor=register(i,host,model);motor.q=message.get('expected_start',[0.]*6)[i-1]
        return motor
    sdk.add_robstride_motor=initialized_motor
    transport=native.NativeXYZTransport
    emit=worker.emit
    def marked_emit(data):
        if data.get('kind')=='result':
            data=dict(data,virtual_sdk=True,virtual_sdk_calls=sdk.calls)
        emit(data)
    with patch('motorbridge.Controller',side_effect=AssertionError('real CAN forbidden')), \
         patch.object(native,'NativeXYZTransport',lambda:transport(factory=lambda **_:sdk,ownership=Owner())), \
         patch.object(worker,'emit',marked_emit):
        if operation=='probe-csp-j6':
            from rebot_mit_hardware import Envelope
            emit(dict(kind='probe',**probe(Envelope.parse(message['config']),
                CspTransport(axis=6,runtime=runtime_for_axis(6),factory=lambda **_:sdk,ownership=Owner()))))
        elif operation=='csp-j6-trial':
            worker.run_trial(message,bus=CspTransport(axis=6,runtime=runtime_for_axis(6),factory=lambda **_:sdk,ownership=Owner()))
        else:worker.run_trial(message)


if __name__=='__main__':main()
