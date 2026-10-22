"""Exercise lifecycle safety without constructing a CAN controller."""
import threading
from types import SimpleNamespace

from rebotarmcontroller.hardware_manager import HardwareManager
from rebotarmcontroller.ros_services import ArmServices


def test_connect_is_readonly_and_does_not_start_control():
    calls = []
    manager = HardwareManager.__new__(HardwareManager)
    manager._cmd_lock = threading.RLock()
    manager._connected = False
    manager._enabled = False
    manager._robot = SimpleNamespace(
        has_gripper=False, connect=lambda: calls.append("connect"),
    )
    manager._start_endpos_loop = lambda: calls.append("start-control")
    manager.connect()
    manager.connect()
    assert calls == ["connect"]
    assert manager._connected and not manager._enabled


def test_shutdown_disables_without_homing_even_with_legacy_option():
    calls = []
    manager = HardwareManager.__new__(HardwareManager)
    manager._cmd_lock = threading.RLock()
    manager._connected = True
    manager.disable = lambda: calls.append("disable")
    manager.safe_home = lambda: calls.append("home")
    manager.set_state_machine = lambda state: calls.append(state)
    manager._robot = SimpleNamespace(disconnect=lambda: calls.append("disconnect"))
    manager.shutdown(disable_after_safe_home=True)
    assert calls == ["disable", "disconnect", "IDLE"]
    assert not manager._connected and not manager._enabled


def test_disable_service_never_calls_safe_home():
    calls = []
    services = ArmServices.__new__(ArmServices)
    services._hardware = SimpleNamespace(
        enabled=True, disable=lambda: calls.append("disable"),
        is_near_safe_home=lambda: False,
    )
    services._safe_home_and_verify = lambda: calls.append("home")
    services._node = SimpleNamespace(publish_arm_status=lambda **kwargs: None)
    response = services.disable(None, SimpleNamespace())
    assert response.success
    assert calls == ["disable"]


def test_disabling_gravity_compensation_never_enters_position_hold():
    calls = []
    manager = HardwareManager.__new__(HardwareManager)
    manager._cmd_lock = threading.RLock()
    manager._gravity_comp_active = True
    manager._control_output_enabled = True
    manager._gripper_assist_active = True
    manager._endpos_ctrl = SimpleNamespace(_running=True)
    manager.stop_motion = lambda: calls.append("stop-motion")
    manager._robot = SimpleNamespace(
        stop_control_loop=lambda: calls.append("stop-loop"),
        disable_all=lambda: calls.append("disable-all"),
    )
    manager.set_state_machine = lambda state: calls.append(state)
    manager.disable()
    assert calls == ["stop-motion", "stop-loop", "disable-all", "IDLE"]
    assert not manager._gravity_comp_active
    assert not manager._control_output_enabled
    assert not manager._gripper_assist_active
