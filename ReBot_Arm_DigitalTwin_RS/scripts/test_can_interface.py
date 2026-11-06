"""Offline regression for Jetson reboot CAN enumeration; never opens CAN."""
import unittest
from unittest.mock import patch

import can_interface
from can_interface import InterfaceChanged, describe, configuration_commands, require_no_receivers


def link(name, *, up=False, state="STOPPED", bitrate=0):
    return {"ifname": name, "flags": ["UP"] if up else [],
            "linkinfo": {"info_kind": "can", "info_data": {
                "state": state, "bittiming": {"bitrate": bitrate}}}}


class CanInterfaceTests(unittest.TestCase):
    def setUp(self):
        self.links = [link("can0", up=True, state="BUS-OFF", bitrate=1000000), link("can1")]
        self.drivers = {"can0": "mttcan", "can1": "peak_usb"}

    def test_reboot_does_not_treat_board_can0_as_usb(self):
        status = describe(self.links, self.drivers)
        self.assertFalse(status["ready"])
        self.assertEqual(status["interface"], "can1")
        self.assertIn("板载", status["message"])

    def test_repair_renames_only_known_idle_usb_and_bus_off_board(self):
        commands = configuration_commands(self.links, self.drivers)
        self.assertEqual(commands, [
            ["link", "set", "can0", "down"],
            ["link", "set", "can0", "name", "can_native0"],
            ["link", "set", "can1", "name", "can0"],
            ["link", "set", "can0", "type", "can", "bitrate", "1000000"],
            ["link", "set", "can0", "up"],
        ])

    def test_ready_usb_is_reused_without_reconfiguration(self):
        links = [link("can0", up=True, state="ERROR-ACTIVE", bitrate=1000000)]
        self.assertTrue(describe(links, {"can0": "peak_usb"})["ready"])
        self.assertEqual(configuration_commands(links, {"can0": "peak_usb"}), [])

    def test_usb_move_waits_for_reenumeration_and_uses_new_port(self):
        moved = {**link("can7"), "ifindex": 31, "parentdev": "2-3.4:1.0"}
        connected = [self.links[0], moved], {"can0": "mttcan", "can7": "peak_usb"}
        with patch.object(can_interface, "inventory", side_effect=[
            (self.links[:1], self.drivers), InterfaceChanged("USB 正在重新枚举"), connected,
        ]), patch.object(can_interface.time, "sleep"):
            links, drivers = can_interface.discover_usb()
        self.assertEqual(describe(links, drivers)["usb_port"], "2-3.4")
        self.assertIn(["link", "set", "can7", "name", "can0"], configuration_commands(links, drivers))

    def test_unidentified_can0_does_not_count_as_usb_feedback_interface(self):
        links = [link("can0", up=True, state="ERROR-ACTIVE", bitrate=1000000)]
        self.assertFalse(describe(links, {"can0": None})["ready"])
        with self.assertRaisesRegex(RuntimeError, "PEAK USB-CAN"):
            configuration_commands(links, {"can0": None})

    def test_replaced_interface_is_not_configured_after_rename(self):
        moved = {**link("can7"), "ifindex": 31}
        with patch.object(can_interface, "discover_usb", return_value=([moved], {"can7": "peak_usb"})), \
             patch.object(can_interface, "require_no_receivers"), \
             patch.object(can_interface.os, "geteuid", return_value=0), \
             patch.object(can_interface, "require_interface", side_effect=[None, InterfaceChanged("设备已改变")]) as check, \
             patch.object(can_interface.subprocess, "run") as run:
            with self.assertRaises(InterfaceChanged):
                can_interface.prepare(apply=True)
        run.assert_called_once_with([can_interface.IP, "link", "set", "can7", "name", "can0"], check=True)
        check.assert_called_with("can0", 31)

    def test_board_only_and_multiple_usb_adapters_are_rejected(self):
        self.assertFalse(describe(self.links[:1], self.drivers)["ready"])
        with self.assertRaisesRegex(RuntimeError, "USB-CAN"):
            configuration_commands(self.links[:1], self.drivers)
        self.links.append(link("can2"))
        self.drivers["can2"] = "peak_usb"
        with self.assertRaisesRegex(RuntimeError, "多个"):
            configuration_commands(self.links, self.drivers)

    def test_active_usb_healthy_board_and_name_collision_are_not_changed(self):
        for replacement in (link("can1", up=True, state="ERROR-ACTIVE", bitrate=1000000),
                            link("can0", up=True, state="ERROR-ACTIVE", bitrate=1000000)):
            links = [replacement if item["ifname"] == replacement["ifname"] else item for item in self.links]
            with self.assertRaises(RuntimeError):
                configuration_commands(links, self.drivers)
        with self.assertRaisesRegex(RuntimeError, "can_native0"):
            configuration_commands([*self.links, link("can_native0")], self.drivers)

    def test_bus_off_usb_is_not_automatically_reset(self):
        with self.assertRaisesRegex(RuntimeError, "BUS-OFF"):
            configuration_commands([self.links[0]], {"can0": "peak_usb"})

    def test_receiver_on_usb_or_any_blocks_configuration(self):
        for name in ("can1", "can0", "any"):
            with self.assertRaisesRegex(RuntimeError, "接收者"):
                require_no_receivers([f"{name} 123 456 789 abc 0 callback"], {"can0", "can1"})
        require_no_receivers(["(can1: no entry)", "can2 123 456 789 abc 0 callback"], {"can0", "can1"})


if __name__ == "__main__":
    unittest.main()
