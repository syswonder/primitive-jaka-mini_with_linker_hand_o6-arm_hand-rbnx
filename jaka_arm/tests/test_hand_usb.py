# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for the USB-serial O6 hand bus (JakaHandUsb) and the driver's
hand_bus routing, against a fake Modbus client — no hardware required.
"""
import unittest

from jaka_arm import driver as D
from jaka_arm import hand_usb as U


class FakeResponse:
    def __init__(self, registers=None, error=False):
        self.registers = list(registers) if registers else []
        self._error = error

    def isError(self):
        return self._error

    def __str__(self):
        return "fake error" if self._error else "ok"


class FakeModbusClient:
    """Answers reads from a register map; records writes."""

    def __init__(self, holding=None, input_regs=None):
        self.holding = dict(holding or {})
        self.input_regs = dict(input_regs or {})
        self.writes = []
        self.connected = False

    def connect(self):
        self.connected = True
        return True

    def close(self):
        self.connected = False

    def read_input_registers(self, address, count, slave):
        return FakeResponse(
            [self.input_regs.get(address + i, 0) for i in range(count)])

    def write_registers(self, address, values, slave):
        self.writes.append((address, list(values)))
        for i, v in enumerate(values):
            self.holding[address + i] = v
        return FakeResponse()


def make_usb_hand(fake_client):
    hand = U.JakaHandUsb(port="/dev/fake", slave_id=0x27)
    hand._client = fake_client
    return hand


class TestUsbHand(unittest.TestCase):
    def test_read_angles_is_one_burst(self):
        client = FakeModbusClient(input_regs={i: 255 - i for i in range(6)})
        hand = make_usb_hand(client)
        self.assertEqual(hand.read_angles(), [255, 254, 253, 252, 251, 250])

    def test_read_failure_returns_none(self):
        client = FakeModbusClient()
        client.read_input_registers = lambda **kw: FakeResponse(error=True)
        hand = make_usb_hand(client)
        self.assertIsNone(hand.read_angles())

    def test_direction_and_dof(self):
        client = FakeModbusClient(input_regs={35: 82, 30: 6})
        hand = make_usb_hand(client)
        self.assertEqual(hand.read_direction(), 82)
        self.assertEqual(hand.read_dof(), 6)

    def test_open_and_grip_write_three_frames(self):
        client = FakeModbusClient()
        hand = make_usb_hand(client)
        self.assertEqual(hand.open(), 0)
        # speed, torque, angle — in that order, six registers each
        self.assertEqual([addr for addr, _ in client.writes],
                         [12, 6, 0])
        self.assertEqual(client.writes[2][1], list(U.OPEN_ANGLES))
        client.writes.clear()
        self.assertEqual(hand.grip(), 0)
        self.assertEqual(client.writes[2][1], list(U.GRASP_ANGLES))

    def test_normalized_write_inverts_angle_polarity(self):
        client = FakeModbusClient()
        hand = make_usb_hand(client)
        self.assertEqual(hand.set_joint_pos([1, 0, 1, 0, 1, 0]), 0)
        # 1.0 (flexed) -> raw 0, 0.0 (straight) -> raw 255
        self.assertEqual(client.writes[2][1], [0, 255, 0, 255, 0, 255])

    def test_limits_persist_and_feed_motion(self):
        client = FakeModbusClient()
        hand = make_usb_hand(client)
        self.assertEqual(hand.set_speed_norm([0.5] * 6), 0)
        self.assertEqual(hand.set_torque_norm([0.25] * 6), 0)
        self.assertEqual(hand.get_motion_speed_norm(), [0.5] * 6)
        self.assertEqual(hand.get_motion_torque_norm(), [0.25] * 6)
        client.writes.clear()
        hand.set_pose([255] * 6)
        self.assertEqual(client.writes[0], (12, [128] * 6))  # 0.5*255 rounded
        self.assertEqual(client.writes[1], (6, [64] * 6))

    def test_out_of_range_rejected(self):
        hand = make_usb_hand(FakeModbusClient())
        with self.assertRaises(ValueError):
            hand.set_pose([256] * 6)
        with self.assertRaises(ValueError):
            hand.set_speed_norm([1.5] * 6)


class FakeUsbLink:
    """Stand-in hand exposing the JakaHandUsb surface used by the driver."""

    slave_id = 0x27

    def __init__(self):
        self.angles = [255] * 6
        self.opened = 0
        self.gripped = 0
        self.posed = []

    def power_on(self):
        pass

    def close(self):
        pass

    def read_direction(self):
        return 82

    def read_angles(self):
        return list(self.angles)

    def open(self, speed=None, torque=None):
        self.opened += 1
        return 0

    def grip(self, speed=None, torque=None):
        self.gripped += 1
        return 0

    def set_pose(self, angles):
        self.posed.append(list(angles))
        return 0

    def set_joint_pos(self, positions):
        return self.set_pose([int(round((1.0 - float(v)) * 255)) for v in positions])


class FakeRC:
    def enable_robot(self):
        return (0,)

    def power_on(self):
        return (0,)

    def get_actual_joint_position(self):
        return (0, [0.0] * 6)


class TestDriverUsbBus(unittest.TestCase):
    def make(self):
        drv = D.JakaDriver(ip="0.0.0.0", end_effector="hand", hand_bus="usb",
                           hand_port="/dev/fake")
        drv._rc = FakeRC()
        drv._hand = FakeUsbLink()
        drv._connected = True
        return drv

    def test_invalid_bus_rejected(self):
        with self.assertRaises(ValueError):
            D.JakaDriver(ip="0.0.0.0", end_effector="hand", hand_bus="can")

    def test_refresh_is_one_burst_and_caches(self):
        drv = self.make()
        drv._hand.angles = [255, 0, 255, 0, 255, 0]
        self.assertTrue(drv.refresh_hand_positions())
        self.assertEqual(drv.get_hand_joint_positions_normalized(),
                         [0.0, 1.0, 0.0, 1.0, 0.0, 1.0])

    def test_refresh_returns_false_when_read_fails(self):
        drv = self.make()
        drv._hand.read_angles = lambda: None
        self.assertFalse(drv.refresh_hand_positions())

    def test_gripper_joint_maps_to_open_grip(self):
        drv = self.make()
        self.assertTrue(drv.set_gripper_open_m(D.GRIPPER_STROKE_M))
        self.assertTrue(drv.set_gripper_open_m(0.0))
        self.assertEqual((drv._hand.opened, drv._hand.gripped), (1, 1))


if __name__ == "__main__":
    unittest.main()
