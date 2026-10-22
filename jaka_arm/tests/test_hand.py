# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for JakaDriver's hand mode (LinkerHand O6) against fakes — no
hardware required. Mirrors test_driver.py's FakeRC style."""
import unittest

from jaka_arm import driver as D


class FakeHand:
    """Records calls; raw angles as configured (255 = straight)."""

    def __init__(self, angles=None):
        self.slave_id = 0x27
        self.angles = list(angles) if angles else [255] * 6
        self.opened = 0
        self.gripped = 0
        self.posed = []
        self.speed_sets = []
        self.torque_sets = []
        self.powered = 0

    def power_on(self):
        self.powered += 1

    def read_angle(self, i, confirm_zero=False):
        return self.angles[i]

    def read_angles(self):
        return list(self.angles)

    def read_direction(self):
        return 82  # right

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
        """Normalized [0,1] entry point (1=flexed) — mirrors hand.py."""
        raws = [int(round((1.0 - float(v)) * 255)) for v in positions]
        return self.set_pose(raws)

    def set_speed_norm(self, speeds):
        self.speed_sets.append(list(speeds))
        return 0

    def set_torque_norm(self, torques):
        self.torque_sets.append(list(torques))
        return 0

    def get_motion_speed_norm(self):
        return self.speed_sets[-1] if self.speed_sets else [200 / 255.0] * 6

    def get_motion_torque_norm(self):
        return self.torque_sets[-1] if self.torque_sets else [200 / 255.0] * 6


class FakeRC:
    def __init__(self):
        self.enabled = 0

    def login(self):
        return (0,)

    def logout(self):
        return (0,)

    def enable_robot(self):
        self.enabled += 1
        return (0,)

    def disable_robot(self):
        return (0,)

    def drag_mode_enable(self, on):
        return (0,)

    def motion_abort(self):
        return (0,)

    def power_off(self):
        return (0,)

    def get_actual_joint_position(self):
        return (0, [0.0] * 6)

    def get_actual_tcp_position(self):
        return (0, [500.0, 0.0, 300.0, 0.0, 3.14159, 0.0])


def make_hand_driver(fake_hand=None):
    """JakaDriver in hand mode with JakaHand/JakaGripper stubbed out."""
    drv = D.JakaDriver(ip="0.0.0.0", end_effector="hand")
    drv._rc = FakeRC()
    drv._hand = fake_hand or FakeHand()
    drv._connected = True
    return drv


class TestModeSelection(unittest.TestCase):
    def test_invalid_mode_rejected(self):
        with self.assertRaises(ValueError):
            D.JakaDriver(ip="0.0.0.0", end_effector="claw")

    def test_invalid_hand_type_rejected(self):
        with self.assertRaises(ValueError):
            D.JakaDriver(ip="0.0.0.0", end_effector="hand", hand_type="middle")

    def test_end_effector_property(self):
        self.assertEqual(D.JakaDriver(ip="0.0.0.0").end_effector, "gripper")
        self.assertEqual(
            D.JakaDriver(ip="0.0.0.0", end_effector="hand").end_effector, "hand")


class TestBinaryOpenGrip(unittest.TestCase):
    def test_full_stroke_opens(self):
        h = FakeHand()
        drv = make_hand_driver(h)
        self.assertTrue(drv.set_gripper_open_m(D.GRIPPER_STROKE_M))
        self.assertEqual((h.opened, h.gripped), (1, 0))

    def test_zero_grips(self):
        h = FakeHand()
        drv = make_hand_driver(h)
        self.assertTrue(drv.set_gripper_open_m(0.0))
        self.assertEqual((h.opened, h.gripped), (0, 1))

    def test_gripper_mode_never_touches_hand(self):
        drv = D.JakaDriver(ip="0.0.0.0")  # gripper mode
        drv._rc = FakeRC()
        drv._hand = FakeHand()
        drv._connected = True
        self.assertFalse(drv.set_gripper_open_m(D.GRIPPER_STROKE_M))  # no JakaGripper -> False
        self.assertEqual((drv._hand.opened, drv._hand.gripped), (0, 0))


class TestHandJointConversion(unittest.TestCase):
    def test_straight_pose_reads_zero_rad(self):
        drv = make_hand_driver(FakeHand([255] * 6))
        self.assertTrue(drv.refresh_hand_positions())
        pos = drv.get_hand_joint_positions_rad()
        self.assertEqual(len(pos), 6)
        for value in pos:
            self.assertAlmostEqual(value, 0.0)

    def test_flexed_pose_reads_upper_rad(self):
        drv = make_hand_driver(FakeHand([0] * 6))
        self.assertTrue(drv.refresh_hand_positions())
        pos = drv.get_hand_joint_positions_rad()
        for value, upper in zip(pos, D.HAND_JOINT_UPPER_RAD, strict=True):
            self.assertAlmostEqual(value, upper)

    def test_write_round_trip(self):
        h = FakeHand()
        drv = make_hand_driver(h)
        target = [0.0, 0.0, D.HAND_JOINT_UPPER_RAD[2], 0.0, 0.0, 0.0]
        self.assertTrue(drv.set_hand_joint_positions_rad(target))
        # index fully flexed -> raw 0; thumb straight -> raw 255
        self.assertEqual(h.posed[-1][2], 0)
        self.assertEqual(h.posed[-1][0], 255)

    def test_write_out_of_range_rejected(self):
        drv = make_hand_driver()
        with self.assertRaises(ValueError):
            drv.set_hand_joint_positions_rad([99.0] * 6)

    def test_dropped_read_is_none_not_fabricated(self):
        h = FakeHand()
        h.read_angle = lambda i, confirm_zero=False: None if i == 3 else 255
        drv = make_hand_driver(h)
        self.assertTrue(drv.refresh_hand_positions())
        pos = drv.get_hand_joint_positions_rad()
        self.assertIsNone(pos[3])
        self.assertAlmostEqual(pos[0], 0.0)

    def test_normalized_reads_and_writes(self):
        drv = make_hand_driver(FakeHand([255, 0, 255, 0, 255, 0]))
        drv.refresh_hand_positions()
        norm = drv.get_hand_joint_positions_normalized()
        self.assertEqual(norm, [0.0, 1.0, 0.0, 1.0, 0.0, 1.0])
        h = FakeHand()
        drv2 = make_hand_driver(h)
        self.assertTrue(drv2.set_hand_joint_positions_normalized([0, 0, 1, 0, 0, 0]))
        self.assertEqual(h.posed[-1][2], 0)  # 1.0 = flexed = raw 0


class TestMotionLimits(unittest.TestCase):
    def test_speed_and_torque_limits_persist(self):
        h = FakeHand()
        drv = make_hand_driver(h)
        self.assertTrue(drv.set_hand_joint_speed_limits([0.8] * 6))
        self.assertTrue(drv.set_hand_joint_torque_limits([0.6] * 6))
        self.assertEqual(h.speed_sets[-1], [0.8] * 6)
        self.assertEqual(h.torque_sets[-1], [0.6] * 6)
        # cached getters serve the values in effect
        self.assertEqual(drv.get_hand_joint_speed_limits(), [0.8] * 6)
        self.assertEqual(drv.get_hand_joint_torque_limits(), [0.6] * 6)


class TestDisconnectedSafety(unittest.TestCase):
    def test_all_hand_ops_refuse_when_disconnected(self):
        drv = D.JakaDriver(ip="0.0.0.0", end_effector="hand")
        self.assertIsNone(drv.get_hand_joint_positions_rad())
        self.assertIsNone(drv.get_hand_joint_positions_normalized())
        self.assertIsNone(drv.get_hand_joint_speed_limits())
        self.assertIsNone(drv.get_hand_joint_torque_limits())
        self.assertFalse(drv.set_hand_joint_positions_rad([0.0] * 6))
        self.assertFalse(drv.set_hand_joint_speed_limits([0.5] * 6))
        self.assertFalse(drv.set_gripper_open_m(D.GRIPPER_STROKE_M))
        self.assertFalse(drv.refresh_hand_positions())


if __name__ == "__main__":
    unittest.main()
