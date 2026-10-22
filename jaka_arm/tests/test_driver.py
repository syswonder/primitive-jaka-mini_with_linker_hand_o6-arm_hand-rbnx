# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for JakaDriver against a FakeRC stub (no hardware required)."""
import time
import unittest

import numpy as np

from jaka_arm.driver import (
    JakaDriver,
    ABS,
    GRIPPER_READ_INTERVAL_S,
    GRIPPER_STROKE_M,
)


class FakeGripper:
    def __init__(self, pos):
        self.pos = pos
        self.reads = 0

    def read_position(self):
        self.reads += 1
        return self.pos


class FakeRC:
    """Minimal jkrc.RC stand-in: record calls, echo targets as feedback."""

    def __init__(self, joints=None, tcp=None):
        self.joints = list(joints) if joints else [0.0] * 6
        self.tcp = tcp or ((0.5, 0.0, 0.3), (0.0, 3.14159, 0.0))
        self.joint_move_calls = []
        self.linear_move_calls = []
        self.ik_calls = []

    def get_actual_joint_position(self):
        return (0, list(self.joints))

    def get_actual_tcp_position(self):
        (x, y, z), (rx, ry, rz) = self.tcp
        return (0, [x * 1000.0, y * 1000.0, z * 1000.0, rx, ry, rz])

    def joint_move(self, rad, abs_flag, is_block, speed):
        self.joint_move_calls.append((list(rad), abs_flag, is_block, speed))
        self.joints = list(rad)
        return (0,)

    def linear_move(self, pose, abs_flag, is_block, speed):
        self.linear_move_calls.append((list(pose), abs_flag, is_block, speed))
        self.tcp = ((pose[0] / 1000.0, pose[1] / 1000.0, pose[2] / 1000.0),
                    (pose[3], pose[4], pose[5]))
        return (0,)

    def kine_inverse(self, ref_rad, pose):
        self.ik_calls.append((list(ref_rad), list(pose)))
        return (0, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])


def make_driver(fake_rc, gripper_pos=None):
    d = JakaDriver(ip="127.0.0.1")
    d._rc = fake_rc
    d._connected = True
    d._gripper = FakeGripper(gripper_pos) if gripper_pos is not None else None
    return d


def read_gripper_now(d):
    """Force a fresh gripper read (bypassing the driver's read cache)."""
    d._gripper_open_m_mono = None
    return d.get_gripper_open_m()


class TestGripperNormalization(unittest.TestCase):
    def test_reverse_travel(self):
        # Measured travel: 0 = open, 1000 = closed
        d = make_driver(FakeRC(), gripper_pos=0)
        d._gripper_open_pos, d._gripper_close_pos = 0, 1000
        self.assertAlmostEqual(read_gripper_now(d), GRIPPER_STROKE_M, places=6)
        d._gripper.read_position = lambda: 1000
        self.assertAlmostEqual(read_gripper_now(d), 0.0, places=6)
        d._gripper.read_position = lambda: 500
        self.assertAlmostEqual(read_gripper_now(d), GRIPPER_STROKE_M / 2, places=6)

    def test_forward_travel(self):
        d = make_driver(FakeRC(), gripper_pos=1000)
        d._gripper_open_pos, d._gripper_close_pos = 1000, 0
        self.assertAlmostEqual(read_gripper_now(d), GRIPPER_STROKE_M, places=6)

    def test_out_of_range_clips(self):
        d = make_driver(FakeRC(), gripper_pos=1500)
        d._gripper_open_pos, d._gripper_close_pos = 0, 1000
        self.assertAlmostEqual(read_gripper_now(d), 0.0, places=6)
        d._gripper.read_position = lambda: -100
        self.assertAlmostEqual(read_gripper_now(d), GRIPPER_STROKE_M, places=6)


class TestGripperReadCache(unittest.TestCase):
    def test_within_interval_serves_cached_value(self):
        d = make_driver(FakeRC(), gripper_pos=0)
        d._gripper_open_pos, d._gripper_close_pos = 0, 1000
        first = d.get_gripper_open_m()
        self.assertEqual(d._gripper.reads, 1)
        d._gripper.pos = 1000  # would read differently if re-read
        self.assertEqual(d.get_gripper_open_m(), first)
        self.assertEqual(d._gripper.reads, 1)

    def test_rereads_after_interval(self):
        d = make_driver(FakeRC(), gripper_pos=0)
        d._gripper_open_pos, d._gripper_close_pos = 0, 1000
        d.get_gripper_open_m()
        d._gripper.pos = 1000
        d._gripper_open_m_mono = time.monotonic() - GRIPPER_READ_INTERVAL_S - 1.0
        self.assertAlmostEqual(d.get_gripper_open_m(), 0.0, places=6)
        self.assertEqual(d._gripper.reads, 2)

    def test_failed_read_backs_off(self):
        d = make_driver(FakeRC())
        d._gripper = FakeGripper(None)
        self.assertIsNone(d.get_gripper_open_m())
        self.assertIsNone(d.get_gripper_open_m())  # no immediate retry
        self.assertEqual(d._gripper.reads, 1)


class TestJointLimits(unittest.TestCase):
    def test_out_of_limit_rejected(self):
        rc = FakeRC()
        d = make_driver(rc)
        target = [0.0, np.radians(200.0), 0.0, 0.0, 0.0, 0.0]  # j2 > 125
        self.assertFalse(d.move_joints_rad(target))
        self.assertEqual(rc.joint_move_calls, [])

    def test_in_limit_accepted(self):
        rc = FakeRC()
        d = make_driver(rc)
        target = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.assertTrue(d.move_joints_rad(target))
        self.assertEqual(len(rc.joint_move_calls), 1)
        rad, abs_flag, is_block, speed = rc.joint_move_calls[0]
        self.assertEqual(rad, target)
        self.assertEqual(abs_flag, ABS)
        self.assertFalse(is_block)  # non-blocking; wait is poll-based
        self.assertGreater(speed, 0)

    def test_wrist_flip(self):
        flipped = JakaDriver._wrist_flip([10.0, 20.0, 30.0, 40.0, 50.0, 60.0])
        self.assertEqual(flipped, [10.0, 20.0, 30.0, -140.0, -50.0, 240.0])


class TestMovePose(unittest.TestCase):
    def test_linear_path_when_not_singular(self):
        rc = FakeRC(joints=[0.0, 0.0, 0.0, 0.0, np.radians(60.0), 0.0])
        d = make_driver(rc)
        self.assertTrue(d.move_pose([0.3, 0.0, 0.2], [0.0, 0.0, 0.0]))
        self.assertEqual(len(rc.linear_move_calls), 1)
        pose, abs_flag, is_block, speed = rc.linear_move_calls[0]
        self.assertEqual(pose[:3], [300.0, 0.0, 200.0])
        self.assertEqual(pose[3:], [0.0, 0.0, 0.0])
        self.assertEqual(abs_flag, ABS)
        self.assertFalse(is_block)
        self.assertGreater(speed, 0)

    def test_singular_start_falls_back_to_ik(self):
        # j5 = 0 (home): Cartesian planner cannot start, must use IK + joints
        rc = FakeRC(joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        d = make_driver(rc)
        self.assertTrue(d.move_pose([0.3, 0.0, 0.2], [0.0, 0.0, 0.0]))
        self.assertEqual(rc.linear_move_calls, [])
        self.assertEqual(len(rc.ik_calls), 1)
        self.assertEqual(len(rc.joint_move_calls), 1)
        for actual, expected in zip(rc.joint_move_calls[0][0],
                                    [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], strict=True):
            self.assertAlmostEqual(actual, expected, places=9)


if __name__ == "__main__":
    unittest.main()
