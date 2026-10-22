# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for the JakaGripper RS485 read protocol against a FakeRC stub.

The controller's signal table misplaces values when it holds 2+ entries, so
the read path must keep exactly one entry ("_rd"), force a poll via
update_tio_rs_signal, delete the entry after each read, and retry on failure.
"""
import unittest
from unittest.mock import patch

from jaka_arm.gripper import REG, JakaGripper


class FakeRC:
    """Minimal jkrc.RC stand-in for the gripper's signal-table reads."""

    def __init__(self, values=None, drop_adds=False):
        self.values = dict(values or {})  # register addr -> value
        self.drop_adds = drop_adds        # simulate the poll never landing
        self.table = {}                   # sig_name -> entry
        self.adds = []
        self.deleted = []
        self.updates = []
        self.max_table_len = 0

    # -- channel config (return codes only) ----------------------------------
    def set_tio_pin_mode(self, *args):
        return (0,)

    def set_rs485_chn_mode(self, *args):
        return (0,)

    def set_rs485_chn_comm(self, *args):
        return (0,)

    def power_on(self):
        return (0,)

    # -- signal table --------------------------------------------------------
    def add_tio_rs_signal(self, sig):
        name = sig["sig_name"]
        if self.table and name not in self.table:
            raise AssertionError("signal table must never hold 2 entries")
        self.adds.append(name)
        if not self.drop_adds:
            self.table[name] = {"sig_name": name,
                                "value": self.values.get(sig["sig_addr"])}
        self.max_table_len = max(self.max_table_len, len(self.table))
        return (0,)

    def update_tio_rs_signal(self, sig):
        self.updates.append(dict(sig))
        return (0,)

    def del_tio_rs_signal(self, name):
        self.table.pop(name, None)
        self.deleted.append(name)
        return (0,)

    def get_rs485_signal_info(self):
        return (0, list(self.table.values()))


class TestGripperReadProtocol(unittest.TestCase):
    def setUp(self):
        self._waits = (JakaGripper._READ_WAIT_ADD, JakaGripper._READ_WAIT_UPDATE)
        JakaGripper._READ_WAIT_ADD = 0
        JakaGripper._READ_WAIT_UPDATE = 0
        # power_on's fixed settle wait is real seconds; skip it in tests
        patch("jaka_arm.gripper.time.sleep").start()
        self.addCleanup(patch.stopall)

    def tearDown(self):
        JakaGripper._READ_WAIT_ADD, JakaGripper._READ_WAIT_UPDATE = self._waits

    def test_init_leaves_no_signal_table_entries(self):
        rc = FakeRC()
        JakaGripper(robot=rc)
        self.assertEqual(rc.table, {})  # no permanent core table

    def test_read_reg_single_entry_protocol(self):
        rc = FakeRC(values={REG["position"]: 1002})
        g = JakaGripper(robot=rc)
        self.assertEqual(g.read_position(), 1002)
        self.assertEqual(rc.adds, ["_rd"])
        self.assertEqual(rc.updates,
                         [{"sig_name": "_rd", "frequency": 5}])
        self.assertEqual(rc.deleted, ["_rd"])  # entry removed after the read
        self.assertEqual(rc.table, {})
        self.assertEqual(rc.max_table_len, 1)

    def test_read_reg_retries_then_returns_none(self):
        rc = FakeRC(drop_adds=True)
        g = JakaGripper(robot=rc)
        self.assertIsNone(g.read_position())
        self.assertEqual(len(rc.adds), 2)  # default retries
        self.assertEqual(rc.deleted, ["_rd"])  # final cleanup

    def test_read_regs_reads_each_register(self):
        rc = FakeRC(values={REG["grip_count"]: 0x1234,
                            REG["grip_count"] + 1: 0x0001})
        g = JakaGripper(robot=rc)
        self.assertEqual(g.read_grip_count(), 0x00011234)  # low word first
        self.assertEqual(rc.max_table_len, 1)

    def test_power_on_ready_on_first_nonzero_current(self):
        rc = FakeRC(values={REG["current"]: 50})
        g = JakaGripper(robot=rc)
        g.power_on()
        self.assertEqual(rc.adds, ["_rd"])  # one read sufficed

    def test_power_on_waits_full_readiness_loop(self):
        rc = FakeRC()  # current reads as None: gripper never ready
        g = JakaGripper(robot=rc)
        g.power_on()
        self.assertEqual(len(rc.adds), 8)


if __name__ == "__main__":
    unittest.main()
