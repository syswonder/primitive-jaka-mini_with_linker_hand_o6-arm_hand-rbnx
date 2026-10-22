import sys
"""main.py hand 契约 handler 的离线测试（不依赖 atlas/ROS）。"""
from jaka_arm import jaka_sdk  # noqa

# 打桩 robonix_api / ROS / hand.msg，再导入 main
import types, unittest
from unittest import mock

class FakePB:
    class _Msg:
        def __init__(self, **kw):
            self.__dict__.update(kw)
    def __getattr__(self, name):
        outer = self
        class _Resp:
            def __init__(self, **kw):
                for k, v in kw.items(): setattr(self, k, v)
                defaults = dict(ok=True, message="", joints=[], targets=[])
                for k, v in defaults.items():
                    if not hasattr(self, k): setattr(self, k, v)
        return _Resp

fake_hand_pb2 = FakePB()
sys.modules["hand_pb2"] = fake_hand_pb2
sys.modules["hand_pb2_grpc"] = types.ModuleType("hand_pb2_grpc")
sys.modules["robonix_api"] = types.SimpleNamespace(
    Primitive=lambda **kw: types.SimpleNamespace(
        on_init=lambda f: f, on_activate=lambda f: f,
        on_deactivate=lambda f: f, on_shutdown=lambda f: f,
        grpc=lambda cid: (lambda f: f),
        create_publisher=lambda **kw: None,
        create_subscription=lambda **kw: None,
        emit=lambda *a, **k: None,
        run=lambda: None),
    Ok=lambda *a, **k: None, Err=lambda msg: msg)
geo = types.ModuleType("geometry_msgs.msg"); geo.Pose = object
sen = types.ModuleType("sensor_msgs.msg"); sen.JointState = object
sys.modules.setdefault("geometry_msgs", types.ModuleType("geometry_msgs"))
sys.modules["geometry_msgs.msg"] = geo
sys.modules.setdefault("sensor_msgs", types.ModuleType("sensor_msgs"))
sys.modules["sensor_msgs.msg"] = sen

import jaka_arm.main as m

class TestOverlay(unittest.TestCase):
    def _t(self, name, value):
        return types.SimpleNamespace(name=name, value=value)

    def test_empty_rejected(self):
        new, err = m._overlay_hand([])
        self.assertIsNone(new); self.assertIn("empty", err)

    def test_unknown_rejected(self):
        new, err = m._overlay_hand([self._t("elbow", 0.5)])
        self.assertIsNone(new); self.assertIn("unknown axis", err)

    def test_duplicate_rejected(self):
        new, err = m._overlay_hand([self._t("index_mcp_pitch", 0.5),
                                    self._t("index_mcp_pitch", 0.6)])
        self.assertIsNone(new); self.assertIn("duplicate", err)

    def test_out_of_range_rejected(self):
        for bad in (-0.1, 1.1, float("nan")):
            new, err = m._overlay_hand([self._t("index_mcp_pitch", bad)])
            self.assertIsNone(new); self.assertIn("out of range", err)

    def test_unknown_pose_rejected(self):
        old = m._hand_targets
        m._hand_targets = None
        try:
            new, err = m._overlay_hand([self._t("index_mcp_pitch", 0.5)])
        finally:
            m._hand_targets = old
        self.assertIsNone(new)
        self.assertIn("not yet known", err)

    def test_hold_unlisted(self):
        base = [0.1] * 6
        new, err = m._overlay_hand([self._t("index_mcp_pitch", 0.7)], base)
        self.assertIsNone(err)
        self.assertEqual(new[2], 0.7)
        self.assertEqual(new[0], 0.1)  # unlisted holds

class TestAxisLayout(unittest.TestCase):
    def test_axis_names_match_driver(self):
        from jaka_arm.driver import HAND_JOINT_NAMES
        self.assertEqual(m._HAND_AXIS_NAMES, HAND_JOINT_NAMES)

    def test_axis_metadata(self):
        names = [a[0] for a in m._AXES]
        self.assertEqual(len(names), 6)
        types_ = [a[2] for a in m._AXES]
        self.assertEqual(types_.count("oppose"), 1)

class TestHandGating(unittest.TestCase):
    def test_hand_info_disabled_when_gripper_mode(self):
        m._hand_mode = False
        r = m.hand_info(None, None)
        self.assertFalse(r.ok)
        self.assertIn("not enabled", r.message)

    def test_move_joint_disabled_when_gripper_mode(self):
        m._hand_mode = False
        req = types.SimpleNamespace(targets=[])
        r = m.hand_move_joint(req, None)
        self.assertFalse(r.ok)

if __name__ == "__main__":
    unittest.main(verbosity=2)
