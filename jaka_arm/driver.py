# SPDX-License-Identifier: MIT
# -*- coding: utf-8 -*-
# Adapted from AIIT-Roboarm (arm/jaka_ctrl_by_sdk.py), MIT License,
# Copyright (c) 2025 Xu Jinyang & Hou Yunlong. See NOTICE.
"""JakaDriver — minimal jkrc SDK adapter for the Robonix jaka_arm primitive.

Surface: connect/disconnect/stop_motion, joint & TCP feedback, joint move,
Cartesian move with controller-side IK fallback, and one of two end
effectors on the controller's RS485 end channel: the TG-9801 adaptive
gripper (default) or a LinkerHand O6 6-axis dexterous hand.

Units: joints in rad (SDK native), TCP pose in m + rpy rad, gripper opening
in m (TG-9801 stroke, normalized), hand joints in rad (URDF convention,
0 = straight, ~1.57 = fully flexed). All RC calls are serialized by one
RLock so the state-publish thread can interleave with motion commands.
"""
from __future__ import annotations

import threading
import time

import numpy as np

from . import jaka_sdk  # noqa: F401  (platform loader; must precede jkrc)
import jkrc  # noqa: E402

from .gripper import JakaGripper
from .hand import JakaHand

ABS = 0  # jkrc absolute motion

JOINT_COUNT = 6
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
GRIPPER_JOINT_NAME = "gripper_joint"
# TG-9801 total stroke (datasheet: 可调行程 0~98 mm)
GRIPPER_STROKE_M = 0.098
# A gripper position read costs ~1.3 s (single-entry RS485 signal-table path,
# see gripper.JakaGripper._read_reg), so the 50 Hz state loop serves a cached
# value and re-reads at most this often.
GRIPPER_READ_INTERVAL_S = 2.0

# ── O6 dexterous hand (end_effector: hand) ────────────────────────────────────
# URDF joint names of the 6 hand axes (match urdf/robot.urdf + the D1 hand
# URDF the chain came from), in firmware index order 0~5.
HAND_JOINT_NAMES = [
    "thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch",
    "middle_mcp_pitch", "ring_mcp_pitch", "pinky_mcp_pitch",
]
# Per-joint URDF upper limit (rad): 0 = straight, upper = fully flexed. The
# hand's raw 0~255 maps linearly onto [0, upper]. These MUST match the
# deployment URDF's joint limits: thumb_cmc_pitch has a physically shorter
# travel (0.52 rad) than the other flexion axes (1.57); thumb_cmc_yaw is
# 1.54. Mismatches make command limits and RViz disagree.
HAND_JOINT_UPPER_RAD = [0.52, 1.54, 1.57, 1.57, 1.57, 1.57]
# O6 RS485 slave ids by hand side (115200 8N1, vendor default).
HAND_SLAVE_IDS = {"right": 0x27, "left": 0x28}

# JAKA Mini soft limits (user manual table 5-1, degrees). Verified against
# the real arm before enabling motion; placeholder limits are from Zu3 docs.
JOINT_LIMITS_DEG = (
    (-360.0, 360.0), (-125.0, 125.0), (-130.0, 130.0),
    (-360.0, 360.0), (-120.0, 120.0), (-360.0, 360.0),
)
# j5 near 0 (wrist singularity) or near its limit cannot start Cartesian moves
SINGULAR_J5_EPS_DEG = 5.0
# Known-good grasp configuration (measured on the real arm), fallback IK seed
GRASP_IK_SEED_DEG = [46.2, -20.6, -76.1, 0.0, -83.4, 76.2]


class JakaDriver:
    """Blocking, poll-to-wait adapter over jkrc.RC. Not thread-safe by itself;
    every RC call is guarded by the internal lock."""

    def __init__(
        self,
        ip: str,
        joint_speed_rad_s: float = 1.0,
        linear_speed_mm_s: float = 100.0,
        gripper_force: int = 50,
        gripper_speed: int = 1100,
        reach_mse_threshold_deg2: float = 1.0,
        move_timeout_s: float = 10.0,
        end_effector: str = "gripper",
        hand_type: str = "right",
    ):
        if end_effector not in ("gripper", "hand"):
            raise ValueError(
                f"end_effector must be 'gripper' or 'hand', got {end_effector!r}")
        if hand_type not in HAND_SLAVE_IDS:
            raise ValueError(f"hand_type must be right|left, got {hand_type!r}")
        self._ip = ip
        self._joint_speed_rad_s = float(joint_speed_rad_s)
        self._linear_speed_mm_s = float(linear_speed_mm_s)
        self._gripper_force = int(gripper_force)
        self._gripper_speed = int(gripper_speed)
        self._reach_mse_threshold_deg2 = float(reach_mse_threshold_deg2)
        self._move_timeout_s = float(move_timeout_s)
        self._end_effector = end_effector
        self._hand_type = hand_type
        self._lock = threading.RLock()
        self._rc = None
        self._gripper = None
        self._hand = None
        # Hand state flows through its own lock/cache: one full sweep blocks
        # ~2.3 s, so the 50 Hz arm state loop must never wait on it (see
        # refresh_hand_positions).
        self._hand_cache_lock = threading.Lock()
        self._hand_raw_cache: list | None = None
        self._gripper_close_pos = 1000  # fallback: 1000 = closed (reverse travel)
        self._gripper_open_pos = 0      # fallback: 0 = open
        self._gripper_open_m = None     # cached reading (see get_gripper_open_m)
        self._gripper_open_m_mono = None
        self._connected = False

    @property
    def end_effector(self) -> str:
        return self._end_effector

    # ---------- lifecycle ----------

    def connect(self) -> None:
        """Login, power on, enable, and bring up the configured end effector.

        Only the configured end effector is constructed: both sit on the same
        controller connection, power-on, and RS485 end channel (the controller
        SDK connection count is limited), so they never coexist.
        """
        with self._lock:
            if self._connected:
                return
            self._rc = jkrc.RC(self._ip)
            ret = self._rc.login()
            if ret[0] != 0:
                raise ConnectionError(f"JAKA login failed ({self._ip}): {ret}")
            if self._end_effector == "hand":
                self._connect_hand()
            else:
                self._connect_gripper()
            self._connected = True

    def _connect_gripper(self) -> None:
        """TG-9801 bring-up on the RS485 end channel (shared-RC mode)."""
        self._gripper = JakaGripper(robot=self._rc)
        self._gripper.power_on()  # re-arms RS485 channel after power cycle
        self._rc.enable_robot()
        self._gripper.set_target_force(self._gripper_force)

        # Cache travel endpoints for opening normalization; controller
        # reads are occasionally bogus (0/0), fall back to measured travel.
        open_pos = self._gripper.read_open_position()
        close_pos = self._gripper.read_close_position()
        endpoints_valid = (
            open_pos is not None and close_pos is not None
            and open_pos != close_pos
            and 0 <= open_pos <= 1000 and 0 <= close_pos <= 1000
        )
        if endpoints_valid:
            self._gripper_open_pos = open_pos
            self._gripper_close_pos = close_pos
        print(
            f"[jaka_arm] TG-9801 travel: close={self._gripper_close_pos} "
            f"open={self._gripper_open_pos}"
            f"{'' if endpoints_valid else ' (fallback, reads invalid)'}",
            flush=True,
        )

    def _connect_hand(self) -> None:
        """O6 hand bring-up on the RS485 end channel (shared-RC mode).

        power_on() powers the arm (the hand is fed through the end connector)
        and waits for the hand to answer; the startup direction read verifies
        the wiring / slave id (82 = right, 76 = left) instead of leaving a
        mis-wired hand to be discovered at grasp time.
        """
        self._hand = JakaHand(
            robot=self._rc, slave_id=HAND_SLAVE_IDS[self._hand_type])
        self._hand.power_on()
        self._rc.enable_robot()
        direction = self._hand.read_direction()
        side = {82: "right", 76: "left"}.get(direction, "unknown")
        print(
            f"[jaka_arm] O6 hand: slave 0x{self._hand.slave_id:02X} "
            f"(configured {self._hand_type}, read {side})",
            flush=True,
        )

    def disconnect(self) -> None:
        """Idempotent teardown: leave drag mode, stop motion, power off, logout.

        Mirrors AIIT disconnect_arm; every step is best-effort so a partially
        connected controller still releases cleanly.
        """
        with self._lock:
            if not self._connected:
                return
            for step in (
                lambda: self._rc.drag_mode_enable(False),
                lambda: self._rc.enable_robot(),
                lambda: self._rc.motion_abort(),
                lambda: self._rc.disable_robot(),
                lambda: self._rc.power_off(),
                lambda: self._rc.logout(),
            ):
                try:
                    step()
                except Exception as exc:  # noqa: BLE001
                    print(f"[jaka_arm] disconnect step failed: {exc}", flush=True)
            self._connected = False
            self._gripper = None
            self._hand = None
            self._rc = None
            self._gripper_open_m = None
            self._gripper_open_m_mono = None

    def stop_motion(self) -> None:
        with self._lock:
            if self._connected:
                try:
                    self._rc.motion_abort()
                except Exception as exc:  # noqa: BLE001
                    print(f"[jaka_arm] motion_abort failed: {exc}", flush=True)

    # ---------- feedback ----------

    def get_joint_positions_rad(self) -> list[float] | None:
        with self._lock:
            if not self._connected:
                return None
            ret = self._rc.get_actual_joint_position()
            if ret[0] != 0:
                print(f"[jaka_arm] get_actual_joint_position error {ret[0]}", flush=True)
                return None
            return [float(a) for a in ret[1]]

    def get_gripper_open_m(self) -> float | None:
        """Last known opening in m, re-read at most every GRIPPER_READ_INTERVAL_S.

        The 50 Hz state loop must not read every cycle: a read stalls ~1.3 s
        and keeps the half-duplex bus busy, risking dropped grip/release
        frames. Between reads the previous value is served; a failed attempt
        (None) also waits a full interval before retrying.
        """
        with self._lock:
            if not self._connected or self._gripper is None:
                return None
            now = time.monotonic()
            if (self._gripper_open_m_mono is None
                    or now - self._gripper_open_m_mono >= GRIPPER_READ_INTERVAL_S):
                pos = self._gripper.read_position()
                self._gripper_open_m_mono = time.monotonic()
                if pos is not None:
                    lo, hi = self._gripper_close_pos, self._gripper_open_pos
                    if hi == lo:
                        value = 0.0
                    else:
                        value = (pos - lo) / (hi - lo)  # 1 = fully open (both travel dirs)
                    self._gripper_open_m = float(np.clip(value, 0, 1)) * GRIPPER_STROKE_M
            return self._gripper_open_m

    def get_tcp_pose(self) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
        """TCP pose as ((x, y, z) m, (rx, ry, rz) rad, R = Rz·Ry·Rx order)."""
        with self._lock:
            if not self._connected:
                return None
            ret = self._rc.get_actual_tcp_position()
            if ret[0] != 0:
                print(f"[jaka_arm] get_actual_tcp_position error {ret[0]}", flush=True)
                return None
            x, y, z, rx, ry, rz = ret[1]
            return (x / 1000.0, y / 1000.0, z / 1000.0), (float(rx), float(ry), float(rz))

    # ---------- motion ----------

    @staticmethod
    def _within_limits(angles_rad: list[float]) -> bool:
        return all(
            lo - 1e-6 <= np.degrees(a) <= hi + 1e-6
            for a, (lo, hi) in zip(angles_rad, JOINT_LIMITS_DEG, strict=True)
        )

    def move_joints_rad(self, positions_rad: list[float]) -> bool:
        """Start a joint move (non-blocking SDK call) and wait until reached.

        State reads keep flowing between polls, so Soma's freshness window is
        not held up by a long blocking SDK call.
        """
        if len(positions_rad) != JOINT_COUNT:
            raise ValueError(f"expected {JOINT_COUNT} joint positions")
        if not all(np.isfinite(positions_rad)):
            raise ValueError("joint positions must be finite")
        if not self._within_limits(positions_rad):
            print(f"[jaka_arm] joint target out of limits: "
                  f"{[round(np.degrees(a), 1) for a in positions_rad]}", flush=True)
            return False
        with self._lock:
            if not self._connected:
                return False
            ret = self._rc.joint_move(positions_rad, ABS, False, self._joint_speed_rad_s)
            if ret[0] != 0:
                print(f"[jaka_arm] joint_move error {ret[0]}", flush=True)
                return False
        return self._wait_reached(positions_rad)

    def move_pose(self, pos_m: list[float], rpy_rad: list[float]) -> bool:
        """Cartesian move; on linear failure or singular start, fall back to
        controller-side IK plus a joint move (AIIT hybrid strategy)."""
        pose = [pos_m[0] * 1000, pos_m[1] * 1000, pos_m[2] * 1000, *rpy_rad]

        joints = self.get_joint_positions_rad()
        if joints is None:
            return False
        j5_deg = np.degrees(joints[4])
        near_singular = abs(j5_deg) < SINGULAR_J5_EPS_DEG or abs(j5_deg) > (
            JOINT_LIMITS_DEG[4][1] - SINGULAR_J5_EPS_DEG
        )
        if not near_singular:
            with self._lock:
                if not self._connected:
                    return False
                ret = self._rc.linear_move(pose, ABS, False, self._linear_speed_mm_s)
            if ret[0] == 0 and self._wait_pose_reached(tuple(pos_m)):
                return True
            print(f"[jaka_arm] linear_move failed ({ret[0]}), falling back to IK",
                  flush=True)

        solution = self._solve_ik_deg(pose, joints)
        if solution is None:
            print(f"[jaka_arm] no IK solution within limits for pose "
                  f"{[round(v, 2) for v in pose]}", flush=True)
            return False
        return self.move_joints_rad([float(np.radians(a)) for a in solution])

    def set_gripper_open_m(self, open_m: float) -> bool:
        """Binary open/close.

        TG-9801 adaptive mode is binary: >= half stroke = release, else grip.
        Hand mode: >= half = open pose, else grip pose (the open/grip poses
        are the probe-verified GRASP_ANGLES / OPEN_ANGLES). Intentional
        asymmetry in hand mode: a grip command targets the fully-closed pose
        (fingers stop on the object); there is no per-object width command.
        """
        with self._lock:
            if not self._connected:
                return False
            if self._end_effector == "hand":
                if self._hand is None:
                    return False
                if open_m >= 0.5 * GRIPPER_STROKE_M:
                    return self._hand.open() == 0
                return self._hand.grip() == 0
            if self._gripper is None:
                return False
            if open_m >= 0.5 * GRIPPER_STROKE_M:
                return self._gripper.release(self._gripper_speed) == 0
            return self._gripper.grip(self._gripper_speed) == 0

    # ---------- hand feedback / commands (end_effector: hand) ----------

    def refresh_hand_positions(self) -> bool:
        """One full hand sweep (~2.3 s total, 6 axes), called by the hand
        state thread. Caches the raw angles; False when not connected or the
        link layer is down.

        The lock is taken per finger, not around the whole sweep: a single
        read holds it for at most ~0.5 s (retries included), so the 50 Hz arm
        state loop and arm commands interleave between fingers instead of
        stalling for the full sweep (Soma's freshness window is 2 s).
        """
        raw = []
        for i in range(len(HAND_JOINT_NAMES)):
            with self._lock:
                if not self._connected or self._hand is None:
                    return False
                # Plain read: the per-finger lock (not the whole sweep) keeps
                # the 50 Hz arm state loop publishing between fingers. A
                # confirm-zero retry here would double each finger's lock hold
                # and push Soma past its 2 s freshness window. Lost answers can
                # therefore show as 0 (fully flexed) in the state stream; the
                # activation seed and verdict readers use confirm_zero instead.
                raw.append(self._hand.read_angle(i))
            # Yield between fingers so the 50 Hz arm state loop can take the
            # lock; without this the hand thread can starve it for a whole
            # sweep and Soma's 2 s freshness window lapses.
            time.sleep(0.02)
        with self._hand_cache_lock:
            self._hand_raw_cache = raw
        return True

    def _cached_hand_raw(self) -> list | None:
        with self._hand_cache_lock:
            return None if self._hand_raw_cache is None else list(self._hand_raw_cache)

    def read_hand_angle(self, index: int, confirm_zero: bool = True):
        """Read one hand axis raw (0~255, 255 = straight); None on failure.

        Direct single read for activation-time seeding and tools; the steady
        state stream goes through refresh_hand_positions. Zeros are confirmed
        by default (a lost answer reads as 0, a legitimate fully-flexed pose
        too); pass confirm_zero=False for a fast single sample.
        """
        with self._lock:
            if not self._connected or self._hand is None:
                return None
            return self._hand.read_angle(index, confirm_zero=confirm_zero)

    def get_hand_joint_positions_rad(self) -> list[float | None] | None:
        """Cached hand joint angles in rad (URDF convention: 0 = straight,
        upper = fully flexed), one entry per HAND_JOINT_NAMES entry.

        None per axis = that axis had no signal-table entry (link-layer
        failure) — callers omit the axis rather than fabricate a pose. Note a
        dropped *answer* is different: the table falls back to raw 0, which
        reads as fully flexed and is published as-is (a genuine full flex and
        a dropout cannot be told apart from one sample; grasp verdicts use
        multi-finger averages over stable readings, see hand.py).
        """
        raw = self._cached_hand_raw()
        if raw is None:
            return None
        return [
            None if value is None
            else (1.0 - float(value) / 255.0) * upper
            for value, upper in zip(raw, HAND_JOINT_UPPER_RAD, strict=True)
        ]

    def get_hand_joint_positions_normalized(self) -> list[float | None] | None:
        """Cached hand axis positions in contract units [0,1] (1 = flexed)."""
        raw = self._cached_hand_raw()
        if raw is None:
            return None
        return [
            None if value is None else 1.0 - float(value) / 255.0
            for value in raw
        ]

    def set_hand_joint_positions_rad(self, positions_rad: list[float]) -> bool:
        """Command hand joint angles in rad (URDF convention). Rejects
        out-of-range values (per the hand contracts: never clamp)."""
        if len(positions_rad) != len(HAND_JOINT_NAMES):
            raise ValueError(
                f"expected {len(HAND_JOINT_NAMES)} hand joint positions")
        if not all(np.isfinite(positions_rad)):
            raise ValueError("hand joint positions must be finite")
        raws = []
        for value, upper in zip(positions_rad, HAND_JOINT_UPPER_RAD, strict=True):
            if not 0.0 <= value <= upper:
                raise ValueError(
                    f"hand joint {value} rad out of range [0, {upper}]")
            raws.append(int(round((1.0 - value / upper) * 255)))
        with self._lock:
            if not self._connected or self._hand is None:
                return False
            return self._hand.set_pose(raws) == 0

    def set_hand_joint_positions_normalized(self, positions01: list[float]) -> bool:
        """Command hand axes in contract units [0,1] (1 = flexed). Values
        outside [0,1] raise (never clamped)."""
        if len(positions01) != len(HAND_JOINT_NAMES):
            raise ValueError(
                f"expected {len(HAND_JOINT_NAMES)} hand axis values")
        with self._lock:
            if not self._connected or self._hand is None:
                return False
            return self._hand.set_joint_pos(positions01) == 0

    def get_hand_joint_speed_limits(self) -> list[float] | None:
        """Persistent per-axis speed limits in [0,1] (1 = max), or None when
        the hand is unavailable.

        Served from the driver's cached limits, NOT a register read: the O6
        firmware zeroes the speed registers after every motion (see hand.py),
        so the live register does not carry the limit in effect — the cached
        copy from the last set_hand_joint_speed_limits is the truth. Defaults
        to 1.0 (max) until first set.
        """
        with self._lock:
            if not self._connected or self._hand is None:
                return None
            return self._hand.get_motion_speed_norm()

    def get_hand_joint_torque_limits(self) -> list[float] | None:
        """Persistent per-axis torque limits in [0,1] (1 = max), or None when
        the hand is unavailable (see get_hand_joint_speed_limits)."""
        with self._lock:
            if not self._connected or self._hand is None:
                return None
            return self._hand.get_motion_torque_norm()

    def set_hand_joint_speed_limits(self, limits: list[float]) -> bool:
        """Set per-axis speed limits in [0,1] (1 = max). The value persists
        across subsequent motions (hand.py records it — the firmware clears
        the speed register after every motion)."""
        with self._lock:
            if not self._connected or self._hand is None:
                return False
            return self._hand.set_speed_norm(limits) == 0

    def set_hand_joint_torque_limits(self, limits: list[float]) -> bool:
        """Set per-axis torque limits in [0,1] (1 = max); persists like
        the speed limits."""
        with self._lock:
            if not self._connected or self._hand is None:
                return False
            return self._hand.set_torque_norm(limits) == 0

    # ---------- internals ----------

    def _wait_reached(self, target_rad: list[float]) -> bool:
        """Poll joint feedback until MSE vs target drops below threshold."""
        deadline = time.monotonic() + self._move_timeout_s
        while time.monotonic() < deadline:
            current = self.get_joint_positions_rad()
            if current is not None and len(current) == len(target_rad):
                mse_deg2 = sum(
                    (np.degrees(c) - np.degrees(t)) ** 2
                    for c, t in zip(current, target_rad, strict=True)
                ) / len(target_rad)
                if mse_deg2 <= self._reach_mse_threshold_deg2:
                    return True
            time.sleep(0.05)
        print(f"[jaka_arm] wait_until_reached timeout ({self._move_timeout_s}s)",
              flush=True)
        return False

    def _wait_pose_reached(self, target_m: tuple[float, float, float]) -> bool:
        deadline = time.monotonic() + self._move_timeout_s
        while time.monotonic() < deadline:
            pose = self.get_tcp_pose()
            if pose is not None:
                pos = pose[0]
                dist = sum((p - t) ** 2 for p, t in zip(pos, target_m, strict=True)) ** 0.5
                if dist <= 0.002:  # 2 mm
                    return True
            time.sleep(0.05)
        print(f"[jaka_arm] wait_pose_reached timeout ({self._move_timeout_s}s)",
              flush=True)
        return False

    def _solve_ik_deg(self, pose_mm_rpy: list[float], ref_rad: list[float]) -> list[float] | None:
        """Controller IK with limit repair: nearest branch, then wrist flip,
        then the known-good grasp seed (AIIT strategy)."""
        for seed in (ref_rad, [np.radians(a) for a in GRASP_IK_SEED_DEG]):
            with self._lock:
                if not self._connected:
                    return None
                ret = self._rc.kine_inverse(seed, pose_mm_rpy)
            if ret[0] != 0:
                continue
            solution = [float(np.degrees(a)) for a in ret[1]]
            if self._within_limits([float(np.radians(a)) for a in solution]):
                return solution
            flipped = self._wrist_flip(solution)
            if self._within_limits([float(np.radians(a)) for a in flipped]):
                return flipped
        return None

    @staticmethod
    def _wrist_flip(angles_deg: list[float]) -> list[float]:
        """Wrist-equivalent configuration (j4∓180, -j5, j6±180); same TCP pose."""
        flip = 180.0 if angles_deg[3] > 0 else -180.0
        return [
            angles_deg[0], angles_deg[1], angles_deg[2],
            angles_deg[3] - flip, -angles_deg[4], angles_deg[5] + flip,
        ]
