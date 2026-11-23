# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""JakaHandUsb：经 USB 串口（CH340 → Modbus RTU）直连 O6 手，不经过 jkrc。

链路：PC → CH340（/dev/ttyUSB*，或 serial/by-id 软链）→ O6 手（Modbus RTU，
右 0x27 / 左 0x28，115200 8N1，与手在控制器末端 RS485L 上的从站参数一致）。

与 [hand.JakaHand]（控制器末端 RS485 通道）的差异只在链路层：
  · 不共享 jkrc.RC：USB 手是独立链路，不占控制器 SDK 会话、不碰 RS485
    信号量表，手写与臂可以并行；
  · 批量读写：角度/力矩/速度一次读写 6 个寄存器（功能码 04/16），一次
    六轴读约 40ms（30ms 帧间隔 + 串口往返），RS485 信号量表逐寄存器读
    一次要 2.3s；
  · 读失败直接报 None（协议层有 CRC 与功能码应答），不再有 RS485 上
    「丢应答回落 0 = 完全弯曲」的歧义——USB 上读到 0 就是真实的完全弯曲；
  · 写仍按「速度 → 力矩 → 目标角度」三帧（手在动作结束后会把速度寄存器
    清 0，三帧缺一不可），但每帧都有应答，无需 RS485 上的堵转补救节奏。

寄存器表、归一化换算、张开/抓取姿势都直接复用 hand.py 的常量与辅助函数：
O6 固件在两种接入方式下是同一套 Modbus 接口，两份副本迟早会漂移。
"""
import threading
import time

from pymodbus.client import ModbusSerialClient

from . import jaka_sdk  # noqa: F401  (jkrc on sys.path; hand.py imports it)
from .hand import (
    FINGER_COUNT,
    GRASP_ANGLES,
    OPEN_ANGLES,
    REG_ANGLE,
    REG_DIRECTION,
    REG_DOF,
    REG_SPEED,
    REG_TORQUE,
    _limit_to_raw,
    _norm_to_raw,
    _raw_to_limit,
    _raw_to_norm,
)


class JakaHandUsb:
    # Modbus 帧间隔：协议只要求 3.5 字符空闲，30ms 与厂商 SDK 一致且留足余量
    _FRAME_GAP_S = 0.03
    # 单次读/写失败（超时、CRC 错、从站异常应答）的前重试次数，含首次
    _ATTEMPTS = 3
    _READ_TIMEOUT_S = 0.2

    # 开合动作默认速度/力矩（0~255，与 hand.py 的 probe 值一致）
    DEFAULT_SPEED = 200
    DEFAULT_TORQUE = 200

    def __init__(self, port: str, slave_id: int = 0x27, baudrate: int = 115200):
        self.port = port
        self.slave_id = slave_id
        self.baudrate = baudrate
        self._lock = threading.RLock()
        self._last_ts = 0.0
        # 持久运动默认值：契约的 set_joint_*_limits 语义是「上限持久有效」，
        # 而每个动作都要三帧全写（手动作结束会清速度寄存器），所以默认值
        # 不能只写一次寄存器——存这里（归一化 [0,1]），由每次 _set_motion
        # 复用。None 表示从未设置，用 DEFAULT_*。
        self._motion_speed_norm = None
        self._motion_torque_norm = None
        self._client = None

    def _open(self) -> None:
        if self._client is not None:
            return
        client = ModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self._READ_TIMEOUT_S,
            handle_local_echo=False,
        )
        if not client.connect():
            raise ConnectionError(
                f"O6 USB 串口打开失败: {self.port}"
                "（查 CH340 驱动是否加载、设备是否在 /dev 下、当前用户在 dialout 组）")
        self._client = client

    def close(self) -> None:
        with self._lock:
            if self._client is not None:
                try:
                    self._client.close()
                except Exception:  # noqa: BLE001
                    pass
                self._client = None

    def power_on(self) -> None:
        """打开串口并等手就绪：读自由度(reg30)应为 6。

        手由 USB 供电，插入后重启，头几秒可能不应答，所以外层给足 ~10s。
        角度 0（握到底）也是合法值，不能作就绪判据。
        """
        self._open()
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            dof = self._read_regs(REG_DOF, 1, attempts=1)
            if dof == [FINGER_COUNT]:
                return
            time.sleep(0.5)
        print(f"Warning: O6 USB 手未就绪（自由度读不到 {FINGER_COUNT}），"
              f"检查 {self.port} 接线与从站号 0x{self.slave_id:02X}", flush=True)

    # ---------- Modbus 读写 ----------
    def _bus_free(self) -> None:
        """保证距离上一帧 ≥ 30ms（半双工链路空闲）。"""
        elapsed = time.perf_counter() - self._last_ts
        if elapsed < self._FRAME_GAP_S:
            time.sleep(self._FRAME_GAP_S - elapsed)

    def _read_regs(self, addr: int, count: int, attempts: int = None):
        """读 count 个输入寄存器（功能码 04），重试后仍失败返回 None。

        失败不打印：状态流按 hand_publish_period_s 调用，链路断开时逐次
        打印会刷屏；链路状态由 refresh_hand_positions 的 False 传播。
        """
        with self._lock:
            for _ in range(max(1, attempts or self._ATTEMPTS)):
                self._bus_free()
                try:
                    rsp = self._client.read_input_registers(
                        address=addr, count=count, slave=self.slave_id)
                    if not rsp.isError():
                        return [int(v) for v in rsp.registers]
                except Exception:  # noqa: BLE001
                    pass
                finally:
                    self._last_ts = time.perf_counter()
        return None

    def _write_regs(self, addr: int, values: list) -> int:
        """写 count 个保持寄存器（功能码 16）。返回 0 = 从站已应答确认。"""
        values = [int(v) for v in values]
        last_err = None
        with self._lock:
            for _ in range(self._ATTEMPTS):
                self._bus_free()
                try:
                    rsp = self._client.write_registers(
                        address=addr, values=values, slave=self.slave_id)
                    if not rsp.isError():
                        return 0
                    last_err = str(rsp)
                except Exception as exc:  # noqa: BLE001
                    last_err = str(exc)
                finally:
                    self._last_ts = time.perf_counter()
        print(f"Warning: O6 USB 写失败 addr={addr} values={values}: {last_err}",
              flush=True)
        return -1

    # ---------- 读 ----------
    def read_angle(self, finger: int, confirm_zero: bool = False):
        """读单指当前角度（0=完全弯曲 ~ 255=伸直），失败返回 None。

        confirm_zero 为 RS485 链路的丢答歧义保留的接口；USB 链路读失败
        直接是 None，读到 0 就是真实的完全弯曲，此处忽略该参数。
        """
        _ = confirm_zero
        if not 0 <= finger < FINGER_COUNT:
            raise ValueError(f"finger 需在 0~{FINGER_COUNT - 1}，实际 {finger}")
        values = self._read_regs(REG_ANGLE + finger, 1)
        return None if values is None else values[0]

    def read_angles(self) -> list:
        """一次读 6 指当前角度（单帧批量读，约 40ms），失败整帧返回 None。"""
        return self._read_regs(REG_ANGLE, FINGER_COUNT)

    def read_direction(self):
        """读方向：82='R'(右手) / 76='L'(左手)。"""
        values = self._read_regs(REG_DIRECTION, 1)
        return None if values is None else values[0]

    def read_dof(self):
        """读自由度（固定 6）。"""
        values = self._read_regs(REG_DOF, 1)
        return None if values is None else values[0]

    # ---------- 写 ----------
    @staticmethod
    def _check_values(values, name: str) -> list:
        values = [int(v) for v in values]
        if len(values) != FINGER_COUNT or not all(0 <= v <= 255 for v in values):
            raise ValueError(
                f"{name}需为 {FINGER_COUNT} 个 0~255 的整数，实际 {values}")
        return values

    def _set_motion(self, angles, speed=None, torque=None) -> int:
        """按「速度 → 力矩 → 目标角度」三帧写入，返回角度帧的返回码。

        未显式给 speed/torque 时用 _motion_speed_norm/_motion_torque_norm
        （set_speed_norm/set_torque_norm 的持久上限），未设置过用 DEFAULT_*。
        """
        angles = self._check_values(angles, "角度")
        with self._lock:
            speed_norm = self._motion_speed_norm
            torque_norm = self._motion_torque_norm
            if speed is None:
                speed = ([_limit_to_raw(v) for v in speed_norm]
                         if speed_norm is not None
                         else [self.DEFAULT_SPEED] * FINGER_COUNT)
            if torque is None:
                torque = ([_limit_to_raw(v) for v in torque_norm]
                          if torque_norm is not None
                          else [self.DEFAULT_TORQUE] * FINGER_COUNT)
            speed = self._check_values(speed, "速度")
            torque = self._check_values(torque, "力矩")
            ret_speed = self._write_regs(REG_SPEED, speed)
            ret_torque = self._write_regs(REG_TORQUE, torque)
            ret_angle = self._write_regs(REG_ANGLE, angles)
            if ret_angle != 0:
                return ret_angle
            return ret_speed or ret_torque
    def open(self, speed=None, torque=None) -> int:
        """张开：拇指对掌 + 其余四指伸直（OPEN_ANGLES）。"""
        return self._set_motion(OPEN_ANGLES, speed, torque)

    def grip(self, speed=None, torque=None) -> int:
        """握取：五指全闭 + 拇指保持对掌位（GRASP_ANGLES）。"""
        return self._set_motion(GRASP_ANGLES, speed, torque)

    def set_pose(self, angles, speed=None, torque=None) -> int:
        """写任意 6 指姿势（0=完全弯曲 ~ 255=伸直）。"""
        return self._set_motion(angles, speed, torque)

    # ---------- 归一化接口（robonix hand 契约单位） ----------
    def read_joint_pos(self) -> list:
        """读 6 轴当前角度 [0,1]（1=完全弯曲），失败返回 None。"""
        values = self.read_angles()
        if values is None:
            return None
        return [_raw_to_norm(v) for v in values]

    def set_joint_pos(self, positions) -> int:
        """写 6 轴目标角度 [0,1]（1=完全弯曲），超界拒绝（不截断）。"""
        return self.set_pose([_norm_to_raw(float(v)) for v in positions])

    def get_motion_speed_norm(self) -> list:
        """后续动作实际生效的持久速度上限 [0,1]（未设置过 = DEFAULT_SPEED）。

        不是寄存器读回：手在每次动作结束会把速度寄存器清 0，寄存器不携带
        「生效值」，以 _set_motion 复用的持久值为准。
        """
        with self._lock:
            if self._motion_speed_norm is not None:
                return list(self._motion_speed_norm)
        return [self.DEFAULT_SPEED / 255.0] * FINGER_COUNT

    def get_motion_torque_norm(self) -> list:
        """后续动作实际生效的持久力矩上限 [0,1]（未设置过 = DEFAULT_TORQUE）。"""
        with self._lock:
            if self._motion_torque_norm is not None:
                return list(self._motion_torque_norm)
        return [self.DEFAULT_TORQUE / 255.0] * FINGER_COUNT

    def set_speed_norm(self, speeds) -> int:
        """写 6 轴速度 [0,1]（1=最大），并作为后续动作的持久速度上限。"""
        norm = [float(v) for v in speeds]
        if len(norm) != FINGER_COUNT or not all(0.0 <= v <= 1.0 for v in norm):
            raise ValueError(
                f"速度需为 {FINGER_COUNT} 个 0~1 的值，实际 {speeds}")
        with self._lock:
            self._motion_speed_norm = list(norm)
            return self._write_regs(REG_SPEED, [_limit_to_raw(v) for v in norm])

    def set_torque_norm(self, torques) -> int:
        """写 6 轴力矩 [0,1]（1=最大），并作为后续动作的持久力矩上限。"""
        norm = [float(v) for v in torques]
        if len(norm) != FINGER_COUNT or not all(0.0 <= v <= 1.0 for v in norm):
            raise ValueError(
                f"力矩需为 {FINGER_COUNT} 个 0~1 的值，实际 {torques}")
        with self._lock:
            self._motion_torque_norm = list(norm)
            return self._write_regs(REG_TORQUE, [_limit_to_raw(v) for v in norm])
