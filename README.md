# primitive-jaka-mini_with_linker_hand_o6-arm_hand-rbnx — JAKA Mini arm + LinkerHand O6 primitive

Robonix primitive for the fixed-base **JAKA Mini** 6-DOF arm with a
**LinkerHand O6** six-axis dexterous hand, driven over TCP by the vendor jkrc
SDK. One process owns the controller's single SDK session and its end RS485
channel, and serves two contract families from the same provider id:

| family | mode | contracts |
|---|---|---|
| arm | always | `robonix/primitive/arm/{joint_states, joint_command, pos_command, end_pose}` |
| hand | `end_effector: hand` | `robonix/primitive/hand/{info, move_joint, state_joint, set_joint_speed_limits, set_joint_torque_limits}` |

`end_effector: gripper` (the default) keeps the TG-9801 behaviour and serves the
arm contracts only; the hand handlers answer `ok=false` ("hand mode not
enabled"). The deployment for the O6 sets `end_effector: hand`.

## Why one package for arm + hand

The O6 hangs on the controller's **end RS485L** channel (channel 1, Modbus RTU,
slave `0x27` right / `0x28` left), behind the same jkrc connection the arm
uses. The controller allows **one active SDK session**: a second process that
logs in makes the first process's `get_actual_joint_position` hang
indefinitely (measured 2026-10-22), and the controller's RS485 signal table is
global across sessions and misaligns with more than one entry. Splitting the
hand into its own process is therefore not viable; the arm and the hand share
one driver, one lock and one signal table here.

Hand state is read the driver-native way: the unchanged RS485 contracts
(above) in `controller` mode, or `hand/state_joint` on `hand_bus: usb`.

## Two hand buses (`hand_bus`)

The O6 asks the same Modbus RTU questions over two different wires:

| `hand_bus` | wire | read path | arm coupling |
|---|---|---|---|
| `controller` (default) | controller end **RS485L**, channel 1, behind the jkrc session | six single-entry signal-table polls, ~2.3 s/sweep | shares the jkrc connection, lock and signal table; needs the arm powered |
| `usb` | serial adapter on the NX (CH340: `1a86:7523`), e.g. through the hand's RS485-USB converter | one 6-register Modbus burst, ~40 ms | independent: no jkrc session, no signal table, works while the arm is off |

`hand_bus: usb` is the wiring to pick when the hand cable reaches the NX
directly (instead of the arm's end flange). It sets `hand_port` and keeps
`end_effector: hand`; the hand contracts, normalized units, open/grip poses
and `gripper_joint` mapping are identical to the controller bus. Over USB a
failed read is reported as `None` (the port has CRC + slave replies), so the
RS485 "lost answer reads as fully flexed" caveat does not apply.

Setup on the NX for `usb`:

1. The CH340 driver (`ch341`) provides `/dev/ttyUSB0`; confirm with
   `lsusb` (expect `1a86:7523`) and `ls /dev/ttyUSB*`.
2. The provider user must be in `dialout` (or the device chmod'd `666`, as
   the vendor SDK's note says). `rbnx` runs as the login user.
3. Prefer `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` as `hand_port`
   so a re-plug or a second adapter does not swap device numbers.

The controller-side constraint (one SDK session, one global signal table)
only applies to `hand_bus: controller`; on `usb` the arm and the hand no
longer contend for either.

## Arm contracts

| contract | dir | topic | notes |
|---|---|---|---|
| `robonix/primitive/arm/joint_states` | out | `/jaka/joint_states` | ~50 Hz; arm joints (rad) + `gripper_joint` (m, gripper mode) + the six hand joints (rad, hand mode) |
| `robonix/primitive/arm/end_pose` | out | `/jaka/end_pose` | TCP pose, base frame |
| `robonix/primitive/arm/joint_command` | in | `/jaka/joint_command` | position-only write; a `gripper_joint` entry drives open/grip |
| `robonix/primitive/arm/pos_command` | in | `/jaka/pos_command` | Cartesian target; IK on provider |

## Hand contracts (`end_effector: hand`)

| contract | dir | topic | notes |
|---|---|---|---|
| `robonix/primitive/hand/info` | rpc | — | six self-describing axes (`JointInfo`) |
| `robonix/primitive/hand/move_joint` | rpc | — | position by axis name, normalized `[0,1]` (1 = flexed) |
| `robonix/primitive/hand/set_joint_speed_limits` | rpc | — | per-axis speed ceilings |
| `robonix/primitive/hand/set_joint_torque_limits` | rpc | — | per-axis torque ceilings |
| `robonix/primitive/hand/state_joint` | out | `/jaka/hand/state_joint` | `hand/JointState`, normalized positions |

The O6 axes, in firmware order: `thumb_cmc_pitch` (thumb/bend),
`thumb_cmc_yaw` (thumb/oppose), `index_mcp_pitch`, `middle_mcp_pitch`,
`ring_mcp_pitch`, `pinky_mcp_pitch` (all bend). The distal joints are
mechanically coupled and are not exposed as separate axes. Positions are
normalized (`0` = extended, `1` = fully flexed; thumb opposition `0` =
neutral, `1` = full); speed/torque limits are `0` = minimum, `1` = maximum.
`move_joint` rejects empty/unknown/duplicate/out-of-range requests and never
clamps; `ok=true` means accepted and dispatched, not reached.

At activation the hand pose is **unknown**: an RS485 read cannot be trusted as
a target seed (the O6 drops answers and a lost answer reads as 0 = fully
flexed), so the first `move_joint` must list all six axes — later requests may
be partial and hold the unlisted axes. The unchanged skill's `gripper_joint`
open/grip is a full six-axis pose and seeds the same cache, so it needs no
special first call.

One full six-axis RS485 sweep costs ~2.3 s (one single-entry poll per register),
so `hand/state_joint` publishes roughly one sweep per `hand_publish_period_s`
cycle. The O6 answers slowly (2~5 s) after a motion; a lost answer reads as 0,
so a sweep during that window can briefly show a moving finger as fully
flexed. Judge grasp results after the motion settles. On `hand_bus: usb` a
sweep is one ~40 ms burst and a lost answer is `None`, so the stream runs at
the configured period and a moving finger is reported as such.

## The unchanged JAKA skill

In hand mode a `gripper_joint` entry in `joint_command` is still the skill's
grip interface: values at/above half the 98 mm stroke open the O6, values below
half run the full-grasp pose. The skill's five-publish burst is idempotent
(each publish re-issues the same open/grip command; the O6 settles on the same
pose). The skill reconstructs a width-like opening from those hand joints
(thumb/index/middle/ring flexion, the same 0.098 m scale and verdict
thresholds), so its empty-grasp/open checks keep working without a
`gripper_joint` entry. The O6's lost-answer clusters can still look fully
closed — the skill requires settled samples, but judge critical grasps with
care. The hand joints are also on `/jaka/hand/state_joint` and in
`joint_states`.

## Configuration

See [config.spec](config.spec). Essentials:

- `ip` — JAKA controller address.
- `end_effector` — `gripper` (TG-9801, default) or `hand` (O6).
- `hand_type` — `right` (`0x27`) or `left` (`0x28`), hand mode only.
- `hand_bus` — `controller` (controller end RS485L, default) or `usb`
  (serial adapter on the NX; the hand cable goes to the NX, not the flange).
- `hand_port` — serial port for `hand_bus: usb` (default `/dev/ttyUSB0`).
- `motion_enabled` — safety gate (default false); gates both arm and hand
  motions.
- `gripper_force` / `gripper_speed` — TG-9801 only (ignored in hand mode).
- `hand_publish_period_s` — hand state stream dwell.
- `shutdown_home_joints` — on teardown, the hand opens first, then the arm
  moves here and powers off.

## Safety

- `motion_enabled: false` by default: boot read-only, verify `joint_states`,
  `end_pose`, `hand/info` and `hand/state_joint`, then enable.
- Clear the workspace before commanding the O6; a grasp closes all five
  fingers. Lower the torque ceiling with `hand/set_joint_torque_limits` for the
  first live grasp.
- No other SDK client and no other RS485 owner may connect to the controller
  while this provider runs (a second login breaks the active session).

## Build

```bash
rbnx build -p /path/to/primitive-jaka-mini_with_linker_hand_o6-arm_hand-rbnx
```

Creates a package-local venv (`--system-site-packages`, so system rclpy is
visible), generates gRPC/MCP/ROS 2 types, and colcon-builds the ROS 2 message
overlay that `scripts/start.sh` sources.

## Tests

```bash
.venv/bin/python -m unittest discover -s jaka_arm/tests -t .
```

52 tests cover the arm driver (fake jkrc), the TG-9801 adapter, the O6 hand
driver/contract mapping, and the USB bus (fake Modbus client); no hardware is
required.

Bench smoke 2026-10-22 (JAKA Mini + right O6): one jkrc session served both
families, `joint_states` carried 6 arm + 6 hand joints, `hand/info` returned
six axes, a partial `move_joint` was rejected until a full six-axis target was
sent, and the arm feedback stayed healthy (no `-3` reads) while the hand moved.
The O6's lost-answer clusters still show up as brief `1.0` (fully flexed)
readings in the state streams — read a verdict after the motion settles.

## JAKA SDK binaries

The aarch64 jkrc SDK (`jkrc.so`, `libjakaAPI.so`, pyV2.1.7.0) ships in this
repository — Apache-2.0, added unmodified from the JAKA official download;
see [NOTICE](NOTICE) and `jaka_arm/jaka_sdk/LICENSE`.

For other platforms (`x86_64-linux-gnu`, `windows`), download the SDK from
the JAKA official download page and place both files under
`jaka_arm/jaka_sdk/<platform>/`; `scripts/build.sh` fails fast with these
instructions when they are missing.

## License

Apache-2.0. `jaka_arm/driver.py`, `jaka_arm/gripper.py` and
`jaka_arm/hand.py` carry MIT-adapted AIIT-Roboarm code — see
[NOTICE](NOTICE).
