# jaka_arm capability notes

JAKA 6-DOF arm (jkrc SDK over TCP) with an end effector on the controller's
end RS485 channel: a TG-9801 adaptive gripper (`end_effector: gripper`, the
default) or a LinkerHand O6 six-axis dexterous hand (`end_effector: hand`).
Provider ID is the deploy instance `name` (default `jaka_arm`).

The arm and the end effector share one jkrc session and one RS485 signal
table. The controller permits only one active SDK session, so both families
are served by this one provider (see the package README).

## Frames and units

- **Base frame** is fixed (stationary arm) and is written to the
  `joint_states` header (`config.base_frame`, default `world`). It must match
  the root link of the deployment URDF in `soma.yaml`.
- `end_pose` / `pos_command` carry unstamped `geometry_msgs/Pose` in the
  **base frame**; positions are the controller's configured TCP pose. The TCP
  definition lives on the controller (user-configured), not in this package.
- Arm joints are revolute, in **rad**, names `joint_1`..`joint_6`.
- Gripper mode: `gripper_joint` is prismatic in **m** (0 = closed, `0.098` =
  fully open; TG-9801 stroke). Adaptive mode is binary: below half stroke
  closes to the configured force, at/above half stroke opens.
- Hand mode: `joint_states` carries the six O6 joints in **rad** (URDF names,
  0 = extended) instead of `gripper_joint`. The hand contracts use normalized
  **[0,1]** positions (0 = extended, 1 = fully flexed; thumb opposition 0 =
  neutral, 1 = full) and speed/torque limits (0 = minimum, 1 = maximum).

## Capabilities

| contract | dir | topic | notes |
|---|---|---|---|
| `robonix/primitive/arm/joint_states` | out | `/jaka/joint_states` | reliable; arm joints + end-effector state, ~50 Hz |
| `robonix/primitive/arm/end_pose` | out | `/jaka/end_pose` | ~50 Hz, base frame |
| `robonix/primitive/arm/joint_command` | in | `/jaka/joint_command` | **position-only write**; `velocity/effort` ignored, speed comes from config |
| `robonix/primitive/arm/pos_command` | in | `/jaka/pos_command` | IK on provider: linear move, falls back to controller IK + joint move |
| `robonix/primitive/hand/info` | rpc | — | hand mode only; six `JointInfo` axes |
| `robonix/primitive/hand/move_joint` | rpc | — | hand mode only; positions by axis name, normalized |
| `robonix/primitive/hand/set_joint_speed_limits` | rpc | — | hand mode only; per-axis ceilings |
| `robonix/primitive/hand/set_joint_torque_limits` | rpc | — | hand mode only; per-axis ceilings |
| `robonix/primitive/hand/state_joint` | out | `/jaka/hand/state_joint` | hand mode only; normalized positions |

Hand axes, in firmware order: `thumb_cmc_pitch` (thumb/bend),
`thumb_cmc_yaw` (thumb/oppose), `index_mcp_pitch`, `middle_mcp_pitch`,
`ring_mcp_pitch`, `pinky_mcp_pitch` (bend). `move_joint` accepts only listed
axes, rejects empty/unknown/duplicate/non-finite/out-of-range requests (never
clamps); `ok=true` means accepted and dispatched, not reached.

At activation the pose is unknown (a lost RS485 answer reads as fully flexed,
so a read is not a safe target seed): the first `move_joint` must list all six
axes; later requests may be partial and hold the unlisted axes. The
`gripper_joint` open/grip command is a full six-axis pose and seeds the same
cache.

## Command semantics

- `joint_command`: subset of `joint_1..joint_6` + optional end-effector entry
  allowed; missing arm joints keep their current values. The end effector is
  driven first, then the joint move. In hand mode the optional entry is still
  `gripper_joint` (metres, the unchanged skill's interface): at/above half the
  stroke opens the O6, below it runs the full-grasp pose. Rejected (logged)
  while `motion_enabled: false`.
- `pos_command`: quaternion converted to the controller's fixed-XYZ RPY
  convention (R = Rz·Ry·Rx). Near j5 singularity or linear failure falls back
  to `kine_inverse` with limit repair.
- Joint targets outside the soft limits are rejected before the SDK call.
- On `CMD_DEACTIVATE` / `CMD_SHUTDOWN` the provider aborts motion; shutdown
  additionally disables and powers off the arm. With `shutdown_home_joints`
  configured, shutdown first opens the end effector (hand mode included), then
  moves the arm to that pose (best-effort, bounded by `move_timeout_s`; see
  config.spec).

## Hand read path

The O6 read path is one single-entry RS485 signal-table poll per register
(~0.38 s; six axes ~2.3 s); `hand/state_joint` publishes about one sweep per
`hand_publish_period_s` cycle. The hand answers slowly (2~5 s) after a motion
and a lost answer reads as 0, so a sweep during that window can briefly report
a moving finger as fully flexed; read verdicts after the motion settles.
