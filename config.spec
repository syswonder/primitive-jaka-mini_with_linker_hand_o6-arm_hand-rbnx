config:
  # string, required, no default.
  # JAKA controller IP address (jkrc manages its own TCP ports).
  ip: 10.5.5.100

  # string, default: gripper. Which end effector is mounted on the RS485 end
  # channel. Both live on the controller's chn1 (RS485L), so exactly one is
  # constructed and driven:
  #   gripper — TG-9801 adaptive gripper (release/grip + position feedback);
  #   hand    — LinkerHand O6 six-axis dexterous hand (open/grip poses +
  #             angle feedback, per-axis control through the
  #             robonix/primitive/hand/* contracts).
  # In hand mode the hand contracts are served and joint_states carries the
  # six hand joints (URDF names, rad) instead of gripper_joint.
  end_effector: gripper

  # string, default: right. O6 hand side → RS485 slave id (right=0x27,
  # left=0x28). Only read when end_effector is hand; the startup direction
  # read warns when the wiring disagrees.
  hand_type: right

  # bool, default: false. SAFETY GATE.
  # When false, joint_command / pos_command are rejected with a rate-limited
  # warning; only feedback is published. Follow the staged acceptance flow:
  # boot with false (read-only arm), verify feedback and estop, then enable.
  motion_enabled: false

  # float, rad/s, default: 1.0; must be > 0.
  # Joint move speed passed to jkrc joint_move (100% ≈ 180 deg/s hardware max).
  joint_speed_rad_s: 1.0

  # float, mm/s, default: 100.0; must be > 0.
  # Cartesian linear_move speed.
  linear_speed_mm_s: 100.0

  # int, 0..100, default: 50.
  # TG-9801 target grip force (adaptive mode). Only read when
  # end_effector is gripper.
  gripper_force: 50

  # int, 200..1500, default: 1100.
  # TG-9801 adaptive grip/release speed. Only read when
  # end_effector is gripper.
  gripper_speed: 1100

  # float, deg^2, default: 1.0.
  # Mean-squared joint error below which a move counts as reached.
  reach_mse_threshold_deg2: 1.0

  # float, seconds, default: 10.0; must be > 0.
  # Timeout for joint/pose move completion waits.
  move_timeout_s: 10.0

  # list of 6 floats (rad), optional, default: unset (no motion on exit).
  # On provider shutdown (Ctrl-C / `rbnx shutdown`), the end effector opens
  # first (the O6 hand in hand mode, releasing any held object), then the six
  # arm joints move to this pose,
  # before the disconnect/power-off sequence — so the arm parks instead of
  # stopping wherever it was commanded to last. Best-effort: bounded by
  # move_timeout_s, failures are logged and power-off proceeds. Skipped
  # while motion_enabled is false.
  # shutdown_home_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  # float, Hz, default: 50.0.
  # joint_states / end_pose publish rate.
  state_publish_hz: 50.0

  # float, seconds, default: 0.5. Minimum period of the hand/state_joint
  # loop (end_effector: hand only). The effective rate is bounded from below
  # by the RS485 read path: one full 6-axis sweep costs ~2.3 s, so the
  # stream publishes at roughly one sweep per cycle.
  hand_publish_period_s: 0.5

  # string, default: world.
  # Frame id written to the joint_states header. Must match the root link of
  # the deployment's whole-robot URDF (soma.yaml urdf.root_link).
  base_frame: world

  # map, optional. Overrides for the ROS 2 topic names. Keys:
  # joint_states / joint_command / pos_command / end_pose / state_joint
  # (state_joint is hand mode only).
  # topics: { joint_states: /jaka/joint_states }
