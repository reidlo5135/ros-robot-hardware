# TF / Odom Runtime Debug Checklist

## Purpose

This checklist is for runtime verification of `cmd_vel`, `/odom`, `/joint_states`,
and TF behavior after bringup changes. It is intentionally command-focused and does
not include build steps.

## Commands

Check current `/cmd_vel` publishers:

```bash
ros2 topic info /cmd_vel -v
```

Motor-only bringup with debug logs:

```bash
ros2 launch robot_bringup motor.launch.py log_level:=debug
```

Direct forward command:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Direct rotation command:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Inspect odom:

```bash
ros2 topic echo /odom
ros2 topic hz /odom
```

Inspect joint states:

```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
```

Inspect TF:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic hz /tf
ros2 run tf2_ros tf2_echo map odom
```

Inspect full TF tree:

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /tf
```

## Expected Behavior

- Forward command should increase `/odom.pose.pose.position.x`.
- Forward command should keep yaw nearly stable.
- Rotate-left command should increase yaw counter-clockwise.
- `/odom.header.frame_id` should be `odom`.
- `/odom.child_frame_id` should be `base_footprint`.
- `/scan.header.frame_id` should be `base_scan`.
- `map -> odom` should come from exactly one localization source outside this repo.
- No teleop node should be running during Nav2 goal tests unless `twist_mux`
  or an equivalent arbitration layer is used.

## Notes

- If `command_mode` is changed away from `body_twist`, verify that the selected
  mode is actually supported before sending motion commands.
- If TF looks split, check whether a namespace was applied only to
  `robot_base_driver` while `robot_state_publisher` remained un-namespaced.

## Motor-only Odom/TF Isolation Test

Start motor-only bringup with debug logs:

```bash
ros2 launch robot_bringup motor.launch.py log_level:=debug
```

Send a direct forward command:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Inspect odom:

```bash
ros2 topic echo /odom
```

Inspect joint states:

```bash
ros2 topic echo /joint_states
```

Inspect odom to base footprint TF:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

Expected for forward:

- Left and right encoder deltas should have the same sign.
- `delta_s` should be positive.
- `delta_theta` should stay near zero.
- `x` should increase.
- `y` should stay near zero.
- `yaw` should stay near zero.
- `odom cmd compare` should show positive commanded and measured `linear_x`.

Expected for rotate left:

- Left and right encoder deltas should have opposite signs.
- `delta_s` should stay near zero.
- `delta_theta` should be positive.
- `yaw` should increase.
- `odom cmd compare` should show positive commanded and measured `angular_z`.

Expected for rotate right:

- `delta_theta` should be negative.
- `yaw` should decrease.
- `odom cmd compare` should show negative commanded and measured `angular_z`.

## Nav2 Retry Preconditions

Do not retry a Nav2 goal until these are true:

- Forward `cmd_vel` makes the real robot move forward.
- Forward `cmd_vel` increases `odom.x`.
- Positive `angular.z` increases odom yaw.
- `odom -> base_footprint` is published only once.
- `map -> odom` is published only once by localization, not by base bringup.
- `base_footprint -> base_link -> base_scan` comes from `robot_state_publisher`.
