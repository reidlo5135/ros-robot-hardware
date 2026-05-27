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
```

Inspect joint states:

```bash
ros2 topic echo /joint_states
```

Inspect TF:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

Inspect full TF tree:

```bash
ros2 run tf2_tools view_frames
```

## Expected Behavior

- Forward command should increase `/odom.pose.pose.position.x`.
- Forward command should keep yaw nearly stable.
- Rotate-left command should increase yaw counter-clockwise.
- `/odom.header.frame_id` should be `odom`.
- `/odom.child_frame_id` should be `base_footprint`.
- `/scan.header.frame_id` should be `base_scan`.
- No teleop node should be running during Nav2 goal tests unless `twist_mux` or an equivalent arbitration layer is used.

## Notes

- If `command_mode` is changed away from `body_twist`, verify that the selected mode is actually supported before sending motion commands.
- If TF looks split, check whether a namespace was applied only to `robot_base_driver` while `robot_state_publisher` remained un-namespaced.
