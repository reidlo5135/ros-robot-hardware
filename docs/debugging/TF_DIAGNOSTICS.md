# TF Diagnostics Workflow

This guide documents the TF responsibilities of `ros-robot-hardware` v0.1.9.

## Expected TF Chain

Runtime chain:

```text
map -> odom                          localization / AMCL / SLAM, not robot_hardware
odom -> base_footprint               robot_base_driver dynamic TF
base_footprint -> base_link          robot_state_publisher static TF
base_link -> base_scan               robot_state_publisher static TF
base_link -> imu_link                robot_state_publisher static TF
base_link -> wheel_left_link         robot_state_publisher + /joint_states
base_link -> wheel_right_link        robot_state_publisher + /joint_states
```

`robot_hardware` must not publish `map -> odom`. That transform belongs to localization/navigation.

## What Each Package Publishes

`robot_base_driver`:

- Publishes `/odom` with `frame_id=odom` and `child_frame_id=base_footprint` by default.
- Publishes `/tf` for `odom -> base_footprint` when `publish_tf=true`.
- Publishes `/imu` with `frame_id=imu_link` when `publish_imu=true`.
- Publishes `/joint_states` when `publish_joint_states=true`.
- Logs `tag=TF component=base_tf event=tf_publish` with `source=wheel_odom` or `source=imu`.

`robot_description`:

- Runs `robot_state_publisher`.
- Defines `base_footprint -> base_link`.
- Defines `base_link -> base_scan` using `scan_yaw_offset`.
- Defines `base_link -> imu_link`.
- Defines wheel links and joints.
- Emits launch-side `tag=TF component=description event=static_frame_config` logs.

`robot_lidar_driver`:

- Publishes `/scan` with `frame_id=base_scan` by default.
- Logs `tag=TF component=lidar event=frame_config` to compare the scan frame with the robot description expectation.

## Bringup Commands

Normal bringup:

```bash
ros2 launch robot_bringup robot.launch.py
```

TF-focused bringup:

```bash
ros2 launch robot_bringup robot.launch.py debug_tf:=true
```

Odom-focused bringup:

```bash
ros2 launch robot_bringup robot.launch.py debug_odom:=true
```

Sensor geometry bringup:

```bash
ros2 launch robot_bringup robot.launch.py debug_scan_geometry:=true
```

Full field debug:

```bash
ros2 launch robot_bringup robot.launch.py debug_tf:=true debug_odom:=true debug_scan_geometry:=true
```

## Useful Log Filters

TF logs:

```bash
./scripts/watch_robot_hw_logs.sh --tag TF --follow --file ~/ws/logs/robot_hw/latest.log
```

Dynamic TF publish only:

```bash
./scripts/watch_robot_hw_logs.sh --tag TF --event tf_publish --follow --file ~/ws/logs/robot_hw/latest.log
```

Scan frame and geometry:

```bash
./scripts/watch_robot_hw_logs.sh --tag SENSOR --event scan_geometry --follow --file ~/ws/logs/robot_hw/latest.log
```

## Basic TF Checks

Run:

```bash
./scripts/check_tf_chain.sh
```

Expected results:

- `odom -> base_footprint` should be available when `robot_base_driver` is running and `publish_tf=true`.
- `base_footprint -> base_link` should be available when `robot_state_publisher` is running.
- `base_link -> base_scan` should be available and should reflect `scan_yaw_offset`.
- `base_link -> imu_link` should be available when the URDF is loaded.
- `map -> odom` is intentionally absent until localization/navigation is active.

## Rotation Drift Workflow

When straight driving is stable but `map -> odom` becomes distorted during rotation, diagnose in this order:

1. Confirm odom yaw sign. During a positive rotate-in-place `/cmd_vel.angular.z`, `event=rotation_consistency` should report `cmd_odom_sign_match=true`.
2. Confirm odom yaw scale. Compare physical 90, 180, or 360 degree rotations with `/odom` yaw, `tf2_echo odom base_footprint`, and `event=rotation_state delta_theta_rad`.
3. Confirm IMU yaw sign and frame. `odom_imu_sign_match` and `cmd_imu_sign_match` should be `true` during active rotation when IMU data is available.
4. Confirm `base_scan` static yaw. In RViz, an obstacle physically in front of the robot should appear in front of the robot model, and `event=scan_geometry` should show `front_angle_rad` near `0` with the closest front obstacle reflected in `front_range_m`.
5. Confirm scan direction flags. Check `scan_angle_offset_rad`, `scan_direction_reversed`, and `reverse_scan` in `event=scan_geometry` before changing localization parameters.
6. Check duplicate TF publishers. Run `./scripts/check_robot_hw_tf_publishers.sh` and verify only the expected owner publishes each TF edge.
7. Tune AMR localization only after hardware sign, scale, scan yaw, and TF ownership are correct.

Useful rotation test:

```bash
./scripts/test_rotation_diagnostics.sh --bag --duration 20
./scripts/test_rotation_diagnostics.sh --publish --angular-z 0.5 --duration 10 --bag
```

The first command records data without commanding motion. The second command publishes a rotate-in-place command and should only be used in a safe test area.

## Wheel Odom Rotation Calibration

Use `odom.angular_scale` for small calibration changes when wheel odom yaw scale is off and command delivery is already correct.

- If odom yaw changes more than physical yaw, increase effective `wheel_separation` or reduce `odom.angular_scale`.
- If odom yaw changes less than physical yaw, decrease effective `wheel_separation` or increase `odom.angular_scale`.
- Test with 90, 180, and 360 degree rotations in both directions.

Use `odom.linear_scale` only for linear distance scale checks. Do not use it to fix rotation distortion.

## Misconfiguration Warnings

Structured warnings are emitted when:

- `publish_tf=false` while navigation expects `odom -> base_footprint`.
- `odom_frame_id` is empty.
- `base_frame_id` is empty.
- `odom_frame_id == base_frame_id`.
- `base_frame_id` does not match the `base_footprint` robot description assumption.
- `imu_frame_id` is empty while `publish_imu=true`.
- LiDAR `frame_id` does not match the `base_scan` robot description assumption.
- A namespace is applied inconsistently between raw configured frames and resolved frames.

## AMR Navigation Integration

`robot_hardware` provides:

- `/scan`
- `/odom`
- `/imu`
- `/joint_states`
- `/tf` for `odom -> base_footprint`
- `/tf_static` for `base_footprint -> base_link -> base_scan / imu_link`

`ros-amr-navigation` provides or depends on:

- `map -> odom` from localization.
- `/cmd_vel` command output.

Do not add a `map -> odom` publisher to `robot_hardware`.
