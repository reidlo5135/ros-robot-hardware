# Robot Hardware Logging Validation Checklist

Use this checklist after launching `robot_bringup` with structured logging enabled.

## Sensor

- [ ] `/scan` is published.
- [ ] `/scan` uses `frame_id=base_scan` or the expected namespaced frame.
- [ ] `event=scan_publish` appears at the configured throttle rate.
- [ ] `event=scan_geometry` confirms the scan angle direction is correct when `debug_scan_geometry=true`.
- [ ] `event=scan_geometry` reports plausible `front_range_m`, `left_range_m`, `right_range_m`, and `rear_range_m` values.
- [ ] Serial read rate is stable with `event=serial_read_rate`.
- [ ] `event=packet_parse` does not show a growing packet error flood.
- [ ] Raw packet logging remains disabled unless explicitly requested with `log_raw_packet=true`.

## Base

- [ ] `/cmd_vel` subscriber exists.
- [ ] `/cmd_vel_stamped` subscriber exists only when `enable_stamped_cmd_vel=true`.
- [ ] `/odom` is published.
- [ ] `/imu` is published when `publish_imu=true` and OpenCR IMU data is available.
- [ ] `/joint_states` is published when `publish_joint_states=true`.
- [ ] `event=cmd_vel_received` changes when navigation sends commands.
- [ ] `event=odom_publish` reports plausible `x`, `y`, `yaw_rad`, `vx`, and `wz`.
- [ ] `event=rotation_state` appears during rotation and reports command, odom, IMU, wheel, and yaw-source fields.
- [ ] `event=rotation_consistency` reports `cmd_odom_sign_match=true` during commanded rotate-in-place tests.
- [ ] `odom.angular_scale` and `odom.linear_scale` are `1.0` unless a measured calibration change is intentionally applied.
- [ ] Wheel encoder signs are correct for forward and rotation tests.
- [ ] `command_mode` is the intended mode, normally `body_twist`.
- [ ] Raw OpenCR packet logging remains disabled unless explicitly requested with `log_serial_packets=true`.

## TF

- [ ] `odom -> base_footprint` exists.
- [ ] `base_footprint -> base_link` exists.
- [ ] `base_link -> base_scan` exists.
- [ ] `base_link -> imu_link` exists.
- [ ] `event=tf_publish` identifies `robot_base_driver` as the `odom -> base_footprint` publisher.
- [ ] `event=static_frame_config` identifies `robot_state_publisher` static frame assumptions.
- [ ] `map -> odom` is not published by `robot_hardware`.
- [ ] No duplicate TF publishers exist for the same parent/child pair.
- [ ] `./scripts/check_robot_hw_tf_publishers.sh` shows expected TF topic publishers and no robot_hardware `map -> odom` publisher.
- [ ] `source=wheel_odom` or `source=imu` matches the configured yaw source.

## Integration With ros-amr-navigation

- [ ] AMR controller publishes `/cmd_vel`.
- [ ] `robot_base_driver` subscribes `/cmd_vel`.
- [ ] AMR localization receives `/scan` and `/odom`.
- [ ] RViz fixed frame `map` works when localization is active.
- [ ] RViz fixed frame `odom` works during hardware-only bringup.
- [ ] TF tree is continuous from `map` to `base_scan` when localization is active.
- [ ] `robot_hardware` does not publish `map -> odom`.
