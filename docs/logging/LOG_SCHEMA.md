# Robot Hardware Structured Log Schema

This document defines the structured logging contract for `ros-robot-hardware` v0.1.10.

## Common Format

All structured hardware logs use one line with stable `key=value` fields:

```text
ROBOT_HW_LOG schema=v1 tag= component= event= key=value ...
```

Required common fields:

- `schema`: schema version. Current value is `v1`.
- `tag`: high-level filter tag.
- `component`: package or hardware component emitting the log.
- `event`: stable event name.

Recommended common fields:

- `node`: ROS node name.
- `namespace`: ROS namespace or `/`.
- `result`: `ok`, `warn`, `failed`, `accepted`, `rejected`, `published`, `configured`, or `scheduled`.
- `reason`: machine-readable reason when `result` is not `ok`.
- `duration_ms`: operation duration.
- `stamp_age_sec`: ROS message stamp age at log time.
- `throttle_sec`: throttle interval used for high-rate logs.

Structured logs use English key names and values. Do not use free-form Korean text inside `ROBOT_HW_LOG` lines.

## Tags

Canonical tags:

- `BRINGUP`
- `SENSOR`
- `BASE`
- `TF`
- `SERIAL`
- `ODOM`
- `IMU`
- `CMD`
- `JOINT`
- `DIAG`

BMS diagnostics currently use `tag=SENSOR component=bms` for parsed/rejected frame
state and `tag=SERIAL component=bms` for raw serial diagnostics.

## Canonical Field Names

Use one canonical name for the same concept:

- Use `distance_to_goal_m`, not mixed names such as `goal_dist`, `dist_goal`, or `distance_to_goal`.
- Use `linear_x` and `angular_z` for command velocity fields.
- Use `yaw_rad`, `roll_rad`, and `pitch_rad` for radians.
- Use `_m`, `_rad`, `_sec`, `_ms`, `_hz`, `_bytes`, and `_count` suffixes where units matter.
- Use `parent_frame` and `child_frame` for TF logs.
- Use `frame_id` and `child_frame_id` for ROS message frame fields.

## Throttle Rules

High-rate hardware logs must be throttled:

- Serial packet, raw bytes, parser state, and read throughput logs must be throttled.
- `/scan`, `/odom`, `/imu`, `/joint_states`, and TF publish summaries must be throttled.
- Raw serial packet logging must remain opt-in only: `log_raw_packet=false` for LiDAR and `log_serial_packets=false` for OpenCR by default.

Default throttle parameters are configured in `robot_bringup/config/robot.yaml` under each node's `logging:` group.

## LiDAR Events

`robot_lidar_driver` uses:

- `tag=SENSOR component=lidar event=sensor_config`
- `tag=SERIAL component=lidar event=serial_open`
- `tag=SERIAL component=lidar event=serial_reconnect`
- `tag=SERIAL component=lidar event=serial_error`
- `tag=SERIAL component=lidar event=serial_read_rate`
- `tag=SENSOR component=lidar event=packet_parse`
- `tag=SENSOR component=lidar event=packet_error`
- `tag=SENSOR component=lidar event=scan_publish`
- `tag=SENSOR component=lidar event=scan_geometry`
- `tag=SENSOR component=lidar event=scan_geometry_stability`
- `tag=TF component=lidar event=frame_config`

`scan_geometry` includes canonical rotation-debug fields:

- `scan_geometry_profile`, `angle_convention`, `mirror_scan_angles`
- `front_angle_rad`, `left_angle_rad`, `right_angle_rad`, `rear_angle_rad`
- `front_range_m`, `left_range_m`, `right_range_m`, `rear_range_m`
- `scan_angle_offset_rad`, `scan_direction_reversed`, and `reverse_scan`

During an RViz obstacle check, an obstacle physically in front of the robot should appear near `front_angle_rad=0` and should primarily affect `front_range_m`.

`scan_geometry_stability` reports whether LaserScan geometry is stable across frames:

- `scan_geometry_profile`, `tb3_compatibility_mode`, `angle_convention`
- `fixed_scan_geometry`, `fixed_scan_samples`
- `current_ranges`, `previous_ranges`, `ranges_size_changed`
- `angle_increment_rad`, `previous_angle_increment_rad`, `angle_increment_changed`
- `scan_time_sec`, `time_increment_sec`
- `tb3_front_index`, `tb3_left_index`, `tb3_right_index`, `tb3_rear_index`, `left_right_mapping_ok`
- `result`, `reason`

When `scan_geometry_profile=tb3_coin_d4`, expected cardinal indexes are front `0`, left `100`, rear `200`, and right `300` for the default 400-sample scan. When `fixed_scan_geometry=true`, `ranges.size`, `angle_min`, `angle_max`, `angle_increment`, `scan_time`, and `time_increment` should remain constant. Missing bins are represented by `+inf` ranges.

Example:

```text
ROBOT_HW_LOG schema=v1 tag=SENSOR component=lidar event=scan_publish node=robot_lidar_driver namespace=/ topic=/scan frame_id=base_scan stamp_age_sec=0.000123 ranges=400 valid_ranges=382 invalid_ranges=18 range_min_m=0.120 range_max_m=12.000 min_range_m=0.142 max_range_m=4.321 angle_min_rad=0.000000 angle_max_rad=6.283185 angle_increment_rad=0.015708 scan_time_sec=0.100000 publish_rate_hz=10.000 throttle_sec=1.000 result=ok
```

## Base/OpenCR Events

`robot_base_driver` and `OpencrClient` use:

- `tag=BASE component=opencr event=base_config`
- `tag=BASE component=opencr event=opencr_startup`
- `tag=BASE component=opencr event=opencr_probe`
- `tag=BASE component=opencr event=opencr_initial_state`
- `tag=BASE component=opencr event=opencr_ready`
- `tag=BASE component=opencr event=base_state`
- `tag=CMD component=opencr event=cmd_vel_received`
- `tag=CMD component=opencr event=cmd_vel_stamped_received`
- `tag=CMD component=opencr event=cmd_vel_rejected`
- `tag=CMD component=opencr event=cmd_vel_write`
- `tag=SERIAL component=opencr event=serial_state`
- `tag=SERIAL component=opencr event=dxl_transaction`
- `tag=SERIAL component=opencr event=dxl_status_packet`
- `tag=SERIAL component=opencr event=dxl_timeout`
- `tag=BASE component=opencr event=poll_cycle`
- `tag=BASE component=opencr event=poll_failure`
- `tag=BASE component=opencr event=poll_recovered`
- `tag=BASE component=opencr event=poll_timing`
- `tag=ODOM component=opencr event=odom_compatibility`
- `tag=ODOM component=opencr event=odom_publish`
- `tag=ODOM component=opencr event=rotation_state`
- `tag=DIAG component=opencr event=rotation_consistency`
- `tag=IMU component=opencr event=imu_publish`
- `tag=IMU component=opencr event=imu_compatibility`
- `tag=JOINT component=opencr event=joint_state_publish`
- `tag=TF component=base_tf event=frame_config`
- `tag=TF component=base_tf event=tf_chain_expected`
- `tag=TF component=base_tf event=tf_publish`

Example:

```text
ROBOT_HW_LOG schema=v1 tag=TF component=base_tf event=tf_publish node=robot_base_driver namespace=/ parent_frame=odom child_frame=base_footprint x=0.123000 y=-0.004000 z=0.000000 roll_rad=0.000000 pitch_rad=0.000000 yaw_rad=0.018000 source=wheel_odom publish_tf=true stamp_age_sec=0.000100 publish_rate_hz=20.000 throttle_sec=1.000 result=published
```

## BMS Events

`robot_bms_driver` uses:

- `tag=SENSOR component=bms event=bms_config`
- `tag=SENSOR component=bms event=bms_frame`
- `tag=SENSOR component=bms event=bms_timeout`
- `tag=SENSOR component=bms event=battery_publish`
- `tag=SERIAL component=bms event=bms_serial_open`
- `tag=SERIAL component=bms event=bms_raw_frame`

The default `bms.protocol=placeholder` does not publish fake battery values.
`battery_publish` is emitted only after a concrete parser returns a valid sample
for `/battery_state`.

Example:

```text
ROBOT_HW_LOG schema=v1 tag=SENSOR component=bms event=bms_config node=robot_bms_driver namespace=/ enabled=false port=/dev/robot/bms baudrate=9600 topic=/battery_state frame_id=base_link protocol=placeholder poll_interval_ms=1000 read_timeout_ms=100 frame_timeout_ms=250 publish_diagnostics=true log_raw_frames=false warn_timeout_ms=5000 result=disabled reason=bms_disabled
```

Odom calibration fields:

- `base_config` reports `odom_linear_scale` and `odom_angular_scale`.
- `odom_publish` reports the same active scale values with the current odom pose and twist.
- `odom.linear_scale` scales integrated linear displacement.
- `odom.angular_scale` scales wheel-derived `delta_theta` in odometry integration. It does not change command writing.

Compatibility diagnostic events:

- `odom_compatibility` reports `odom_frame_id`, `base_frame_id`, `child_frame_id`, `odom_linear_scale`, `odom_angular_scale`, `wheel_separation`, `wheel_radius`, `tb3_odom_zero_covariance`, effective `pose_covariance_diagonal`, effective `twist_covariance_diagonal`, `result`, and `reason`.
- `imu_compatibility` reports `imu_frame_id`, `imu_orientation_yaw_rad`, `odom_yaw_rad`, `odom_imu_yaw_delta_rad`, `imu_angular_velocity_z`, `odom_angular_z`, `orientation_covariance_0`, `orientation_covariance_8`, `result`, and `reason`.

Rotation diagnostic events:

- `rotation_state` reports `cmd_angular_z`, `odom_angular_z`, `integrated_yaw_rad`, `odom_yaw_rad`, `imu_yaw_rad`, `imu_angular_velocity_z`, `wheel_left_velocity`, `wheel_right_velocity`, `left_delta_m`, `right_delta_m`, `delta_theta_rad`, `dt_sec`, and `yaw_source`.
- `rotation_consistency` reports `cmd_angular_z`, `odom_angular_z`, `imu_angular_velocity_z`, `cmd_odom_sign_match`, `odom_imu_sign_match`, `cmd_imu_sign_match`, `odom_imu_yaw_delta_rad`, `odom_cmd_ratio`, `result`, and `reason`.
- Sign match fields are `true`, `false`, or `unknown`. `unknown` is used when a compared value is too small or unavailable.

Example:

```text
ROBOT_HW_LOG schema=v1 tag=DIAG component=opencr event=rotation_consistency node=robot_base_driver namespace=/ cmd_angular_z=0.500000 odom_angular_z=0.492000 imu_angular_velocity_z=0.488000 cmd_odom_sign_match=true odom_imu_sign_match=true cmd_imu_sign_match=true odom_imu_yaw_delta_rad=0.018000 odom_cmd_ratio=0.984000 throttle_sec=1.000 result=ok reason=none
```

## Robot Description Events

`robot_description/launch/description.launch.py` emits launch-side static frame config logs:

- `tag=TF component=description event=static_frame_config`

The runtime static transforms are published by `robot_state_publisher` from `robot_description/urdf/robot.urdf.xacro`.

## TF Responsibility

Expected runtime TF chain:

- `map -> odom`: localization, AMCL, or SLAM. Not published by `robot_hardware`.
- `odom -> base_footprint`: dynamic TF from `robot_base_driver` when `publish_tf=true`.
- `base_footprint -> base_link`: static TF from `robot_state_publisher`.
- `base_link -> base_scan`: static TF from `robot_state_publisher`, using `scan_yaw_offset`.
- `base_link -> imu_link`: static TF from `robot_state_publisher`.
- `base_link -> wheel_left_link/right_link`: `robot_state_publisher` with `/joint_states`.

`robot_hardware` must not publish `map -> odom`.
