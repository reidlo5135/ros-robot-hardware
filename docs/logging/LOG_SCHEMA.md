# Robot Hardware Structured Log Schema

This document defines the structured logging contract for `ros-robot-hardware` v0.1.8.

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
- `tag=TF component=lidar event=frame_config`

Example:

```text
ROBOT_HW_LOG schema=v1 tag=SENSOR component=lidar event=scan_publish node=robot_lidar_driver namespace=/ topic=/scan frame_id=base_scan stamp_age_sec=0.000123 ranges=360 valid_ranges=342 invalid_ranges=18 range_min_m=0.120 range_max_m=12.000 min_range_m=0.142 max_range_m=4.321 angle_min_rad=-3.141593 angle_max_rad=3.141593 angle_increment_rad=0.017502 scan_time_sec=0.100000 publish_rate_hz=10.000 throttle_sec=1.000 result=ok
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
- `tag=ODOM component=opencr event=odom_publish`
- `tag=IMU component=opencr event=imu_publish`
- `tag=JOINT component=opencr event=joint_state_publish`
- `tag=TF component=base_tf event=frame_config`
- `tag=TF component=base_tf event=tf_chain_expected`
- `tag=TF component=base_tf event=tf_publish`

Example:

```text
ROBOT_HW_LOG schema=v1 tag=TF component=base_tf event=tf_publish node=robot_base_driver namespace=/ parent_frame=odom child_frame=base_footprint x=0.123000 y=-0.004000 z=0.000000 roll_rad=0.000000 pitch_rad=0.000000 yaw_rad=0.018000 source=wheel_odom publish_tf=true stamp_age_sec=0.000100 publish_rate_hz=20.000 throttle_sec=1.000 result=published
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
