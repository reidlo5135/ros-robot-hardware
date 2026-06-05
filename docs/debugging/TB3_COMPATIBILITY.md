# TB3 Compatibility Diagnostics

This guide explains how to compare `turtlebot3_bringup` and `ros-robot-hardware` outputs when AMR localization works with TurtleBot3 bringup but `map -> odom` distorts during rotation with robot hardware bringup.

## Why This Points At Hardware Semantics

If the same `ros-amr-navigation` stack localizes correctly with `turtlebot3_bringup`, the AMR localization configuration is less likely to be the primary bug. The replacement hardware stack must match the semantics that localization expects from TurtleBot3 bringup.

The important contracts are:

- `/scan` geometry, order, frame, timing, and obstacle direction.
- `/odom` frame IDs, twist signs, covariance, and yaw scale.
- `/imu` frame ID, covariance, angular velocity sign, and orientation yaw semantics.
- `/tf` and `/tf_static` ownership for the static and dynamic frame chain.

`robot_hardware` still must not publish `map -> odom`; that transform belongs to localization.

## Capture Baselines

Capture TurtleBot3 bringup first:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
./scripts/compare_tb3_compatibility.sh --samples 10 > ~/ws/logs/tb3_baseline.txt
```

Capture robot hardware bringup next:

```bash
ros2 launch robot_bringup robot.launch.py debug_tf:=true debug_odom:=true debug_scan_geometry:=true
./scripts/compare_tb3_compatibility.sh --samples 10 > ~/ws/logs/robot_hw_compat.txt
```

Compare:

```bash
diff -u ~/ws/logs/tb3_baseline.txt ~/ws/logs/robot_hw_compat.txt
```

Focus first on `scan.ranges_length`, `scan.ranges_length.unique`, `scan.angle_min`, `scan.angle_max`, `scan.angle_increment`, `scan.angle_increment.unique`, `scan.front.index`, `scan.left.index`, `scan.right.index`, `scan.rear.index`, `scan.nearest.angle`, `odom.child_frame_id`, IMU covariance, and the four TF edges.

`compare_tb3_compatibility.sh` subscribes to `/scan` and `/imu` with SensorDataQoS-compatible settings: `BEST_EFFORT`, `VOLATILE`, `KEEP_LAST`, depth `10`. This matches TurtleBot3 `single_coin_d4_node`, TB3/OpenCR-style IMU topics, and robot_hw sensor publishers without reliability QoS mismatch warnings. `/odom` keeps the script's default subscription QoS.

The snapshot prints the assumed subscriber QoS and discovered publisher QoS for `/scan` and `/imu`. If a RELIABILITY QoS warning appears for `/imu`, treat that first as a comparison-script subscription QoS problem, not proof that the IMU publisher is broken. Inspect the publisher with:

```bash
ros2 topic info -v /imu
```

## LaserScan Geometry

v0.1.10 defaults to TurtleBot3/Coin D4/LDS-03 style scan geometry through `scan_geometry_profile: "tb3_coin_d4"`:

```yaml
scan_geometry_profile: "tb3_coin_d4"
fixed_scan_geometry: true
fixed_scan_samples: 400
fixed_angle_min: 0.0
fixed_angle_max: 6.283185307179586
fixed_scan_time: 0.1
fixed_time_increment: 0.0
mirror_scan_angles: true
```

The TB3 profile publishes `/scan` with `frame_id=base_scan`, `angle_min=0`, `angle_max=2*pi`, and `angle_increment=2*pi/fixed_scan_samples`. With the default 400 samples, expected cardinal indexes are front `0`, left `100`, rear `200`, and right `300`. A right-side obstacle should therefore affect the right index range, not the left index range.

Available scan profiles:

- `tb3_coin_d4`: TB3-compatible default, fixed 400 samples, `0..2*pi`, mirrored raw angle mapping.
- `ros_standard_360`: fixed 360 samples, `-pi..pi`, no angle mirroring.
- `legacy`: variable geometry, `-pi..pi`, no angle mirroring.
- `custom`: uses the explicit YAML values as-is.

When `fixed_scan_geometry=true`:

- `ranges.size` remains constant.
- `angle_min`, `angle_max`, `angle_increment`, and `scan_time` remain constant.
- Missing bins are filled with `+inf`.
- Partial scans are still published; a scan is not dropped only because some bins are missing.

If the TB3 baseline shows a different stable sample count, set `fixed_scan_samples` to match it, commonly `360` or `400`.

Watch:

```bash
./scripts/watch_robot_hw_logs.sh --tag SENSOR --event scan_geometry_stability --follow --file ~/ws/logs/robot_hw/latest.log
```

`result=warn` with `fixed_scan_geometry=true` means robot_hw is still emitting unstable LaserScan geometry and should be investigated before tuning localization.

## Scan Direction Check

Use a simple obstacle test:

1. Put a clear obstacle physically in front of the robot.
2. Under TurtleBot3 bringup, capture `scan.nearest.angle`, `scan.front.angle`, and `scan.front.range` from `compare_tb3_compatibility.sh`.
3. Under robot_hw, capture the same values.
4. They should match within a small tolerance.

Relevant parameters:

- `scan_geometry_profile`: selects the TB3, ROS-standard, legacy, or custom geometry contract.
- `mirror_scan_angles`: mirrors raw LiDAR angles before filling `ranges[]`; enabled by the TB3 profile to match TurtleBot3 left/right ordering.
- `scan_angle_offset`: rotates LaserScan data before filling `ranges[]`.
- `scan_direction_reversed`: reverses the published scan arrays.
- `reverse_scan`: secondary reversal flag combined with `scan_direction_reversed`.
- `scan_yaw_offset`: URDF/static TF yaw between `base_link` and `base_scan`.

Do not apply both `scan_angle_offset` and `scan_yaw_offset` blindly. Use `scan_angle_offset` when the LaserScan bins are rotated relative to the scan frame. Use `scan_yaw_offset` when the physical LiDAR frame is mounted with a yaw offset relative to `base_link`.

## IMU Compatibility

Robot hardware logs:

```text
event=imu_compatibility
```

Compare these fields with the TB3 baseline:

- `imu_frame_id`
- `imu_orientation_yaw_rad`
- `odom_yaw_rad`
- `odom_imu_yaw_delta_rad`
- `imu_angular_velocity_z`
- `odom_angular_z`
- `orientation_covariance_0`
- `orientation_covariance_8`

If AMR localization uses only `/imu.angular_velocity.z`, an absolute IMU yaw offset may be acceptable. If localization uses IMU orientation yaw as an absolute heading, the yaw offset must be calibrated, or the orientation covariance should indicate unavailable or low-trust orientation. Compare TurtleBot3 bringup `/imu` covariance with robot_hw `/imu` covariance before changing localization settings.

If the snapshot cannot receive `/imu`, it prints `imu.available=false`, `imu.timeout=true`, `imu.qos_profile=best_effort_sensor_data`, and `imu.error=timeout_waiting_for_imu`. With an active publisher, this points to discovery, namespace, or runtime publisher issues after the script's subscriber QoS has already been set to best-effort sensor data.

## Odom Compatibility

Robot hardware logs once at startup:

```text
event=odom_compatibility
```

Check:

- `odom_frame_id=odom`
- `child_frame_id=base_footprint`
- `odom_linear_scale=1.0` unless deliberately calibrated
- `odom_angular_scale=1.0` unless deliberately calibrated
- `wheel_separation` and `wheel_radius` match the intended TurtleBot3 model
- `tb3_odom_zero_covariance=true` when matching TurtleBot3 zero covariance behavior
- Pose and twist covariance are close to the TurtleBot3 baseline, or all zero when the TB3 zero covariance option is enabled

## TF Compatibility

Expected TF chain:

```text
map -> odom                          localization, not robot_hardware
odom -> base_footprint               robot_base_driver
base_footprint -> base_link          robot_state_publisher
base_link -> base_scan               robot_state_publisher
base_link -> imu_link                robot_state_publisher
```

Run:

```bash
./scripts/check_robot_hw_tf_publishers.sh
./scripts/compare_tb3_compatibility.sh --timeout 10
```

There should be no duplicate publisher for the same TF edge.

## AMR Map To Odom Check

After scan geometry, scan direction, odom, IMU, and TF match the TB3 baseline, run the same AMR localization scenario again.

During in-place rotation:

- `odom -> base_footprint` should rotate continuously with small translation drift.
- `/scan` should retain stable geometry and a stable front direction.
- `map -> odom` should correct gradually rather than distort heavily.

Only tune AMR localization parameters after the hardware output is compatible with the TB3 baseline.
