# TB3 Bringup Compatibility Diagnosis

## Scope

This diagnosis inspects only the current `ros-robot-hardware` repository.

Standard TurtleBot3 Burger references available in this environment:

- `/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf`
- `robot_lidar_driver/src/robot_lidar_driver/lds03_parser.cpp`
  The parser comment states it was verified against the TurtleBot3 Humble
  `coin_d4_driver` M1CT_TOF parser path.

Files and functions inspected:

- `robot_lidar_driver/src/robot_lidar_driver/lds03_parser.cpp`
  `Lds03Parser::decodePacket()`
- `robot_lidar_driver/src/robot_lidar_driver/laser_scan_builder.cpp`
  `LaserScanBuilder::buildScan()`
- `robot_lidar_driver/src/robot_lidar_driver/lidar_driver_node.cpp`
  `publishCompletedScans()`, `publishMockScan()`
- `robot_description/urdf/robot.urdf.xacro`
- `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp`
  `publishOdometry()`, `publishImu()`
- `robot_bringup/config/robot.yaml`
- `robot_base_driver/config/base.yaml`
- `robot_lidar_driver/config/lidar.yaml`

## Scan Compatibility

### Current `/scan` semantics

- `header.frame_id`
  `base_scan`
- `timestamp source`
  `completed_scan.stamp`, which is set in `Lds03Parser::consume()` when a full
  ring is completed
- `scan_time` / `time_increment`
  derived from configured `publish_rate_hint_hz`, not from the measured
  `completed_scan.scan_frequency_hz`
- `angle_min`
  `-pi`
- `angle_max`
  `+pi`
- `angle_increment`
  positive, computed as `(angle_max - angle_min) / (bin_count - 1)`
- `range_min`
  `0.12`
- `range_max`
  `12.0`
- `ranges ordering`
  produced by angle binning in `LaserScanBuilder::buildScan()`
- `ranges[0]`
  corresponds to approximately `-pi`, so it is a rear-facing ray, not the front
- front ray
  near the middle of the array, around the index for `0 rad`

### Comparison to standard TB3 LDS expectations

- `frame_id = base_scan`
  matches TB3 convention
- `angle_min ~= -pi`, `angle_max ~= +pi`
  matches common TB3 LDS convention
- positive `angle_increment`
  matches standard `sensor_msgs/msg/LaserScan` usage
- `ranges[0]` is rear, not front
  this is normal for a `[-pi, +pi]` scan and is not itself a bug

### Compatibility risk

The likely scan risk is not the nominal `LaserScan` envelope. The likely risk is:

- the physical LiDAR is yaw-rotated relative to `base_scan`, or
- the raw scan ordering is mirrored relative to the robot front

That would make AMCL or other localization rotate `map -> odom` aggressively
even when `/odom` itself looks reasonable.

Secondary scan risk:

- `scan_time` and `time_increment` are currently compatibility approximations
  based on `publish_rate_hint_hz`
- exact TB3 binary behavior was not modified here

## TF Compatibility

### `base_link -> base_scan`

Current custom URDF:

- `robot_description/urdf/robot.urdf.xacro`
- `origin xyz="-0.032 0 0.172" rpy="0 0 0"`

Standard TB3 Burger in this environment:

- `/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf`
- `origin xyz="-0.032 0 0.172" rpy="0 0 0"`

Judgment:

- `base_scan` fixed transform currently matches standard TB3 Burger convention
- this specific transform is unlikely to be the main cause of the observed
  `map -> odom` rotation

### `base_link -> imu_link`

Current custom URDF:

- `origin xyz="-0.032 0 0.068" rpy="0 0 0"`

Standard TB3 Burger in this environment:

- `origin xyz="-0.032 0 0.068" rpy="0 0 0"`

Judgment:

- matches TB3 Burger convention

## Odom Covariance

Current `/odom` covariance is not all-zero.

Configured defaults:

- `odom_pose_covariance_diagonal = [0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.05]`
- `odom_twist_covariance_diagonal = [0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.05]`

Applied in:

- `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp`
  `publishOdometry()`

Judgment:

- odom covariance is already conservative and configurable
- it is unlikely that all-zero odom covariance is the present localization issue

## IMU Covariance

Before this change, `publishImu()` filled orientation, angular velocity, and
linear acceleration values but left all three covariance matrices at their
message default of all zeros.

Risk:

- all-zero IMU covariance can be interpreted as unrealistically certain data by
  downstream consumers
- even if localization does not fuse IMU directly, all-zero covariance is poor
  compatibility behavior

Implemented change:

- added configurable IMU covariance matrices in
  `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp`
  and `robot_base_driver/include/robot_base_driver/robot_base_driver_node.hpp`
- added YAML defaults in:
  - `robot_bringup/config/robot.yaml`
  - `robot_base_driver/config/base.yaml`

Current defaults:

- `imu_orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.01]`
- `imu_angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.04]`
- `imu_linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]`

Exact stock TB3 binary IMU covariance values remain `UNCERTAIN` in this
environment, so these defaults should be treated as conservative compatibility
values rather than a byte-for-byte TB3 clone.

## Most Likely Cause Of `map -> odom` Rotation

Most likely code-level cause in this hardware repo:

1. Scan-to-base semantic mismatch rather than odom math.
2. Specifically, a front-direction mismatch in `/scan` caused by physical LiDAR
   mounting orientation, mirrored scan ordering, or a required yaw offset that
   is not represented in the current scan publication path.

Why this is the leading suspect:

- `odom -> base_footprint` already looks numerically reasonable
- current `base_scan` fixed transform matches standard TB3 Burger
- current `/scan` envelope also matches standard TB3-style `[-pi, +pi]`
- the remaining high-impact incompatibility is whether the data inside the scan
  array is aligned to the robot front the way TB3 localization expects

## Implemented Diagnostics

Added to `robot_lidar_driver`:

- `debug_scan_geometry: bool = false`
- `scan_angle_offset: double = 0.0`
- `reverse_scan: bool = false`

When `debug_scan_geometry=true`, the driver logs:

- `angle_min`
- `angle_max`
- `angle_increment`
- `ranges.size`
- computed index for front, left, right, rear
- the range currently observed at those directions

These diagnostics were added in:

- `robot_lidar_driver/src/robot_lidar_driver/lidar_driver_node.cpp`
- `robot_lidar_driver/include/robot_lidar_driver/lidar_driver_node.hpp`
- `robot_lidar_driver/src/robot_lidar_driver/laser_scan_builder.cpp`
- `robot_lidar_driver/include/robot_lidar_driver/laser_scan_builder.hpp`

## Recommended Next Check

1. Launch sensor-only bringup with `debug_scan_geometry=true`.
2. In RViz, set `Fixed Frame=base_link`.
3. Place an obstacle directly in front of the robot.
4. Verify that the obstacle appears in front in the `LaserScan` display.
5. If not, try:
   - `reverse_scan: true`
   - `scan_angle_offset: 3.141592653589793`
   - `scan_angle_offset: 1.5707963267948966`
   - `scan_angle_offset: -1.5707963267948966`

## Overall Judgment

Within this hardware repo, the strongest remaining TB3 compatibility risk is
scan direction / scan yaw semantics, not the nominal TF tree shape and not
all-zero odom covariance.
