# TurtleBot3 Compatibility Diagnosis

## Scope

This report compares the current custom hardware stack against the standard
TurtleBot3 Burger bringup conventions available in this environment:

- TB3 launch: `/opt/ros/humble/share/turtlebot3_bringup/launch/robot.launch.py`
- TB3 state publisher launch: `/opt/ros/humble/share/turtlebot3_bringup/launch/turtlebot3_state_publisher.launch.py`
- TB3 params: `/opt/ros/humble/share/turtlebot3_bringup/param/humble/burger.yaml`
- TB3 URDF: `/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf`

Current custom implementation was inspected in:

- `robot_base_driver/src/robot_base_driver/odometry_integrator.cpp`
- `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp`
- `robot_lidar_driver/src/robot_lidar_driver/lidar_driver_node.cpp`
- `robot_lidar_driver/src/robot_lidar_driver/laser_scan_builder.cpp`
- `robot_description/urdf/robot.urdf.xacro`

## TB3 Expected

| Item | TB3 Burger convention | Source |
| --- | --- | --- |
| `/odom.header.frame_id` | `odom` | `burger.yaml` `diff_drive_controller.odometry.frame_id` |
| `/odom.child_frame_id` | `base_footprint` | `burger.yaml` `diff_drive_controller.odometry.child_frame_id` |
| `odom -> base_footprint` TF | Published by base odometry stack | `burger.yaml` `publish_tf: true` |
| `base_footprint -> base_link` | Fixed transform, `xyz=(0, 0, 0.010)` | `turtlebot3_burger.urdf:52-56` |
| `base_link -> base_scan` | Fixed transform, `xyz=(-0.032, 0, 0.172)` | `turtlebot3_burger.urdf:170-174` |
| `base_link -> imu_link` | Fixed transform, `xyz=(-0.032, 0, 0.068)` | `turtlebot3_burger.urdf:162-166` |
| `/scan.header.frame_id` | `base_scan` | `robot.launch.py` passes `frame_id='base_scan'` to lidar launch |
| `/imu.header.frame_id` | `imu_link` | `turtlebot3_ros` binary strings expose `imu_link`; URDF uses `imu_link` |
| `/joint_states` wheel joint names | `wheel_left_joint`, `wheel_right_joint` | TB3 URDF and `turtlebot3_ros` binary strings |
| Odom covariance defaults | `UNCERTAIN` in this environment; `turtlebot3_ros` is installed as a binary, not source | Could not verify from installed binary |
| Odom publish rate | `UNCERTAIN` exact default from installed binary | Could not verify from installed binary |
| TF publish rate | `UNCERTAIN` exact default from installed binary | Could not verify from installed binary |

## Current Implementation

| Item | Current implementation | Risk |
| --- | --- | --- |
| `/odom.header.frame_id` | `odom` via `OdometryIntegrator::buildOdometryMessage()` | Low |
| `/odom.child_frame_id` | `base_footprint` via `OdometryIntegrator::buildOdometryMessage()` | Low |
| `/odom.pose.covariance` | Previously all-zero, now configurable non-zero diagonal via `RobotBaseDriverNode::publishOdometry()` | High before fix, reduced after fix |
| `/odom.twist.covariance` | Previously all-zero, now configurable non-zero diagonal via `RobotBaseDriverNode::publishOdometry()` | High before fix, reduced after fix |
| `/imu.header.frame_id` | `imu_link` via `RobotBaseDriverNode::publishImu()` | Low |
| `/imu` covariance semantics | Previously `-1` for all three covariance groups while still publishing valid data; now left at all-zero meaning unknown covariance | Medium before fix, reduced after fix |
| `/scan.header.frame_id` | `base_scan` via `LaserScanBuilder::buildScan()` | Low |
| `/scan.header.stamp` | Previously `now()` at publish time; now parser-completed scan stamp is preserved | Medium before fix, reduced after fix |
| `odom` and TF timestamps | Same `stamp` from `handleOpencrState()` for odom and TF | Low |
| `base_footprint -> base_link` | Fixed `xyz=(0, 0, 0.0335)` in custom URDF | Medium; differs from TB3 but mostly affects vertical geometry |
| `base_link -> base_scan` | Previously `xyz=(0, 0, 0.105)`, now aligned to TB3 `(-0.032, 0, 0.172)` | High before fix, reduced after fix |
| `base_link -> imu_link` | Previously `xyz=(0, 0, 0.060)`, now aligned to TB3 `(-0.032, 0, 0.068)` | Medium before fix, reduced after fix |
| Wheel joint names | `wheel_left_joint`, `wheel_right_joint` | Low |
| Wheel link names | `left_wheel_link`, `right_wheel_link` instead of TB3 `wheel_left_link`, `wheel_right_link` | Low; not used by AMCL |

## Compatibility Risks For AMCL / Localization

### 1. All-zero odom covariance

Before this update, `robot_base_driver/src/robot_base_driver/odometry_integrator.cpp`
populated pose and twist fields only, and `publishOdometry()` forwarded the message
without any covariance assignment.

That leaves both `pose.covariance` and `twist.covariance` as all zeros.

Risk:

- Some localization stacks interpret all-zero odom covariance as unrealistically
  confident motion.
- That can make `map -> odom` oscillate because the localization side is forced
  to fight highly trusted odom even when scan matching disagrees.

Status:

- Fixed.

### 2. Scan timestamp mismatch

`robot_lidar_driver/src/robot_lidar_driver/lidar_driver_node.cpp` previously built
the outgoing `LaserScan` with `now()` even though the parser already stored
`completed_scan.stamp`.

Risk:

- `/scan` can be timestamped later than the actual measurement completion time.
- That weakens scan-to-TF time alignment and can make localization jitter worse.

Status:

- Fixed.

### 3. Base scan transform mismatch from TB3 Burger

The custom URDF previously placed `base_scan` at `(0, 0, 0.105)` relative to
`base_link`, while TB3 Burger uses `(-0.032, 0, 0.172)`.

Risk:

- If the physical platform and Nav2 stack assumptions are TB3-like, scan origin
  mismatch changes the effective laser pose used by costmap and localization.
- This is a direct source of AMCL instability even when odom numerically looks
  reasonable.

Status:

- Fixed to match TB3 Burger convention.

### 4. IMU covariance semantics mismatch

The custom base driver previously published IMU orientation/angular velocity/
linear acceleration values while setting covariance element `0` to `-1` for all
three blocks.

Risk:

- `-1` means the associated estimate is not provided, not merely that the
  covariance is unknown.
- Consumers can legitimately discard the IMU data entirely.

Status:

- Fixed to use all-zero covariance, which correctly means covariance unknown.

### 5. Base footprint to base link vertical offset mismatch

The custom URDF still uses `base_footprint -> base_link` `z=0.0335`, while TB3
Burger uses `z=0.010`.

Risk:

- Low to medium for 2D AMCL itself because this is mainly a vertical offset.
- It is still a geometry mismatch from TB3, so downstream 3D visualization and
  collision interpretation may differ.

Status:

- Not changed in this pass.
- Left as a documented difference because changing it cleanly should also review
  wheel and caster placement together.

## Minimal Compatibility Fixes

| Fix | Recommended | Implemented |
| --- | --- | --- |
| Non-zero odom pose covariance defaults | Yes | Yes |
| Non-zero odom twist covariance defaults | Yes | Yes |
| Odom covariance parameterization in YAML | Yes | Yes |
| Preserve parser-completed scan timestamps | Yes | Yes |
| Align `base_link -> base_scan` to TB3 Burger | Yes | Yes |
| Align `base_link -> imu_link` to TB3 Burger | Yes | Yes |
| Keep IMU covariance as unknown when covariance is not characterized | Yes | Yes, using all-zero covariance |
| Align `base_footprint -> base_link` vertical offset to TB3 | Maybe, but review wheel/caster geometry together | No |

## Implemented Changes

### Odom covariance

Implemented in:

- `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp`
- `robot_base_driver/include/robot_base_driver/robot_base_driver_node.hpp`
- `robot_bringup/config/robot.yaml`
- `robot_base_driver/config/base.yaml`

Current defaults:

- `odom_pose_covariance_diagonal = [0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.05]`
- `odom_twist_covariance_diagonal = [0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.05]`

### Scan timestamp preservation

Implemented in:

- `robot_lidar_driver/src/robot_lidar_driver/lidar_driver_node.cpp`

The outgoing `LaserScan` now uses `completed_scan.stamp` instead of a fresh
publish-time `now()`.

### TB3 sensor-frame alignment

Implemented in:

- `robot_description/urdf/robot.urdf.xacro`

Current fixed transforms now match TB3 Burger for:

- `base_link -> base_scan`
- `base_link -> imu_link`

## Remaining Differences

- `base_footprint -> base_link` vertical offset is still not the same as TB3 Burger.
- Exact stock TB3 odom covariance defaults and exact odom/TF output rate remain
  `UNCERTAIN` from this environment because `turtlebot3_ros` is installed as a
  binary executable rather than source code.
- `joint_states.velocity` in this custom stack is still not TB3-perfect
  semantically because it is based on the existing hardware driver path rather
  than the stock TB3 implementation. This is low risk for AMCL.

## Overall Judgment

The most likely code-level incompatibilities affecting localization were:

1. All-zero `/odom` covariance.
2. Non-TB3 `base_scan` fixed transform.
3. `/scan` publish timestamp not using the parser-completed scan time.

Those are now corrected in a minimal way without changing odom integration math,
command semantics, or TF ownership.
