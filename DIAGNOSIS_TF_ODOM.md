# TF / Odom Diagnosis

## Summary

Overall judgment:
- TF ownership inside this repository is mostly clean and matches the intended split when `use_description=true` and no namespace is used.
- The highest-risk code-level issue is in the OpenCR command path: `robot_base_driver` computes left/right wheel goal velocities in `RobotBaseDriverNode::handleVelocityCommand()` and `OpencrClient::writeVelocityCommand()`, but the actual payload written to OpenCR contains only body-frame `linear_x` and `angular_z` raw fields, not the computed per-wheel goal values.
- The second clear issue is `sensor_msgs/msg/JointState.velocity`: the code publishes values derived in meters/second, not radians/second.
- A launch-level mismatch can occur when `namespace` is set in `robot_bringup`: `robot_base_driver` prefixes frame IDs and joint names with the namespace, but `robot_state_publisher` is launched without that namespace and without namespaced link names.

Things that look correct:
- `robot_base_driver` is the only code-level `/tf` broadcaster in this repo, and it publishes `odom -> base_footprint` only when `publish_tf=true`.
- `robot_description` defines `base_footprint -> base_link`, `base_link -> base_scan`, `base_link -> imu_link`, and wheel joints in URDF.
- `robot_lidar_driver` publishes `LaserScan.header.frame_id` only; it does not publish TF.

## TF Ownership

| TF edge | expected publisher | actual publisher | status |
| --- | --- | --- | --- |
| `map -> odom` | localization outside this repo | none found in repo | PASS |
| `odom -> base_footprint` | `robot_base_driver` | `robot_base_driver` via `tf2_ros::TransformBroadcaster` in `RobotBaseDriverNode::publishOdometry()` at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:905-920` | PASS |
| `base_footprint -> base_link` | `robot_state_publisher` / URDF | fixed joint `base_footprint_joint` in `robot_description/urdf/robot.urdf.xacro:35-39`, published by `robot_state_publisher` from `robot_description/launch/description.launch.py:22-33` | PASS |
| `base_link -> base_scan` | `robot_state_publisher` / URDF | fixed joint `base_scan_joint` in `robot_description/urdf/robot.urdf.xacro:62-66` | PASS |
| `base_link -> imu_link` | `robot_state_publisher` / URDF | fixed joint `imu_joint` in `robot_description/urdf/robot.urdf.xacro:89-93` | PASS |
| wheel links | `robot_state_publisher` driven by `/joint_states` | wheel joints `wheel_left_joint` and `wheel_right_joint` in `robot_description/urdf/robot.urdf.xacro:116-149`; `/joint_states` from `RobotBaseDriverNode::publishJointStates()` at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:881-902` | PASS with caveat |

Repo-wide TF ownership evidence:
- `tf2_ros::TransformBroadcaster` appears only in `robot_base_driver/include/robot_base_driver/robot_base_driver_node.hpp` and `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:439-441, 913-919`.
- No `tf2_ros::StaticTransformBroadcaster` found.
- No `static_transform_publisher` found.
- `robot_state_publisher` is launched only by `robot_description/launch/description.launch.py:22-33`, and `robot_bringup/launch/robot.launch.py:48-56` includes it only when `use_description=true`.

Possible duplicated TF edge:
- No duplicate TF publisher was found inside this repository for `odom -> base_footprint`, `base_footprint -> base_link`, `base_link -> base_scan`, or `base_link -> imu_link`.
- Duplicate TF is still possible at runtime if another external stack also publishes `odom -> base_footprint` or the same static edges, but that cannot be verified from this repo alone.

Sensor driver TF behavior:
- `robot_lidar_driver` declares and loads only `frame_id` and `topic_name` parameters in `robot_lidar_driver/src/robot_lidar_driver/lidar_driver_node.cpp:82-139`.
- `LaserScanBuilder::buildScan()` writes `scan_message.header.frame_id = frame_id_` at `robot_lidar_driver/src/robot_lidar_driver/laser_scan_builder.cpp:25-31`.
- No TF broadcaster exists in `robot_lidar_driver`.

`robot_description` and `odom` link:
- `robot_description/urdf/robot.urdf.xacro` defines `base_footprint`, `base_link`, `base_scan`, `imu_link`, wheel links, and caster link, but no `odom` link.
- That matches the expected ownership because `odom` should be a TF frame, not a URDF link, in this stack.

## Frame ID Consistency

| topic/field | expected | actual/default | status |
| --- | --- | --- | --- |
| `/odom.header.frame_id` | `odom` | `odom_frame_id`, default `"odom"` in `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:31,124,180`; published in `odometry_integrator.cpp:133-152` | PASS |
| `/odom.child_frame_id` | `base_footprint` | `base_frame_id`, default `"base_footprint"` in `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:32,125,181`; published in `odometry_integrator.cpp:137-138` | PASS |
| `/tf` transform parent | `odom` | same as `odom_frame_id` in `odometry_integrator.cpp:155-170` | PASS |
| `/tf` transform child | `base_footprint` | same as `base_frame_id` in `odometry_integrator.cpp:159-160` | PASS |
| `/scan.header.frame_id` | `base_scan` | LiDAR `frame_id`, default `"base_scan"` in `robot_lidar_driver/src/robot_lidar_driver/lidar_driver_node.cpp:25,87,117` and `robot_bringup/config/robot.yaml:7` | PASS |
| `/imu.header.frame_id` | `imu_link` | `imu_frame_id`, default `"imu_link"` in `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:33,126,182,862-878` | PASS |
| `/joint_states.header.frame_id` | not relied on for TF | set to `base_frame_id` in `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:890-892` | CAUTION |
| `/joint_states.name[0]` | `wheel_left_joint` | `wheel_left_joint_name`, default `"wheel_left_joint"` in `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:34,127,183,893` | PASS |
| `/joint_states.name[1]` | `wheel_right_joint` | `wheel_right_joint_name`, default `"wheel_right_joint"` in `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:35,128,184,894` | PASS |

Namespace caveat:
- `RobotBaseDriverNode::resolveFrameId()` prefixes relative frame IDs with the node namespace at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:982-995`.
- `robot_description/launch/description.launch.py` does not accept or forward a namespace to `robot_state_publisher`.
- If `robot_bringup/launch/robot.launch.py` is run with `namespace:=robot1`, the driver can publish `robot1/odom`, `robot1/base_footprint`, and `robot1/imu_link`, while `robot_state_publisher` still publishes un-namespaced URDF frames. That is a real TF-tree split risk.

## CmdVel to Wheel Command Path

Code path:
1. `/cmd_vel` subscription is created in `RobotBaseDriverNode::setupSubscriptions()` at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:445-487`.
2. `RobotBaseDriverNode::handleVelocityCommand()` receives `geometry_msgs::msg::Twist` at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:655-739`.
3. It stores only `linear_x_mps`, `angular_z_rps`, and `source` into `VelocityCommand` and queues that to `OpencrClient::setVelocityCommand()` at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:728-732`.
4. `OpencrClient::workerLoop()` drains the latest pending command and calls `writeVelocityCommand()` at `robot_base_driver/src/robot_base_driver/opencr_client.cpp:151-168`.
5. `OpencrClient::writeVelocityCommand()` constructs the actual OpenCR payload at `robot_base_driver/src/robot_base_driver/opencr_client.cpp:382-459`.

Formulas in `RobotBaseDriverNode::handleVelocityCommand()`:
- `left_wheel_mps = linear.x - angular.z * wheel_separation / 2.0`
- `right_wheel_mps = linear.x + angular.z * wheel_separation / 2.0`
- `left_wheel_radps = left_wheel_mps / wheel_radius`
- `right_wheel_radps = right_wheel_mps / wheel_radius`
- `left_goal_velocity = clamp(left_wheel_mps * 1263.632956882, -337.0, 337.0)`
- `right_goal_velocity = clamp(right_wheel_mps * 1263.632956882, -337.0, 337.0)`

Formulas in `OpencrClient::writeVelocityCommand()`:
- Same left/right wheel formulas are recomputed at `robot_base_driver/src/robot_base_driver/opencr_client.cpp:406-417`.
- But the payload actually written is:
  - `velocity_values[0] = int32(linear_x_mps * 100.0)` to `CMD_VELOCITY_LINEAR_X`
  - `velocity_values[1] = 0` to `CMD_VELOCITY_LINEAR_Y`
  - `velocity_values[2] = 0` to `CMD_VELOCITY_LINEAR_Z`
  - `velocity_values[3] = 0` to `CMD_VELOCITY_ANGULAR_X`
  - `velocity_values[4] = 0` to `CMD_VELOCITY_ANGULAR_Y`
  - `velocity_values[5] = int32(angular_z_rps * 100.0)` to `CMD_VELOCITY_ANGULAR_Z`

Current finding:
- The computed `left_goal_velocity` and `right_goal_velocity` are diagnostic only in this repo. They are logged, but never inserted into the outgoing payload.
- Therefore the actual command interface assumed by this code is “body twist registers at 150..173,” not “per-wheel goal velocity registers.”
- Whether that is correct depends on the stock OpenCR firmware register map and semantics. From this repo alone, the left/right wheel goal values are not the bytes being sent.

## Wheel Encoder to Odom Path

Readback path:
1. `OpencrClient::readRequiredStateGroup()` reads:
   - `PRESENT_VELOCITY_LEFT` address `128`, int32
   - `PRESENT_VELOCITY_RIGHT` address `132`, int32
   - `PRESENT_POSITION_LEFT` address `136`, int32
   - `PRESENT_POSITION_RIGHT` address `140`, int32
   at `robot_base_driver/src/robot_base_driver/opencr_client.cpp:652-675` with addresses defined in `robot_base_driver/include/robot_base_driver/control_table.hpp:48-51`.
2. `RobotBaseDriverNode::handleOpencrState()` forwards those raw values to `OdometryIntegrator::update()` at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:814-824`.
3. `OdometryIntegrator::update()` computes deltas, pose, and twist at `robot_base_driver/src/robot_base_driver/odometry_integrator.cpp:41-120`.

Exact formulas:
- `left_tick_delta = raw_left_ticks - last_left_ticks`
- `right_tick_delta = raw_right_ticks - last_right_ticks`
- `left_delta_rad = left_tick_delta * TICK_TO_RAD`
- `right_delta_rad = right_tick_delta * TICK_TO_RAD`
- `TICK_TO_RAD = 0.001533981` from `robot_base_driver/include/robot_base_driver/odometry_integrator.hpp:19`
- `joint_velocity_left = raw_left_velocity * RPM_TO_MS`
- `joint_velocity_right = raw_right_velocity * RPM_TO_MS`
- `RPM_TO_MS = 0.229 * 0.0034557519189487725` from `robot_base_driver/include/robot_base_driver/odometry_integrator.hpp:20`
- `delta_s = wheel_radius * (right_delta_rad + left_delta_rad) / 2.0`
- `delta_theta = wheel_radius * (right_delta_rad - left_delta_rad) / wheel_separation`
- `x += delta_s * cos(yaw + delta_theta / 2.0)`
- `y += delta_s * sin(yaw + delta_theta / 2.0)`
- `yaw = normalizeAngle(yaw + delta_theta)`
- `odom.linear.x = delta_s / dt`
- `odom.linear.y = 0.0`
- `odom.angular.z = delta_theta / dt`

Current findings:
- Odom is integrated from position delta, not from present velocity.
- Present velocity is used only for `joint_states.velocity`.
- No encoder wrap-around handling exists. Raw `int32_t` tick deltas are used directly.
- No explicit sign correction exists for left or right encoder channels.
- No explicit conversion from present velocity to radians/second exists; published joint velocity unit is meters/second by variable name and formula.

## Left/Right and Sign Convention Check

Forward command (`linear.x > 0`, `angular.z = 0`):
- `left_wheel_mps = linear.x`
- `right_wheel_mps = linear.x`
- `left_goal_velocity > 0`, `right_goal_velocity > 0`
- If encoder signs match the formulas, `delta_s > 0`, `odom.twist.linear.x > 0`, and `x` increases.
- Status: PASS in code assumption.

Rotate left command (`linear.x = 0`, `angular.z > 0`):
- `left_wheel_mps = -angular.z * wheel_separation / 2`
- `right_wheel_mps = +angular.z * wheel_separation / 2`
- `left_goal_velocity < 0`, `right_goal_velocity > 0`
- `delta_theta = wheel_radius * (right_delta_rad - left_delta_rad) / wheel_separation`
- If right encoder increases and left encoder decreases for CCW rotation, `delta_theta > 0` and yaw increases.
- Status: PASS in code assumption.

Rotate right command (`linear.x = 0`, `angular.z < 0`):
- `left_wheel_mps > 0`
- `right_wheel_mps < 0`
- `delta_theta < 0` if encoder signs match the assumption above.
- Status: PASS in code assumption.

Potential sign-violation points:
- There is no sign inversion layer between raw OpenCR encoder values and odom integration. If the firmware reports one side with opposite polarity, odom sign will be wrong.
- There is no code-level evidence of left/right encoder calibration, motor inversion, or per-wheel sign compensation.
- URDF wheel joints both use `<axis xyz="0 0 1"/>` with mirrored origins at `robot_description/urdf/robot.urdf.xacro:116-149`. Whether positive joint motion visually matches positive encoder ticks for both sides is `UNCERTAIN` from code alone.

Pass/fail summary:
- Left/right differential-drive formulas: PASS
- Yaw sign formula in odometry integrator: PASS
- Encoder sign correction layer: FAIL, none exists
- Firmware sign agreement with this code: UNCERTAIN

## Launch and Parameter Check

Launch orchestration:
- `robot_bringup/launch/sensor.launch.py` launches only `robot_lidar_driver` with `params_file`, `namespace`, `use_sim_time`, and `log_level` at `robot_bringup/launch/sensor.launch.py:18-26`.
- `robot_bringup/launch/motor.launch.py` launches only `robot_base_driver` with the same shared parameter plumbing at `robot_bringup/launch/motor.launch.py:18-26`.
- `robot_bringup/launch/robot.launch.py` conditionally includes:
  - `sensor.launch.py` when `use_sensor=true`
  - `motor.launch.py` when `use_motor=true`
  - `robot_description/launch/description.launch.py` when `use_description=true`
  at `robot_bringup/launch/robot.launch.py:22-56`

`use_description=false` behavior:
- Yes, it prevents `robot_state_publisher` from starting because the include itself is wrapped in `IfCondition(use_description)` at `robot_bringup/launch/robot.launch.py:48-56`.

Static TF elsewhere:
- No `static_transform_publisher` found.
- No `tf2_ros::StaticTransformBroadcaster` found.

`params_file` propagation:
- `robot_bringup/launch/sensor.launch.py` and `robot_bringup/launch/motor.launch.py` both pass `parameters=[params_file, {"use_sim_time": use_sim_time}]`.
- `robot_description/launch/description.launch.py` does not consume the shared `params_file`; it only accepts `use_sim_time` and `model`.

`publish_tf` centralization:
- `publish_tf` exists in `robot_bringup/config/robot.yaml:63-66`.
- `robot_bringup/launch/motor.launch.py` passes that YAML directly to `robot_base_driver`.
- Therefore `publish_tf` can be controlled from `robot_bringup/config/robot.yaml`.

Conflicts or duplication:
- `robot_bringup/config/robot.yaml` and `robot_base_driver/config/base.yaml` duplicate nearly the same base-driver parameters.
- `robot_bringup/config/robot.yaml` and `robot_lidar_driver/config/lidar.yaml` duplicate LiDAR parameters.
- Code defaults duplicate many YAML defaults again in node constructors and `declare_parameter()` calls.

## OpenCR Register Mapping Check

Control-table map:
- `DEVICE_STATUS` = `18`, `uint8`
- `HEARTBEAT` = `19`, `uint8`
- `IMU_RECALIBRATION` = `59`, `uint8`
- `PRESENT_VELOCITY_LEFT` = `128`, `int32`
- `PRESENT_VELOCITY_RIGHT` = `132`, `int32`
- `PRESENT_POSITION_LEFT` = `136`, `int32`
- `PRESENT_POSITION_RIGHT` = `140`, `int32`
- `MOTOR_TORQUE_ENABLE` = `149`, `uint8`
- `CMD_VELOCITY_LINEAR_X` = `150`, `int32`
- `CMD_VELOCITY_LINEAR_Y` = `154`, `int32`
- `CMD_VELOCITY_LINEAR_Z` = `158`, `int32`
- `CMD_VELOCITY_ANGULAR_X` = `162`, `int32`
- `CMD_VELOCITY_ANGULAR_Y` = `166`, `int32`
- `CMD_VELOCITY_ANGULAR_Z` = `170`, `int32`
- `PROFILE_ACCELERATION_LEFT` = `174`, `int32`
- `PROFILE_ACCELERATION_RIGHT` = `178`, `int32`
- Source: `robot_base_driver/include/robot_base_driver/control_table.hpp:29-64`

Velocity command payload:
- Start address: `150`
- Register span: `24` bytes for six `int32` fields from `150` through `173`
- Byte order: host-endian `int32_t` bytes copied directly with `reinterpret_cast<uint8_t *>` at `robot_base_driver/src/robot_base_driver/opencr_client.cpp:400-404`
- On the current target platform this implies little-endian layout, but the code does not enforce endian conversion explicitly.
- Number of motor command fields actually written: 6 body-twist fields, not 2 per-wheel fields.
- Left/right goal velocities are not written to separate offsets anywhere in this repo.
- `linear_x_raw` and `angular_z_raw` are not diagnostic-only; they are the actual transmitted values in the payload.
- `left_goal_velocity` and `right_goal_velocity` are diagnostic-only in this repo.

Firmware-map match:
- The payload layout matches the local `ControlTable` definition.
- Whether that local control-table definition matches the actual stock OpenCR firmware is `UNCERTAIN` from repository code alone.

## OpenCR Command Semantics Decision

Decision:
- Treat the current workspace implementation as `A. body twist command`.

Why this is the best-supported interpretation from workspace code:
- The control table names six consecutive writeable command registers:
  - `CMD_VELOCITY_LINEAR_X`
  - `CMD_VELOCITY_LINEAR_Y`
  - `CMD_VELOCITY_LINEAR_Z`
  - `CMD_VELOCITY_ANGULAR_X`
  - `CMD_VELOCITY_ANGULAR_Y`
  - `CMD_VELOCITY_ANGULAR_Z`
  in `robot_base_driver/include/robot_base_driver/control_table.hpp:53-58`.
- `OpencrClient::writeVelocityCommand()` writes a contiguous 24-byte block starting at `CMD_VELOCITY_LINEAR_X.address` and fills six `int32` slots in that same axis order at `robot_base_driver/src/robot_base_driver/opencr_client.cpp:382-404`.
- The workspace README describes the package as targeting the “stock OpenCR control table layout used by TurtleBot3 Humble” and says the driver uses an OpenCR “velocity command register” path in `robot_base_driver/README.md:7-16`.
- No per-wheel command register names, offsets, or comments are present anywhere in the workspace.

Why `B. per-wheel velocity command` is not supported by workspace evidence:
- The repo contains no `LEFT_GOAL_VELOCITY`, `RIGHT_GOAL_VELOCITY`, motor-ID-indexed wheel command registers, or payload offsets for left/right wheel writes.
- The computed left/right goal velocities exist only as host-side diagnostic math in `RobotBaseDriverNode::handleVelocityCommand()` and `OpencrClient::writeVelocityCommand()`.

Remaining uncertainty:
- It is still `UNCERTAIN` whether the workspace `ControlTable` labels exactly match the real stock OpenCR firmware implementation, because the firmware source or official register reference is not included in this repository.
- The missing artifact is an authoritative OpenCR firmware/register document or firmware source mapping that proves addresses `150..173` are body-twist command fields on the target hardware.

## Suspicious Findings

1. `robot_base_driver` computes left/right wheel target velocities but never transmits them.
   - Evidence: `RobotBaseDriverNode::handleVelocityCommand()` computes wheel values at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:669-677`.
   - Evidence: `OpencrClient::writeVelocityCommand()` logs left/right goal values at `robot_base_driver/src/robot_base_driver/opencr_client.cpp:406-417, 426-455`.
   - Evidence: actual payload writes only `linear_x*100` and `angular_z*100` to body-command registers at `robot_base_driver/src/robot_base_driver/opencr_client.cpp:388-404`.
   - Risk: if firmware expects per-wheel commands, robot motion and odom behavior can diverge badly.

2. Namespace handling can split the TF tree.
   - Evidence: `resolveFrameId()` prefixes relative frame IDs with namespace at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:982-995`.
   - Evidence: `robot_bringup/launch/robot.launch.py` forwards namespace to sensor and motor includes at `robot_bringup/launch/robot.launch.py:27-45`.
   - Evidence: `robot_description/launch/description.launch.py` does not accept namespace and launches un-namespaced `robot_state_publisher` at `robot_description/launch/description.launch.py:22-33`.
   - Risk: `odom -> robot1/base_footprint` can exist while `base_footprint -> base_link` is published as un-namespaced links.

3. `joint_states.velocity` is in meters/second, not radians/second.
   - Evidence: `RPM_TO_MS` constant and `joint_velocities_mps_` naming at `robot_base_driver/include/robot_base_driver/odometry_integrator.hpp:20,33`.
   - Evidence: `joint_velocities_mps_[i] = raw_velocity * RPM_TO_MS` at `robot_base_driver/src/robot_base_driver/odometry_integrator.cpp:69-70`.
   - Evidence: those values are published into `sensor_msgs/msg/JointState.velocity` at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:896-901`.
   - Risk: `robot_state_publisher` uses position only for TF, so TF may still work, but tooling that reads joint velocity semantics will be wrong.

4. Encoder sign and wrap handling are absent.
   - Evidence: raw `int32_t` deltas are subtracted directly at `robot_base_driver/src/robot_base_driver/odometry_integrator.cpp:62-67`.
   - No wrap correction, no side inversion, no calibration offsets.
   - Risk: sign inversion or wrap events can create odom jumps or reversed yaw.

5. All odom timestamps are `now()` from the ROS node, not hardware timestamps.
   - Evidence: `RobotBaseDriverNode::handleOpencrState()` sets `rclcpp::Time stamp = now()` at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:754-758`.
   - Risk: serial delay and poll jitter are folded into odom `dt`.

6. `joint_states.header.frame_id` is set to `base_frame_id`.
   - Evidence: `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:890-892`.
   - Risk: not a TF producer by itself, but semantically unusual and can confuse debugging.

## Recommended Next Hardware Tests

These are proposed commands only. They were not run here.

- Motor-only launch test:
  - `ros2 launch robot_bringup motor.launch.py`
- Direct forward `cmd_vel`:
  - `ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"`
- Direct rotation `cmd_vel`:
  - `ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"`
- Echo `/odom`:
  - `ros2 topic echo /odom`
- Echo `/joint_states`:
  - `ros2 topic echo /joint_states`
- Echo TF `odom -> base_footprint`:
  - `ros2 run tf2_ros tf2_echo odom base_footprint`
- Dump TF tree:
  - `ros2 run tf2_tools view_frames`

## Recommended Fix Plan

Do not implement yet.

1. Confirm the stock OpenCR firmware register semantics for addresses `150..173`.
   - If those registers are body twist commands, keep the payload as-is and remove misleading left/right “goal velocity” wording from logs.
   - If firmware expects per-wheel commands, change the payload to transmit actual left/right wheel values to the correct registers.

2. Decide whether odom should remain encoder-only or optionally use IMU yaw.
   - Current code already supports `use_imu_for_yaw`, but only when `poll_mode=full`.

3. Add explicit encoder sign calibration parameters if hardware reveals reversed forward or reversed yaw.
   - Example future knobs: `invert_left_encoder`, `invert_right_encoder`, `invert_left_command`, `invert_right_command`.

4. Correct `JointState.velocity` units.
   - Publish radians/second for wheel joints, not meters/second.

5. If namespacing is required, make launch and frame naming consistent.
   - Either namespace `robot_state_publisher` and URDF frame/joint names consistently, or stop prefixing driver frame IDs.

6. Consider adding bounds/guarding for odom `dt`.
   - Reject unusually large `dt` after reconnects or stalled polls to reduce pose jumps.

7. Consider making timestamp source explicit.
   - If hardware timestamps are unavailable, at least document that odom timing is poll-loop timing.

## Task-by-Task Notes

TF ownership inspection:
- Confirmed by repo-wide search and the files above.

Base-driver odom publishing logic:
- Implemented across `RobotBaseDriverNode::handleVelocityCommand()`, `RobotBaseDriverNode::handleOpencrState()`, `OpencrClient::writeVelocityCommand()`, `OpencrClient::readRequiredStateGroup()`, and `OdometryIntegrator::update()`.

Left/right wheel mapping:
- Consistent names are used across config, node parameters, `/joint_states`, and URDF: `wheel_left_joint`, `wheel_right_joint`.
- Explicit motor IDs are not present in this repo; only a single OpenCR device ID `200` exists in `ControlTable::OPENCR_ID`.
- Per-wheel command register mapping is therefore `UNCERTAIN` because the repo never names separate left/right command registers.

Timing and watchdog:
- Poll loop period is `poll_interval_ms`, now set to `50` in both `robot_bringup/config/robot.yaml` and `robot_base_driver/config/base.yaml`, and forwarded through `RobotBaseDriverNode::startRealMode()`.
- Heartbeat period is `heartbeat_interval_ms`, default `100`.
- Command queue behavior is “latest command only”: `OpencrClient::setVelocityCommand()` overwrites `pending_velocity_command_` at `robot_base_driver/src/robot_base_driver/opencr_client.cpp:101-109`.
- No `cmd_vel` timeout stop watchdog exists; see log message at `robot_base_driver/src/robot_base_driver/robot_base_driver_node.cpp:512-515`.
- Polling and command writes occur in the same worker thread in `OpencrClient::workerLoop()` at `robot_base_driver/src/robot_base_driver/opencr_client.cpp:144-210`, so long command transactions can delay polls.

Cmd_vel source assumptions:
- The driver cannot distinguish an external zero command from an intentional stop command; both are just `source="cmd_vel"`.
- It does not record publisher identity.
- It does not internally generate periodic stop commands on timeout.
