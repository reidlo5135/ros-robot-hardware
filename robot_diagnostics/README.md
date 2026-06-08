# robot_diagnostics

`robot_diagnostics` validates the ROS contract expected by the TurtleBot3 Burger/OpenCR/LDS-03 compatible hardware stack. It is intended primarily as a live monitor for TF, topic, scan, odom, and IMU consistency, with one-shot mode reserved for explicit snapshot, CI, or report runs.

## Contract

Default contract file:

```bash
ros2 launch robot_diagnostics diagnostics.launch.py
```

The default `config/tb3_contract.yaml` expects:

- Dynamic TF: `odom -> base_footprint`
- Static TF: `base_footprint -> base_link`, `base_link -> base_scan`, `base_link -> imu_link`
- Optional external TF: `map -> odom`
- Topics: `/scan`, `/odom`, `/imu`, `/joint_states`, `/cmd_vel`
- `/scan`: `frame_id=base_scan`, `400` ranges, `angle_min=0`, `angle_max=2*pi`, cardinal indexes front `0`, left `100`, rear `200`, right `300`
- `/odom`: `frame_id=odom`, `child_frame_id=base_footprint`
- `/imu`: `frame_id=imu_link`
- `/battery_state`: optional by default, `frame_id=base_link`, voltage range `0..30 V`
- `/joint_states`: `wheel_left_joint`, `wheel_right_joint`

`map -> odom` is optional because it belongs to localization, AMCL, SLAM, or navigation. Its absence is not a failure. If it appears to come from the robot hardware stack, the node reports a warning.

## Run

Primary live monitor:

```bash
ros2 launch robot_diagnostics diagnostics.launch.py
```

Repository helper for live monitoring:

```bash
./scripts/robot_diag_watch.sh
./scripts/robot_diag_watch.sh --summary-period 5.0
```

One-shot snapshot, CI, or report run:

```bash
ros2 launch robot_diagnostics diagnostics.launch.py once:=true summary_period_sec:=5.0
```

`once:=true` exits normally after the first summary and should only be used when a finite diagnostic snapshot is desired.

With a custom contract:

```bash
ros2 launch robot_diagnostics diagnostics.launch.py contract_file:=/path/to/contract.yaml
```

From the repository snapshot helper script:

```bash
./scripts/robot_diag_snapshot.sh --summary-period 5.0
```

## Checks

The node subscribes to `/tf`, `/tf_static`, `/scan`, `/odom`, `/imu`, and `/joint_states`, and uses a `tf2_ros::Buffer` plus `TransformListener` for TF lookup checks.

It reports:

- Required TF edge availability
- Static TF geometry tolerance
- Odom frame and child frame
- Odom pose versus `odom -> base_footprint` TF consistency
- Scan frame, sample count, angle fields, and geometry stability
- IMU frame availability
- Optional `/battery_state` publisher/message, frame, finite voltage, and range
- Required wheel joints in `/joint_states`
- `/cmd_vel` subscriber availability
- Optional external `map -> odom` state

`odom_tf_consistency` is timestamp-sensitive. It compares the latest received
`/odom.pose.pose` against the `odom.header.frame_id -> odom.child_frame_id` TF
transform looked up at the same `odom.header.stamp`. If TF is not available at
that odom stamp, the node reports `WARN` with
`reason=tf_unavailable_at_odom_stamp` and may include latest TF fallback values
only for visibility. During motion, an unsynchronized latest-TF comparison can
produce large yaw differences, so this condition is not classified as a hardware
failure by default.

Battery is an optional contract item by default. Missing `/battery_state` does
not fail the summary unless the contract sets `battery_state.required: true`.
When a message exists, the check warns on `present=false`, frame mismatch, `NaN`
voltage, or voltage outside the configured range. The default v0.1.13 base-driver
path follows the TurtleBot3 OpenCR control table battery fields; diagnostics only
verifies the ROS message contract. Custom-register battery mappings remain a
vehicle-specific field-validation item in `robot_base_driver`.

## Logs

Every check emits structured logs using the repository schema:

```text
ROBOT_HW_LOG schema=v1 tag=DIAG component=diagnostics event=... key=value result=...
```

The periodic human-readable summary is concise:

```text
Diagnostics summary: overall=PASS pass=... warn=... fail=...
```

## Motion Diagnostics

`robot_diagnostics` remains the static/live contract monitor. For v0.1.12
commanded-motion checks, use the repository-level motion workflow:

```bash
./scripts/test_motion_diagnostics.sh --mode rotate --publish --angular-z 0.5 --duration 10 --bag
./scripts/test_motion_diagnostics.sh --mode linear --publish --linear-x 0.05 --duration 10 --bag
```

That workflow summarizes `/cmd_vel` versus `/odom`, `/imu`, TF, and `/scan`
behavior without changing this package's launch file or contract checks. See
`docs/debugging/MOTION_DIAGNOSTICS.md`.
