# Motion Diagnostics

v0.1.12 adds a motion diagnostics workflow for checking whether live motion
commands produce semantically consistent `/odom`, `/imu`, TF, and `/scan`
behavior.

## Purpose

The v0.1.11 `robot_diagnostics` node validates the static ROS contract:
required topics, TF edges, frame IDs, scan geometry, joint names, and
timestamp-synchronized odom pose versus TF consistency.

The v0.1.12 motion workflow goes one step later in the field checklist. It runs
or observes conservative motion and summarizes whether `/cmd_vel` agrees with
the direction and shape of `/odom`, `/imu`, `odom -> base_footprint`, and scan
geometry.

Use it before Nav2/AMCL integration. It is meant to catch sign mistakes, broken
motion feedback, TF/odom disagreement, and scan geometry instability before
localization is asked to solve the harder problem.

## Script

```bash
./scripts/test_motion_diagnostics.sh --help
```

Defaults are conservative for a TurtleBot3 Burger/OpenCR-compatible base:

- `mode=rotate`
- `duration=10`
- `linear_x=0.05`
- `angular_z=0.5`
- publish enabled
- bag recording disabled

The script publishes several zero `/cmd_vel` messages when it exits after a
published test.

## Rotate Mode

Rotate in place and compare yaw direction and angular velocity across odom, IMU,
and TF:

```bash
./scripts/test_motion_diagnostics.sh --mode rotate --publish --angular-z 0.5 --duration 10 --bag
```

Observe only, without commanding the robot:

```bash
./scripts/test_motion_diagnostics.sh --mode rotate --no-publish --duration 10
```

Important summary fields:

- `odom_yaw_delta_rad`: unwrapped yaw change accumulated sample-to-sample from `/odom.pose.pose`
- `tf_yaw_delta_rad`: unwrapped yaw change accumulated sample-to-sample from `odom -> base_footprint`
- `odom_yaw_delta_normalized_rad`: start/end odom yaw difference normalized to `[-pi, pi]` for reference only
- `tf_yaw_delta_normalized_rad`: start/end TF yaw difference normalized to `[-pi, pi]` for reference only
- `odom_angular_z_mean`: mean `/odom.twist.twist.angular.z`
- `imu_angular_z_mean`: mean `/imu.angular_velocity.z`
- `xy_drift_m`: translational drift during rotate-in-place
- `scan_geometry_stable`: whether scan size and angle fields stayed stable

Expected behavior:

- Positive commanded `angular.z` should produce positive unwrapped odom yaw
  delta and positive mean odom angular velocity.
- Negative commanded `angular.z` should produce negative unwrapped odom yaw
  delta and negative mean odom angular velocity.
- Mean odom angular velocity is the first sign check for commanded rotate
  direction. The unwrapped odom yaw delta must also have the same sign.
- IMU angular velocity should usually have the same sign as command and odom;
  IMU sign mismatches are reported as `WARN` first so that odom/command
  agreement is not hidden by an IMU convention issue.
- Odom yaw delta and TF yaw delta should be close. Because they can be sampled
  at different timing/rates, yaw delta error is treated as `PASS` at
  `<= 0.10 rad`, `WARN` at `0.10..0.25 rad`, and `FAIL` above `0.25 rad`.
- X/Y drift is a quality metric. The initial threshold is conservative:
  `0.05 m` or more is reported as `WARN`, not an immediate contract failure.

Yaw values published in odom and TF are bounded angles. For rotate tests longer
than 180 degrees, a simple `end_yaw - start_yaw` value normalized to `[-pi, pi]`
can appear to flip sign. For example, a real `+4.7 rad` rotation normalizes to
about `-1.6 rad`. Rotate mode therefore sums the shortest angular delta between
adjacent yaw samples and leaves the accumulated value unwrapped. The normalized
start/end delta is still printed for debugging, but it is not used as a FAIL
criterion for rotate direction.

## Linear Mode

Drive slowly and compare distance and direction across odom and TF:

```bash
./scripts/test_motion_diagnostics.sh --mode linear --publish --linear-x 0.05 --duration 10 --bag
```

Expected behavior:

- Positive commanded `linear.x` should produce forward odom progress or clear
  odom distance increase.
- Negative commanded `linear.x` should produce reverse odom progress or clear
  odom distance increase.
- Mean odom linear velocity should match the command sign.
- Heading drift is reported as `WARN` when it grows beyond the initial
  conservative threshold.
- Scan geometry stability is treated as a hard requirement because scan size or
  angle-field changes during motion can confuse localization.

## Square Mode

Square mode is a conservative smoke test. It alternates short straight and
rotate segments four times:

```bash
./scripts/test_motion_diagnostics.sh --mode square --publish --duration 16 --bag
```

The square summary is intentionally weaker than rotate and linear mode. Use it
to verify that a short combined motion sequence produces nontrivial odom and yaw
changes without scan geometry instability.

## Bag Recording

Add `--bag` to record the main hardware topics while the test runs:

```bash
./scripts/test_motion_diagnostics.sh --mode rotate --publish --duration 10 --bag
```

Default topics:

- `/cmd_vel`
- `/odom`
- `/imu`
- `/scan`
- `/tf`
- `/tf_static`
- `/joint_states`

Default output root:

```text
~/ws/logs/robot_hw/motion
```

Example bag path:

```text
~/ws/logs/robot_hw/motion/motion_rotate_YYYYmmdd_HHMMSS
```

Use `--output-dir DIR` to choose a different root. Use `--rosbag-duration SEC`
when you want the bag recorder to stop earlier than the observer.

## Summary

At exit the script prints a human-readable summary:

```text
Motion diagnostics summary
- mode: rotate
- duration_sec: 10.000
- commanded_linear_x: 0.000
- commanded_angular_z: 0.500
- odom_yaw_delta_rad: ...
- tf_yaw_delta_rad: ...
- odom_yaw_delta_normalized_rad: ...
- tf_yaw_delta_normalized_rad: ...
- imu_angular_z_mean: ...
- odom_angular_z_mean: ...
- xy_drift_m: ...
- scan_geometry_stable: true
- bag_path: ...
- result: PASS
- reason: none
```

It also prints a structured line:

```text
ROBOT_HW_LOG schema=v1 tag=DIAG component=motion event=motion_summary ...
```

## PASS, WARN, FAIL

`PASS` means the observed motion matched the selected command well enough for
this field workflow.

`WARN` means the data deserves attention but is not immediately classified as a
hardware contract failure. Examples include missing IMU angular velocity, IMU
sign mismatch, TF samples unavailable during the test, small odom-vs-TF yaw
delta mismatch, heading drift, or rotate-mode X/Y drift over `0.05 m`.

`FAIL` means the main semantic motion contract failed. Examples include missing
odom samples, commanded rotate direction disagreeing with odom angular velocity
or unwrapped odom yaw, large odom-vs-TF yaw delta mismatch above `0.25 rad`,
commanded linear motion not appearing in odom, or scan geometry changing during
the run.

## Interpreting Mismatches

Odom/IMU sign mismatch usually means an IMU axis convention, yaw sign, driver
mapping, or sensor frame assumption needs review.

Odom/TF mismatch means the pose being published in `/odom` and the dynamic
`odom -> base_footprint` transform are not describing the same motion. The
v0.1.11 contract monitor already checks timestamp-synchronized odom pose versus
TF; this workflow checks whether the deltas remain meaningful during a command.

Scan geometry instability means `/scan` fields such as range count, angle min,
angle max, or angle increment changed during motion. That can break downstream
localization even when motion feedback looks correct.

X/Y drift during rotate-in-place is a quality indicator. It should be watched,
but it is not treated as an immediate hardware contract failure by default.
