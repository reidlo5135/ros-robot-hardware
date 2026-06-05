# ros-robot-hardware

ROS 2 Humble hardware bringup packages for a TurtleBot3/OpenCR-compatible mobile base, LiDAR, robot description, and integrated launch.

## Packages

- `robot_bringup`: integrated launch entrypoint and shared `robot.yaml` parameters.
- `robot_lidar_driver`: LDS-03 / Coin D4 style serial LiDAR driver publishing `/scan`.
- `robot_base_driver`: OpenCR-compatible base driver for `/cmd_vel`, `/odom`, `/imu`, `/joint_states`, and `odom -> base_footprint` TF.
- `robot_description`: URDF/xacro and `robot_state_publisher` static frame chain.

## Normal Bringup

```bash
ros2 launch robot_bringup robot.launch.py
```

Launch all hardware components explicitly:

```bash
ros2 launch robot_bringup robot.launch.py use_description:=true use_sensor:=true use_motor:=true
```

## Field Debug Bringup

TF-focused:

```bash
ros2 launch robot_bringup robot.launch.py debug_tf:=true
```

Sensor geometry focused:

```bash
ros2 launch robot_bringup robot.launch.py debug_scan_geometry:=true
```

Base/odom focused:

```bash
ros2 launch robot_bringup robot.launch.py debug_odom:=true
```

Full field debug:

```bash
ros2 launch robot_bringup robot.launch.py debug_tf:=true debug_odom:=true debug_scan_geometry:=true
```

`log_level` sets the ROS logger level for included driver nodes. `debug_tf`, `debug_odom`, and `debug_scan_geometry` enable diagnostic summaries, but raw packet logging remains opt-in only through `log_raw_packet=true` for LiDAR and `log_serial_packets=true` for OpenCR.

## Structured Hardware Logs

v0.1.8 introduced structured hardware logs using:

```text
ROBOT_HW_LOG schema=v1 tag= component= event= key=value ...
```

Default structured logging is enabled in `robot_bringup/config/robot.yaml` under each node's `logging:` group. High-rate logs such as serial reads, scan publish, odom publish, TF publish, IMU publish, joint state publish, packet/parser state, and poll timing are throttled.

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

See [docs/logging/LOG_SCHEMA.md](docs/logging/LOG_SCHEMA.md) for the schema and event list.

## Rotation Diagnostics

v0.1.9 adds rotation-focused odom, IMU, scan, and TF diagnostics for cases where straight driving is stable but `map -> odom` drifts during turns.

Important logs:

- `event=rotation_state`: command angular velocity, odom angular velocity, integrated yaw, odom yaw, IMU yaw/gyro, wheel velocities, wheel deltas, and yaw source.
- `event=rotation_consistency`: command/odom/IMU sign checks, odom-vs-IMU yaw delta, odom-vs-command angular ratio, and result/reason.
- `event=scan_geometry`: front/left/right/rear scan angle and range fields for checking `base_scan` yaw.

Calibration parameters in `robot_bringup/config/robot.yaml`:

```yaml
odom:
  linear_scale: 1.0
  angular_scale: 1.0
```

`odom.angular_scale` affects wheel-derived odom yaw integration only; it does not change `/cmd_vel` command writing.

Safe rotation diagnostics:

```bash
./scripts/test_rotation_diagnostics.sh --bag --duration 20
./scripts/check_robot_hw_tf_publishers.sh
```

Explicit rotate-in-place command, only in a safe test area:

```bash
./scripts/test_rotation_diagnostics.sh --publish --angular-z 0.5 --duration 10 --bag
```

See [docs/debugging/TF_DIAGNOSTICS.md](docs/debugging/TF_DIAGNOSTICS.md) for the rotation drift workflow and calibration notes.

## TB3 Compatibility Diagnostics

v0.1.10 adds TB3 bringup comparison tooling and TurtleBot3/Coin D4/LDS-03 compatible LaserScan defaults. This is useful when TurtleBot3 bringup localizes correctly but robot_hw causes `map -> odom` distortion during rotation.

Default LiDAR geometry stabilization:

```yaml
scan_geometry_profile: "tb3_coin_d4"
fixed_scan_geometry: true
fixed_scan_samples: 400
fixed_angle_min: 0.0
fixed_angle_max: 6.283185307179586
fixed_scan_time: 0.1
fixed_time_increment: 0.0
mirror_scan_angles: true

tb3_compatibility:
  odom_zero_covariance: true
```

The default scan convention is `angle_min=0`, `angle_max=2*pi`, `angle_increment=2*pi/400`, with expected cardinal indexes front `0`, left `100`, rear `200`, and right `300`.

Capture TB3 baseline:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
./scripts/compare_tb3_compatibility.sh --samples 10 > ~/ws/logs/tb3_baseline.txt
```

Capture robot_hw:

```bash
ros2 launch robot_bringup robot.launch.py debug_tf:=true debug_odom:=true debug_scan_geometry:=true
./scripts/compare_tb3_compatibility.sh --samples 10 > ~/ws/logs/robot_hw_compat.txt
```

Compare:

```bash
diff -u ~/ws/logs/tb3_baseline.txt ~/ws/logs/robot_hw_compat.txt
```

The compatibility script subscribes to `/scan` and `/imu` with SensorDataQoS-compatible `best_effort`, `volatile`, `keep_last`, depth `10`, so it can read both TurtleBot3 bringup and robot_hw sensor topics without reliability QoS mismatch warnings. It also prints subscriber/publisher QoS metadata for those sensor topics. A RELIABILITY warning on `/imu` should be treated first as a comparison subscriber QoS problem; inspect publisher QoS with `ros2 topic info -v /imu`.

See [docs/debugging/TB3_COMPATIBILITY.md](docs/debugging/TB3_COMPATIBILITY.md) for the full workflow.

## TF Responsibility

Expected runtime TF chain:

```text
map -> odom                          localization / AMCL / SLAM, not robot_hardware
odom -> base_footprint               robot_base_driver dynamic TF
base_footprint -> base_link          robot_state_publisher static TF
base_link -> base_scan               robot_state_publisher static TF
base_link -> imu_link                robot_state_publisher static TF
base_link -> wheel_left/right_link   robot_state_publisher + /joint_states
```

`robot_hardware` must not publish `map -> odom`. That transform belongs to localization/navigation.

See [docs/debugging/TF_DIAGNOSTICS.md](docs/debugging/TF_DIAGNOSTICS.md) for the TF workflow.

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

Do not add a `map -> odom` publisher to this repository.

## Diagnostic Scripts

Filter structured logs:

```bash
./scripts/watch_robot_hw_logs.sh --tag TF --follow --file ~/ws/logs/robot_hw/latest.log
./scripts/watch_robot_hw_logs.sh --tag SENSOR --event scan_publish < bringup.log
```

Check the TF chain:

```bash
./scripts/check_tf_chain.sh
```

Inspect topics:

```bash
./scripts/inspect_robot_hw_topics.sh
```

Record a lightweight bag:

```bash
./scripts/record_robot_hw_bag_light.sh
```

Run rotation diagnostics:

```bash
./scripts/test_rotation_diagnostics.sh --bag --duration 20
./scripts/check_robot_hw_tf_publishers.sh
```

Compare TurtleBot3 bringup and robot_hw outputs:

```bash
./scripts/compare_tb3_compatibility.sh > ~/ws/logs/robot_hw_compat.txt
```

Run bringup under `nohup` and store logs:

```bash
./scripts/run_robot_hw_nohup.sh debug_tf:=true debug_odom:=true
```

See [scripts/README.md](scripts/README.md) for script options.

## Validation

Use [docs/checklists/ROBOT_HW_LOGGING_CHECKLIST.md](docs/checklists/ROBOT_HW_LOGGING_CHECKLIST.md) after bringup to verify sensor, base, TF, and AMR navigation integration behavior.
