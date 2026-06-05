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

v0.1.8 introduces structured hardware logs using:

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

Run bringup under `nohup` and store logs:

```bash
./scripts/run_robot_hw_nohup.sh debug_tf:=true debug_odom:=true
```

See [scripts/README.md](scripts/README.md) for script options.

## Validation

Use [docs/checklists/ROBOT_HW_LOGGING_CHECKLIST.md](docs/checklists/ROBOT_HW_LOGGING_CHECKLIST.md) after bringup to verify sensor, base, TF, and AMR navigation integration behavior.
