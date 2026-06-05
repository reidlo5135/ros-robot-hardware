# Robot Hardware Diagnostic Scripts

All scripts are bash scripts, support `--help`, and do not require a package rebuild.

## watch_robot_hw_logs.sh

Filter `ROBOT_HW_LOG` lines by tag and event.

```bash
./scripts/watch_robot_hw_logs.sh --tag TF --follow --file ~/ws/logs/robot_hw/latest.log
./scripts/watch_robot_hw_logs.sh --tag SENSOR --event scan_publish < bringup.log
```

## check_tf_chain.sh

Run basic `tf2_echo` checks for the hardware TF chain.

```bash
./scripts/check_tf_chain.sh
./scripts/check_tf_chain.sh --timeout 10
```

Expected ownership:

- `odom -> base_footprint`: `robot_base_driver`
- `base_footprint -> base_link`: `robot_state_publisher`
- `base_link -> base_scan`: `robot_state_publisher`
- `base_link -> imu_link`: `robot_state_publisher`

`map -> odom` is not expected from `robot_hardware`.

## check_robot_hw_tf_publishers.sh

Inspect `/tf`, `/tf_static`, and relevant running nodes to catch duplicate or misplaced TF publishers.

```bash
./scripts/check_robot_hw_tf_publishers.sh
```

Expected ownership:

- `odom -> base_footprint`: `robot_base_driver`
- `base_footprint -> base_link`: `robot_state_publisher`
- `base_link -> base_scan`: `robot_state_publisher`
- `base_link -> imu_link`: `robot_state_publisher`
- `map -> odom`: localization/navigation, not `robot_hardware`

## test_rotation_diagnostics.sh

Run a rotation-focused diagnostic session. It does not publish motion unless `--publish` is set.

```bash
./scripts/test_rotation_diagnostics.sh
./scripts/test_rotation_diagnostics.sh --bag --duration 20
./scripts/test_rotation_diagnostics.sh --publish --angular-z 0.5 --duration 10 --bag
```

Watch `event=rotation_state` and `event=rotation_consistency` while comparing `/odom`, `/imu`, `/tf`, and `/scan`.

## compare_tb3_compatibility.sh

Capture a one-shot compatibility snapshot of `/scan`, `/odom`, `/imu`, `/tf`, and `/tf_static`.

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

The scan section prints requested/received sample count, timeout status, range count stability, angle increment stability, nearest-hit angle/range statistics, and representative frame, angle, timing, front/left/right/rear indexes, and ranges. `/scan` and `/imu` use SensorDataQoS-compatible best-effort subscriptions and include subscriber/publisher QoS metadata in the snapshot; `/odom` uses the default subscription QoS. Odom, IMU, and TF sections print the fields most likely to affect localization compatibility.

TB3/OpenCR IMU topics may publish with SensorDataQoS. A RELIABILITY QoS warning for `/imu` means the comparison subscriber QoS is wrong or stale, not necessarily that the publisher is broken. Inspect publisher QoS with:

```bash
ros2 topic info -v /imu
```

## record_robot_hw_bag_light.sh

Record a lightweight hardware bag.

```bash
./scripts/record_robot_hw_bag_light.sh
./scripts/record_robot_hw_bag_light.sh --duration 30
```

Default output path:

```text
~/ws/bags/robot_hw_light_YYYYmmdd_HHMMSS
```

## inspect_robot_hw_topics.sh

Inspect core topic publishers/subscribers.

```bash
./scripts/inspect_robot_hw_topics.sh
```

## echo_battery_state.sh

Echo the optional BMS `sensor_msgs/msg/BatteryState` topic.

```bash
./scripts/echo_battery_state.sh
./scripts/echo_battery_state.sh --once
```

`/battery_state` is only expected when BMS bringup is enabled and a concrete BMS parser has produced a valid frame.

## run_robot_hw_nohup.sh

Run integrated bringup under `nohup` and store logs.

```bash
./scripts/run_robot_hw_nohup.sh
./scripts/run_robot_hw_nohup.sh debug_tf:=true debug_odom:=true
```

Environment variables:

- `ROBOT_HW_LOG_DIR`: log directory, default `~/ws/logs/robot_hw`
- `ROBOT_HW_WS`: workspace root to source, default `~/ws`
- `ROS_DOMAIN_ID`: passed through
- `RMW_IMPLEMENTATION`: passed through
