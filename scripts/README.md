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
