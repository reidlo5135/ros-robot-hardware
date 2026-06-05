# robot_bms_driver

## Purpose

`robot_bms_driver` is an optional ROS 2 Humble BMS bringup package for publishing
`sensor_msgs/msg/BatteryState` on `/battery_state`.

The package is intentionally isolated from `robot_base_driver` so OpenCR odom, IMU,
joint state, TF, LiDAR, and scan behavior remain unchanged when BMS support is not
enabled.

## Current Protocol Status

No concrete BMS vendor protocol is documented in this repository yet. The default
`bms.protocol: "placeholder"` opens the extension point but does not invent binary
commands, registers, CRC rules, or fake battery values.

With the placeholder protocol, the node may read serial bytes and log diagnostics,
but it does not publish `/battery_state` until a valid parser implementation is
added and configured.

## Run

Standalone launch is disabled by default:

```bash
ros2 launch robot_bms_driver bms.launch.py
```

Enable BMS polling explicitly:

```bash
ros2 launch robot_bms_driver bms.launch.py enabled:=true
```

Integrated bringup:

```bash
ros2 launch robot_bringup robot.launch.py use_bms:=true
```

## Parameters

The standalone default file is [config/bms.yaml](config/bms.yaml). Integrated
bringup uses [../robot_bringup/config/robot.yaml](../robot_bringup/config/robot.yaml).

| Parameter | Description |
| --- | --- |
| `bms.enabled` | Enables serial polling. Default is `false`. |
| `bms.port` | BMS serial device path, for example `/dev/robot/bms`. |
| `bms.baudrate` | Serial baudrate. Default `9600`. |
| `bms.frame_id` | `BatteryState.header.frame_id`. Default `base_link`. |
| `bms.topic_name` | BatteryState publish topic. Default `/battery_state`. |
| `bms.poll_interval_ms` | Timer period for nonblocking serial reads. |
| `bms.read_timeout_ms` | Reserved protocol read timeout setting for parser implementations. |
| `bms.frame_timeout_ms` | Reserved frame assembly timeout setting for parser implementations. |
| `bms.protocol` | Parser key. Current safe default is `placeholder`. |
| `bms.publish_diagnostics` | Enables structured BMS diagnostics logs. |
| `bms.log_raw_frames` | Logs raw serial bytes as hex when enabled. Default `false`. |
| `bms.warn_timeout_ms` | Warns when no valid BMS frame is parsed within this period. |
| `bms.read_buffer_size` | Per-poll serial read buffer size. |
| `logging.structured_enabled` | Enables `ROBOT_HW_LOG` structured logs. |
| `logging.diagnostics_throttle_sec` | Throttle interval for repeated BMS diagnostics. |

## Published Topic

When enabled and a concrete parser returns a valid sample, the node publishes:

- `/battery_state` (`sensor_msgs/msg/BatteryState`)

Unknown numeric values should be set to `NaN` by parser implementations. Unknown
enum values should use the `UNKNOWN` constants from `BatteryState`.

The node fills standard BatteryState fields when available:

- `header.stamp`, `header.frame_id`
- `voltage`, `current`, `charge`, `capacity`, `design_capacity`, `percentage`
- `temperature`, `cell_voltage`, `cell_temperature`
- `power_supply_status`, `power_supply_health`, `power_supply_technology`
- `present`, `location`, `serial_number`

## Adding A Vendor Parser

Add documented protocol handling in `BmsParser::parseBytes()`:

1. Add a protocol key, for example `my_vendor_v1`.
2. Implement frame boundary detection and frame timeout behavior.
3. Validate checksum/CRC exactly as documented by the vendor.
4. Convert parsed fields into `BatterySample`.
5. Leave unavailable values unset so the node publishes `NaN` or `UNKNOWN`.

Do not add guessed command bytes, register layouts, or checksum rules without a
protocol reference.

## Check Commands

```bash
ros2 topic echo /battery_state sensor_msgs/msg/BatteryState
../scripts/echo_battery_state.sh --once
```

Structured diagnostics use `component=bms`:

```bash
../scripts/watch_robot_hw_logs.sh --tag SENSOR --follow --file ~/ws/logs/robot_hw/latest.log
```