# robot_lidar_driver

## Purpose

`robot_lidar_driver` is a ROS 2 Humble C++ LiDAR bringup package that keeps hardware
specifications in YAML while implementing its own POSIX serial, epoll, parser, and
`sensor_msgs/msg/LaserScan` publishing pipeline.

## Supported Model

- `lds_03`

## Implementation Summary

- POSIX serial open/close/read with `termios`
- epoll-based background serial reader with `eventfd` stop wakeup
- internal ring buffer for partial and merged UART packet handling
- independent LDS-03 parser based on packet format analysis from `coin_d4_driver`
- mock mode for RViz and topic verification without hardware

## Run

Default launch:

```bash
ros2 launch robot_lidar_driver lidar.launch.py
```

Integrated launch from `robot_bringup`:

```bash
ros2 launch robot_bringup sensor.launch.py
```

`robot_lidar_driver/launch/lidar.launch.py` is kept as a standalone launch, while
the preferred integrated bringup path uses `robot_bringup/config/robot.yaml`.

Override the parameter file:

```bash
ros2 launch robot_lidar_driver lidar.launch.py params_file:=/path/to/lidar.yaml
```

Launch inside a namespace:

```bash
ros2 launch robot_lidar_driver lidar.launch.py namespace:=robot1
```

When `namespace` is set and `topic_name` remains the default `"/scan"`, the node
publishes on the relative topic `scan`, which resolves under the namespace.

## Mock Mode

Set `mock_mode: true` in [config/lidar.yaml](config/lidar.yaml), then launch:

```bash
ros2 launch robot_lidar_driver lidar.launch.py
```

Mock mode does not open the serial port. It publishes a synthetic 360-degree scan
using the configured frame, topic, angle, and range parameters so TF wiring and RViz
display can be checked before hardware bringup.

## Real LiDAR Mode

Update these YAML parameters for the target device:

- `lidar_model`
- `port`
- `baudrate`
- `frame_id`
- `topic_name`

Then launch:

```bash
ros2 launch robot_lidar_driver lidar.launch.py
```

The default baudrate is `230400`. Some LDS-03 related documents or bridges may expect
`115200`, so this value should be treated as a deployment parameter rather than a
hardcoded hardware assumption.

## Parameters

The default file is [config/lidar.yaml](config/lidar.yaml).

| Parameter | Description |
| --- | --- |
| `lidar_model` | Parser selection key. Current supported value is `lds_03`. |
| `port` | Serial device path such as `/dev/ttyUSB0` or `/dev/robot/lidar`. |
| `baudrate` | Serial baudrate used by the POSIX serial layer. |
| `frame_id` | `LaserScan.header.frame_id`. |
| `topic_name` | Publish topic name. |
| `range_min` | Minimum valid range in meters. |
| `range_max` | Maximum valid range in meters. |
| `angle_min` | Published scan minimum angle in radians. |
| `angle_max` | Published scan maximum angle in radians. |
| `scan_angle_offset` | Adds a yaw offset to raw LiDAR angles before filling `ranges[]`. |
| `scan_direction_reversed` | Reverses the published scan arrays. |
| `reverse_scan` | Secondary scan reversal flag combined with `scan_direction_reversed`. |
| `debug_scan_geometry` | Logs `angle_*`, front/left/right/rear ranges and angles, cardinal indexes, sector minima, nearest hit, and raw-to-scan angle mapping. |
| `publish_rate_hint_hz` | Used for `scan_time`, `time_increment`, and mock timer period. |
| `read_buffer_size` | Per-read byte buffer size used by the serial reader thread. |
| `ring_buffer_size` | Internal byte stream buffer size for parser input. |
| `use_epoll` | Enables epoll-based wait instead of polling fallback. |
| `reconnect_on_error` | Retries serial open/read failures without exiting the node. |
| `reconnect_interval_ms` | Delay between reconnect attempts. |
| `serial_read_timeout_ms` | epoll wait timeout. |
| `mock_mode` | Publishes fake scans without opening serial. |
| `log_raw_packet` | Throttled raw packet dump logging. |
| `log_packet_error` | Throttled checksum and packet parse warning logging. |

## Check Commands

```bash
ros2 topic echo /scan
ros2 topic hz /scan
rviz2
```
