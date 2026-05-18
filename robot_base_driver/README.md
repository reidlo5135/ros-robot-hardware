# robot_base_driver

## Purpose

`robot_base_driver` is a ROS 2 Humble C++ base bringup package for TurtleBot3
Burger-compatible OpenCR hardware. It uses POSIX serial and an in-package
Dynamixel Protocol 2.0 implementation to talk to stock OpenCR control-table
registers without depending on `turtlebot3_bringup`.

## Compatibility Target

- TurtleBot3 Burger-compatible OpenCR base interface
- Stock OpenCR control table layout used by TurtleBot3 Humble
- `cmd_vel` write path through OpenCR velocity command registers
- `odom`, `imu`, and `joint_states` derived from OpenCR feedback registers

## Run

Default launch:

```bash
ros2 launch robot_base_driver base.launch.py
```

Override the parameter file:

```bash
ros2 launch robot_base_driver base.launch.py params_file:=/path/to/base.yaml
```

Launch inside a namespace:

```bash
ros2 launch robot_base_driver base.launch.py namespace:=robot1
```

## Parameters

The default file is [config/base.yaml](config/base.yaml).

| Parameter | Description |
| --- | --- |
| `port` | OpenCR serial device path. |
| `baudrate` | Serial baudrate for the OpenCR connection. |
| `opencr_id` | Dynamixel device ID used by the stock OpenCR firmware. |
| `protocol_version` | Dynamixel protocol version. This driver supports `2.0`. |
| `cmd_vel_topic` | Command velocity subscription topic. |
| `odom_topic` | Odometry publish topic. |
| `imu_topic` | IMU publish topic. |
| `joint_states_topic` | Joint state publish topic. |
| `odom_frame_id` | `Odometry.header.frame_id`. |
| `base_frame_id` | `Odometry.child_frame_id` and TF child frame. |
| `imu_frame_id` | `Imu.header.frame_id`. |
| `wheel_left_joint_name` | Left wheel joint name used in `joint_states`. |
| `wheel_right_joint_name` | Right wheel joint name used in `joint_states`. |
| `wheel_separation` | Wheel separation in meters. |
| `wheel_radius` | Wheel radius in meters. |
| `publish_tf` | Publishes `odom -> base_footprint` TF when enabled. |
| `use_imu_for_yaw` | Uses OpenCR IMU orientation for yaw integration. |
| `publish_imu` | Enables `/imu` publishing. |
| `publish_joint_states` | Enables `/joint_states` publishing. |
| `poll_mode` | `minimal` reads only device status and wheel feedback. `full` adds optional IMU polling. |
| `max_consecutive_poll_failures` | Required poll failure threshold before reconnect is triggered. |
| `heartbeat_enabled` | Sends stock heartbeat writes to OpenCR. |
| `heartbeat_interval_ms` | Heartbeat write period. |
| `poll_interval_ms` | OpenCR feedback polling period. |
| `startup_delay_ms` | Delay before the startup ping/recalibration sequence. |
| `response_timeout_ms` | Timeout used for request/response transactions. |
| `reconnect_on_error` | Retries on serial/protocol failures. |
| `reconnect_interval_ms` | Delay between reconnect attempts. |
| `enable_stamped_cmd_vel` | Also subscribes to `geometry_msgs/msg/TwistStamped`. |
| `imu_recalibration_on_startup` | Sends the stock IMU recalibration write on startup. |
| `imu_recalibration_requires_ack` | Waits for a status packet for IMU recalibration if true. |
| `profile_acceleration_requires_ack` | Waits for a status packet for profile acceleration writes if true. |
| `heartbeat_requires_ack` | Waits for a status packet for heartbeat writes if true. |
| `startup_require_initial_state_read` | Requires a successful initial OpenCR state read during startup. |
| `startup_initial_state_read_retries` | Retry count for the initial required state read. |
| `startup_initial_state_read_retry_interval_ms` | Delay between initial state read retries. |
| `log_serial_packets` | Throttled packet hex logging for debugging. |
| `log_read_rate` | Logs serial read throughput every second. |

## Check Commands

```bash
ros2 topic echo /odom
ros2 topic echo /imu
ros2 topic echo /joint_states
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.2}}"
```

## Troubleshooting

- If the serial port cannot be opened, check the device path and user permissions.
- If the driver reconnects repeatedly, confirm that the OpenCR firmware is running
  and that the configured `opencr_id` and `baudrate` match the stock firmware.
- If topics appear in the global namespace while using a node namespace, keep the
  default topic names or switch the YAML topic names to relative names explicitly.

### Ping Succeeds But Startup Fails At IMU Recalibration

IMU recalibration is optional for stock TurtleBot3 Burger OpenCR bringup. If ping
works but the node used to fail right after startup, disable recalibration first
and keep startup focused on the required ping and initial state read.

Recommended first bringup settings:

```yaml
imu_recalibration_on_startup: false
heartbeat_enabled: false
response_timeout_ms: 500
log_serial_packets: false
```

Turn packet logging on only when you need protocol debugging:

```yaml
log_serial_packets: true
```

### Startup Succeeds But Poll Fails With 172 Bytes Expected

Older bringup logic that reads one large contiguous OpenCR block from address `10`
through `181` is not reliable on stock TurtleBot3 Burger firmware. This driver now
uses grouped polling instead:

- required minimal polling:
  - `DEVICE_STATUS`
  - wheel velocity/position block
- optional full polling:
  - IMU angular velocity block
  - IMU linear acceleration block
  - IMU orientation block

Recommended first bringup settings:

```yaml
poll_mode: "minimal"
max_consecutive_poll_failures: 5
response_timeout_ms: 500
publish_imu: false
use_imu_for_yaw: false
log_serial_packets: false
```

If you need deeper protocol inspection, temporarily enable:

```yaml
log_serial_packets: true
```

This package is an independently implemented serial driver that is intended to stay
compatible with the stock TurtleBot3 OpenCR control table.
