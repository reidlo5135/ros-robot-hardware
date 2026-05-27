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

## ROS Interface

- Subscribe: `/cmd_vel` `geometry_msgs/msg/Twist`
- Publish: `/odom` `nav_msgs/msg/Odometry`
- Publish: `/joint_states` `sensor_msgs/msg/JointState`
- Publish: `/imu` `sensor_msgs/msg/Imu`
- Publish: `tf` `odom -> base_footprint` when `publish_tf=true`

`TwistStamped` is supported only as an optional secondary interface through
`cmd_vel_stamped_topic` when `enable_stamped_cmd_vel=true`. The default
TurtleBot3/Nav2/teleop-compatible command input is always `geometry_msgs/msg/Twist`
on `/cmd_vel`.

## Run

Default launch:

```bash
ros2 launch robot_base_driver base.launch.py
```

Integrated launch from `robot_bringup`:

```bash
ros2 launch robot_bringup motor.launch.py
```

`robot_base_driver/launch/base.launch.py` is kept as a standalone launch, while the
preferred integrated bringup path uses `robot_bringup/config/robot.yaml`.

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
| `cmd_vel_stamped_topic` | Optional `TwistStamped` command velocity subscription topic. |
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
| `command_mode` | OpenCR command payload mode. `body_twist` is the current verified workspace mode. `wheel_velocity` is guarded and will refuse to send commands until per-wheel firmware register mapping is verified. |
| `publish_tf` | Publishes `odom -> base_footprint` TF when enabled. |
| `use_imu_for_yaw` | Uses OpenCR IMU orientation for yaw integration. |
| `publish_imu` | Enables `/imu` publishing. Default is `true`. |
| `publish_joint_states` | Enables `/joint_states` publishing. |
| `poll_mode` | `minimal` reads required wheel feedback only. `full` adds IMU polling. Default is `full`. |
| `max_consecutive_poll_failures` | Required poll failure threshold before reconnect is triggered. |
| `poll_device_status` | Enables optional `DEVICE_STATUS` polling. Disabled by default for first bringup. |
| `require_device_status` | Makes `DEVICE_STATUS` a required poll item when enabled. |
| `require_imu` | Makes IMU polling required in `full` mode when enabled. |
| `reconnect_on_poll_failure` | Allows repeated required poll failures to trigger reconnect handling. |
| `reopen_serial_on_poll_failure` | Reopens `/dev/ttyACM0` on poll failure reconnect when enabled. |
| `probe_registers_on_startup` | Probes known OpenCR registers individually after ping for bringup debugging. |
| `heartbeat_enabled` | Sends stock heartbeat writes to OpenCR. |
| `heartbeat_interval_ms` | Heartbeat write period. |
| `poll_interval_ms` | OpenCR feedback polling period. Recommended hardware bringup default is `50 ms`; `300 ms` is too slow for stable Nav2 odom/TF feedback. |
| `startup_delay_ms` | Delay before the startup ping/recalibration sequence. |
| `response_timeout_ms` | Timeout used for request/response transactions. |
| `transaction_gap_us` | Small gap inserted between Dynamixel transactions to reduce CDC framing pressure. |
| `reconnect_on_error` | Retries on serial/protocol failures. |
| `reconnect_interval_ms` | Delay between reconnect attempts. |
| `debug_motor_command` | Enables throttle logs for transmitted raw command fields, payload bytes, expected wheel goal velocities, and motor readiness state. |
| `enable_stamped_cmd_vel` | Enables an additional `geometry_msgs/msg/TwistStamped` subscription on `cmd_vel_stamped_topic`. |
| `motor_torque_enable_on_startup` | Sends `MOTOR_TORQUE_ENABLE=1` during startup. |
| `motor_torque_enable_requires_ack` | Waits for a status packet for torque enable writes if true. |
| `imu_recalibration_on_startup` | Sends the stock IMU recalibration write on startup. |
| `imu_recalibration_requires_ack` | Waits for a status packet for IMU recalibration if true. |
| `profile_acceleration_requires_ack` | Waits for a status packet for profile acceleration writes if true. |
| `profile_acceleration_on_startup` | Sends profile acceleration writes during startup when enabled. |
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
- If `/cmd_vel` is published as `geometry_msgs/msg/Twist`, keep `enable_stamped_cmd_vel: false`
  unless you also want a separate `TwistStamped` input on `cmd_vel_stamped_topic`.
- If `/imu` is missing, check `publish_imu: true` and `poll_mode: "full"` first.
- If `/cmd_vel` is acknowledged but the robot does not move, confirm `heartbeat_enabled: true`
  and inspect `motor_torque_enable` diagnostics in the node logs.
- If topics appear in the global namespace while using a node namespace, keep the
  default topic names or switch the YAML topic names to relative names explicitly.
- If startup succeeds but runtime polling still jitters, keep `poll_mode: "minimal"`
  first and use `log_serial_packets: true` only while capturing parser diagnostics.
- If `poll_device_status` is disabled, the node cannot determine whether `device_status=-1`
  is a real motor power fault. Enable `poll_device_status: true` when debugging motor bringup.

### Ping Succeeds But Startup Fails At IMU Recalibration

IMU recalibration is optional for stock TurtleBot3 Burger OpenCR bringup. If ping
works but the node used to fail right after startup, disable recalibration first
and keep startup focused on the required ping and initial state read.

Recommended first bringup settings:

```yaml
imu_recalibration_on_startup: false
heartbeat_enabled: false
response_timeout_ms: 500
profile_acceleration_on_startup: false
log_serial_packets: false
```

Turn packet logging on only when you need protocol debugging:

```yaml
log_serial_packets: true
```

### Startup Succeeds But Poll Fails With 172 Bytes Expected

Older bringup logic that reads one large contiguous OpenCR block from address `10`
through `181` is not reliable on stock TurtleBot3 Burger firmware. Host-side
contiguous multi-item reads are also fragile even when addresses look adjacent,
because the OpenCR firmware registers control items individually. This driver now
polls control items one by one, keeps a persistent RX stream buffer, and only
extracts complete Dynamixel 2.0 packets after header/length/CRC validation:

- required minimal polling:
  - `PRESENT_VELOCITY_LEFT`
  - `PRESENT_VELOCITY_RIGHT`
  - `PRESENT_POSITION_LEFT`
  - `PRESENT_POSITION_RIGHT`
- optional polling:
  - `DEVICE_STATUS`
  - IMU angular velocity x/y/z
  - IMU linear acceleration x/y/z
  - IMU orientation w/x/y/z

Recommended first bringup settings:

```yaml
poll_mode: "full"
max_consecutive_poll_failures: 5
poll_device_status: true
response_timeout_ms: 500
transaction_gap_us: 10000
poll_interval_ms: 50
publish_imu: true
heartbeat_enabled: true
debug_motor_command: true
use_imu_for_yaw: false
require_device_status: false
require_imu: false
reconnect_on_poll_failure: false
reopen_serial_on_poll_failure: false
probe_registers_on_startup: true
log_serial_packets: false
```

Recommended motion verification commands:

```bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

If you need deeper protocol inspection, temporarily enable:

```yaml
log_serial_packets: true
```

With packet logging enabled, the driver logs:

- raw RX byte chunks
- packet extraction boundaries
- CRC success/failure recovery
- buffer sizes before and after extraction
- periodic parser stats such as `crc_failures`, `sync_recoveries`, `partial_reads`,
  `packets_decoded`, and `packets_dropped`

Short reads by themselves are not treated as fatal. The driver keeps the serial
port open, preserves partial packets in the RX buffer, and only escalates to
reconnect handling on hard transport failures or explicit timeout policies.

The driver now flushes stale serial input before each request-response transaction
and only accepts read status packets whose parameter length exactly matches the
current register request. Same-ID packets with the wrong payload length are
treated as stale and ignored until timeout.

With `probe_registers_on_startup: true`, the driver logs one-by-one probe results for:

- `18` length `1`
- `128` length `4`
- `132` length `4`
- `136` length `4`
- `140` length `4`
- `150` length `4`
- `170` length `4`

This package is an independently implemented serial driver that is intended to stay
compatible with the stock TurtleBot3 OpenCR control table.

## Command Semantics

The current workspace implementation treats the OpenCR velocity write path as a
body-twist command block:

- `CMD_VELOCITY_LINEAR_X`
- `CMD_VELOCITY_LINEAR_Y`
- `CMD_VELOCITY_LINEAR_Z`
- `CMD_VELOCITY_ANGULAR_X`
- `CMD_VELOCITY_ANGULAR_Y`
- `CMD_VELOCITY_ANGULAR_Z`

In `command_mode: "body_twist"`, the driver transmits:

- `linear_x_raw = linear.x * 100`
- `angular_z_raw = angular.z * 100`
- all other body fields as zero

The driver still computes expected left/right wheel goal velocities for diagnostics,
but those values are not transmitted in `body_twist` mode.

`command_mode: "wheel_velocity"` is intentionally blocked for now. This repository
does not include a verified per-wheel OpenCR command register map, so the driver
logs a clear error and suppresses unsafe writes instead of guessing firmware behavior.

## Timing Guidance

`poll_interval_ms` controls OpenCR feedback polling and therefore the effective
publish cadence of `/odom`, `/joint_states`, and `odom -> base_footprint` TF.

- `300 ms` is too slow for Nav2 controller feedback and can make TF/odom look jumpy.
- `50 ms` is the current recommended default for hardware bringup.
- Further tuning can typically stay in the `20~50 ms` range depending on serial stability.
