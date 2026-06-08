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
- `/battery_state` owned by the OpenCR/base-driver path when battery voltage is available

## ROS Interface

- Subscribe: `/cmd_vel` `geometry_msgs/msg/Twist`
- Publish: `/odom` `nav_msgs/msg/Odometry`
- Publish: `/joint_states` `sensor_msgs/msg/JointState`
- Publish: `/imu` `sensor_msgs/msg/Imu`
- Publish: `/battery_state` `sensor_msgs/msg/BatteryState` when OpenCR battery voltage is valid
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
| `battery.publish_battery_state` | Creates the `/battery_state` publisher in the base driver. Messages are published with `present=false` and `voltage=NaN` while voltage is unavailable. |
| `battery.frame_id` | `BatteryState.header.frame_id`. Default `base_link`. |
| `battery.read_enabled` | Enables optional OpenCR battery raw register reads. Default `true` so battery telemetry is observable. |
| `battery.register_address` | Candidate OpenCR battery register address. Default `0`, unused while `battery.read_enabled=false`. |
| `battery.register_length` | Raw register byte length: `1`, `2`, or `4`. Default `2`. |
| `battery.raw_type` | Raw register type: `uint8`, `uint16`, `uint32`, `int32`, or `float32`. Default `uint16`. |
| `battery.raw_scale` | Scale applied to the raw value before voltage calibration. Default `1.0`. |
| `battery.raw_offset` | Offset applied after raw scale. Default `0.0`. |
| `battery.voltage_scale` | Final voltage scale. Default `1.0`. |
| `battery.voltage_offset` | Final voltage offset. Default `0.0`. |
| `battery.publish_percentage` | Enables voltage-based percentage calculation. Disabled by default because voltage SOC is approximate. |
| `battery.min_voltage` | Lower voltage bound for optional percentage calculation. Must be lower than `battery.max_voltage`. |
| `battery.max_voltage` | Upper voltage bound for optional percentage calculation. Must be higher than `battery.min_voltage`. |
| `battery.warn_low_voltage` | Enables throttled low-voltage structured warnings when valid voltage is at or below `battery.low_voltage`. |
| `battery.low_voltage` | Low-voltage warning threshold in volts. |
| `battery.log_battery_state` | Enables throttled `battery_state` structured logs after valid voltage is available. |
| `battery.mapping_state` | Human-readable mapping status, such as `unconfirmed` or `field_validation`. |
| `odom_frame_id` | `Odometry.header.frame_id`. |
| `base_frame_id` | `Odometry.child_frame_id` and TF child frame. |
| `imu_frame_id` | `Imu.header.frame_id`. |
| `wheel_left_joint_name` | Left wheel joint name used in `joint_states`. |
| `wheel_right_joint_name` | Right wheel joint name used in `joint_states`. |
| `wheel_separation` | Wheel separation in meters. |
| `wheel_radius` | Wheel radius in meters. |
| `odom.linear_scale` | Multiplies integrated linear odom displacement. Default `1.0`. |
| `odom.angular_scale` | Multiplies wheel-derived odom yaw delta. Default `1.0`; does not change command writing. |
| `tb3_compatibility.odom_zero_covariance` | Publishes zero pose/twist covariance on `/odom` for TurtleBot3 compatibility when `true`. Set `false` to use `odom_pose_covariance_diagonal` and `odom_twist_covariance_diagonal`. |
| `left_encoder_sign` | Multiplies the left OpenCR encoder/velocity feedback by `-1` or `1` before odom and joint-state integration. |
| `right_encoder_sign` | Multiplies the right OpenCR encoder/velocity feedback by `-1` or `1` before odom and joint-state integration. |
| `swap_wheel_encoders` | Swaps left/right OpenCR wheel feedback before sign correction, odom integration, and joint-state publishing. |
| `odom_pose_covariance_diagonal` | Six-element pose covariance diagonal for `/odom` in row-major order `(x, y, z, roll, pitch, yaw)`. |
| `odom_twist_covariance_diagonal` | Six-element twist covariance diagonal for `/odom` in row-major order `(vx, vy, vz, vroll, vpitch, vyaw)`. |
| `command_mode` | OpenCR command payload mode. `body_twist` is the current verified workspace mode. `wheel_velocity` is guarded and will refuse to send commands until per-wheel firmware register mapping is verified. |
| `publish_tf` | Publishes `odom -> base_footprint` TF when enabled. |
| `use_imu_for_yaw` | Uses OpenCR IMU orientation for yaw integration. |
| `publish_imu` | Enables `/imu` publishing. Default is `true`. |
| `publish_joint_states` | Enables `/joint_states` publishing. |
| `poll_mode` | `odom` reads only wheel velocity/position for `/odom` and `/joint_states`. `minimal` also keeps optional status/torque reads. `full` adds IMU polling and is the TurtleBot3-compatible default when `/imu` should be published. |
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
| `debug_odom` | Logs raw wheel feedback, adjusted wheel feedback, wheel deltas, integrated odom, odom quaternion yaw, and recent `cmd_vel` comparison. |
| `debug_tf` | Logs published `odom -> base_footprint` TF translation, quaternion, and recovered yaw. |
| `debug_poll_timing` | Enables once-per-second OpenCR poll timing summaries, including required read, IMU read, device status read, command write, and wait durations. |
| `target_odom_rate_hz` | Expected `/odom` and `/joint_states` target rate used in poll timing diagnostics. |
| `logging.rotation_diagnostics_throttle_sec` | Throttle interval for `event=rotation_state` and `event=rotation_consistency`. Default `1.0`. |
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
ros2 topic echo /battery_state sensor_msgs/msg/BatteryState
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.2}}"
```

## Battery State

TurtleBot3 Burger does not have a separate default BMS serial device. Battery
state should flow through the same OpenCR connection as odom, IMU, joint state,
and device status feedback.

The current repository does not document a verified OpenCR battery-voltage control
table field. For that reason the base driver owns the `/battery_state` publisher
and provides an optional raw register read path. `/battery_state` messages are
published even while voltage is unavailable so operators can immediately tell
whether battery telemetry is disabled, unreadable, or still unconfirmed.

Default policy:

- `battery.publish_battery_state: true`
- `battery.read_enabled: true`
- `battery.mapping_state: "unconfirmed"`
- Register read failures never fail the base driver poll loop.
- Unavailable battery telemetry publishes `present=false`, `voltage=NaN`, and
  `percentage=NaN`.

Field validation flow:

1. Keep `battery.read_enabled: true` while validating telemetry.
2. Set the candidate `battery.register_address`, `battery.register_length`, and
   `battery.raw_type`.
3. Adjust `battery.raw_scale`, `battery.raw_offset`, `battery.voltage_scale`, and
   `battery.voltage_offset`.
4. Compare `/battery_state.voltage` against a meter on the real battery pack.
5. Set `battery.mapping_state: "field_validation"` while validating, and only
   promote it after the register/scaling is confirmed.

When voltage becomes available, the message uses:

- `header.stamp`: current base-driver state update time
- `header.frame_id`: `battery.frame_id`, default `base_link`
- `voltage`: OpenCR battery voltage, or `NaN` while unavailable
- `current`, `charge`, `capacity`, `design_capacity`, `temperature`: `NaN`
- `percentage`: `NaN` unless `battery.publish_percentage=true` and voltage bounds are valid
- `power_supply_status`, `power_supply_health`: `UNKNOWN`
- `power_supply_technology`: `LION` for the TurtleBot3 battery assumption
- `present`: `true` when voltage is valid, `false` while unavailable

Unavailable example:

```yaml
header:
  frame_id: base_link
voltage: .nan
current: .nan
percentage: .nan
power_supply_status: 0
power_supply_health: 0
power_supply_technology: 3
present: false
```

Voltage-based percentage is a coarse approximation, not a battery fuel gauge. Set
`battery.min_voltage` and `battery.max_voltage` explicitly for the battery pack in
use before enabling `battery.publish_percentage`.

Structured logs:

- `event=battery_config`: publisher, raw read, scaling, percentage, and mapping state.
- `event=battery_raw`: raw register value and converted voltage when `read_enabled=true`.
- `event=battery_state_unavailable`: `present=false` message was published; reason indicates disabled read, register read failure, invalid raw value, or unconfirmed mapping.
- `event=battery_state`: throttled publish log when `battery.log_battery_state=true`.
- `event=battery_low_voltage`: warning when enabled and voltage is below `battery.low_voltage`.

Rotation diagnostics:

```bash
../scripts/test_rotation_diagnostics.sh --bag --duration 20
../scripts/test_rotation_diagnostics.sh --publish --angular-z 0.5 --duration 10 --bag
```

The first command records without publishing motion. The second command publishes a rotate-in-place command and should only be used in a safe test area.

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
- If `/battery_state` exists but no messages arrive, the OpenCR battery voltage
  field mapping is still unconfirmed or disabled. Check `battery.read_enabled`,
  `battery.register_address`, `battery.mapping_state`, and `ROBOT_HW_LOG
  event=battery_config`, `event=battery_raw`, and
  `event=battery_state_unavailable`.

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
debug_poll_timing: false
target_odom_rate_hz: 20.0
use_imu_for_yaw: false
require_device_status: false
require_imu: false
reconnect_on_poll_failure: false
reopen_serial_on_poll_failure: false
probe_registers_on_startup: true
log_serial_packets: false
```

Switch `poll_mode` to `odom` only when you intentionally want wheel-only polling
for serial stress isolation. That mode is not a full TurtleBot3 ROS contract
because `/imu` will not be published.

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

Because the active command path is body-twist rather than per-wheel goal velocity,
this repository intentionally does not add `left_command_sign`,
`right_command_sign`, or `swap_wheel_commands` parameters. Those would imply a
verified per-wheel command mapping that this workspace does not currently have.
The current diagnostics instead focus on verifying whether OpenCR feedback signs
and wheel ordering are consistent with REP-103 odom semantics.

## Timing Guidance

`poll_interval_ms` controls OpenCR feedback polling and therefore the effective
publish cadence of `/odom`, `/joint_states`, and `odom -> base_footprint` TF.

- `300 ms` is too slow for Nav2 controller feedback and can make TF/odom look jumpy.
- `50 ms` is the current recommended default for hardware bringup.
- Further tuning can typically stay in the `20~50 ms` range depending on serial stability.
- Expected `/odom`, `/joint_states`, and `odom -> base_footprint` TF rate is typically `15~20 Hz` or higher.

AMCL and other localization stacks are also sensitive to message semantics, not
just the numeric odom path. This package now assigns non-zero default odom pose
and twist covariance diagonals instead of leaving the covariance matrices at
all-zero. All-zero odom covariance can be interpreted as unrealistically
perfect motion and may destabilize `map -> odom` updates.

`poll_mode: "full"` performs the TurtleBot3-compatible OpenCR feedback cycle:

- 1 bulk wheel feedback read for odom/joint states
- optional `DEVICE_STATUS` and `MOTOR_TORQUE_ENABLE` reads
- 1 bulk IMU feedback read

The bulk reads keep `/odom`, `/joint_states`, `/imu`, and `odom -> base_footprint`
on the same ROS-time sampling path while avoiding the old per-register IMU
transaction cost.

Useful runtime checks:

```bash
ros2 topic hz /odom
ros2 topic hz /joint_states
ros2 topic hz /tf
```

## Odom Sign Verification

Run motor-only bringup first so Nav2, SLAM, and AMCL cannot hide a bad odom sign:

```bash
ros2 launch robot_bringup motor.launch.py log_level:=debug
```

Recommended direct checks:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}, angular: {z: 0.0}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}"
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_tools view_frames
```

Expected behavior:

- `linear.x > 0` should move the real robot forward and increase `odom.x`.
- `linear.x > 0` should keep `yaw` close to zero and keep left/right wheel deltas with the same sign.
- `angular.z > 0` should rotate the real robot counter-clockwise and increase odom yaw.
- `angular.z < 0` should rotate the real robot clockwise and decrease odom yaw.
- Only `odom -> base_footprint` should come from `robot_base_driver`; `base_footprint -> base_link` and sensor links should come from `robot_state_publisher`.

When `debug_odom=true`, the node logs:

- raw left/right position and velocity from OpenCR
- adjusted left/right position and velocity after swap/sign correction
- left/right wheel delta in radians and meters
- `delta_s`, `delta_theta`, `dt`, integrated `x/y/yaw`
- odom quaternion yaw and joint angular velocities
- recent `cmd_vel` compared against published odom twist

If forward motion makes `delta_s` negative, or left rotation makes `delta_theta`
negative, investigate `left_encoder_sign`, `right_encoder_sign`, and
`swap_wheel_encoders` before retrying Nav2 goals.
