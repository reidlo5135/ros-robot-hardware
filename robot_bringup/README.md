# robot_bringup

`robot_bringup` centralizes runtime launch orchestration for the robot hardware stack
while keeping the driver internals and package-level standalone launches intact.

Integrated bringup uses [config/robot.yaml](config/robot.yaml) as the shared runtime
parameter file for both `robot_lidar_driver` and `robot_base_driver`, and can also
launch `robot_description` for TF and URDF publication.

## Launch Files

- `launch/sensor.launch.py`: integrated LDS LiDAR bringup
- `launch/motor.launch.py`: integrated OpenCR motor/base bringup
- `launch/bms.launch.py`: optional integrated BMS bringup
- `launch/robot.launch.py`: top-level orchestrator for description, sensor, motor, and optional BMS launch files

The existing launch files under `robot_lidar_driver/launch` and
`robot_base_driver/launch` are still available as standalone launches or deprecated
examples, but the integrated entrypoint is `robot_bringup`.

## Run

```bash
ros2 launch robot_bringup sensor.launch.py
ros2 launch robot_bringup motor.launch.py
ros2 launch robot_bringup bms.launch.py enabled:=true
ros2 launch robot_bringup robot.launch.py
```

Override the shared parameter file:

```bash
ros2 launch robot_bringup robot.launch.py params_file:=/absolute/path/to/robot.yaml
ros2 launch robot_bringup sensor.launch.py params_file:=/absolute/path/to/robot.yaml
ros2 launch robot_bringup motor.launch.py params_file:=/absolute/path/to/robot.yaml
```

Selectively enable subsystems from the integrated launcher:

```bash
ros2 launch robot_bringup robot.launch.py use_sensor:=true use_motor:=false
ros2 launch robot_bringup robot.launch.py use_sensor:=false use_motor:=true
ros2 launch robot_bringup robot.launch.py use_sensor:=true use_motor:=true use_description:=true
ros2 launch robot_bringup robot.launch.py use_sensor:=false use_motor:=true use_description:=true
ros2 launch robot_bringup robot.launch.py use_bms:=true
```

`use_bms` defaults to `false`. When it is false, no BMS node is launched and existing OpenCR, LiDAR, odom, scan, IMU, joint state, and TF behavior is unchanged.

## Parameters

The default parameter file is [config/robot.yaml](config/robot.yaml).

- `robot_lidar_driver` parameters cover LiDAR serial port, baudrate, frame, topic,
  scan limits, fixed LaserScan geometry, reconnect behavior, and mock mode.
- `robot_base_driver` parameters cover OpenCR serial port, baudrate, topics, frame
  IDs, TF publishing, odometry/IMU/joint state publishing, odom scale calibration,
  rotation diagnostics, and polling behavior.
- `robot_bms_driver` parameters cover optional BMS serial port, baudrate,
  `/battery_state` frame/topic, polling, timeout, parser protocol, and diagnostics.

BMS defaults are conservative: `bms.enabled: false` and `bms.protocol: "placeholder"`.
The placeholder parser does not publish fake battery values. `/battery_state` is
published only when a concrete BMS parser produces a valid `BatteryState` sample.

When needed, `use_sim_time` can be overridden from the launch command line and is
forwarded consistently to both driver nodes.

`robot_bringup/config/robot.yaml` is the integrated runtime source of truth for
hardware bringup. The standalone driver configs remain available, but integrated
launches should follow the shared bringup YAML first.

## Timing Guidance

`robot_base_driver.poll_interval_ms` controls OpenCR feedback polling and the update
cadence of `/odom`, `/joint_states`, and `odom -> base_footprint` TF.

- `300 ms` is too slow for Nav2 controller feedback.
- `50 ms` is the current recommended default for hardware bringup.
- Further tuning can often stay in the `20~50 ms` range depending on serial stability.
- Expected `/odom`, `/joint_states`, and `/tf` update rate is `15~20 Hz` or higher.

For TurtleBot3 bringup compatibility, `robot_bringup/config/robot.yaml` defaults
to `robot_base_driver.poll_mode: "full"` so `/odom`, `/joint_states`, and `/imu`
are all published from OpenCR feedback. The driver reads wheel and IMU register
groups in bulk to keep the default `50 ms` polling cadence practical.

`heartbeat_interval_ms` remains separate and does not need to match the polling
period.

Useful runtime checks:

```bash
ros2 topic hz /odom
ros2 topic hz /joint_states
ros2 topic hz /tf
```

For TurtleBot3 compatibility comparisons, capture a baseline with TurtleBot3 bringup and compare it with robot_hw:

```bash
../scripts/compare_tb3_compatibility.sh > ~/ws/logs/robot_hw_compat.txt
```

## Runtime Interfaces

With the integrated bringup enabled, the expected topics and TF interfaces are:

- `/scan`
- `/cmd_vel`
- `/odom`
- `/imu`
- `/joint_states`
- optional `/battery_state`
- `/tf`
- `/tf_static`
- `robot_description` parameter from `robot_state_publisher`
