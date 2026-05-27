# robot_bringup

`robot_bringup` centralizes runtime launch orchestration for the robot hardware stack
while keeping the driver internals and package-level standalone launches intact.

Integrated bringup uses [config/robot.yaml](config/robot.yaml) as the shared runtime
parameter file for both `robot_lidar_driver` and `robot_base_driver`, and can also
launch `robot_description` for TF and URDF publication.

## Launch Files

- `launch/sensor.launch.py`: integrated LDS LiDAR bringup
- `launch/motor.launch.py`: integrated OpenCR motor/base bringup
- `launch/robot.launch.py`: top-level orchestrator for description, sensor, and motor launch files

The existing launch files under `robot_lidar_driver/launch` and
`robot_base_driver/launch` are still available as standalone launches or deprecated
examples, but the integrated entrypoint is `robot_bringup`.

## Run

```bash
ros2 launch robot_bringup sensor.launch.py
ros2 launch robot_bringup motor.launch.py
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
```

## Parameters

The default parameter file is [config/robot.yaml](config/robot.yaml).

- `robot_lidar_driver` parameters cover LiDAR serial port, baudrate, frame, topic,
  scan limits, reconnect behavior, and mock mode.
- `robot_base_driver` parameters cover OpenCR serial port, baudrate, topics, frame
  IDs, TF publishing, odometry/IMU/joint state publishing, and polling behavior.

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

For Nav2 bringup, `robot_bringup/config/robot.yaml` now defaults to
`robot_base_driver.poll_mode: "odom"`, which reads only wheel velocity/position
feedback needed for `/odom` and `/joint_states`. Use `poll_mode: "full"` for
OpenCR IMU diagnostics or experiments that need `/imu`.

`heartbeat_interval_ms` remains separate and does not need to match the polling
period.

Useful runtime checks:

```bash
ros2 topic hz /odom
ros2 topic hz /joint_states
ros2 topic hz /tf
```

## Runtime Interfaces

With the integrated bringup enabled, the expected topics and TF interfaces are:

- `/scan`
- `/cmd_vel`
- `/odom`
- `/imu`
- `/joint_states`
- `/tf`
- `/tf_static`
- `robot_description` parameter from `robot_state_publisher`
