# robot_bringup

`robot_bringup` centralizes runtime launch orchestration for the robot hardware stack
while keeping the driver internals and package-level standalone launches intact.

Integrated bringup uses [config/robot.yaml](config/robot.yaml) as the shared runtime
parameter file for both `robot_lidar_driver` and `robot_base_driver`.

## Launch Files

- `launch/sensor.launch.py`: integrated LDS LiDAR bringup
- `launch/motor.launch.py`: integrated OpenCR motor/base bringup
- `launch/robot.launch.py`: top-level orchestrator for sensor and motor launch files

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
```

## Parameters

The default parameter file is [config/robot.yaml](config/robot.yaml).

- `robot_lidar_driver` parameters cover LiDAR serial port, baudrate, frame, topic,
  scan limits, reconnect behavior, and mock mode.
- `robot_base_driver` parameters cover OpenCR serial port, baudrate, topics, frame
  IDs, TF publishing, odometry/IMU/joint state publishing, and polling behavior.

When needed, `use_sim_time` can be overridden from the launch command line and is
forwarded consistently to both driver nodes.
