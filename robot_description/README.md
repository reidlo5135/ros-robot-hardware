# robot_description

`robot_description` provides a simple TurtleBot3 Burger-compatible URDF/Xacro and
`robot_state_publisher` launch for the hardware bringup stack.

The model is intentionally lightweight and uses primitive geometry so the TF and
joint structure stay easy to inspect during hardware integration.

## TF Tree

The default model publishes this core tree:

- `base_footprint`
- `base_link`
- `base_scan`
- `imu_link`
- `left_wheel_link`
- `right_wheel_link`
- `caster_back_link`

Relationship overview:

```text
base_footprint
  └── base_link
      ├── base_scan
      ├── imu_link
      ├── left_wheel_link
      ├── right_wheel_link
      └── caster_back_link
```

The wheel joint names match the base driver defaults exactly:

- `wheel_left_joint`
- `wheel_right_joint`

## Run

```bash
ros2 launch robot_description description.launch.py
```

Override the model path if needed:

```bash
ros2 launch robot_description description.launch.py model:=/absolute/path/to/robot.urdf.xacro
```
