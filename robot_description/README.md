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
- `wheel_left_link`
- `wheel_right_link`
- `caster_back_link`

Relationship overview:

```text
base_footprint
  └── base_link
      ├── base_scan
      ├── imu_link
      ├── wheel_left_link
      ├── wheel_right_link
      └── caster_back_link
```

The wheel joint names match the base driver defaults exactly:

- `wheel_left_joint`
- `wheel_right_joint`

For TurtleBot3 Burger compatibility, the fixed sensor transforms follow the
standard Humble description layout:

- `base_footprint -> base_link`: `xyz = (0.0, 0.0, 0.010)`
- `base_link -> base_scan`: `xyz = (-0.032, 0.0, 0.172)`
- `base_link -> imu_link`: `xyz = (-0.032, 0.0, 0.068)`

When `namespace` is set through `robot_bringup`, link and joint names receive
the same `robot1/`-style prefix used by TurtleBot3 Humble. This keeps
`robot_state_publisher` aligned with the frame and joint names emitted by
`robot_base_driver`.

## Run

```bash
ros2 launch robot_description description.launch.py
```

Override the model path if needed:

```bash
ros2 launch robot_description description.launch.py model:=/absolute/path/to/robot.urdf.xacro
```
