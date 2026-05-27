# TB3 Scan Compatibility

## Goal

Compare this hardware stack's `/scan` behavior against standard
`turtlebot3_bringup robot.launch.py` behavior before blaming localization.

## Quick Comparison

1. Launch standard TurtleBot3 bringup on a known-good robot and observe `/scan`.
2. Launch this hardware repo and observe `/scan` with the same RViz setup.
3. Compare obstacle direction, not just whether scans are published.

## RViz Check

Use RViz with:

- `Fixed Frame = base_link`
- `LaserScan` display subscribed to `/scan`

Place a clear obstacle directly in front of the robot.

Expected:

- the obstacle cloud appears in front of `base_link`
- left-side obstacles appear on the left
- right-side obstacles appear on the right

Important:

- with `angle_min = -pi` and `angle_max = pi`, `ranges[0]` is not the front ray
- the front ray is near the middle of the array, around the index for `0 rad`

## Runtime Commands

```bash
ros2 launch robot_bringup sensor.launch.py log_level:=debug
ros2 topic echo /scan --once
ros2 topic hz /scan
rviz2
```

To inspect frame wiring:

```bash
ros2 run tf2_ros tf2_echo base_link base_scan
```

## Geometry Debug Parameters

These parameters are available on `robot_lidar_driver`:

```yaml
debug_scan_geometry: true
scan_angle_offset: 0.0
reverse_scan: false
```

When `debug_scan_geometry=true`, the node logs:

- `angle_min`
- `angle_max`
- `angle_increment`
- `ranges.size`
- index and range for front (`0 deg`)
- index and range for left (`+90 deg`)
- index and range for right (`-90 deg`)
- index and range for rear (`180 deg`)

## If Front Is Wrong

Try one change at a time:

1. `reverse_scan: true`
2. `scan_angle_offset: 3.141592653589793`
3. `scan_angle_offset: 1.5707963267948966`
4. `scan_angle_offset: -1.5707963267948966`

Interpretation:

- obstacle appears behind instead of in front:
  likely needs `scan_angle_offset` near `pi`
- left and right appear swapped:
  likely needs `reverse_scan: true`
- obstacle is rotated by a fixed angle:
  likely needs `scan_angle_offset`

## Compare Against TB3 Bringup

The closest compatibility target is:

- `/scan.header.frame_id = base_scan`
- `angle_min ~= -pi`
- `angle_max ~= +pi`
- positive `angle_increment`
- front obstacle aligned with `0 rad` in `base_link`

If the scan looks correct in RViz with `Fixed Frame=base_link`, but localization
still rotates `map -> odom`, the next suspect is timestamping or covariance
quality rather than pure scan direction.
