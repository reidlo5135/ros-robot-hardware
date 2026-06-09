# Changelog

## 0.2.0

- Added TB3 bringup compatibility, rotation drift, LiDAR geometry, and TF/URDF validation docs.
- Added conservative yaw-offset monitoring parameters while keeping default odom behavior on wheel odom.
- Extended rotation diagnostics with absolute angular values, sign-check threshold/skip reasons, yaw-delta threshold, and odom/cmd ratio thresholds.
- Added OpenCR timing/stamp fields to make poll-rate degradation easier to diagnose from logs.
- Added LiDAR geometry validation state logging without auto-claiming left/right mapping.
- Added `scripts/extract_robot_hw_quality.py` for offline `ROBOT_HW_LOG` summaries.

Build, ROS launch, and hardware execution were intentionally not part of this validation step.
