# Rotation Drift Diagnosis

목표 버전: `0.2.0`

증상은 제자리 회전 이후 AMViz 기준 `map -> odom`이 약간 `-x` 방향으로 틀어지는 현상이다. 이 문서는 현재 `ros-robot-hardware` 코드 관점에서 가능한 원인 후보를 확정 원인과 실차 검증 필요 원인으로 분리한다.

## 확정 가능한 코드 사실

- odom yaw는 기본적으로 wheel odom만 사용한다. `use_imu_for_yaw=false`, `odom.yaw_source=wheel_odom`이 기본이다.
- `odom -> base_footprint` TF와 `/odom`은 같은 `OdometryIntegrator` pose에서 나온다.
- `/imu`가 발행되어도 기본 주행 odom yaw 보정에는 자동 반영되지 않는다.
- OpenCR cmd write, heartbeat write, state read는 같은 worker loop와 transaction path를 공유한다.
- full poll은 wheel state 외에 optional device status, motor torque, battery, IMU read를 수행한다.
- LiDAR scan cardinal indexes는 TB3 profile 기준 front `0`, left `100`, right `300`, rear `200`로 계산된다.

## 원인 후보 점검

| 후보 | 코드 관점 확인 결과 | 판단 |
| --- | --- | --- |
| wheel odom yaw와 imu yaw 사이 고정 offset 또는 누적 drift | `odom_imu_yaw_delta_rad`가 로그로 계산된다. 관측 요약상 `0.42~0.48rad` 지속 offset이 있음 | 추가 실차 검증 필요 |
| wheel odom만 yaw source로 쓰는 한계 | 기본 구조가 wheel yaw 적분이다. IMU yaw는 기본 보정에 미사용 | 구조적 한계 가능 |
| actual odom rate 13~15 Hz | poll timing 로그에서 확인 가능. 목표 20 Hz 대비 낮음 | 원인 후보 |
| cmd_vel write와 read polling contention | 같은 worker loop 및 serial transaction path 사용 | 원인 후보 |
| LaserScan front/left/right mapping 미확정 | 코드상 index는 계산되나 실제 장착/좌우 방향은 장애물 없이 확정 불가 | 실차 검증 필요 |
| base_scan static transform yaw/position 오차 | URDF `scan_yaw_offset` 기본 0, xyz `-0.032 0 0.172` | 실측 필요 |
| odom/tf/message stamp 불일치 | `/odom`과 TF는 같은 stamp 사용. `/imu`, `/joint_states`도 같은 state callback stamp 사용 | 큰 불일치 가능성 낮음 |
| odom/imu covariance 신뢰도 문제 | odom zero covariance TB3 호환 기본, IMU covariance는 non-zero 기본 | localization 입력 설정과 함께 검증 필요 |
| 저속/전환 sign mismatch false positive | 기존 로직은 낮은 angular 구간에서 mismatch warning 가능. 0.2.0에서 threshold 이하 skip 처리 추가 | 개선됨 |

## 확정 원인

현재 코드만으로 `map -> odom`의 `-x` drift 단일 확정 원인은 특정할 수 없다. 다만 기본 odom yaw가 wheel-only이고, IMU yaw offset이 관측되며, poll rate가 목표보다 낮다는 사실은 확인된다.

## 추가 실차 검증 필요 원인

- 제자리 회전 중 `/odom.twist.twist.angular.z`와 `/imu.angular_velocity.z` ratio 및 sign.
- 회전 전/후 `odom_imu_yaw_delta_rad`가 고정 offset인지 누적 drift인지.
- LiDAR front/left/right/rear mapping이 물리 방향과 일치하는지.
- `base_scan`의 실제 장착 yaw와 URDF `scan_yaw_offset` 차이.
- full poll 대비 `poll_mode=odom` 또는 status read 축소 시 actual odom rate가 개선되는지.

## 0.2.0 관측 방법

저장 로그에서 다음을 확인한다.

```bash
python3 scripts/extract_robot_hw_quality.py /path/to/ros.log
```

중점 필드:

- `event=rotation_consistency`: `sign_check_enabled`, `sign_check_reason`, `odom_imu_yaw_delta_rad`, `odom_cmd_ratio`.
- `event=imu_compatibility`: `observed_yaw_offset_rad`, `yaw_offset_warn_rad`.
- `event=poll_timing`: `actual_odom_rate_hz`, `avg_total_ms`, `avg_required_state_read_ms`, `avg_optional_imu_read_ms`.
- `event=scan_geometry_stability`: `mapping_validation_state`.
