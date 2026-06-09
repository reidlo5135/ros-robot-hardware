# TF and URDF Validation

목표 버전: `0.2.0`

이 문서는 `robot_description`의 frame 구성과 base driver/lidar driver frame 계약을 점검한다.

## 현재 TF 소유권

- `map -> odom`: 외부 localization 소유. robot hardware가 발행하지 않는다.
- `odom -> base_footprint`: `robot_base_driver` 동적 TF.
- `base_footprint -> base_link`: `robot_state_publisher` static TF.
- `base_link -> base_scan`: `robot_state_publisher` static TF.
- `base_link -> imu_link`: `robot_state_publisher` static TF.
- wheel links: `/joint_states`와 URDF wheel joints로 robot_state_publisher가 계산.

## URDF 점검 결과

| 항목 | 현재 값 | 판단 |
| --- | --- | --- |
| `base_footprint -> base_link` | xyz `0 0 0.010`, rpy `0 0 0` | TB3 계열 footprint/base 분리 |
| `base_link -> base_scan` | xyz `-0.032 0 0.172`, yaw `${scan_yaw_offset}` 기본 `0.0` | 실제 LiDAR 장착 위치/방향 실측 필요 |
| `base_link -> imu_link` | xyz `-0.032 0 0.068`, rpy `0 0 0` | 실제 IMU 축 방향 실측 필요 |
| wheel joints | left y `+0.08`, right y `-0.08`, axis `0 0 1` | joint name은 base driver 기본값과 일치 |
| `/joint_states` names | `wheel_left_joint`, `wheel_right_joint` | URDF와 일치 |

## 실측 필요 항목

- LiDAR 중심이 실제 `x=-0.032`, `z=0.172`에 가까운지.
- `base_scan` yaw 0에서 scan front index가 로봇 전방을 보는지.
- `imu_link` rpy 0이 OpenCR IMU quaternion frame과 일치하는지.
- 좌/우 wheel joint 이름과 encoder 좌/우가 실제로 뒤바뀌지 않았는지.

## 검증 명령

빌드 없이 실행 가능한 정적/로그 중심 확인:

```bash
./scripts/check_robot_hw_tf_publishers.sh
./scripts/check_tf_chain.sh
```

실차 실행 시 확인할 로그:

- `event=tf_chain_expected`
- `event=tf_publish`
- `event=frame_config`
- `event=joint_state_publish`
