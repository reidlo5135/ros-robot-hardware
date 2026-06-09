# TB3 Bringup Compatibility Review

목표 버전: `0.2.0`

이 문서는 `ros-robot-hardware`의 자체 TurtleBot3/OpenCR 호환 bringup이 기존 `turtlebot3_bringup` 계열과 어떤 ROS 계약 차이를 가질 수 있는지 정리한다. 현재 작업공간에는 `turtlebot3_bringup` 소스가 없으므로, 외부 패키지 내부 구현은 로컬 직접 비교 불가 / 확인 필요로 둔다. 아래의 "현재 구현" 항목은 이 저장소 코드 기준으로 확인한 내용이다.

## 요약

- 자체 구현은 `/odom`, `/imu`, `/joint_states`, `/scan`, `odom -> base_footprint` TF를 모두 직접 발행한다.
- `map -> odom`은 발행하지 않으며 외부 localization 소유로 둔다.
- 기본 odom yaw source는 기존과 동일하게 wheel odom이다. `odom.yaw_source` skeleton은 추가되었지만 기본 동작을 바꾸지 않는다.
- LiDAR는 TB3 Coin D4 스타일 `0..2*pi`, 400 samples, front/left/rear/right index `0/100/200/300` 계약을 기본으로 둔다.
- OpenCR full poll은 wheel, optional status/torque/battery, IMU read를 같은 worker loop에서 처리한다. 목표 20 Hz 대비 실제 13~15 Hz 저하는 read transaction 수와 `transaction_gap_us=10000`의 영향을 받을 수 있다.

## 항목별 비교

| 항목 | 현재 구현 | 기존 TB3 bringup 대비 상태 |
| --- | --- | --- |
| OpenCR odom 산출 | `robot_base_driver`가 OpenCR present position/velocity를 읽고 `OdometryIntegrator`에서 wheel odom 적분 | 외부 소스 로컬 비교 불가 / 확인 필요 |
| IMU orientation/angular velocity | full poll에서 IMU quaternion, gyro, accel을 읽어 `/imu` 발행. yaw 비교 로그는 odom yaw와 IMU yaw delta를 계산 | TB3 IMU covariance, orientation 신뢰도는 실측 baseline 필요 |
| `/joint_states` | wheel_left/right joint name으로 wheel position/velocity 발행. stamp는 같은 OpenCR state callback stamp 사용 | joint 이름은 URDF와 일치 |
| `odom -> base_footprint` TF | base driver가 odom message와 같은 stamp로 동적 TF 발행 | duplicate TF publisher 여부는 실차에서 확인 필요 |
| static TF chain | URDF: `base_footprint -> base_link -> base_scan`, `base_link -> imu_link`, wheel links | TB3 Burger 계열 형상을 따르나 실제 LiDAR/IMU 장착 방향은 실측 필요 |
| LaserScan angle convention | `tb3_coin_d4`: `angle_min=0`, `angle_max=2*pi`, 400 samples, `mirror_scan_angles=true` | TB3 baseline과 sample count/order 비교 필요 |
| scan cardinal index | front `0`, left `100`, rear `200`, right `300` | 코드상 계산 가능. 실제 좌우 mapping은 장애물 테스트 필요 |
| timestamp 정책 | 같은 OpenCR state callback에서 `/imu`, `/joint_states`, `/odom`, TF가 같은 stamp를 사용 | read 완료 시각 기준에 가까움. 펌웨어 sample time은 없음 |
| covariance 정책 | `tb3_compatibility.odom_zero_covariance=true`가 기본. IMU covariance는 YAML 설정값 사용 | TB3 baseline covariance와 비교 필요 |
| publish/poll rate | `poll_interval_ms=50`, `target_odom_rate_hz=20`. poll timing 로그가 actual rate와 avg/max timing을 보고 | 실제 로그는 13~15 Hz로 관측됨 |
| cmd_vel/state contention | cmd write, heartbeat, state poll이 동일 OpenCR client worker와 transaction path를 공유 | 같은 serial transaction mutex/gap의 영향 가능성 있음 |

## 0.2.0에서 보강한 관측 필드

- `rotation_state`: 절대 angular 값, configured/active yaw source, IMU yaw offset 설정값.
- `rotation_consistency`: sign check threshold, skip 사유, yaw delta warning threshold, odom/cmd ratio threshold.
- `imu_compatibility`: configured yaw offset, observed yaw offset, monitor threshold.
- `base_state`, `poll_timing`: state read/cycle duration과 stamp age 계열 필드.
- `scan_geometry_stability`: `geometry_validation_mode`, `mapping_validation_state`.

## 추가 확인 필요

- TB3 공식 bringup의 현재 OpenCR register read grouping, IMU covariance, LaserScan order는 로컬 소스가 없어 직접 비교하지 못했다.
- `/scan` 좌우 mapping은 코드만으로 확정하지 않는다. `docs/LIDAR_SCAN_GEOMETRY_VALIDATION.md` 절차로 실차 검증해야 한다.
- IMU yaw orientation을 localization에서 절대 heading으로 쓰는지, gyro만 쓰는지는 `ros-amr-navigation` 설정 확인이 필요하나 이 저장소 수정 대상은 아니다.
