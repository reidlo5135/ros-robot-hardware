# TODO - ros-robot-hardware Roadmap

## Goal

TurtleBot3 Burger 기준 하드웨어 계층을 `turtlebot3_bringup` 의존 없이 자체 ROS 2 Humble 패키지로 내재화한다.

최종 목표는 다음 ROS 인터페이스를 자체 패키지에서 제공하는 것이다.

- `/scan`
- `/cmd_vel`
- `/odom`
- `/imu`
- `/joint_states`
- `/tf`
- `/tf_static`

---

## Phase 1. LiDAR Driver

Status: DONE

### Scope

- LDS LiDAR UART 통신
- LaserScan 변환
- `/scan` publish
- udev 기반 device path 사용
- launch/config 정리

### Output

- `robot_lidar_driver`
- `/scan`

---

## Phase 2. Base Driver

Status: TODO

### Scope

OpenCR 기반 구동부 브릿지를 자체 구현한다.

### Tasks

- [ ] `robot_base_driver` 패키지 생성
- [ ] OpenCR UART serial connection 구현
- [ ] `/cmd_vel` subscriber 구현
- [ ] OpenCR velocity command 송신
- [ ] wheel encoder feedback 수신
- [ ] `/odom` publisher 구현
- [ ] OpenCR IMU feedback 수신
- [ ] `/imu` publisher 구현
- [ ] wheel joint feedback 수신
- [ ] `/joint_states` publisher 구현
- [ ] serial/device/frame/topic parameter YAML 분리
- [ ] launch file 작성
- [ ] README 작성

### Output

- `/cmd_vel`
- `/odom`
- `/imu`
- `/joint_states`

---

## Phase 3. TF Publisher

Status: TODO

### Scope

TurtleBot3 Burger 기준 TF tree를 자체 패키지에서 발행한다.

### Tasks

- [ ] `robot_tf_publisher` 또는 `robot_description` 패키지 생성
- [ ] Burger 기준 frame 이름 정리
- [ ] static TF 작성
- [ ] odom dynamic TF 정책 정리
- [ ] URDF/Xacro 작성 여부 결정
- [ ] `robot_state_publisher` 연동 여부 결정

### Target TF Tree

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── base_scan
            └── imu_link