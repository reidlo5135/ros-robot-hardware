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

Status: IMPLEMENTED - FIELD VALIDATION

### Scope

OpenCR 기반 구동부 브릿지를 자체 구현한다.

### Tasks

- [x] `robot_base_driver` 패키지 생성
- [x] OpenCR UART serial connection 구현
- [x] `/cmd_vel` subscriber 구현
- [x] OpenCR velocity command 송신
- [x] wheel encoder feedback 수신
- [x] `/odom` publisher 구현
- [x] OpenCR IMU feedback 수신
- [x] `/imu` publisher 구현
- [x] wheel joint feedback 수신
- [x] `/joint_states` publisher 구현
- [x] serial/device/frame/topic parameter YAML 분리
- [x] launch file 작성
- [x] README 작성

현재 상태: Phase 2 기능은 구현되어 있으며 TurtleBot3/OpenCR 호환 하드웨어에서 field validation 중이다.

### Output

- `/cmd_vel`
- `/odom`
- `/imu`
- `/joint_states`

---

## Phase 3. TF Publisher

Status: IMPLEMENTED - FIELD VALIDATION

### Scope

TurtleBot3 Burger 기준 TF tree를 자체 패키지에서 발행한다.

### Tasks

- [x] `robot_description` 패키지 생성
- [x] Burger 기준 frame 이름 정리
- [x] static TF 작성
- [x] odom dynamic TF 정책 정리
- [x] URDF/Xacro 작성 여부 결정
- [x] `robot_state_publisher` 연동 여부 결정

현재 상태: Phase 3 TF/description 기능은 구현되어 있으며 static frame chain과 odom dynamic TF를 field validation 중이다.

### Target TF Tree

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── base_scan
            └── imu_link
```

---

## Version Roadmap

### v0.1.11. `robot_diagnostics` MVP Package

Status: IMPLEMENTED - FIELD VALIDATION

#### Scope

진단 도구와 호환성 점검 로직을 전용 패키지로 분리하고, TB3 Burger/OpenCR/LDS-03 기준 robot_hardware ROS contract를 one-shot/live로 검증하는 MVP를 제공한다.

#### Tasks

- [x] `robot_diagnostics` 패키지 신설
- [x] `config/tb3_contract.yaml` 추가
- [x] `robot_diagnostics_node` 구현
- [x] TF/topic/odom/scan/IMU/joint_states contract check 구현
- [x] `diagnostics.launch.py` 추가
- [x] `scripts/robot_diag_snapshot.sh` 추가
- [x] 패키지 README 작성
- [x] 기존 `scripts/` 진단 도구의 이관 대상은 v0.1.12에서 계속 정리

#### Output

- `robot_diagnostics`
- `robot_diagnostics_node`
- `tb3_contract.yaml`
- `robot_diag_snapshot.sh`

---

### v0.1.12. Motion Diagnostics

Status: IMPLEMENTED - FIELD VALIDATION

#### Scope

정적 ROS contract 검증을 넘어서 실제 `/cmd_vel` 주행 명령 중 `/odom`, `/imu`, TF, `/scan`이 의미론적으로 맞는지 요약하는 motion diagnostics workflow를 제공한다.

#### Tasks

- [x] `scripts/test_motion_diagnostics.sh` 추가
- [x] rotate / linear / square mode 지원
- [x] optional `/cmd_vel` publish 및 observe-only mode 지원
- [x] optional rosbag record 지원
- [x] motion summary 및 `ROBOT_HW_LOG event=motion_summary` 출력
- [x] 기존 `test_rotation_diagnostics.sh`는 legacy/specialized rotation workflow로 유지
- [x] `docs/debugging/MOTION_DIAGNOSTICS.md` 추가
- [x] root README, scripts README, robot_diagnostics README 갱신

#### Output

- `test_motion_diagnostics.sh`
- `MOTION_DIAGNOSTICS.md`

---

### v0.1.13. TurtleBot3 OpenCR Battery State Parity

Status: IMPLEMENTED - FIELD VALIDATION

#### Scope

TurtleBot3 ROS 2 원본 `turtlebot3_node`의 OpenCR battery state 발행 방식과
동등하게 `/battery_state` publish 경로를 정리한다. 기본 경로는 TB3 OpenCR
control table `battery_voltage`/`battery_percentage` 항목을 사용하고, custom
register path는 non-TB3 차량 확장용 fallback으로 유지한다.

#### Tasks

- [x] TurtleBot3 원본 battery_state 발행 경로 조사
- [x] TB3 OpenCR battery protocol/read path 반영
- [x] /battery_state voltage publish path 정리
- [x] unavailable fallback 유지
- [x] robot_diagnostics optional battery check 유지
- [ ] 실차에서 /battery_state.voltage 확인
- [ ] 실제 voltage meter와 `/battery_state.voltage` 비교
- [ ] 멀티미터 전압과 비교해 scaling 검증

#### Output

- TurtleBot3 OpenCR battery state parity path
- optional `/battery_state` diagnostics contract check

---

### v0.1.14. TB3 Compatibility Scripts Migration

Status: TODO

#### Scope

TB3 compatibility 관련 스크립트를 `robot_diagnostics`로 이관한다.

#### Tasks

- [ ] `compare_tb3_compatibility.sh`를 `robot_diagnostics`로 이관
- [ ] TF/topic/rotation compatibility 보조 스크립트 이관 범위 결정
- [ ] 기존 경로 호환 wrapper 또는 migration 안내 제공
- [ ] `docs/debugging/TB3_COMPATIBILITY.md`와 `scripts/README.md` 경로 갱신

#### Output

- `robot_diagnostics` 기반 TB3 compatibility diagnostics

---

### v0.1.15. Robot Profile / Contract YAML Generalization

Status: TODO

#### Scope

v0.1.11의 TB3 contract YAML MVP를 확장해 하드웨어별 ROS 인터페이스 계약을 profile로 일반화한다.

#### Tasks

- [ ] robot profile / contract YAML 스키마 정의
- [ ] `/scan`, `/odom`, `/imu`, `/tf`, `/tf_static` 계약 필드 정리
- [ ] TB3/OpenCR/LDS-03 기본 profile 초안 작성
- [ ] bringup config와 diagnostics가 contract YAML을 참조하는 흐름 설계
- [ ] contract mismatch 진단 출력 형식 정의

#### Output

- robot profile / contract YAML

---

### v0.1.16. Custom Vehicle Template

Status: TODO

#### Scope

TB3가 아닌 차량에 맞춰 profile, description, bringup 설정을 시작할 수 있는 template을 추가한다.

#### Tasks

- [ ] custom vehicle template 디렉터리 구조 정의
- [ ] frame/topic/sensor/base parameter template 작성
- [ ] custom URDF/xacro 시작점 제공
- [ ] custom profile 작성 가이드 추가

#### Output

- custom vehicle template

---

### v0.2.0. Reference Profile Reorganization

Status: TODO

#### Scope

TB3/OpenCR/LDS-03를 기본 전제가 아닌 하나의 reference profile로 격하한다.

#### Tasks

- [ ] TB3/OpenCR/LDS-03 assumptions를 reference profile로 이동
- [ ] 기본 bringup이 explicit profile 선택을 요구하도록 정책 정리
- [ ] generic robot contract와 reference profile의 경계 정리
- [ ] 문서에서 TB3 중심 표현을 reference example 표현으로 전환

#### Output

- TB3/OpenCR/LDS-03 reference profile
- profile 기반 generic hardware bringup 구조
