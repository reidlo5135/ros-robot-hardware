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
```

---

## Version Roadmap

### v0.1.11. `robot_diagnostics` Package

Status: TODO

#### Scope

진단 도구와 호환성 점검 로직을 전용 패키지로 분리할 기반을 만든다.

#### Tasks

- [ ] `robot_diagnostics` 패키지 신설
- [ ] 진단 스크립트/문서/런치 진입점 구조 정의
- [ ] 기존 `scripts/` 진단 도구의 이관 대상 목록 정리
- [ ] 패키지 README 작성

#### Output

- `robot_diagnostics`

---

### v0.1.12. TB3 Compatibility Scripts Migration

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

### v0.1.13. Robot Profile / Contract YAML

Status: TODO

#### Scope

하드웨어별 ROS 인터페이스 계약을 YAML profile로 명시한다.

#### Tasks

- [ ] robot profile / contract YAML 스키마 정의
- [ ] `/scan`, `/odom`, `/imu`, `/tf`, `/tf_static` 계약 필드 정리
- [ ] TB3/OpenCR/LDS-03 기본 profile 초안 작성
- [ ] bringup config와 diagnostics가 contract YAML을 참조하는 흐름 설계
- [ ] contract mismatch 진단 출력 형식 정의

#### Output

- robot profile / contract YAML

---

### v0.1.14. Custom Vehicle Template

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
