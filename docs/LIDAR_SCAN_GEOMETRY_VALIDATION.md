# LiDAR Scan Geometry Validation

목표 버전: `0.2.0`

`robot_lidar_driver`는 TB3 Coin D4 호환 profile에서 `/scan`을 `base_scan` frame, 400 samples, `0..2*pi` angle convention으로 발행한다. 코드상 cardinal index는 계산할 수 있지만, 실제 센서 장착 방향과 좌우 mapping은 현장 장애물 테스트 없이 확정하지 않는다.

## 기본 설정

```yaml
scan_geometry_profile: "tb3_coin_d4"
scan_angle_offset: 0.0
scan_direction_reversed: false
reverse_scan: false
mirror_scan_angles: true
fixed_scan_geometry: true
fixed_scan_samples: 400
fixed_angle_min: 0.0
fixed_angle_max: 6.283185307179586
frame_id: "base_scan"
lidar:
  geometry_validation_mode: "manual"
```

기대 index:

- front: `0`
- left: `100`
- rear: `200`
- right: `300`

## 수동 검증 절차

1. 로봇 정면 약 0.5~1.0 m 위치에 장애물을 둔다.
2. `/scan`에서 front index 또는 `front_range_m`이 가장 크게 감소하는지 확인한다.
3. 장애물을 로봇 왼쪽에 둔다.
4. left index 또는 `left_range_m`이 감소하는지 확인한다.
5. 장애물을 로봇 오른쪽에 둔다.
6. right index 또는 `right_range_m`이 감소하는지 확인한다.
7. RViz/AMViz에서 scan cloud가 로봇의 실제 전방/좌/우 방향과 일치하는지 확인한다.

## 로그 해석

`event=scan_geometry_stability`에서 확인한다.

- `geometry_validation_mode=manual`: 자동 확정하지 않았으며 현장 확인이 필요하다.
- `left_right_mapping_ok=manual_required`: 장애물 테스트 전 상태.
- `mapping_validation_state=unverified`: 아직 좌우 mapping 미확정.
- `result=warn`: fixed geometry에서 sample count 또는 angle increment가 흔들렸다.

수동 검증이 끝난 뒤 운영 기록상 TB3 mapping이 맞는 것으로 판단되면 `lidar.geometry_validation_mode: "assumed_tb3"`로 바꿀 수 있다. 이 설정은 데이터를 회전하거나 뒤집지 않고 로그 상태만 명확히 한다.

## 조정 기준

- 전방 장애물이 front가 아니라 일정 각도만큼 밀려 보이면 `scan_angle_offset` 후보를 검토한다.
- RViz에서 scan frame 자체가 실제 장착 yaw와 다르면 URDF/launch의 `scan_yaw_offset` 후보를 검토한다.
- 좌우가 뒤집혀 보이면 `mirror_scan_angles`, `scan_direction_reversed`, `reverse_scan`을 한 번에 여러 개 바꾸지 말고 한 항목씩 비교한다.
