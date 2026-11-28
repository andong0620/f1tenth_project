# LiDAR 확인과 base_link 기준 장착 값

배정 업무: "라이다 나오는 것 확인, base link 기준으로 차량 파라미터 수정 (라이다 중심점 대비 차량 스펙 반영)"

## 1. 좌표 약속 (팀 `mount.yaml`)

- **base_link = 후륜축 중심을 지면에 내린 점.** x 앞, y 왼쪽, z 위 (REP-103).
- 센서 위치 = `plate`(실측) + `*_on_plate`(도면). launch가 static TF `base_link → laser` 를 만든다.
- `/scan_filtered` 의 frame_id 는 `laser`. 각도 0 = 정면, 반시계 +.

```
          y (왼쪽)
          ▲
  ┌───────┼───────────────┐
  │  ●────┼──────── ◎ ────┼──► x (앞)       ● base_link (후륜축 중심, 지면)
  │       │   0.27 m  TEA │                  ◎ LiDAR (TEA 중심, z ≈ 0.154 m)
  └───────┼───────────────┘
        후륜축            앞범퍼 (LiDAR 에서 약 0.1 m, 실측)
```

## 2. 지금 값

| 값 | 팀 레포 위치 | 지금 | 상태 |
|---|---|---|---|
| 후륜축 → 판 원점 `plate.x` | `mount.yaml` | 0.20 | ❗ 임시값. **실측** |
| 판 원점 → TEA 중심 `laser_on_plate.x` | `mount.yaml` | 0.070 | 도면 |
| **후륜축 → LiDAR x** | (계산) | **0.27** | plate.x 실측 후 확정 |
| 지면 → 판 아랫면 `plate.z` | `mount.yaml` | 0.094 | 11/26 실측 반영 |
| 판 → 레이저 평면 `laser_on_plate.z` | `mount.yaml` | 0.0596 | 좌석 3.2 mm + 데이터시트 56.4 mm |
| **레이저 높이** | (계산) | **≈ 0.154** | 파이프 높이보다 낮아야 파이프가 보인다 |
| 축간 거리 `wheelbase` | `vesc.yaml`, `lane_follow` | 0.25 | ❗ 임시값 (RoboRacer 표준 ≈ 0.33). **실측** |
| LiDAR → 앞범퍼 `ttc_margin` | `safe.yaml` | 0.1 | 범퍼까지 실측 |

## 3. LiDAR 위치가 들어가는 곳

| 어디 | 왜 |
|---|---|
| `mount.yaml` `plate.x` | TF. RViz에서 스캔이 차 모양과 맞는 자리에 찍힌다 |
| `mode_manager` `ttc_margin` | AEB 거리 = LiDAR 거리 − 범퍼까지 거리 |
| `mode_manager` `alongside_x_behind` | 차 뒤끝은 LiDAR 보다 (0.27 + 후륜축→뒤범퍼) 뒤에 있다. 박스 옆을 지날 때 이 범위까지 "옆에 있음"으로 본다 |
| gap `half_width` | LiDAR가 앞에 있으면 코너에서 앞이 먼저 꺾여 뒤가 안쪽 모서리를 긁는다 → 0.35 (README 지연 실험) |
| gap `lidar_x` (pursuit 모드) | 목표점을 base_link 로 옮겨 조향 계산 |
| 시뮬 `--lidar-x`, f1sim_ms `scan_distance_to_base_link` | 시뮬과 실차를 같게 |

## 4. 실차 확인 순서

```bash
export ROS_DOMAIN_ID=2
source ~/2026AUTOLAB_team2/install/setup.bash
ros2 topic hz /scan                  # TEA 약 20 Hz (설정에 따라 다름)
ros2 topic hz /scan_filtered         # 같은 주기로 나와야 scan_filter 가 산다
ros2 run tf2_ros tf2_echo base_link laser   # x ≈ 0.27, z ≈ 0.154 가 나와야 한다
python3 scripts/scan_report.py bags/pipe_1  # 주파수, 유효 각도, 잡음
```

1. **방향**: 차 **왼쪽 앞 45°, 1 m** 에 박스를 둔다. RViz(Fixed Frame `base_link`)에서 점이 +x, +y 쪽에 찍혀야 한다. 반대면 `scan_filter.flip`, 90° 돌아가 있으면 `yaw_offset_deg` 를 고친다.
2. **거리**: 정면 1.00 m 에 박스 → `/scan_filtered` 정면 빔이 약 1.00 m (LiDAR 기준). RViz에서는 base_link 기준 약 1.27 m 에 찍힌다.
3. **시야**: ±150° 안에 차체(카메라 타워, 케이블)가 찍히지 않는지 본다.
4. **줄자 실측**: 후륜축 중심 → TEA 중심(x), TEA 중심 → 앞범퍼, 후륜축 → 앞바퀴축(= wheelbase), 후륜축 → 뒤범퍼.
5. 값을 팀 레포 `mount.yaml`, `vesc.yaml`, `safe.yaml` 과 이 레포 `config/pursuit.yaml` 에 넣고, 시뮬은 `--lidar-x <실측값>` 으로 다시 돌린다.
