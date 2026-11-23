# RoboRacer LiDAR Planning — Gap Follow · Wall/Obstacle Clustering · Wall Follow

**2026 Automotive Engineering Lab, Team 2** project:
a 1/10-scale autonomous race car (RoboRacer / F1TENTH-style) that drives a closed-loop track **without a map**, reacting to each sensor frame.

- **Follow the Gap** with a disparity extender, for sections with obstacles
- **Wall / obstacle classification** by clustering each scan, used to switch modes
- **Wall following** (centre of two walls, PID), for the pipe-shaped sections

> The full team stack (ROS 2 Humble nodes, camera lane following, motor control, mode manager) lives in a private team repository.
> This repository contains only the LiDAR planning part assigned to me, as plain Python with no ROS dependency.

## Results

![Follow-the-Gap demo](docs/gap_follow_demo.png)

**Follow the Gap.** The scan is cleaned, the closest obstacle is wiped with a safety bubble, and every edge is widened by the car's half width so that gaps the car cannot fit through disappear. The car aims at the deepest point near the centre of the largest remaining gap. In the middle panel, the 0.25 m slit between the box and the wall is rejected and the car passes on the open side.

![Clustering demo](docs/clustering_demo.png)

**Wall vs. obstacle.** Each scan is split into clusters of consecutive points with an adaptive breakpoint threshold. A cluster is an obstacle only if it is **short** and lies **inside the forward corridor**. The length check is what stops the outer wall of a corner from being mistaken for an obstacle (right panel).

**Tests.** 69 contract tests for my functions pass, 0 fail (`python -m pytest`).
The remaining 31 tests are marked `xfail` because they also call two speed-selection functions that belong to a teammate (hardware/control) and are not part of this repository.

## Pipeline

```
/scan ─ normalize ─┬─ cluster_scan ─ find_obstacles ──────────────► mode switch (WALL ↔ GAP)
                   ├─ wall_distance ×2 ─ center_error ─ PID ───────► steering (wall follow)
                   └─ preprocess ─ safety_bubble ─ disparity_extend
                        ─ find_max_gap ─ best_point ───────────────► steering (gap follow)
```

Convention: REP-103 (x forward, y left, angle 0 straight ahead, counter-clockwise positive).

## What I implemented

| File | Function | Idea |
|---|---|---|
| `src/control/control/core/gap.py` | `preprocess` | Interpolate invalid beams (NaN/inf/≤0) from neighbours instead of treating them as obstacles, clip to max range, edge-padded moving average |
| | `safety_bubble` | Zero every beam within ±asin(R / r) of the closest point (±90° if r ≤ R) |
| | `disparity_extend` | At each range jump, copy the near distance d over the far side for atan(half_width / d); edges detected on the input, written to a copy |
| | `find_max_gap` | Longest run of free beams (ties keep the first) |
| | `best_point` | Among beams ≥ 0.9 × deepest, pick the one closest to the gap centre |
| `src/control/control/core/wall.py` | `wall_distance` | F1TENTH Lab 3 geometry: α = atan2(a·cosθ − b, a·sinθ), D = b·cosα + L·sinα; median over a small angular window to survive dropouts; reject far beams before the trigonometry (corner case) |
| | `center_error` | Error to the corridor centre, continuous when one wall disappears |
| | `PID` | Discrete PID with integral anti-windup and safe handling of dt ≤ 0 |
| `src/perception/perception/core/scan_utils.py` | `cluster_scan` | Adaptive breakpoint: split when the gap between consecutive valid points exceeds `break_dist + break_ratio × r` |
| | `find_obstacles` | Short cluster (≤ max length) with a point inside the forward corridor |

## Run it

```bash
pip install -r requirements.txt
python -m pytest          # contract tests
python demo.py            # regenerates the figures in docs/
```

## Status and next steps

- [x] Gap follow — unit tests pass
- [x] Wall / obstacle clustering — unit tests pass
- [x] Wall follow — unit tests pass
- [ ] Gap follow validated in the f1tenth_gym simulator (3 collision-free laps on the box map)
- [ ] Mode switching WALL ↔ GAP in simulation, then on the real car (pipe section)
- [ ] Final track demo (Dec 17, 2026)

## Credits

- Project framework, interfaces, function specifications (docstrings) and contract tests: team lead, 2026 Automotive Engineering Lab Team 2.
- `normalize_scan`, `crop_fov`, `front_distance`, `follow_gap`, `follow_walls` and the synthetic scan generator `synth.py` were provided by the team lead.
- Implementation of the functions listed in **What I implemented**: Donghyun An ([@andong0620](https://github.com/andong0620)).
- Algorithms follow the F1TENTH course labs ([Lab 3 Wall Following](https://github.com/f1tenth/f1tenth_lab3_template), [Lab 4 Follow the Gap](https://github.com/f1tenth/f1tenth_lab4_template)).

---

### 한국어 요약

한양대학교 미래자동차공학과 2026 자동차공학실험 2조의 RoboRacer 자율주행 프로젝트에서 **라이다 플래닝**을 맡아 구현한 부분입니다.
맵 없이 매 스캔만 보고 판단합니다. 장애물 구간은 gap follow로 피하고, 벽과 장애물은 클러스터링으로 구분하며, 파이프 구간은 양쪽 벽 중앙을 PID로 추종합니다.
전체 팀 코드(ROS 2 노드, 카메라, 모터 제어)는 비공개 팀 저장소에 있고, 이 저장소에는 제 담당 부분만 담았습니다.
