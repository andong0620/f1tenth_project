# 플래너 조사: 방향을 어떤 기준으로 정할까

배정 업무: "판단(planner) 관련 알고리즘, 어떤 기준으로 방향을 정할건지, lookahead, pure pursuit 등 어떤 방법을 사용할건지 찾아보기"

## 1. 결론

| 구간 | 방향을 정하는 기준 | 조향으로 바꾸는 방법 | 담당 |
|---|---|---|---|
| 파이프 (장애물 없음) | 양쪽 벽의 가운데. 두 빔으로 lookahead L 앞의 벽 거리를 예측 | PID (`wall.py`) | 라이다 |
| 파이프 + 박스 | 가장 긴 빈틈 안에서 깊은 점 중 틈 가운데에 가까운 점 (`best_point`) | 지금: 목표점 각도 그대로 → **제안: pure pursuit** (`config/pursuit.yaml`) | 라이다 |
| 카메라 (노란 테이프) | 차선 waypoint 중 lookahead 거리의 점 | pure pursuit (`lane_follow`) | 카메라 구현, 컨트롤이 L 튜닝 |

- **맵을 쓰지 않는 반응형(reactive) 방식을 유지한다.** 박스 2개의 위치가 랜덤이고, 1/4 구간은 카메라 구간이라 LiDAR 맵만으로 한 바퀴를 다 덮을 수 없다.
- 레이싱 라인 최적화 + MPC는 랩타임이 가장 좋지만, 맵, 측위, 정확한 차량 모델이 필요하다. 우리 속도(1~2 m/s)와 일정에서는 이득보다 위험이 크다.
- **gap follow의 조향만 pure pursuit으로 바꾸는 것**은 비용이 거의 없다(함수 하나). 시뮬에서 흔들림이 절반으로 줄었고, 0.3 s 지연까지 모든 맵을 완주했다 (README, `config/pursuit.yaml`).

## 2. 방식 비교

| 방식 | 맵 필요 | 랜덤 장애물 | 장점 | 단점 |
|---|---|---|---|---|
| Follow the Gap (+disparity extender) | ✗ | ✅ | 단순하고 튜닝이 쉽다. 국소 최소(local minimum)가 없다 | 목표점이 매 스캔 튀면 지그재그. 차 동역학을 모른다 |
| Wall follow (PID) | ✗ | ✗ (박스를 벽으로 본다) | 직선에서 매끈하다 | 장애물 회피가 없다 → mode_manager가 GAP으로 넘긴다 |
| Pure Pursuit | 경로(waypoint) 필요 | 경로가 장애물을 피해야 함 | 기하학만으로 조향, 축간 거리 반영 | lookahead에 매우 민감: 짧으면 진동, 길면 코너를 자른다 |
| 레이싱 라인 최적화 + 추종 | ✅ | ✗ (따로 회피 필요) | 가장 빠르다 | SLAM, 측위, 맵 필요 |
| MPC / MPCC | 보통 ✅ | 제약식으로 가능 | 동역학과 제약을 직접 다룬다 | 계산량, 모델 정확도, 튜닝 부담 |
| 강화학습 (end-to-end) | ✗ | 학습한 범위 안에서만 | 빠를 수 있다 | 학습 비용, sim-to-real 차이, 설명하기 어렵다 |

## 3. lookahead와 pure pursuit 정리

후륜축(base_link) 기준으로 목표점 (x, y)를 지나는 원호를 그리면:

```
곡률 κ = 2y / ld²        (ld² = x² + y²)
조향각 δ = atan(L_wb · κ) = atan(2 · L_wb · sin α / ld)
```

- **lookahead ld가 짧으면** 조금만 벗어나도 크게 꺾는다 → 반응은 빠르지만 진동.
- **ld가 길면** 부드럽지만 늦게 꺾는다 → 코너 안쪽을 자르거나, 가까운 장애물을 늦게 피한다.
- 우리 시뮬에서도 그대로 보였다: `pursuit_max_dist` 1.5 m, 1.2 m는 통로 가운데 박스를 못 피했고, 1.0 m는 다 피했다.
- 목표점 각도를 그대로 조향각으로 쓰는 지금 방식은 `ld = 2·L_wb ≈ 0.66 m`인 pure pursuit과 거의 같다(작은 각도). 그래서 더 짧은 lookahead처럼 흔들렸다.
- LiDAR가 후륜축보다 0.27 m 앞에 있으므로 목표점을 **base_link로 옮긴 뒤** 계산해야 한다 (`pure_pursuit_steer` 의 `lidar_x`).
- 속도에 비례해 ld를 늘리는 adaptive lookahead(`ld = k·v + ld_min`)도 많이 쓴다. 지금은 gap 구간 속도가 0.7 m/s로 거의 일정해서 상한만 두었다.

## 4. 읽은 논문 (arXiv, 읽는 순서)

| # | 논문 | 한 줄 요약 | 우리에게 |
|---|---|---|---|
| 1 | Evans et al., *Unifying F1TENTH Autonomous Racing: Survey, Methods and Benchmarks*, [arXiv:2402.18558](https://arxiv.org/abs/2402.18558) | F1TENTH 고전 방식(측위, 경로 최적화+추종, MPCC, follow-the-gap)과 강화학습을 같은 조건에서 비교. 최적화+추종이 가장 빠르다 | 전체 지도. 우리 방식(follow-the-gap)의 위치와 한계를 설명할 근거 |
| 2 | O'Kelly et al., *F1/10: An Open-Source Autonomous Cyber-Physical Platform*, [arXiv:1901.08567](https://arxiv.org/abs/1901.08567) | 플랫폼 원조 논문. 장애물 회피 예제로 Follow the Gap 사용 | gap 배열 → 최대 틈 가운데로 향하는 기본 구조 |
| 3 | Sukhil & Behl, *Adaptive Lookahead Pure-Pursuit for Autonomous Racing*, [arXiv:2111.08873](https://arxiv.org/abs/2111.08873) | waypoint마다 최적 lookahead를 미리 정해 랩타임과 경로 이탈을 줄인다 | lookahead가 결과를 크게 바꾼다는 근거. 카메라 구간 L 튜닝에도 참고 |
| 4 | Elgouhary & El-Wakeel, *Dynamic Lookahead Distance via RL-Based Pure Pursuit*, [arXiv:2603.28625](https://arxiv.org/abs/2603.28625) (2026) | PPO로 속도·곡률을 보고 lookahead를 실시간으로 고른다. 실차에서 고정 lookahead보다 랩타임 개선 | "lookahead = 안정성과 반응성의 균형"이라는 문제 정의. 학습은 하지 않는다 |
| 5 | Baumann et al., *ForzaETH Race Stack*, [arXiv:2403.11784](https://arxiv.org/abs/2403.11784) | 국제 F1TENTH 대회 우승팀의 공개 전체 스택 (측위, 계획, 제어, 상태 추정) | 상위 팀의 모듈 구성 참고. 우리 범위보다 훨씬 크다 |
| 6 | (선택) *High-speed model-based lateral control for F1TENTH*, [arXiv:2209.04346](https://arxiv.org/abs/2209.04346) | 타이어 동역학을 넣은 제어기로 11 m/s에서 pure pursuit보다 추종 정확도 4배 | 반대로, 1~2 m/s에서는 기하학 방식으로 충분하다는 근거 |

FGM 원 논문: Sezer & Gokasan, *A novel obstacle avoidance algorithm: "Follow the Gap Method"*, Robotics and Autonomous Systems, 2012 (arXiv 아님).

## 5. 다음에 할 것

- [ ] 실차에서 `wheelbase`, 후륜축 → LiDAR 거리 실측 → `config/pursuit.yaml` 값 교체 ([lidar_check.md](lidar_check.md))
- [ ] RViz(ROS 브리지)에서 `steer_mode: pursuit` 로 AUTO 모드 주행 비교
- [ ] 결과가 좋으면 팀 레포 `lidar` 브랜치 PR: `GapParams` 에 기본값 있는 필드 4개 추가라서 `safe.yaml` 을 안 고쳐도 기존 동작은 그대로다
- [ ] 카메라 구간 `lane_follow` 의 lookahead(1.5 m)와 gap의 1.0 m가 다른 이유를 Proposal에 한 줄로 정리
