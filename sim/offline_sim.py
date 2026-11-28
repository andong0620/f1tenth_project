"""ROS·시뮬레이터 없이 우리 core 함수(follow_gap, follow_walls)로 파이프 트랙을 달려 본다.

Windows, macOS, Ubuntu 어디서나 numpy + matplotlib + pyyaml 만 있으면 돈다.
f1sim_ms 대신 아주 단순한 모델을 쓴다. 결과가 좋아도 시뮬레이터와 실차 검증은 따로 해야 한다.
- 차: 운동학 자전거 모델 (미끄러짐, 조향·속도 지연 없음)
- LiDAR: synth.raycast, TEA 20 Hz와 같은 0.24° 간격 300° (노이즈 옵션 --noise)
- 충돌: 차를 반지름 car_radius 의 원으로 보고 벽·박스 선분과의 거리로 판정

예
    python sim/offline_sim.py gap                 # 박스 3개 좌우 번갈아, 3바퀴
    python sim/offline_sim.py wall --boxes none   # 박스 없이 벽 추종
    python sim/offline_sim.py gap --boxes row --noise 0.03 --plot docs/offline_gap_row.png
"""
import argparse
import pathlib
import sys

import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src' / 'perception'), str(ROOT / 'src' / 'control')]
from control.core.gap import GapParams, follow_gap  # noqa: E402
from control.core.wall import PID, WallParams, follow_walls  # noqa: E402
from perception.core.synth import beam_angles, raycast  # noqa: E402

# 트랙: 바깥 사각형 OUTER, 통로 폭 WIDTH (sim/make_pipe_map.py 와 같은 모양). 반시계로 돈다.
BOX_SETS = {  # 박스 중심 (x, y), 아래쪽 직선(y 0~WIDTH)에 놓는다
    'alt': [(4.0, 0.35), (7.0, 0.85), (10.0, 0.35)],
    'row': [(5.0, 0.35), (6.0, 0.85)],
    'wall': [(6.0, 0.20)],
    'none': [],
}


def rect(x0, y0, x1, y1):
    return [[x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0]]


def build_world(outer, width, boxes, box_size):
    w, h = outer
    segs = rect(0, 0, w, h) + rect(width, width, w - width, h - width)
    for cx, cy in boxes:
        s = box_size / 2
        segs += rect(cx - s, cy - s, cx + s, cy + s)
    return np.asarray(segs, dtype=float)


def scan_from(world, x, y, yaw, angles, noise, rng):
    """세계 선분을 차 기준으로 옮겨 레이캐스트한다. 결과는 REP-103, 무효는 NaN."""
    c, s = np.cos(-yaw), np.sin(-yaw)
    px, py = world[:, [0, 2]] - x, world[:, [1, 3]] - y
    local = np.stack([c * px[:, 0] - s * py[:, 0], s * px[:, 0] + c * py[:, 0],
                      c * px[:, 1] - s * py[:, 1], s * px[:, 1] + c * py[:, 1]], axis=1)
    ranges = raycast(angles, local)
    if noise > 0:
        ranges = ranges + rng.normal(0, noise, ranges.size)
    return ranges


def min_dist_to_segments(world, x, y):
    a, b = world[:, :2], world[:, 2:]
    ab = b - a
    t = np.clip(((np.array([x, y]) - a) * ab).sum(1) / (ab * ab).sum(1), 0, 1)
    proj = a + ab * t[:, None]
    return float(np.min(np.hypot(proj[:, 0] - x, proj[:, 1] - y)))


def load_params(path):
    cfg = yaml.safe_load(open(path, encoding='utf-8'))
    def pick(cls, node):
        p = cfg[node]['ros__parameters']
        return cls(**{k: p[k] for k in cls.__dataclass_fields__ if k in p})  # 없는 값은 기본값
    return pick(GapParams, 'gap_follow'), pick(WallParams, 'wall_follow')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ctrl', choices=['gap', 'wall'])
    ap.add_argument('--boxes', choices=list(BOX_SETS), default='alt')
    ap.add_argument('--width', type=float, default=1.2, help='통로 폭 (m). 실측값으로 바꾼다')
    ap.add_argument('--box-size', type=float, default=0.4)
    ap.add_argument('--params', default=str(ROOT / 'config' / 'params.yaml'),
                    help='팀 저장소의 src/bringup/config/safe.yaml 을 줘도 된다')
    ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--noise', type=float, default=0.0, help='LiDAR 거리 잡음 표준편차 (m)')
    ap.add_argument('--wheelbase', type=float, default=0.25)
    ap.add_argument('--car-radius', type=float, default=0.15, help='충돌 판정용 차 반지름 (m)')
    ap.add_argument('--plot', default='', help='궤적 그림을 저장할 경로 (.png)')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    outer = (14.0, 8.0)
    world = build_world(outer, args.width, BOX_SETS[args.boxes], args.box_size)
    gp, wp = load_params(args.params)
    pid = PID(wp.kp, wp.ki, wp.kd, wp.i_limit)
    angles = beam_angles()
    rng = np.random.default_rng(args.seed)

    dt = 0.05  # 20 Hz, TEA 와 같다
    x, y, yaw = 2.0, args.width / 2, 0.0
    cx, cy = outer[0] / 2, outer[1] / 2
    prev_ang = np.arctan2(y - cy, x - cx)
    turned = 0.0
    path = [(x, y)]
    result, t = 'timeout', 0.0
    for step in range(int(600 / dt)):
        t = step * dt
        ranges = scan_from(world, x, y, yaw, angles, args.noise, rng)
        if args.ctrl == 'gap':
            steer, speed = follow_gap(ranges, angles, gp)
        else:
            steer, speed = follow_walls(ranges, angles, pid, dt, wp)
        x += speed * np.cos(yaw) * dt
        y += speed * np.sin(yaw) * dt
        yaw += speed / args.wheelbase * np.tan(steer) * dt
        path.append((x, y))
        ang = np.arctan2(y - cy, x - cx)
        turned += (ang - prev_ang + np.pi) % (2 * np.pi) - np.pi
        prev_ang = ang
        if min_dist_to_segments(world, x, y) < args.car_radius:
            result = 'crash'
            break
        if turned >= 2 * np.pi * args.laps:
            result = 'done'
            break

    laps = turned / (2 * np.pi)
    if result == 'done':
        print(f'✅ {args.ctrl}: {args.laps}바퀴 완주 {t:.1f}s (랩당 {t / args.laps:.1f}s)')
    elif result == 'crash':
        print(f'💥 {args.ctrl}: 충돌 t={t:.1f}s x={x:.2f} y={y:.2f} ({laps:.2f}바퀴)')
    else:
        print(f'⏱ {args.ctrl}: 600초 안에 못 끝냄 ({laps:.2f}바퀴)')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 6))
        for s in world:
            ax.plot(s[[0, 2]], s[[1, 3]], 'k-', lw=2)
        p = np.array(path)
        ax.plot(p[:, 0], p[:, 1], '-', color='tab:blue', lw=1.2, label='path')
        ax.plot(*path[0], 'go', label='start')
        ax.plot(*path[-1], 'rx' if result == 'crash' else 'bo', ms=10, label='end')
        ax.set_aspect('equal')
        ax.set_title(f'{args.ctrl} / boxes={args.boxes} / noise={args.noise} m → {result}, {laps:.2f} laps, {t:.1f} s')
        ax.legend(loc='upper right')
        pathlib.Path(args.plot).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.plot, dpi=120, bbox_inches='tight')
        print('그림 저장:', args.plot)
    sys.exit({'done': 0, 'crash': 1}.get(result, 2))


if __name__ == '__main__':
    main()
