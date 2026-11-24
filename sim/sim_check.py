"""f1sim_ms(gym, ROS 없음)에서 우리 core 함수(follow_gap, follow_walls)를 돌려 본다. Ubuntu + f1sim_ms 필요.
팀 저장소 TODO/week12_LIDAR.md 4-2절의 스크립트를 이 저장소에서도 돌게 고쳤다.
파라미터는 기본으로 config/params.yaml, --params 로 팀 safe.yaml 을 줄 수 있다.

    python3 sim/sim_check.py gap --render
    python3 sim/sim_check.py wall --map maps/pipe_obs_wall
"""
import argparse
import pathlib
import sys

import gym
import numpy as np
import yaml
from f110_gym.envs import Integrator

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src' / 'perception'), str(ROOT / 'src' / 'control')]
from control.core.gap import GapParams, follow_gap  # noqa: E402
from control.core.wall import PID, WallParams, follow_walls  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('ctrl', choices=['gap', 'wall'])
ap.add_argument('--map', default='maps/pipe_obs', help='확장자 없이')
ap.add_argument('--start', type=float, nargs=3, default=[2.0, 0.6, 0.0], metavar=('X', 'Y', 'THETA'))
ap.add_argument('--laps', type=int, default=3)
ap.add_argument('--params', default=str(ROOT / 'config' / 'params.yaml'))
ap.add_argument('--render', action='store_true')
args = ap.parse_args()

cfg = yaml.safe_load(open(args.params, encoding='utf-8'))
def params(cls, node):
    p = cfg[node]['ros__parameters']
    return cls(**{k: p[k] for k in cls.__dataclass_fields__})

if args.ctrl == 'gap':
    gp = params(GapParams, 'gap_follow')
    control = lambda r, a, dt: follow_gap(r, a, gp)
else:
    wp = params(WallParams, 'wall_follow')
    pid = PID(wp.kp, wp.ki, wp.kd, wp.i_limit)
    control = lambda r, a, dt: follow_walls(r, a, pid, dt, wp)

env = gym.make('f110_gym:f110-v0', map=str(pathlib.Path(args.map).resolve()), map_ext='.png',
               num_agents=1, timestep=0.01, integrator=Integrator.RK4)
obs, _, done, _ = env.reset(np.array([args.start]))
CTRL_EVERY = 5  # 0.01 s × 5 = 20 Hz, TEA 스캔 주기와 같다
steer = speed = 0.0
for step in range(int(300 / 0.01)):  # 최대 300초
    if step % CTRL_EVERY == 0:
        ranges = obs['scans'][0]
        angles = np.linspace(-4.7 / 2, 4.7 / 2, ranges.size)  # 시뮬 LiDAR 270°, 이미 REP-103
        steer, speed = control(ranges, angles, 0.01 * CTRL_EVERY)
    obs, _, done, _ = env.step(np.array([[steer, speed]]))
    if args.render:
        env.render(mode='human')
    if obs['collisions'][0]:
        print(f'💥 충돌 t={step * 0.01:.1f}s x={obs["poses_x"][0]:.2f} y={obs["poses_y"][0]:.2f}')
        sys.exit(1)
    if obs['lap_counts'][0] >= args.laps:
        print(f'✅ {args.laps}바퀴 완주 {step * 0.01:.1f}s (랩당 {step * 0.01 / args.laps:.1f}s)')
        sys.exit(0)
print('⏱ 300초 안에 끝나지 않음 (멈췄거나 제자리)')
sys.exit(2)
