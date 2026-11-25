"""f1sim_ms C++ 코어로 우리 core 함수(follow_gap, follow_walls)를 돌려 지연·잡음에 대한 튼튼함을 본다.

sim_check.py 와 달리 실차에 가까운 조건을 넣을 수 있다.
- --delay  : 명령이 바퀴에 반영되기까지의 지연 (s). 11/24 bag 분석에서 0.2~0.3 s
- --noise  : LiDAR 거리 잡음 표준편차 (m)
- mode_manager 와 같은 출력 제한: 조향 변화율 90°/s(--steer-rate), 가속 1.5, 감속 3.0 m/s², 속도 0.3~1.5 m/s
  (safe.yaml mode_manager 기본값. --no-limits 로 끈다)
- --set    : 파라미터 덮어쓰기. 예) --set gap_follow.max_speed=0.8 --set wall_follow.kd=0.3

화면 없이 돈다. 결과 한 줄: 완주 여부, 랩타임, 벽까지 최소 거리, 조향 흔들림.

    python3 sim/sim_eval.py gap maps/pipe_obs_alt.yaml --delay 0.25
    python3 sim/sim_eval.py wall maps/pipe.yaml --delay 0.25 --set wall_follow.kd=0.3
    python3 sim/sim_eval.py gap maps/pipe_obs_*.yaml --delay 0.25 --steer-rate 180 \
        --params config/params.yaml config/delay_tuned.yaml
"""
import argparse
import collections
import pathlib
import sys

import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src' / 'perception'), str(ROOT / 'src' / 'control')]
from control.core.gap import GapParams, follow_gap  # noqa: E402
from control.core.wall import PID, WallParams, follow_walls  # noqa: E402

try:
    from f110_gym.envs.cpp_simulator import CppSimulator
    from f110_gym.envs.vehicle_params import normalize_vehicle_params
except ImportError as e:  # pragma: no cover
    sys.exit(f'f1sim_ms 가 필요하다 (팀 TODO 4-0절 설치, PYTHONPATH=~/f1sim_ms/gym): {e}')

FOV, BEAMS = 4.7, 1080          # f1sim_ms 기본 LiDAR (270°)
SIM_DT, CTRL_DT = 0.01, 0.05    # 물리 100 Hz, 제어 20 Hz (TEA 와 같다)


def load_params(paths, overrides):
    """yaml 여러 개를 차례로 덮어쓴 뒤 --set 을 적용한다."""
    cfg = {}
    for path in ([paths] if isinstance(paths, str) else paths):
        for node, body in yaml.safe_load(open(path, encoding='utf-8')).items():
            cfg.setdefault(node, {'ros__parameters': {}})['ros__parameters'].update(
                body['ros__parameters'])
    for item in overrides:
        key, val = item.split('=', 1)
        node, name = key.split('.', 1)
        cfg[node]['ros__parameters'][name] = yaml.safe_load(val)
    def pick(cls, node):
        p = cfg[node]['ros__parameters']
        return cls(**{k: p[k] for k in cls.__dataclass_fields__})
    return pick(GapParams, 'gap_follow'), pick(WallParams, 'wall_follow')


def run(ctrl, map_yaml, gp, wp, delay=0.0, noise=0.0, limits=True, laps=3, start=None,
        max_time=300.0, seed=0, steer_rate_deg=90.0):
    """한 번 달리고 결과 dict 를 돌려준다."""
    meta = yaml.safe_load(open(map_yaml, encoding='utf-8'))
    ext = pathlib.Path(meta['image']).suffix
    sim = CppSimulator(normalize_vehicle_params(None), 1, seed, SIM_DT, 0, 1, 0.0, BEAMS, FOV)
    sim.set_map(str(map_yaml), ext)
    from PIL import Image
    w_px, h_px = Image.open(pathlib.Path(map_yaml).with_name(meta['image'])).size
    cx = meta['origin'][0] + w_px * meta['resolution'] / 2
    cy = meta['origin'][1] + h_px * meta['resolution'] / 2
    width = float(meta.get('corridor_width', 1.2))
    x0, y0, th0 = start or (2.0, width / 2, 0.0)
    sim.reset(np.array([[x0, y0, th0]]))

    rng = np.random.default_rng(seed)
    angles = np.linspace(-FOV / 2, FOV / 2, BEAMS)
    pid = PID(wp.kp, wp.ki, wp.kd, wp.i_limit)
    queue = collections.deque([(0.0, 0.0)] * int(round(delay / CTRL_DT)))
    cmd_steer = cmd_speed = 0.0       # 바퀴에 실제로 들어가는 명령
    out_steer = out_speed = 0.0       # mode_manager 제한을 거친 명령
    obs = sim.step(np.array([[0.0, 0.0]]))
    prev_ang, turned = np.arctan2(y0 - cy, x0 - cx), 0.0
    min_clear, steer_hist = np.inf, []
    per = int(round(CTRL_DT / SIM_DT))
    for k in range(int(max_time / SIM_DT)):
        if k % per == 0:
            ranges = np.asarray(obs['scans'][0], dtype=float)
            if noise > 0:
                ranges = ranges + rng.normal(0, noise, ranges.size)
            min_clear = min(min_clear, float(np.nanmin(ranges)))
            if ctrl == 'gap':
                steer, speed = follow_gap(ranges, angles, gp)
            else:
                steer, speed = follow_walls(ranges, angles, pid, CTRL_DT, wp)
            if limits:  # safe.yaml mode_manager 기본값
                ds = np.radians(steer_rate_deg) * CTRL_DT
                out_steer = float(np.clip(steer, out_steer - ds, out_steer + ds))
                target = float(np.clip(speed, 0.3, 1.5)) if speed > 0 else 0.0
                out_speed = float(np.clip(target, out_speed - 3.0 * CTRL_DT, out_speed + 1.5 * CTRL_DT))
            else:
                out_steer, out_speed = steer, speed
            queue.append((out_steer, out_speed))
            cmd_steer, cmd_speed = queue.popleft()
            steer_hist.append(cmd_steer)
        obs = sim.step(np.array([[cmd_steer, cmd_speed]]))
        x, y = obs['poses_x'][0], obs['poses_y'][0]
        ang = np.arctan2(y - cy, x - cx)
        turned += (ang - prev_ang + np.pi) % (2 * np.pi) - np.pi
        prev_ang = ang
        t = (k + 1) * SIM_DT
        if obs['collisions'][0]:
            return dict(result='crash', t=t, laps=turned / (2 * np.pi), x=x, y=y,
                        min_clear=min_clear, wobble=_wobble(steer_hist))
        if turned >= 2 * np.pi * laps:
            return dict(result='done', t=t, laps=laps, lap_time=t / laps, x=x, y=y,
                        min_clear=min_clear, wobble=_wobble(steer_hist))
    return dict(result='timeout', t=max_time, laps=turned / (2 * np.pi), x=x, y=y,
                min_clear=min_clear, wobble=_wobble(steer_hist))


def _wobble(hist):
    """조향 흔들림: 제어 주기마다 조향이 바뀐 크기의 평균 (deg/step). 클수록 지그재그."""
    h = np.degrees(np.asarray(hist))
    return float(np.mean(np.abs(np.diff(h)))) if h.size > 1 else 0.0


def fmt(r):
    head = {'done': '✅', 'crash': '💥', 'timeout': '⏱'}[r['result']]
    body = (f"랩 {r['lap_time']:.1f}s" if r['result'] == 'done'
            else f"{r['laps']:.2f}바퀴 t={r['t']:.1f}s ({r['x']:.1f},{r['y']:.1f})")
    return f"{head} {body} | 최소거리 {r['min_clear']:.2f} m | 흔들림 {r['wobble']:.2f}°"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ctrl', choices=['gap', 'wall'])
    ap.add_argument('maps', nargs='+', help='맵 yaml (여러 개 가능)')
    ap.add_argument('--params', nargs='+', default=[str(ROOT / 'config' / 'params.yaml')],
                    help='여러 개면 뒤의 것이 덮어쓴다. 예) config/params.yaml config/delay_tuned.yaml')
    ap.add_argument('--set', action='append', default=[], metavar='NODE.KEY=VALUE')
    ap.add_argument('--delay', type=float, default=0.0)
    ap.add_argument('--noise', type=float, default=0.0)
    ap.add_argument('--no-limits', action='store_true')
    ap.add_argument('--steer-rate', type=float, default=90.0, help='mode_manager max_steer_rate_deg (deg/s)')
    ap.add_argument('--laps', type=int, default=3)
    args = ap.parse_args()
    gp, wp = load_params(args.params, args.set)
    ok = True
    for m in args.maps:
        r = run(args.ctrl, m, gp, wp, args.delay, args.noise, not args.no_limits, args.laps,
                steer_rate_deg=args.steer_rate)
        ok &= r['result'] == 'done'
        print(f'{args.ctrl:4s} {pathlib.Path(m).stem:15s} delay={args.delay:.2f} noise={args.noise:.2f} {fmt(r)}')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
