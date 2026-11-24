"""양쪽 벽 중앙 추종 (Lab 3 응용). rclpy import 금지. (담당: 라이다)

참고: https://github.com/f1tenth/f1tenth_lab3_template
Lab 3과 다른 점: 한쪽 벽까지의 목표 거리 대신 양쪽 벽의 중앙을 따른다.
speed_from_steer 만 하드웨어·컨트롤 담당이다.

입력 스캔은 REP-103이다: 각도 0 = 정면, 왼쪽 +. 무효 빔은 NaN.
조향각도 왼쪽이 + 이다. (ackermann_msgs 규칙)
"""
from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass(frozen=True)
class WallParams:
    """wall_follow 노드 파라미터. 값은 bringup/config/*.yaml 에 있다."""

    theta_deg: float               # 두 빔 a, b 사이의 각. 0 < θ <= 70
    lookahead: float               # m, Lab 3의 L
    beam_window_deg: float         # 빔 하나 대신 이 폭 안의 중앙값을 쓴다 (드롭아웃 대비)
    max_wall_dist: float           # m, 이보다 먼 벽은 없는 것으로 친다
    half_width: float              # m, 한쪽 벽만 보일 때 그 벽에서 유지할 거리
    corner_front_dist: float       # m, 벽 한쪽이 없고 앞 벽이 이보다 가까우면 코너로 본다
    kp: float
    ki: float
    kd: float
    i_limit: float                 # 적분항 크기 제한
    max_steer_deg: float           # 조향각 제한
    steer_breaks_deg: List[float]  # 속도 구간 경계. 오름차순. 예: [10, 20]
    speeds: List[float]            # m/s, 경계보다 1개 많다. 예: [1.0, 0.7, 0.4]


def wall_distance(ranges, angles, side, theta_deg, lookahead, window_deg, max_dist):
    """한쪽 벽까지의 미래 거리와 벽에 대한 각도를 구한다. (담당: 라이다)

    Lab 3의 삼각법 (θ = theta_deg)
        빔 b: 벽 쪽 90°    (왼쪽 +90°, 오른쪽 -90°)
        빔 a: b에서 정면 쪽으로 θ만큼 돌린 빔 (왼쪽 90°-θ, 오른쪽 -90°+θ)
        α = atan2(a·cosθ − b, a·sinθ)
        D(t) = b·cosα
        D(t+1) = D(t) + L·sinα

    α의 부호: 차가 그 벽에서 멀어지는 방향을 보면 +, 벽을 향하면 − 이다.
    (왼쪽 벽이든 오른쪽 벽이든 같다. 그래서 공식이 양쪽에 똑같이 적용된다.)

    인덱스로 각도를 가정하면 안 된다. 항상 angles 배열로 찾는다.
    또 빔 하나만 믿으면 드롭아웃(NaN) 한 번에 벽을 잃는다. 그래서 a, b 는 각각
    목표각 ±window_deg/2 안에서 유효한 값들의 중앙값으로 정한다.
        vals = ranges[np.abs(angles - 목표각) <= np.radians(window_deg) / 2]
        vals = vals[np.isfinite(vals)]

    Args:
        ranges, angles: 같은 길이의 배열 (m, rad)
        side: 'left' 또는 'right'
        theta_deg: a, b 사이의 각 (도)
        lookahead: L (m)
        window_deg: 중앙값을 낼 전체 각도 폭 (도)
        max_dist: 이보다 먼 벽은 없는 것으로 친다 (m)

    Returns:
        (dist, alpha): D(t+1) (m)와 α (rad). 아래 경우는 벽이 없는 것으로 보고 (nan, nan).
        - a나 b의 창 안에 유효한 값이 하나도 없다.
        - b > max_dist 이거나 a > max_dist / cosθ 이다. 반드시 삼각법 전에 검사한다.
          코너에서는 b가 11 m 밖을 보고 a는 앞 벽에 맞는다. 이 값으로 계산하면
          짧은 D가 나와서 "벽이 가깝다"며 엉뚱한 쪽으로 꺾는다.
    """
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    theta = np.radians(theta_deg)
    half_window = np.radians(window_deg) / 2.0

    if side == 'left':
        b_angle, a_angle = np.pi / 2.0, np.pi / 2.0 - theta
    elif side == 'right':
        b_angle, a_angle = -np.pi / 2.0, -np.pi / 2.0 + theta
    else:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")

    def median_near(target):
        vals = ranges[np.abs(angles - target) <= half_window]
        vals = vals[np.isfinite(vals)]
        return float(np.median(vals)) if vals.size else np.nan

    a, b = median_near(a_angle), median_near(b_angle)
    if np.isnan(a) or np.isnan(b):
        return np.nan, np.nan
    # 삼각법 전에 거리 검사. 코너에서 b는 멀리, a는 앞 벽에 맞아 엉터리 D가 나온다.
    if b > max_dist or a > max_dist / np.cos(theta):
        return np.nan, np.nan

    alpha = float(np.arctan2(a * np.cos(theta) - b, a * np.sin(theta)))
    dist_now = b * np.cos(alpha)
    return float(dist_now + lookahead * np.sin(alpha)), alpha


def front_distance(ranges, angles, window_deg):
    """정면까지의 거리. 정면 ±window_deg/2 안 유효 값의 중앙값이고, 없으면 inf 이다.

    조립 함수 follow_walls 가 코너를 판단할 때 쓴다.
    """
    vals = ranges[np.abs(angles) <= np.radians(window_deg) / 2.0]
    vals = vals[np.isfinite(vals)]
    return float(np.median(vals)) if vals.size else np.inf


def center_error(d_left, d_right, half_width):
    """통로 중앙에 대한 오차를 구한다. (담당: 라이다)

    오차가 + 이면 차가 중앙보다 오른쪽에 있다. 조향도 +(왼쪽)가 나와야 중앙으로 돌아간다.
    (주의: LaneState.lateral_error 는 왼쪽이 + 라서 부호 의미가 반대다.)

    - 둘 다 있음: (d_left − d_right) / 2
    - 왼쪽만 있음: d_left − half_width
    - 오른쪽만 있음: half_width − d_right
    - 둘 다 NaN: nan

    Args:
        d_left, d_right: 벽까지의 거리 (m). 벽이 없으면 NaN.
        half_width: 한쪽 벽만 보일 때 유지할 거리 (m)

    Returns:
        float: 오차 (m) 또는 nan
    """
    has_left, has_right = not np.isnan(d_left), not np.isnan(d_right)
    if has_left and has_right:
        return (d_left - d_right) / 2.0
    if has_left:
        return d_left - half_width
    if has_right:
        return half_width - d_right
    return np.nan


class PID:
    """이산 PID 제어기. (담당: 라이다)

    u = kp·e + ki·∫e dt + kd·de/dt
    - 적분항 ki·∫e dt 의 크기는 i_limit 으로 제한한다. (windup 방지)
    - dt <= 0 이면(첫 프레임 등) 적분과 미분을 건너뛰고 kp·e 만 낸다.
    """

    def __init__(self, kp, ki, kd, i_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.reset()

    def step(self, error, dt):
        """오차 하나를 넣어 제어 입력을 돌려준다. 다음 호출을 위해 상태를 저장한다."""
        if dt <= 0.0:
            self._prev_error = error
            return self.kp * error

        # 적분: ki·∫e dt 가 ±i_limit 을 넘지 않도록 적분 상태 자체를 묶는다 (windup 방지).
        self._integral += error * dt
        if self.ki != 0.0:
            bound = self.i_limit / abs(self.ki)
            self._integral = float(np.clip(self._integral, -bound, bound))

        # 미분: 이전 오차가 없으면(첫 호출, reset 직후) 0으로 둔다.
        derivative = 0.0 if self._prev_error is None else (error - self._prev_error) / dt
        self._prev_error = error

        return self.kp * error + self.ki * self._integral + self.kd * derivative

    def reset(self):
        """적분값과 이전 오차를 지운다."""
        self._integral = 0.0
        self._prev_error = None


def speed_from_steer(steer, table):
    """조향각 구간별 속도를 고른다. (담당: 하드웨어·컨트롤)

    table = ([10°, 20°를 rad로], [1.0, 0.7, 0.4]) 이면
        |steer| < 10°          → 1.0
        10° <= |steer| < 20°   → 0.7
        20° <= |steer|         → 0.4

    Args:
        steer: 조향각 (rad). 왼쪽/오른쪽을 구분하지 않는다.
        table: (breaks, speeds). breaks는 오름차순 rad, len(speeds) == len(breaks) + 1

    Returns:
        float: 속도 (m/s)
    """
    # TODO(컨트롤 담당): 임시 구현. 담당자 버전이 오면 교체한다.
    breaks, speeds = table
    i = int(np.searchsorted(np.asarray(breaks, dtype=float), abs(steer), side='right'))
    return float(speeds[i])


def follow_walls(ranges, angles, pid, dt, p):
    """스캔 하나로 (조향각, 속도)를 계산한다. 위 함수들을 순서대로 부르는 조립 함수다."""
    d_left, _ = wall_distance(
        ranges, angles, 'left', p.theta_deg, p.lookahead, p.beam_window_deg, p.max_wall_dist)
    d_right, _ = wall_distance(
        ranges, angles, 'right', p.theta_deg, p.lookahead, p.beam_window_deg, p.max_wall_dist)
    max_steer = np.radians(p.max_steer_deg)

    # 코너: 벽 한쪽이 사라졌고 앞에 벽이 있다. 남은 벽을 따라가면 코너 정면으로 직진하므로,
    # 사라진 쪽으로 꺾는다. 앞에 벽이 없으면 코너가 아니라 파이프가 잠깐 끊긴 곳이다.
    if np.isnan(d_left) != np.isnan(d_right) and \
            front_distance(ranges, angles, p.beam_window_deg) <= p.corner_front_dist:
        return (max_steer if np.isnan(d_left) else -max_steer), p.speeds[-1]

    error = center_error(d_left, d_right, p.half_width)
    if np.isnan(error):
        return 0.0, p.speeds[-1]  # 양쪽 벽이 다 없다. 곧고 느리게 가며 mode_manager 판단을 기다린다.

    steer = float(np.clip(pid.step(error, dt), -max_steer, max_steer))
    speed = speed_from_steer(steer, (np.radians(p.steer_breaks_deg), p.speeds))
    return steer, speed
