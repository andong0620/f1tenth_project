"""Follow the Gap (Lab 4 응용). rclpy import 금지. (담당: 라이다)

참고: https://github.com/f1tenth/f1tenth_lab4_template
Lab 4와 다른 점: disparity extender를 더해 차가 못 들어가는 틈을 없앤다.

speed_from_front 만 하드웨어·컨트롤 담당이다. 처리 순서는 맨 아래 follow_gap 함수에 있다. 배열 안에서 0은 "막힘"을 뜻한다.
"""
from dataclasses import dataclass

import numpy as np

from perception.core.scan_utils import crop_fov


@dataclass(frozen=True)
class GapParams:
    """gap_follow 노드 파라미터. 값은 bringup/config/*.yaml 에 있다."""

    fov_deg: float              # 사용할 전체 시야각. 180이면 ±90°
    max_range: float            # m, 이보다 먼 값은 잘라낸다
    window: int                 # 이동평균 창 크기 (빔 개수)
    bubble_radius: float        # m, safety bubble 반경
    disparity_threshold: float  # m, 이웃 빔 거리 차가 이보다 크면 경계로 본다
    half_width: float           # m, 차 반폭 + 여유
    depth_ratio: float          # 최대 거리의 이 비율 이상이면 "먼 점"으로 본다
    front_half_deg: float       # 속도 계산에 쓸 정면 범위. 10이면 ±10°
    min_speed: float            # m/s
    max_speed: float            # m/s
    slow_dist: float            # m, 전방 거리가 이 이하면 min_speed
    fast_dist: float            # m, 전방 거리가 이 이상이면 max_speed
    max_steer_deg: float        # 조향각 제한


def preprocess(ranges, angles, max_range, window):
    """스캔을 다듬는다. (담당: 라이다)

    1. 무효 값(NaN, inf, 0 이하)을 양옆의 유효 값으로 선형 보간한다. (np.interp)
       유효 빔이 하나도 없으면 전부 0으로 돌려준다.
       TEA는 "너무 멀어서 안 돌아옴"과 "측정 오류"를 구분 없이 무효로 낸다.
       이를 0(막힘)으로 두면 드롭아웃 한 빔이 틈을 둘로 쪼개고, 정면이 먼 통로가 막혀 보인다.
    2. max_range 보다 큰 값은 max_range 로 자른다.
    3. window 크기의 이동평균을 한다. 배열 양끝 값이 작아지지 않도록
       np.pad(mode='edge') 로 늘린 뒤 평균한다.

    Args:
        ranges, angles: 같은 길이의 배열
        max_range: 거리 상한 (m)
        window: 이동평균 창 크기 (빔 개수). 1이면 평균 없음.

    Returns:
        입력과 길이가 같은 새 배열. NaN 없음. 입력을 수정하지 않는다.
    """
    raise NotImplementedError('preprocess: 라이다 담당 구현 필요')


def safety_bubble(ranges, angles, radius):
    """가장 가까운 점 주변을 0으로 지운다. (담당: 라이다)

    1. 0이 아닌 값 중 가장 가까운 점을 찾는다. 거리 r.
    2. 그 점에서 각도 ±asin(radius / r) 안의 빔을 모두 0으로 만든다.
       r <= radius 이면 ±90° 로 본다. (asin의 입력이 1을 넘지 않게)

    Args:
        ranges, angles: 같은 길이의 배열. 0은 막힘.
        radius: 버블 반경 (m)

    Returns:
        새 배열. 전부 0이면 그대로 돌려준다.
    """
    raise NotImplementedError('safety_bubble: 라이다 담당 구현 필요')


def disparity_extend(ranges, angles, threshold, half_width):
    """거리가 급변하는 경계를 차 반폭만큼 넓힌다. (담당: 라이다)

    Lab 4의 팁 "차가 들어갈 수 있는 틈인가"에 대한 답이다.

    주의: 이 함수는 이동평균 뒤에 돈다. 평균이 경계를 window 개 빔으로 번지게 해서
    이웃 빔의 거리 차가 "실제 점프 / window" 로 줄어든다. window를 키우면 threshold도 낮춘다.

    이웃한 두 빔의 거리 차가 threshold 보다 크면 경계(disparity)다.
    1. 가까운 쪽 거리를 d 라 한다. 둘 중 0이 있으면 건너뛴다. (버블은 이미 처리됨)
    2. 먼 쪽 방향으로 각도 atan(half_width / d) 안의 빔을 min(기존값, d) 로 덮어쓴다.

    덮어쓴 결과로 다른 경계를 또 찾지 않도록, 경계는 입력 배열에서 찾고 결과는 복사본에 쓴다.

    Returns:
        입력과 길이가 같은 새 배열. 값이 커지는 빔은 없다.
    """
    raise NotImplementedError('disparity_extend: 라이다 담당 구현 필요')


def find_max_gap(ranges):
    """0이 아닌 값이 가장 길게 이어진 구간을 찾는다. (담당: 라이다)

    Returns:
        (start, end): 양쪽 다 포함하는 인덱스. 길이가 같으면 앞쪽 구간.
        0이 아닌 값이 없으면 (-1, -1).
    """
    raise NotImplementedError('find_max_gap: 라이다 담당 구현 필요')


def best_point(ranges, start, end, depth_ratio):
    """틈 안의 목표점을 고른다. (담당: 라이다)

    맨 먼 점을 고르면 틈 가장자리로 붙기 쉽다. Lab 4의 "Better Idea"처럼:
    1. 틈 안의 최대 거리 m 을 구한다.
    2. 거리가 depth_ratio × m 이상인 빔을 후보로 삼는다.
    3. 후보 중 틈 중앙 (start + end) / 2 에 가장 가까운 인덱스를 고른다.

    Returns:
        int: start <= idx <= end 인 인덱스
    """
    raise NotImplementedError('best_point: 라이다 담당 구현 필요')


def speed_from_front(front_dist, min_speed, max_speed, slow_dist, fast_dist):
    """전방 거리로 속도를 정한다. (담당: 하드웨어·컨트롤)

    front_dist <= slow_dist 이면 min_speed, front_dist >= fast_dist 이면 max_speed,
    그 사이는 직선으로 잇는다. (np.interp)
    """
    raise NotImplementedError('speed_from_front: 하드웨어·컨트롤 담당 구현 필요')


def follow_gap(ranges, angles, p):
    """스캔 하나로 (조향각, 속도)를 계산한다. 위 함수들을 순서대로 부르는 조립 함수다."""
    ranges, angles = crop_fov(ranges, angles, p.fov_deg)
    clean = preprocess(ranges, angles, p.max_range, p.window)
    free = safety_bubble(clean, angles, p.bubble_radius)
    free = disparity_extend(free, angles, p.disparity_threshold, p.half_width)

    start, end = find_max_gap(free)
    if start < 0:
        return 0.0, 0.0  # 틈이 없다. 멈춘다.

    idx = best_point(free, start, end, p.depth_ratio)
    max_steer = np.radians(p.max_steer_deg)
    steer = float(np.clip(angles[idx], -max_steer, max_steer))

    front = np.abs(angles) <= np.radians(p.front_half_deg)
    speed = speed_from_front(
        float(clean[front].min()), p.min_speed, p.max_speed, p.slow_dist, p.fast_dist)
    return steer, speed
