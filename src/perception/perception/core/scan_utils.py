"""LiDAR 스캔을 다루는 순수 함수. rclpy import 금지.

좌표계는 REP-103이다: x 전방, y 왼쪽, 각도 0 = 정면, 반시계 방향 +.
normalize_scan 을 거친 뒤에는 무효 빔이 전부 NaN 이다.

담당
- normalize_scan, crop_fov: 구현 완료 (총괄)
- cluster_scan, find_obstacles: 라이다 담당
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ScanFilterParams:
    """scan_filter 노드 파라미터. 값은 bringup/config/*.yaml 에 있다."""

    yaw_offset_deg: float  # 장착 yaw 보정
    flip: bool             # 좌우 반전 장착이면 true
    range_min: float       # m, 이보다 가까우면 무효 (차체에 맞는 빔 포함)
    range_max: float       # m, 이보다 멀면 무효
    fov_deg: float         # 남길 전체 시야각. 300이면 ±150°
    output_frame: str      # /scan_filtered 의 frame_id


@dataclass
class Cluster:
    """스캔에서 연속된 점들의 묶음."""

    idx_start: int      # 입력 배열에서 첫 유효 점의 인덱스
    idx_end: int        # 입력 배열에서 마지막 유효 점의 인덱스 (포함)
    points: np.ndarray  # (N, 2) 배열, 각 행은 [x, y], 유효 점만
    length: float       # 첫 점과 마지막 점 사이의 직선 거리 (m)


def normalize_scan(ranges, angle_min, angle_inc, yaw_offset, flip, range_min, range_max):
    """드라이버가 준 스캔을 REP-103 격자로 옮긴다.

    TEA 드라이버는 무효 빔을 0.0으로 채우고 시야 밖 60°도 0.0으로 내보낸다.
    0.0을 그대로 두면 "바로 앞에 물체가 있다"로 읽히므로 여기서 NaN으로 바꾼다.

    출력은 항상 [-π, π)를 빠짐없이 덮는 균일 격자라서, 장착 yaw가 얼마든
    angles 는 오름차순이고 간격이 일정하다. 데이터가 없는 칸은 NaN이다.

    Args:
        ranges: (N,) 거리 배열 (m)
        angle_min: 첫 빔의 각도 (rad)
        angle_inc: 빔 간격 (rad)
        yaw_offset: 장착 yaw 보정 (rad). 표준 각도 = 원래 각도(반전 후) + yaw_offset
        flip: True면 각도 부호를 뒤집는다. yaw_offset보다 먼저 적용한다.
        range_min, range_max: 이 범위를 벗어난 값은 NaN (m)

    Returns:
        (ranges, angles): 같은 길이의 배열. 각도는 rad.
    """
    ranges = np.asarray(ranges, dtype=float)
    raw_angles = angle_min + np.arange(ranges.size) * angle_inc
    std_angles = (-raw_angles if flip else raw_angles) + yaw_offset

    num_beams = int(round(2.0 * np.pi / abs(angle_inc)))
    grid_inc = 2.0 * np.pi / num_beams
    wrapped = (std_angles + np.pi) % (2.0 * np.pi)
    grid_idx = np.round(wrapped / grid_inc).astype(int) % num_beams

    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    out_ranges = np.full(num_beams, np.nan)
    out_ranges[grid_idx[valid]] = ranges[valid]
    out_angles = -np.pi + np.arange(num_beams) * grid_inc
    return out_ranges, out_angles


def crop_fov(ranges, angles, fov_deg):
    """정면 기준 전체 시야각 fov_deg 안의 빔만 남긴다. fov_deg=180이면 ±90°."""
    half_fov = np.radians(fov_deg) / 2.0
    keep = np.abs(angles) <= half_fov + 1e-9
    return ranges[keep], angles[keep]


def cluster_scan(ranges, angles, break_dist, break_ratio):
    """스캔을 연속된 점의 묶음으로 나눈다. (담당: 라이다, 맵이 아니라 매 스캔 계산)

    방법 (적응형 breakpoint)
    1. NaN 빔은 건너뛰고 유효 점만 순서대로 본다. NaN 때문에 벽이 끊기면 안 된다.
    2. 이웃한 유효 점 사이의 거리가 임계값보다 크면 거기서 끊는다.
       임계값 = break_dist + break_ratio × (앞 점의 거리)
       먼 곳은 점 간격이 원래 넓으므로 임계값도 거리에 비례해 키운다.
    3. 각 묶음의 점 좌표는 x = r·cosθ, y = r·sinθ 로 계산한다.

    Args:
        ranges, angles: 같은 길이의 배열. NaN 포함 가능.
        break_dist: 임계값의 고정 부분 (m)
        break_ratio: 임계값의 거리 비례 계수 (m/m)

    Returns:
        list[Cluster]: 각도 순서대로. 유효 점이 하나도 없으면 빈 리스트.
    """
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    valid = np.flatnonzero(np.isfinite(ranges))
    if valid.size == 0:
        return []

    r = ranges[valid]
    points = np.column_stack((r * np.cos(angles[valid]), r * np.sin(angles[valid])))

    # 이웃한 유효 점 사이 거리가 (break_dist + break_ratio × 앞 점 거리) 를 넘으면 끊는다.
    gaps = np.hypot(*np.diff(points, axis=0).T)
    splits = np.flatnonzero(gaps > break_dist + break_ratio * r[:-1]) + 1
    bounds = np.concatenate(([0], splits, [valid.size]))

    clusters = []
    for s, e in zip(bounds[:-1], bounds[1:]):
        pts = points[s:e]
        clusters.append(Cluster(
            idx_start=int(valid[s]),
            idx_end=int(valid[e - 1]),
            points=pts,
            length=float(np.hypot(*(pts[-1] - pts[0]))),
        ))
    return clusters


def find_obstacles(clusters, corridor_width, d_obs, max_obstacle_len):
    """클러스터 중 전방 통로 안의 장애물만 고른다. (담당: 라이다)

    장애물의 조건 (둘 다 만족)
    - 짧다: cluster.length <= max_obstacle_len. 길면 파이프 벽이다.
    - 통로 안: 점 하나라도 0 < x <= d_obs 이고 |y| <= corridor_width / 2.

    전방 사각형 안에 점이 있는지만 보면 코너의 바깥 벽을 장애물로 오인한다.
    길이 조건이 그 오인을 막는 핵심이다.

    Args:
        clusters: cluster_scan 의 결과
        corridor_width: 전방 통로 폭 (m). 차폭 + 여유.
        d_obs: 전방 탐지 거리 (m)
        max_obstacle_len: 장애물로 볼 최대 길이 (m)

    Returns:
        list[Cluster]: 장애물 클러스터. 없으면 빈 리스트.
    """
    half = corridor_width / 2.0
    obstacles = []
    for cluster in clusters:
        if cluster.length > max_obstacle_len:
            continue  # 길면 벽이다. 코너의 바깥 벽을 장애물로 오인하지 않게 하는 핵심 조건.
        x, y = cluster.points[:, 0], cluster.points[:, 1]
        if np.any((x > 0.0) & (x <= d_obs) & (np.abs(y) <= half)):
            obstacles.append(cluster)
    return obstacles
