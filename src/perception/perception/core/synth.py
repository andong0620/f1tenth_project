"""테스트용 가짜 스캔 생성기. 실제 rosbag 없이 벽과 박스로 스캔을 만든다.

차는 항상 원점에서 +x를 바라본다 (REP-103). 벽과 박스는 선분 배열로 표현한다.
선분 배열은 (M, 4) 모양이고 각 행은 [x1, y1, x2, y2] 이다.
"""
import numpy as np


def beam_angles(fov_deg=300.0, inc_deg=0.24):
    """TEA 20 Hz 설정과 같은 해상도의 각도 배열을 만든다. (rad, 오름차순)"""
    half = np.radians(fov_deg) / 2.0
    num_beams = int(round(fov_deg / inc_deg)) + 1
    return np.linspace(-half, half, num_beams)


def raycast(angles, segments, max_range=25.0):
    """각 빔이 선분과 만나는 가장 가까운 거리를 구한다. 안 맞으면 NaN."""
    segments = np.atleast_2d(np.asarray(segments, dtype=float))
    dir_x, dir_y = np.cos(angles)[:, None], np.sin(angles)[:, None]
    p_x, p_y = segments[:, 0], segments[:, 1]
    s_x, s_y = segments[:, 2] - p_x, segments[:, 3] - p_y

    denom = dir_x * s_y - dir_y * s_x
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (p_x * s_y - p_y * s_x) / denom
        u = (p_x * dir_y - p_y * dir_x) / denom
    hit = (t > 0.0) & (u >= 0.0) & (u <= 1.0)
    t = np.where(hit, t, np.inf)

    ranges = t.min(axis=1)
    ranges[ranges > max_range] = np.nan
    return ranges


def to_car_frame(segments, x=0.0, y=0.0, yaw_deg=0.0):
    """세계 좌표의 선분을, (x, y)에서 yaw_deg 만큼 돌아 있는 차 기준 좌표로 옮긴다."""
    segments = np.atleast_2d(np.asarray(segments, dtype=float))
    yaw = np.radians(yaw_deg)
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    out = np.empty_like(segments)
    for start in (0, 2):
        dx = segments[:, start] - x
        dy = segments[:, start + 1] - y
        out[:, start] = cos_yaw * dx + sin_yaw * dy
        out[:, start + 1] = -sin_yaw * dx + cos_yaw * dy
    return out


def corridor(width, length=30.0):
    """x축 방향의 직선 통로. 왼쪽 벽 y = +width/2, 오른쪽 벽 y = -width/2."""
    half = width / 2.0
    return np.array([
        [-length, half, length, half],
        [-length, -half, length, -half],
    ])


def box(x, y, size_x, size_y):
    """(x, y)가 중심인 직사각형 박스의 네 변."""
    x1, x2 = x - size_x / 2.0, x + size_x / 2.0
    y1, y2 = y - size_y / 2.0, y + size_y / 2.0
    return np.array([
        [x1, y1, x2, y1],
        [x2, y1, x2, y2],
        [x2, y2, x1, y2],
        [x1, y2, x1, y1],
    ])
