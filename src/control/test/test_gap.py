"""Gap follow 계약 테스트. (라이다 담당) 구현하면 xfail이 pass나 fail로 바뀐다.

마지막 TestFollowGap은 Lab 4 채점 기준과 같은 상황이다: 앞의 기둥, 한쪽 벽에 붙은 통로,
막다른 곳에서 옆으로 이어지는 통로. 각 상황과 그 좌우 반전에서 넓은 쪽으로 꺾어야 한다.
"""
import numpy as np
import pytest

from control.core.gap import (
    GapParams, best_point, disparity_extend, find_max_gap, follow_gap, preprocess,
    safety_bubble, speed_from_front)
from perception.core.synth import beam_angles, box, corridor, raycast, to_car_frame

pytestmark = pytest.mark.xfail(raises=NotImplementedError)

PARAMS = GapParams(
    fov_deg=180.0, max_range=3.0, window=3, bubble_radius=0.3, disparity_threshold=0.2,
    half_width=0.2, depth_ratio=0.9, front_half_deg=10.0, min_speed=0.4, max_speed=1.0,
    slow_dist=0.8, fast_dist=2.5, max_steer_deg=45.0)


def degrees_grid(start, stop, step=1.0):
    return np.radians(np.arange(start, stop + step / 2.0, step))


class TestPreprocess:

    def test_dropout_is_filled_from_neighbors(self):
        ranges = np.array([1.0, np.nan, 2.0])
        assert preprocess(ranges, degrees_grid(-1, 1), 3.0, 1)[1] == pytest.approx(1.5)

    def test_dropout_does_not_split_a_gap(self):
        # 드롭아웃 한 빔을 막힘으로 두면 틈이 둘로 쪼개져 차가 흔들린다.
        ranges = np.full(21, 2.0)
        ranges[10] = np.nan
        assert find_max_gap(preprocess(ranges, degrees_grid(-10, 10), 3.0, 1)) == (0, 20)

    def test_beyond_lidar_range_is_free_not_blocked(self):
        # 정면이 멀면 빔이 안 돌아온다. 뚫린 곳이 막힌 것으로 보이면 안 된다.
        ranges = np.array([20.0, np.nan, np.nan, np.nan, 20.0])
        out = preprocess(ranges, degrees_grid(-2, 2), 3.0, 1)
        assert np.all(out == 3.0)

    @pytest.mark.parametrize('bad', [np.nan, np.inf, 0.0, -1.0])
    def test_all_kinds_of_invalid_values_are_filled(self, bad):
        ranges = np.array([1.0, bad, 1.0])
        assert preprocess(ranges, degrees_grid(-1, 1), 3.0, 1)[1] == pytest.approx(1.0)

    def test_edges_take_the_nearest_valid_value(self):
        ranges = np.array([np.nan, 1.0, 2.0, np.nan])
        assert preprocess(ranges, degrees_grid(-1, 2), 3.0, 1) == pytest.approx([1.0, 1.0, 2.0, 2.0])

    def test_no_valid_beam_is_all_zero(self):
        out = preprocess(np.full(5, np.nan), degrees_grid(-2, 2), 3.0, 3)
        assert np.all(out == 0.0)

    def test_clips_to_max_range(self):
        out = preprocess(np.array([5.0, 2.0, 5.0]), degrees_grid(-1, 1), 3.0, 1)
        assert out[0] == 3.0 and out[1] == 2.0

    def test_moving_average(self):
        ranges = np.array([1.0, 1.0, 2.0, 1.0, 1.0])
        out = preprocess(ranges, degrees_grid(-2, 2), 3.0, 3)
        assert out[2] == pytest.approx(4.0 / 3.0)

    def test_average_does_not_shrink_the_edges(self):
        out = preprocess(np.full(5, 2.0), degrees_grid(-2, 2), 3.0, 3)
        assert out == pytest.approx(np.full(5, 2.0))

    def test_same_length_and_input_untouched(self):
        ranges = np.array([np.nan, 5.0, 1.0, 1.0])
        original = ranges.copy()
        out = preprocess(ranges, degrees_grid(-1, 2), 3.0, 3)
        assert out.size == 4
        assert not np.isnan(out).any()
        assert np.array_equal(ranges, original, equal_nan=True)


class TestSafetyBubble:

    def test_clears_around_closest_point(self):
        angles = degrees_grid(-30, 30)
        ranges = np.full(angles.size, 3.0)
        ranges[30] = 1.0  # 정면, asin(0.3 / 1.0) ≈ 17.5°
        out = safety_bubble(ranges, angles, 0.3)
        assert out[30] == 0.0
        assert out[30 + 17] == 0.0 and out[30 - 17] == 0.0
        assert out[30 + 19] == 3.0 and out[30 - 19] == 3.0

    def test_ignores_zero_when_finding_closest(self):
        angles = degrees_grid(-30, 30)
        ranges = np.full(angles.size, 3.0)
        ranges[0] = 0.0
        ranges[50] = 1.0
        assert safety_bubble(ranges, angles, 0.1)[50] == 0.0

    def test_closer_than_radius_does_not_crash(self):
        angles = degrees_grid(-90, 90)
        ranges = np.full(angles.size, 3.0)
        ranges[90] = 0.1
        out = safety_bubble(ranges, angles, 0.3)
        assert out[90] == 0.0

    def test_all_zero_input(self):
        angles = degrees_grid(-5, 5)
        assert safety_bubble(np.zeros(angles.size), angles, 0.3).sum() == 0.0


class TestDisparityExtend:

    def test_extends_near_distance_over_far_side(self):
        angles = degrees_grid(-30, 30)
        ranges = np.full(angles.size, 3.0)
        ranges[:30] = 1.0  # 인덱스 29(1.0)와 30(3.0) 사이가 경계
        out = disparity_extend(ranges, angles, 0.3, 0.2)
        # atan(0.2 / 1.0) ≈ 11.3° 만큼 먼 쪽으로 넓어진다.
        assert out[30 + 9] == 1.0
        assert out[30 + 13] == 3.0

    def test_extends_in_the_other_direction_too(self):
        angles = degrees_grid(-30, 30)
        ranges = np.full(angles.size, 3.0)
        ranges[31:] = 1.0  # 왼쪽(+)이 가깝다
        out = disparity_extend(ranges, angles, 0.3, 0.2)
        assert out[30 - 9] == 1.0
        assert out[30 - 13] == 3.0

    def test_small_difference_is_untouched(self):
        angles = degrees_grid(-30, 30)
        ranges = np.linspace(2.0, 2.2, angles.size)
        assert disparity_extend(ranges, angles, 0.3, 0.2) == pytest.approx(ranges)

    def test_never_increases_a_range(self):
        angles = degrees_grid(-30, 30)
        ranges = 2.0 + np.sin(angles * 40.0)
        assert np.all(disparity_extend(ranges, angles, 0.3, 0.2) <= ranges + 1e-9)

    def test_zero_from_bubble_is_not_a_disparity(self):
        angles = degrees_grid(-30, 30)
        ranges = np.full(angles.size, 3.0)
        ranges[20:40] = 0.0
        assert np.array_equal(disparity_extend(ranges, angles, 0.3, 0.2), ranges)

    def test_narrow_slit_disappears(self):
        # Lab 4의 팁: 차가 못 들어가는 틈은 목표가 되면 안 된다.
        angles = degrees_grid(-30, 30, 0.5)
        ranges = np.full(angles.size, 1.0)
        center = angles.size // 2
        ranges[center - 2:center + 3] = 3.0  # 1 m 앞의 폭 약 9 cm 틈
        out = disparity_extend(ranges, angles, 0.3, 0.2)
        assert out.max() == 1.0


class TestFindMaxGap:

    def test_longest_run(self):
        assert find_max_gap(np.array([0, 1, 1, 0, 1, 1, 1, 0], dtype=float)) == (4, 6)

    def test_run_at_the_end(self):
        assert find_max_gap(np.array([0, 0, 1, 1], dtype=float)) == (2, 3)

    def test_whole_array(self):
        assert find_max_gap(np.ones(4)) == (0, 3)

    def test_tie_keeps_first(self):
        assert find_max_gap(np.array([1, 1, 0, 1, 1], dtype=float)) == (0, 1)

    def test_no_gap(self):
        assert find_max_gap(np.zeros(4)) == (-1, -1)


class TestBestPoint:

    def test_plateau_picks_the_center(self):
        assert best_point(np.full(11, 3.0), 0, 10, 0.9) == 5

    def test_prefers_deep_points_over_center(self):
        ranges = np.array([3.0, 3.0, 3.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        assert best_point(ranges, 0, 10, 0.9) == 2

    def test_stays_inside_the_gap(self):
        ranges = np.array([9.0, 0.0, 1.0, 2.0, 1.0, 0.0, 9.0])
        assert best_point(ranges, 2, 4, 0.9) == 3


class TestSpeedFromFront:

    @pytest.mark.parametrize('front, expected', [
        (0.0, 0.4), (0.8, 0.4), (1.65, 0.7), (2.5, 1.0), (10.0, 1.0)])
    def test_interpolates(self, front, expected):
        assert speed_from_front(front, 0.4, 1.0, 0.8, 2.5) == pytest.approx(expected)


def dead_end(mirror=False):
    """2 m 앞이 막혔고 오른쪽으로 통로가 이어진다. mirror면 왼쪽으로."""
    segments = np.array([
        [-5.0, 1.0, 2.5, 1.0],    # 왼쪽 벽
        [2.5, 1.0, 2.5, -6.0],    # 앞의 벽
        [-5.0, -1.0, 1.0, -1.0],  # 오른쪽 벽, x=1에서 끊긴다
        [1.0, -1.0, 1.0, -6.0],   # 오른쪽 통로의 벽
    ])
    if mirror:
        segments[:, [1, 3]] *= -1.0
    return segments


class TestFollowGap:
    """Lab 4 채점 기준의 상황들. 각 상황과 좌우 반전을 함께 본다."""

    def drive(self, segments):
        angles = beam_angles()
        return follow_gap(raycast(angles, segments), angles, PARAMS)

    @pytest.mark.parametrize('side, sign', [(1.0, -1.0), (-1.0, 1.0)])
    def test_pillar_steers_to_the_wider_side(self, side, sign):
        segments = np.vstack([corridor(3.0), box(1.5, side * 0.5, 0.2, 0.2)])
        assert self.drive(segments)[0] * sign > 0.0

    @pytest.mark.parametrize('y, sign', [(0.6, -1.0), (-0.6, 1.0)])
    def test_close_to_a_wall_steers_away(self, y, sign):
        segments = to_car_frame(corridor(2.0), y=y)
        assert self.drive(segments)[0] * sign > 0.0

    @pytest.mark.parametrize('mirror, sign', [(False, -1.0), (True, 1.0)])
    def test_dead_end_aims_into_the_exit(self, mirror, sign):
        assert self.drive(dead_end(mirror))[0] * sign > 0.0

    def test_open_corridor_goes_nearly_straight(self):
        # 정면의 먼 빔은 NaN이다. 막힘으로 처리하면 틈이 쪼개져 크게 꺾는다.
        steer, speed = self.drive(corridor(2.0))
        assert abs(steer) < np.radians(15.0)
        assert speed == pytest.approx(PARAMS.max_speed)

    def test_blocked_front_slows_down(self):
        _, speed = self.drive(np.vstack([corridor(2.0), [[0.7, -1.0, 0.7, 0.2]]]))
        assert speed == pytest.approx(PARAMS.min_speed)

    def test_box_too_close_to_wall_is_not_a_gap(self):
        # 벽과 박스 사이 틈(0.25 m)은 차가 못 지나간다. 그 틈으로 가면 안 되고, 박스의 안쪽
        # 모서리(y=0.45, x=1.65)도 차 반폭(0.15 m)만큼은 비켜 가는 방향이어야 한다.
        segments = np.vstack([corridor(2.0), box(1.5, 0.6, 0.3, 0.3)])
        steer = self.drive(segments)[0]
        assert 1.65 * np.tan(steer) <= 0.45 - 0.15

    def test_steering_is_limited(self):
        params = GapParams(**{**PARAMS.__dict__, 'max_steer_deg': 5.0})
        angles = beam_angles()
        steer, _ = follow_gap(raycast(angles, dead_end()), angles, params)
        assert abs(steer) <= np.radians(5.0) + 1e-9

    def test_no_gap_stops(self):
        angles = beam_angles()
        steer, speed = follow_gap(np.full(angles.size, np.nan), angles, PARAMS)
        assert steer == 0.0 and speed == 0.0
