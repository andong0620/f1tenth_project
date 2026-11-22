"""Wall follow 계약 테스트. (라이다 담당) 구현하면 xfail이 pass나 fail로 바뀐다.

마지막 TestFollowWalls는 Lab 3 채점 기준과 같은 상황이다: 직선 통로에서 한쪽 벽에 가까울 때와
한쪽 벽을 향해 돌아 있을 때, 어느 쪽이든 올바른 방향으로 꺾어야 한다.
"""
import numpy as np
import pytest

from control.core.wall import (
    PID, WallParams, center_error, follow_walls, speed_from_steer, wall_distance)
from perception.core.synth import beam_angles, corridor, raycast, to_car_frame

pytestmark = pytest.mark.xfail(raises=NotImplementedError)

THETA = 50.0
WINDOW = 3.0
MAX_DIST = 1.8
WIDTH = 2.0
PARAMS = WallParams(
    theta_deg=THETA, lookahead=0.5, beam_window_deg=WINDOW, max_wall_dist=MAX_DIST, half_width=1.0, corner_front_dist=2.0, kp=1.0, ki=0.0, kd=0.1, i_limit=0.2,
    max_steer_deg=20.0, steer_breaks_deg=[10.0, 20.0], speeds=[1.0, 0.7, 0.4])


def scan(y=0.0, yaw_deg=0.0):
    angles = beam_angles()
    return raycast(angles, to_car_frame(corridor(WIDTH), y=y, yaw_deg=yaw_deg)), angles


class TestWallDistance:

    @pytest.mark.parametrize('side', ['left', 'right'])
    def test_parallel_and_centered(self, side):
        dist, alpha = wall_distance(*scan(), side, THETA, 0.0, WINDOW, MAX_DIST)
        assert dist == pytest.approx(1.0, abs=0.01)
        assert alpha == pytest.approx(0.0, abs=0.01)

    def test_distance_changes_with_position(self):
        ranges, angles = scan(y=0.4)  # 왼쪽으로 치우침
        assert wall_distance(ranges, angles, 'left', THETA, 0.0, WINDOW, MAX_DIST)[0] == pytest.approx(0.6, abs=0.01)
        assert wall_distance(ranges, angles, 'right', THETA, 0.0, WINDOW, MAX_DIST)[0] == pytest.approx(1.4, abs=0.01)

    def test_alpha_is_negative_toward_the_wall(self):
        ranges, angles = scan(yaw_deg=10.0)  # 왼쪽 벽을 향해 돌아 있음
        _, alpha_left = wall_distance(ranges, angles, 'left', THETA, 0.0, WINDOW, MAX_DIST)
        _, alpha_right = wall_distance(ranges, angles, 'right', THETA, 0.0, WINDOW, MAX_DIST)
        assert alpha_left == pytest.approx(np.radians(-10.0), abs=0.01)
        assert alpha_right == pytest.approx(np.radians(10.0), abs=0.01)

    def test_perpendicular_distance_ignores_yaw(self):
        # D(t) = b·cosα 는 돌아 있어도 벽까지의 수직 거리다.
        dist, _ = wall_distance(*scan(yaw_deg=10.0), 'left', THETA, 0.0, WINDOW, MAX_DIST)
        assert dist == pytest.approx(1.0, abs=0.01)

    def test_lookahead_projects_into_the_future(self):
        # D(t+1) = 1.0 + 0.5·sin(-10°)
        dist, _ = wall_distance(*scan(yaw_deg=10.0), 'left', THETA, 0.5, WINDOW, MAX_DIST)
        assert dist == pytest.approx(1.0 + 0.5 * np.sin(np.radians(-10.0)), abs=0.01)

    def test_missing_wall_is_nan(self):
        angles = beam_angles()
        ranges = raycast(angles, [[-30.0, -1.0, 30.0, -1.0]])  # 오른쪽 벽만 있음
        dist, alpha = wall_distance(ranges, angles, 'left', THETA, 0.5, WINDOW, MAX_DIST)
        assert np.isnan(dist) and np.isnan(alpha)
        assert not np.isnan(wall_distance(ranges, angles, 'right', THETA, 0.5, WINDOW, MAX_DIST)[0])

    def test_far_wall_is_missing(self):
        # 파이프가 끝나면 NaN이 아니라 멀리 있는 교실 벽이 잡힌다.
        angles = beam_angles()
        ranges = raycast(angles, [[-30.0, 6.0, 30.0, 6.0]])
        dist, alpha = wall_distance(ranges, angles, 'left', THETA, 0.5, WINDOW, MAX_DIST)
        assert np.isnan(dist) and np.isnan(alpha)

    def test_far_side_beam_with_near_front_wall_is_missing(self):
        # 코너: 옆 빔 b는 멀리 보고 앞쪽 빔 a는 앞 벽에 맞는다. 계산하면 짧은 거리가 나온다.
        angles = beam_angles()
        segments = [
            [1.0, -5.0, 1.0, 5.0],     # 앞 벽. 빔 a(왼쪽 40°)는 약 1.3 m에서 맞는다.
            [-30.0, 8.0, 30.0, 8.0],   # 왼쪽 멀리 있는 벽. 빔 b는 8 m에서 맞는다.
        ]
        dist, alpha = wall_distance(
            raycast(angles, segments), angles, 'left', THETA, 0.5, WINDOW, MAX_DIST)
        assert np.isnan(dist) and np.isnan(alpha)

    def test_single_dropout_does_not_lose_the_wall(self):
        ranges, angles = scan()
        ranges[np.argmin(np.abs(angles - np.pi / 2))] = np.nan  # 빔 b
        dist, _ = wall_distance(ranges, angles, 'left', THETA, 0.0, WINDOW, MAX_DIST)
        assert dist == pytest.approx(1.0, abs=0.02)

    def test_does_not_assume_beam_index(self):
        # 시야각이 달라져도(Hokuyo 270°, TEA 300°) 결과가 같아야 한다.
        for fov in (270.0, 300.0):
            angles = beam_angles(fov, 0.25)
            ranges = raycast(angles, corridor(WIDTH))
            dist, _ = wall_distance(ranges, angles, 'left', THETA, 0.0, WINDOW, MAX_DIST)
            assert dist == pytest.approx(1.0, abs=0.01)


class TestCenterError:

    def test_both_walls(self):
        assert center_error(1.5, 0.5, 1.0) == pytest.approx(0.5)

    def test_positive_when_car_is_right_of_center(self):
        assert center_error(1.4, 0.6, 1.0) > 0.0

    def test_only_left_wall(self):
        assert center_error(1.5, np.nan, 1.0) == pytest.approx(0.5)

    def test_only_right_wall(self):
        assert center_error(np.nan, 1.5, 1.0) == pytest.approx(-0.5)

    def test_one_wall_matches_two_walls_at_full_width(self):
        # 벽이 하나 사라지는 순간 오차가 튀면 안 된다.
        both = center_error(1.3, 0.7, 1.0)
        assert center_error(1.3, np.nan, 1.0) == pytest.approx(both)
        assert center_error(np.nan, 0.7, 1.0) == pytest.approx(both)

    def test_no_walls_is_nan(self):
        assert np.isnan(center_error(np.nan, np.nan, 1.0))


class TestPID:

    def test_proportional(self):
        assert PID(2.0, 0.0, 0.0, 1.0).step(0.5, 0.05) == pytest.approx(1.0)

    def test_integral_accumulates(self):
        pid = PID(0.0, 1.0, 0.0, 10.0)
        pid.step(1.0, 0.1)
        assert pid.step(1.0, 0.1) == pytest.approx(0.2)

    def test_integral_is_limited(self):
        pid = PID(0.0, 1.0, 0.0, 0.15)
        for _ in range(10):
            output = pid.step(1.0, 0.1)
        assert output == pytest.approx(0.15)

    def test_derivative(self):
        pid = PID(0.0, 0.0, 1.0, 1.0)
        pid.step(0.0, 0.1)
        assert pid.step(0.5, 0.1) == pytest.approx(5.0)

    def test_zero_dt_does_not_blow_up(self):
        pid = PID(1.0, 1.0, 1.0, 1.0)
        assert pid.step(0.5, 0.0) == pytest.approx(0.5)

    def test_reset_clears_state(self):
        pid = PID(0.0, 1.0, 1.0, 10.0)
        pid.step(1.0, 0.1)
        pid.reset()
        assert pid.step(0.0, 0.1) == pytest.approx(0.0)


class TestSpeedFromSteer:
    TABLE = (np.radians([10.0, 20.0]), [1.0, 0.7, 0.4])

    @pytest.mark.parametrize('degrees, expected', [
        (0.0, 1.0), (9.9, 1.0), (10.0, 0.7), (19.9, 0.7), (20.0, 0.4), (45.0, 0.4)])
    def test_tiers(self, degrees, expected):
        assert speed_from_steer(np.radians(degrees), self.TABLE) == pytest.approx(expected)

    def test_left_and_right_are_the_same(self):
        steer = np.radians(15.0)
        assert speed_from_steer(-steer, self.TABLE) == speed_from_steer(steer, self.TABLE)


class TestFollowWalls:
    """Lab 3 채점 기준의 네 가지 상황."""

    def steer(self, y=0.0, yaw_deg=0.0):
        pid = PID(PARAMS.kp, PARAMS.ki, PARAMS.kd, PARAMS.i_limit)
        return follow_walls(*scan(y, yaw_deg), pid, 0.05, PARAMS)[0]

    def test_too_close_to_right_wall_steers_left(self):
        assert self.steer(y=-0.5) > 0.0

    def test_too_close_to_left_wall_steers_right(self):
        assert self.steer(y=0.5) < 0.0

    def test_turned_toward_left_wall_steers_right(self):
        assert self.steer(yaw_deg=15.0) < 0.0

    def test_turned_toward_right_wall_steers_left(self):
        assert self.steer(yaw_deg=-15.0) > 0.0

    def test_centered_goes_straight(self):
        assert self.steer() == pytest.approx(0.0, abs=0.02)

    def test_steering_is_limited(self):
        pid = PID(100.0, 0.0, 0.0, 1.0)
        steer, _ = follow_walls(*scan(y=-0.8), pid, 0.05, PARAMS)
        assert steer == pytest.approx(np.radians(PARAMS.max_steer_deg))

    def test_one_wall_still_follows(self):
        angles = beam_angles()
        ranges = raycast(angles, [[-30.0, 0.5, 30.0, 0.5]])  # 왼쪽 벽만, half_width보다 가까움
        pid = PID(PARAMS.kp, PARAMS.ki, PARAMS.kd, PARAMS.i_limit)
        assert follow_walls(ranges, angles, pid, 0.05, PARAMS)[0] < 0.0

    def test_far_wall_counts_as_missing(self):
        # 파이프가 끝나면 NaN이 아니라 멀리 있는 교실 벽이 잡힌다. 그쪽으로 꺾으면 안 된다.
        angles = beam_angles()
        ranges = raycast(angles, [[-30.0, 6.0, 30.0, 6.0], [-30.0, -1.0, 30.0, -1.0]])
        pid = PID(PARAMS.kp, PARAMS.ki, PARAMS.kd, PARAMS.i_limit)
        assert follow_walls(ranges, angles, pid, 0.05, PARAMS)[0] == pytest.approx(0.0, abs=0.02)

    @pytest.mark.parametrize('mirror, sign', [(False, 1.0), (True, -1.0)])
    def test_corner_steers_toward_the_missing_wall(self, mirror, sign):
        # 왼쪽 벽이 끝났고 앞에 벽이 있다. 남은 벽만 따라가면 코너 정면으로 직진한다.
        segments = np.array([
            [-5.0, -1.0, 1.5, -1.0],  # 오른쪽(바깥) 벽
            [1.5, -1.0, 1.5, 6.0],    # 앞 벽
            [-5.0, 1.0, -1.0, 1.0],   # 왼쪽(안쪽) 벽, 차보다 뒤에서 끝난다
        ])
        if mirror:
            segments[:, [1, 3]] *= -1.0
        angles = beam_angles()
        pid = PID(PARAMS.kp, PARAMS.ki, PARAMS.kd, PARAMS.i_limit)
        steer, speed = follow_walls(raycast(angles, segments), angles, pid, 0.05, PARAMS)
        assert steer == pytest.approx(sign * np.radians(PARAMS.max_steer_deg))
        assert speed == PARAMS.speeds[-1]

    def test_pipe_gap_without_front_wall_is_not_a_corner(self):
        # Lab 3의 문과 같다. 앞에 벽이 없으면 꺾지 말고 남은 벽을 따라간다.
        angles = beam_angles()
        ranges = raycast(angles, [[-30.0, -1.0, 30.0, -1.0]])
        pid = PID(PARAMS.kp, PARAMS.ki, PARAMS.kd, PARAMS.i_limit)
        steer, _ = follow_walls(ranges, angles, pid, 0.05, PARAMS)
        assert steer == pytest.approx(0.0, abs=0.02)

    def test_no_walls_goes_straight_and_slow(self):
        angles = beam_angles()
        pid = PID(PARAMS.kp, PARAMS.ki, PARAMS.kd, PARAMS.i_limit)
        steer, speed = follow_walls(np.full(angles.size, np.nan), angles, pid, 0.05, PARAMS)
        assert steer == 0.0
        assert speed == PARAMS.speeds[-1]
