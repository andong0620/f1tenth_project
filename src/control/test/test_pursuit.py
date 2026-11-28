"""gap 목표점 → pure pursuit 조향 변환 테스트 (ROS 없이)."""
import numpy as np
import pytest

from control.core.gap import pure_pursuit_steer

L, LX, MAXD = 0.33, 0.27, 1.5


def test_straight_ahead_is_zero():
    assert pure_pursuit_steer(1.0, 0.0, L, LX, MAXD) == pytest.approx(0.0)


def test_sign_follows_rep103():
    # 왼쪽(+각도) 목표 → 왼쪽(+) 조향
    assert pure_pursuit_steer(1.0, np.radians(20), L, LX, MAXD) > 0
    assert pure_pursuit_steer(1.0, np.radians(-20), L, LX, MAXD) < 0


def test_nearer_target_steers_more():
    a = np.radians(20)
    assert abs(pure_pursuit_steer(0.6, a, L, LX, MAXD)) > abs(pure_pursuit_steer(1.4, a, L, LX, MAXD))


def test_far_target_clipped_to_max_dist():
    a = np.radians(15)
    assert pure_pursuit_steer(3.0, a, L, LX, MAXD) == pytest.approx(pure_pursuit_steer(MAXD, a, L, LX, MAXD))


def test_matches_formula():
    r, a = 1.0, np.radians(30)
    x, y = LX + r * np.cos(a), r * np.sin(a)
    assert pure_pursuit_steer(r, a, L, LX, MAXD) == pytest.approx(np.arctan(2 * L * y / (x * x + y * y)))
