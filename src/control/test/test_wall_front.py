"""follow_walls 의 코너 판단에 쓰는 front_distance 테스트. 구현 완료된 함수다."""
import numpy as np
import pytest

from control.core.wall import front_distance
from perception.core.synth import beam_angles, raycast


def test_wall_ahead():
    angles = beam_angles()
    ranges = raycast(angles, [[1.5, -5.0, 1.5, 5.0]])
    assert front_distance(ranges, angles, 3.0) == pytest.approx(1.5, abs=0.01)


def test_nothing_ahead_is_infinite():
    angles = beam_angles()
    assert front_distance(np.full(angles.size, np.nan), angles, 3.0) == np.inf
