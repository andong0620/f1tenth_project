"""scan_utils 테스트. normalize_scan, crop_fov는 구현 완료, 클러스터링은 라이다 담당 계약 테스트."""
import numpy as np
import pytest

from perception.core.scan_utils import (
    Cluster, cluster_scan, crop_fov, find_obstacles, normalize_scan)
from perception.core.synth import beam_angles, box, corridor, raycast

not_implemented_yet = pytest.mark.xfail(raises=NotImplementedError)


def value_at(ranges, angles, degrees):
    return ranges[np.argmin(np.abs(angles - np.radians(degrees)))]


class TestNormalizeScan:
    INC = np.radians(1.0)

    def raw_scan(self, angle_deg, distance=2.0):
        """-180° ~ +179° 스캔에서 빔 하나만 물체를 보고 나머지는 드라이버처럼 0.0이다."""
        ranges = np.zeros(360)
        ranges[angle_deg + 180] = distance
        return ranges

    def normalize(self, ranges, yaw_offset=0.0, flip=False):
        return normalize_scan(ranges, -np.pi, self.INC, yaw_offset, flip, 0.05, 25.0)

    def test_zero_from_driver_becomes_nan(self):
        # TEA 드라이버는 무효 빔을 0.0으로 낸다. 0.0을 믿으면 AEB가 즉시 작동한다.
        ranges, _ = self.normalize(self.raw_scan(30))
        assert np.isnan(ranges).sum() == 359

    def test_out_of_range_inf_and_nan_become_nan(self):
        raw = np.array([np.inf, np.nan, 0.01, 30.0, 2.0] + [0.0] * 355)
        ranges, _ = self.normalize(raw)
        assert np.isfinite(ranges).sum() == 1

    def test_identity_keeps_angle(self):
        ranges, angles = self.normalize(self.raw_scan(30))
        assert value_at(ranges, angles, 30) == 2.0

    def test_flip_mirrors_left_and_right(self):
        ranges, angles = self.normalize(self.raw_scan(30), flip=True)
        assert value_at(ranges, angles, -30) == 2.0

    def test_yaw_offset_rotates_counter_clockwise(self):
        ranges, angles = self.normalize(self.raw_scan(30), yaw_offset=np.radians(90))
        assert value_at(ranges, angles, 120) == 2.0

    def test_flip_applies_before_yaw(self):
        ranges, angles = self.normalize(self.raw_scan(30), yaw_offset=np.radians(90), flip=True)
        assert value_at(ranges, angles, 60) == 2.0

    def test_wraps_around_pi(self):
        ranges, angles = self.normalize(self.raw_scan(170), yaw_offset=np.radians(30))
        assert value_at(ranges, angles, -160) == 2.0

    def test_output_grid_is_uniform_even_with_partial_fov(self):
        # 300° 스캔을 90° 돌려도 LaserScan의 angle_min + i × inc 공식이 맞아야 한다.
        raw_angles = beam_angles(300.0, 1.0)
        ranges, angles = normalize_scan(
            np.full(raw_angles.size, 2.0), raw_angles[0], self.INC, np.radians(90), False,
            0.05, 25.0)
        assert ranges.size == angles.size
        assert np.allclose(np.diff(angles), np.diff(angles)[0])
        assert angles[0] == pytest.approx(-np.pi)

    def test_real_tea_resolution_has_a_beam_at_zero(self):
        raw_angles = np.linspace(-np.pi, np.pi, 1501)
        _, angles = normalize_scan(
            np.full(1501, 2.0), -np.pi, raw_angles[1] - raw_angles[0], 0.0, False, 0.05, 25.0)
        assert np.min(np.abs(angles)) < 1e-6


class TestCropFov:

    def test_fov_is_total_width(self):
        angles = np.radians(np.arange(-150, 151, dtype=float))
        ranges = np.ones_like(angles)
        cropped_ranges, cropped_angles = crop_fov(ranges, angles, 180.0)
        assert cropped_ranges.size == cropped_angles.size == 181
        assert np.degrees(cropped_angles[0]) == pytest.approx(-90.0)
        assert np.degrees(cropped_angles[-1]) == pytest.approx(90.0)


def corridor_scan(*extra):
    angles = beam_angles()
    segments = np.vstack([corridor(2.0)] + list(extra))
    return raycast(angles, segments), angles


@not_implemented_yet
class TestClusterScan:
    BREAK_DIST, BREAK_RATIO = 0.1, 0.05

    def clusters(self, ranges, angles):
        return cluster_scan(ranges, angles, self.BREAK_DIST, self.BREAK_RATIO)

    def test_empty_scan_gives_no_clusters(self):
        angles = beam_angles()
        assert self.clusters(np.full(angles.size, np.nan), angles) == []

    def test_single_wall_is_one_cluster(self):
        angles = np.radians(np.arange(20.0, 160.0, 0.24))
        ranges = raycast(angles, [[-30.0, 1.0, 30.0, 1.0]])
        clusters = self.clusters(ranges, angles)
        assert len(clusters) == 1
        assert isinstance(clusters[0], Cluster)

    def test_box_is_separate_from_walls(self):
        ranges, angles = corridor_scan(box(1.5, 0.0, 0.3, 0.3))
        clusters = self.clusters(ranges, angles)
        # 벽의 먼 부분은 점 간격이 넓어 짧은 조각으로 끊길 수 있다. 벽(|y|≈1)은 빼고 본다.
        short = [c for c in clusters
                 if c.length < 1.0 and np.abs(c.points[:, 1]).max() < 0.5]
        assert len(short) == 1
        # 앞에서 보이는 변의 폭이 0.3 m이다.
        assert short[0].length == pytest.approx(0.3, abs=0.05)

    def test_points_are_xy_of_valid_beams(self):
        ranges, angles = corridor_scan(box(1.5, 0.0, 0.3, 0.3))
        for cluster in self.clusters(ranges, angles):
            assert cluster.points.ndim == 2 and cluster.points.shape[1] == 2
            assert not np.isnan(cluster.points).any()
            first = cluster.idx_start
            expected = ranges[first] * np.array([np.cos(angles[first]), np.sin(angles[first])])
            assert cluster.points[0] == pytest.approx(expected)

    def test_nan_beams_do_not_split_a_wall(self):
        angles = np.radians(np.arange(20.0, 160.0, 0.24))
        ranges = raycast(angles, [[-30.0, 1.0, 30.0, 1.0]])
        ranges[::7] = np.nan
        assert len(self.clusters(ranges, angles)) == 1

    def test_far_wall_with_wide_point_spacing_stays_one_cluster(self):
        # 벽을 비스듬히 멀리 보면 점 간격이 넓다. 거리 비례 임계값이 없으면 잘게 끊긴다.
        angles = np.radians(np.arange(5.0, 20.0, 0.24))
        ranges = raycast(angles, [[-30.0, 1.0, 30.0, 1.0]])
        assert len(self.clusters(ranges, angles)) == 1


@not_implemented_yet
class TestFindObstacles:
    PARAMS = dict(corridor_width=0.6, d_obs=2.0, max_obstacle_len=0.8)

    def obstacles(self, ranges, angles):
        return find_obstacles(cluster_scan(ranges, angles, 0.1, 0.05), **self.PARAMS)

    def test_box_ahead_is_an_obstacle(self):
        assert len(self.obstacles(*corridor_scan(box(1.5, 0.0, 0.3, 0.3)))) == 1

    def test_empty_corridor_has_no_obstacle(self):
        assert self.obstacles(*corridor_scan()) == []

    def test_box_beyond_d_obs_is_ignored(self):
        assert self.obstacles(*corridor_scan(box(3.0, 0.0, 0.3, 0.3))) == []

    def test_box_outside_corridor_is_ignored(self):
        assert self.obstacles(*corridor_scan(box(1.5, 0.7, 0.2, 0.2))) == []

    def test_outer_wall_of_a_corner_is_not_an_obstacle(self):
        # mission_plan 4.3: 코너의 바깥 벽을 장애물로 오인하면 안 된다.
        angles = beam_angles()
        outer_wall = [[1.5, -5.0, 1.5, 5.0]]
        assert self.obstacles(raycast(angles, outer_wall), angles) == []
