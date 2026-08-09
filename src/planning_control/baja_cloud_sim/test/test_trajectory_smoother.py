"""Tests for trajectory_smoother.py."""

from __future__ import annotations

import math
import unittest

from baja_cloud_sim.trajectory_smoother import (
    TrajectorySmoother,
    TrajectoryTable,
    _BezierSegment,
    _curvature,
    _fit_bezier_segment,
    _build_speed_profile,
)


class TestBezierHelpers(unittest.TestCase):
    """Unit tests for Bézier math utilities."""

    def test_straight_bezier_zero_curvature(self):
        """A straight line Bézier should have zero curvature everywhere."""
        pts = [(0.0, 0.0), (3.0, 0.0), (6.0, 0.0), (9.0, 0.0)]
        seg = _fit_bezier_segment(pts)
        for u in [0.0, 0.25, 0.5, 0.75, 1.0]:
            k = _curvature(seg.p0, seg.p1, seg.p2, seg.p3, u)
            self.assertAlmostEqual(k, 0.0, delta=1e-9,
                                   msg=f"Straight Bezier curv≠0 at u={u}")

    def test_curved_bezier_nonzero_curvature(self):
        """A genuinely curved Bézier should have non-zero curvature."""
        # Non-collinear points — form a clear arc
        pts = [(0.0, 0.0), (3.0, 2.0), (4.0, 1.0), (6.0, 3.0)]
        seg = _fit_bezier_segment(pts)
        k_mid = _curvature(seg.p0, seg.p1, seg.p2, seg.p3, 0.5)
        self.assertNotEqual(k_mid, 0.0,
                            msg="Curved Bézier should have non-zero curvature")

    def test_segment_arc_length_positive(self):
        """Arc length of any non-degenerate segment must be > 0."""
        pts = [(0.0, 0.0), (2.0, 1.0), (4.0, 3.0), (6.0, 2.0)]
        seg = _fit_bezier_segment(pts)
        self.assertGreater(seg.arc_length, 0.0)


class TestSpeedProfile(unittest.TestCase):

    def test_straight_full_speed(self):
        """Zero curvature → speed should reach target after sufficient distance."""
        n = 60
        curvatures = [0.0] * n
        speeds = _build_speed_profile(curvatures, ds=0.2, target_speed=6.0,
                                       a_lat_max=3.0, a_accel=2.0, a_decel=2.5)
        # After 60*0.2=12m of straight, speed should be near target
        self.assertGreater(speeds[-1], 5.5)

    def test_tight_curve_speed_limit(self):
        """κ = 0.3 rad/m → v_max = sqrt(3.0 / 0.3) ≈ 3.16 m/s."""
        n = 20
        curvatures = [0.3] * n
        speeds = _build_speed_profile(curvatures, ds=0.15, target_speed=6.0,
                                       a_lat_max=3.0, a_accel=2.0, a_decel=2.5)
        for v in speeds:
            self.assertLessEqual(v, 3.2,
                                 msg=f"Speed {v:.2f} exceeds curvature limit")

    def test_backward_pass_deceleration(self):
        """Curve in middle of path → speed must drop BEFORE the curve."""
        n = 60
        # straight (0-19) → curve (20-39) → straight (40-59)
        curvatures = [0.0] * 20 + [0.4] * 20 + [0.0] * 20
        speeds = _build_speed_profile(curvatures, ds=0.15, target_speed=6.0,
                                       a_lat_max=3.0, a_accel=2.0, a_decel=2.5)
        # Speed before curve (index 18) should be higher than speed in curve (index 25)
        # i.e. backward pass forced deceleration before the turn
        self.assertGreater(speeds[18], speeds[25],
                           msg="Speed should drop AT curve entrance (backward pass)")
        # Speed after curve (index 45) should recover
        self.assertGreater(speeds[45], speeds[25],
                           msg="Speed should recover after curve")


class TestTrajectorySmoother(unittest.TestCase):

    def setUp(self):
        self.smoother = TrajectorySmoother(wheelbase=1.43)

    def test_short_path_returns_none(self):
        """Path with < 4 points → None."""
        result = self.smoother.generate([(0.0, 0.0), (1.0, 0.0)])
        self.assertIsNone(result)

    def test_straight_path_generates_table(self):
        """A straight 20-point path should produce a valid trajectory table."""
        path = [(float(i), 0.0) for i in range(20)]
        table = self.smoother.generate(path, target_speed=6.0)
        self.assertIsNotNone(table)
        self.assertIsInstance(table, TrajectoryTable)
        self.assertGreater(len(table.points), 0)
        self.assertGreater(table.total_length, 0.0)

    def test_straight_path_steering_near_zero(self):
        """On a straight path, steering_ff should be ~0."""
        path = [(float(i), 0.0) for i in range(20)]
        table = self.smoother.generate(path, target_speed=6.0,
                                        num_lookahead_pts=12)
        self.assertIsNotNone(table)
        for pt in table.points:
            self.assertAlmostEqual(pt.steering_ff, 0.0, delta=0.02,
                                   msg=f"steering_ff={pt.steering_ff:.4f} on straight path")

    def test_curved_path_nonzero_steering(self):
        """On a curved path, some steering_ff should be non-zero."""
        # Simple arc: points on a circle of radius 10m
        path = []
        for i in range(20):
            angle = i * 0.15  # ~0.15 rad per step
            x = 10.0 * math.sin(angle)
            y = 10.0 * (1.0 - math.cos(angle))
            path.append((x, y))
        table = self.smoother.generate(path, target_speed=6.0,
                                        num_lookahead_pts=12)
        self.assertIsNotNone(table)
        max_abs_steer = max(abs(pt.steering_ff) for pt in table.points)
        self.assertGreater(max_abs_steer, 0.01,
                           msg="Curved path should have non-zero steering_ff")

    def test_speed_limit_respected(self):
        """All speed_limits should be positive and ≤ target_speed."""
        path = [(float(i), 0.0) for i in range(20)]
        table = self.smoother.generate(path, target_speed=6.0)
        self.assertIsNotNone(table)
        for pt in table.points:
            self.assertGreater(pt.speed_limit, 0.0)
            self.assertLessEqual(pt.speed_limit, 6.0)

    def test_points_are_monotonic_in_arc_length(self):
        """Arc-length s should increase monotonically."""
        path = [(float(i), 0.0) for i in range(20)]
        table = self.smoother.generate(path, target_speed=6.0)
        self.assertIsNotNone(table)
        prev_s = -1.0
        for pt in table.points:
            self.assertGreater(pt.s, prev_s)
            prev_s = pt.s


if __name__ == "__main__":
    unittest.main()
