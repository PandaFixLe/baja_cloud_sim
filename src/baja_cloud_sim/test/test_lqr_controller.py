"""Tests for lqr_controller.py."""

from __future__ import annotations

import math
import unittest

import numpy as np

from baja_cloud_sim.lqr_controller import (
    LQRConfig,
    LQRController,
    LQRGainTable,
    VehicleState,
    _linearise_bicycle,
    _discretise,
    _solve_dare,
    _clamp,
)
from baja_cloud_sim.trajectory_smoother import (
    TrajectorySmoother,
    TrajectoryTable,
)


class TestLQRMath(unittest.TestCase):
    """Low-level DARE / linearisation / discretisation."""

    def setUp(self):
        self.cfg = LQRConfig()

    def test_linearise_bicycle_shapes(self):
        """A is 4×4, B is 4×1."""
        A, B = _linearise_bicycle(5.0, self.cfg)
        self.assertEqual(A.shape, (4, 4))
        self.assertEqual(B.shape, (4, 1))

    def test_linearise_bicycle_finite(self):
        """All entries should be finite."""
        A, B = _linearise_bicycle(5.0, self.cfg)
        self.assertTrue(np.all(np.isfinite(A)))
        self.assertTrue(np.all(np.isfinite(B)))

    def test_discretise_shapes(self):
        """A_d is 4×4, B_d is 4×1."""
        A, B = _linearise_bicycle(5.0, self.cfg)
        A_d, B_d = _discretise(A, B, 0.05)
        self.assertEqual(A_d.shape, (4, 4))
        self.assertEqual(B_d.shape, (4, 1))

    def test_solve_dare_gain_finite(self):
        """K should be finite for a stable system."""
        A, B = _linearise_bicycle(5.0, self.cfg)
        A_d, B_d = _discretise(A, B, 0.05)
        Q = np.diag(self.cfg.Q_diag)
        R = np.array([[self.cfg.R_scalar]])
        K = _solve_dare(A_d, B_d, Q, R)
        self.assertEqual(K.shape, (1, 4))
        self.assertTrue(np.all(np.isfinite(K)))


class TestLQRGainTable(unittest.TestCase):

    def setUp(self):
        self.cfg = LQRConfig(speed_min=1.0, speed_max=5.0, speed_step=1.0)

    def test_build_table_nonempty(self):
        table = LQRGainTable(self.cfg)
        self.assertGreater(len(table._table), 0)

    def test_lookup_clamp_low(self):
        table = LQRGainTable(self.cfg)
        K_low = table.lookup(0.1)
        self.assertIsNotNone(K_low)
        self.assertEqual(K_low.shape, (1, 4))

    def test_lookup_clamp_high(self):
        table = LQRGainTable(self.cfg)
        K_high = table.lookup(100.0)
        self.assertIsNotNone(K_high)
        self.assertEqual(K_high.shape, (1, 4))

    def test_lookup_interpolation(self):
        """K at 3.0 should be between K at 2.0 and K at 4.0 (by norm)."""
        cfg = LQRConfig(speed_min=1.0, speed_max=5.0, speed_step=1.0)
        table = LQRGainTable(cfg)
        K2 = table.lookup(2.0)
        K3 = table.lookup(3.0)
        K4 = table.lookup(4.0)
        n2 = float(np.linalg.norm(K2))
        n3 = float(np.linalg.norm(K3))
        n4 = float(np.linalg.norm(K4))
        # K3 should lie between K2 and K4 in norm (monotonic)
        self.assertTrue(n2 <= n3 <= n4 or n2 >= n3 >= n4,
                        msg=f"K norms: {n2:.4f} {n3:.4f} {n4:.4f}")


class TestLQRController(unittest.TestCase):

    def setUp(self):
        self.cfg = LQRConfig()
        self.controller = LQRController(self.cfg)
        self.smoother = TrajectorySmoother(wheelbase=1.43)

    def _straight_table(self) -> TrajectoryTable:
        path = [(float(i), 0.0) for i in range(20)]
        table = self.smoother.generate(path, target_speed=6.0)
        assert table is not None
        return table

    def _curved_table(self) -> TrajectoryTable:
        path = []
        for i in range(20):
            angle = i * 0.15
            x = 10.0 * math.sin(angle)
            y = 10.0 * (1.0 - math.cos(angle))
            path.append((x, y))
        table = self.smoother.generate(path, target_speed=6.0)
        assert table is not None
        return table

    def test_compute_straight_no_steering(self):
        """On a straight path, steering should be near zero."""
        table = self._straight_table()
        vehicle = VehicleState(speed=6.0, cte=0.0, heading_error=0.0, yaw_rate=0.0)
        v_cmd, delta, delta_ff, delta_fb = self.controller.compute(
            table, vehicle, target_speed=6.0,
        )
        self.assertAlmostEqual(delta_ff, 0.0, delta=0.02)
        self.assertAlmostEqual(delta_fb, 0.0, delta=0.02)

    def test_compute_curved_nonzero_feedforward(self):
        """On a curved path, feed-forward should be non-zero."""
        table = self._curved_table()
        vehicle = VehicleState(speed=4.0, cte=0.0, heading_error=0.0, yaw_rate=0.0)
        _v, _d, delta_ff, _fb = self.controller.compute(
            table, vehicle, target_speed=6.0,
        )
        self.assertNotEqual(delta_ff, 0.0,
                            msg="Feed-forward steering should be non-zero on curves")

    def test_compute_cte_produces_correcting_feedback(self):
        """Positive CTE (left of path) → negative feedback (steer right)."""
        table = self._straight_table()
        vehicle = VehicleState(speed=6.0, cte=0.5, heading_error=0.0, yaw_rate=0.0)
        _v, _d, _ff, delta_fb = self.controller.compute(
            table, vehicle, target_speed=6.0,
        )
        # With a right-side steering convention (positive = CCW/left in math frame):
        # CTE > 0 (vehicle left of path, need to steer right / negative)
        self.assertLess(delta_fb, 0.0,
                        msg="Positive CTE should produce negative feedback steering")

    def test_feedback_within_limit(self):
        """Feedback should never exceed fb_limit_deg."""
        table = self._straight_table()
        limit = math.radians(self.cfg.fb_limit_deg)
        # Large errors to stress the controller
        vehicle = VehicleState(speed=6.0, cte=2.0, heading_error=0.5, yaw_rate=0.3)
        _v, _d, _ff, delta_fb = self.controller.compute(
            table, vehicle, target_speed=6.0,
        )
        self.assertGreaterEqual(delta_fb, -limit - 1e-9)
        self.assertLessEqual(delta_fb, limit + 1e-9)

    def test_speed_cmd_not_exceed_target(self):
        """Commanded speed should not exceed target_speed."""
        table = self._straight_table()
        vehicle = VehicleState(speed=6.0, cte=0.0, heading_error=0.0, yaw_rate=0.0)
        v_cmd, _d, _ff, _fb = self.controller.compute(
            table, vehicle, target_speed=3.0,
        )
        self.assertLessEqual(v_cmd, 3.0)

    def test_speed_reduced_in_curve(self):
        """Speed should be lower on curved path than target_speed."""
        table = self._curved_table()
        vehicle = VehicleState(speed=4.0, cte=0.0, heading_error=0.0, yaw_rate=0.0)
        v_cmd, _d, _ff, _fb = self.controller.compute(
            table, vehicle, target_speed=6.0,
        )
        # Curved path with R=10m gives κ≈0.1, v_curve≈sqrt(3/0.1)=5.48, accel-limited
        self.assertLess(v_cmd, 6.0 - 0.1,
                        msg="Curve should slow the vehicle below target_speed")

    def test_empty_table_graceful(self):
        """Empty table should return safe defaults."""
        empty = TrajectoryTable(points=[], total_length=0.0)
        vehicle = VehicleState(speed=6.0, cte=0.0, heading_error=0.0, yaw_rate=0.0)
        v_cmd, delta, delta_ff, delta_fb = self.controller.compute(
            empty, vehicle, target_speed=6.0,
        )
        self.assertEqual(v_cmd, 6.0)
        self.assertEqual(delta, 0.0)


if __name__ == "__main__":
    unittest.main()
