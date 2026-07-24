import math
import unittest

from baja_cloud_sim.core import (
    ControllerConfig,
    PlannerConfig,
    generate_boundaries,
    generate_centerline,
    generate_obstacles,
    legacy_path_control,
    plan_frenet_path,
    signed_lateral,
)


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.centerline = generate_centerline(100.0, 0.5)
        left, right = generate_boundaries(self.centerline)
        self.left = [signed_lateral(left[i], self.centerline[i]) for i in range(len(self.centerline))]
        self.right = [signed_lateral(right[i], self.centerline[i]) for i in range(len(self.centerline))]

    def test_centerline_spacing_and_length(self):
        self.assertEqual(len(self.centerline), 201)
        self.assertAlmostEqual(self.centerline[-1]["s"], 100.0)
        for first, second in zip(self.centerline, self.centerline[1:]):
            self.assertAlmostEqual(second["s"] - first["s"], 0.5)

    def test_random_obstacles_are_reproducible(self):
        first = generate_obstacles(self.centerline, 42, 5)
        second = generate_obstacles(self.centerline, 42, 5)
        self.assertEqual(first, second)
        self.assertEqual(len(first), 5)

    def test_planner_and_existing_control_law(self):
        obstacles = generate_obstacles(self.centerline, 42, 5)[:2]
        start = (self.centerline[0]["x"], self.centerline[0]["y"])
        result = plan_frenet_path(
            self.centerline, 0, start, self.left, self.right,
            obstacles, PlannerConfig(horizon_m=30.0),
        )
        self.assertTrue(result.feasible, result.reason)
        self.assertGreater(len(result.path), 20)
        yaw_navigation = math.pi * 0.5 - self.centerline[0]["yaw"]
        command = legacy_path_control(start, yaw_navigation, result.path, ControllerConfig())
        self.assertGreater(command["speed"], 0.0)
        self.assertLessEqual(abs(command["steering"]), math.radians(35.0) + 1e-9)

    def test_terrain_height_is_zero_outside_features(self):
        from baja_cloud_sim.core import terrain_height
        # Outside any hill/bump zones the profile must be exactly zero.
        self.assertEqual(terrain_height(5.0), 0.0)
        self.assertEqual(terrain_height(50.0), 0.0)
        # Inside a known hill the value must be positive and bounded.
        peak = terrain_height(23.0)  # mid of first smooth_hill (s=16..30)
        self.assertGreater(peak, 0.3)
        self.assertLess(peak, 0.6)

    def test_centerline_carries_elevation_and_turn(self):
        # First straight segment must lie on x-axis with zero yaw.
        self.assertAlmostEqual(self.centerline[0]["y"], 0.0, places=6)
        self.assertAlmostEqual(self.centerline[0]["yaw"], 0.0, places=6)
        self.assertIn("z", self.centerline[0])
        # Well into the 90° turn (s = 50..73.56, radius 15 m), the centre
        # line must have non-trivial y and yaw. At s = 70 m, angle = 20/15
        # ≈ 1.333 rad → y ≈ 11.2 m and yaw ≈ 76°.
        late = next(p for p in self.centerline if p["s"] >= 70.0)
        self.assertGreater(late["y"], 5.0)
        self.assertGreater(abs(late["yaw"]), 0.5)
        # After the turn (s ≥ 80 m), yaw must be ≈ π/2.
        after = next(p for p in self.centerline if p["s"] >= 80.0)
        self.assertGreater(after["y"], 15.0)
        self.assertAlmostEqual(abs(after["yaw"]), math.pi * 0.5, places=1)

    def test_segment_is_safe_rejects_obstacle_hit(self):
        """Regression: segment_is_safe must reject paths that pass through
        an obstacle, and accept paths that clearly miss it.

        The obstacle is larger than the vehicle so a corner of the swept
        rectangle always reaches the obstacle's interior at every sample.
        """
        from baja_cloud_sim.core import segment_is_safe
        obstacles = [{
            "id": 0, "x": 5.0, "y": 0.0, "yaw": 0.0,
            "length": 3.0, "width": 2.0, "height": 0.5,
        }]
        # Path crossing the obstacle — must be rejected.
        self.assertFalse(segment_is_safe(
            start=(3.0, 0.0), end=(7.0, 0.0),
            obstacles=obstacles,
            vehicle_half_length=1.0, vehicle_half_width=0.9,
            samples=4,
        ))
        # Path 2.5 m above the obstacle (vehicle edges reach y≈1.6 at the
        # midpoint, so 2.5 m clearance is well outside the box) — accepted.
        self.assertTrue(segment_is_safe(
            start=(3.0, 2.5), end=(7.0, 2.5),
            obstacles=obstacles,
            vehicle_half_length=1.0, vehicle_half_width=0.9,
            samples=4,
        ))


if __name__ == "__main__":
    unittest.main()
