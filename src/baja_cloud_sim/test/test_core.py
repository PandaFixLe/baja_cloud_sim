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
        # After s = 60 m (well past the turn that ends near s ≈ 65 m)
        # the centerline must have non-zero y and yaw.
        late = next(p for p in self.centerline if p["s"] >= 60.0)
        self.assertGreater(late["y"], 5.0)
        self.assertGreater(abs(late["yaw"]), 0.3)

    def test_segment_is_safe_catches_rear_corner_graze(self):
        """Regression: reference regressed this to a single-point test,
        allowing a swept vehicle rectangle to clip an obstacle's rear
        corner even when the centre line clears. The four-corner sweep
        must reject this path."""
        from baja_cloud_sim.core import segment_is_safe
        # Path that arcs around an obstacle: start in front, end behind.
        obstacles = [{
            "id": 0, "x": 5.0, "y": 0.0, "yaw": 0.0,
            "length": 1.0, "width": 1.0, "height": 0.5,
        }]
        # End point sits behind the obstacle but offset laterally so a
        # *point* test passes; the rear corner of the vehicle does not.
        unsafe = segment_is_safe(
            start=(4.0, -1.5), end=(6.0, 1.5),
            obstacles=obstacles,
            vehicle_half_length=1.0, vehicle_half_width=0.9,
            samples=4,
        )
        self.assertFalse(unsafe, "rear-corner graze must be detected")


if __name__ == "__main__":
    unittest.main()
