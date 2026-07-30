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
    terrain_height,
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

    def test_centerline_is_straight_then_turns_ninety_degrees(self):
        straight_end = self.centerline[100]
        self.assertAlmostEqual(straight_end["x"], 50.0)
        self.assertAlmostEqual(straight_end["y"], 0.0)
        self.assertAlmostEqual(straight_end["yaw"], 0.0)
        finish = self.centerline[-1]
        self.assertAlmostEqual(finish["x"], 65.0, places=3)
        self.assertGreater(finish["y"], 40.0)
        self.assertAlmostEqual(finish["yaw"], math.pi * 0.5, places=6)

    def test_terrain_contains_hills_and_speed_bumps(self):
        self.assertGreater(terrain_height(23.0), 0.50)
        self.assertGreater(terrain_height(39.0), 0.07)
        self.assertAlmostEqual(terrain_height(45.0), 0.0)
        maximum_hill_grade = 0.55 * math.pi / 14.0
        self.assertLess(math.degrees(math.atan(maximum_hill_grade)), 10.0)

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


if __name__ == "__main__":
    unittest.main()
