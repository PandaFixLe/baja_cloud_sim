import math
import os
import tempfile
import unittest

from baja_cloud_sim.core import (
    ControllerConfig,
    PlannerConfig,
    StanleyControllerConfig,
    StanleyState,
    augment_path_for_cte,
    generate_boundaries,
    generate_centerline,
    generate_obstacles,
    legacy_path_control,
    load_algorithm_defaults,
    plan_frenet_path,
    preview_curvature,
    signed_lateral,
    stanley_path_control,
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

    # --- v1.1 Stanley tests ---

    def _straight_path(self):
        return [(float(i), 0.0) for i in range(20)]

    def test_stanley_straight_path_zero_steering(self):
        path = self._straight_path()
        state = StanleyState()
        command = stanley_path_control(
            (0.0, 0.0), math.pi * 0.5, path,
            StanleyControllerConfig(), state, dt=0.05,
        )
        self.assertGreater(command["speed"], 0.0)
        self.assertLessEqual(abs(command["steering"]), math.radians(35.0) + 1e-9)
        self.assertLess(abs(command["cte"]), 1e-9)
        self.assertTrue(command["state"].initialized)

    def test_stanley_cte_sign_corrects(self):
        path = self._straight_path()
        state = StanleyState()
        # Vehicle 0.5 m to the LEFT of the polyline (positive math-frame CTE,
        # since signed_lateral uses left-positive convention). With heading_gain=0
        # the Stanley term dominates: cte > 0 should produce steering < 0
        # (right turn) per the project's compass-bearing sign convention.
        command = stanley_path_control(
            (5.0, 0.5), 0.0, path,
            StanleyControllerConfig(target_speed=2.5, k_stanley=0.8,
                                    heading_gain=0.0),
            state, dt=0.05,
        )
        self.assertGreater(command["cte"], 0.0)
        self.assertLess(command["steering"], 0.0)  # negative = right turn

    def test_stanley_state_persists_across_ticks(self):
        path = self._straight_path()
        state = StanleyState()
        a = stanley_path_control(
            (0.0, 0.0), math.pi * 0.5, path,
            StanleyControllerConfig(), state, dt=0.05,
        )
        b = stanley_path_control(
            (1.0, 0.0), math.pi * 0.5, path,
            StanleyControllerConfig(), a["state"], dt=0.05,
        )
        # Same state object must be threaded through.
        self.assertIs(b["state"], a["state"])
        # Steering rate-limited to max_steer_rate_deg per period.
        self.assertLessEqual(
            abs(b["steering"] - a["steering"]),
            math.radians(8.0) + 1e-9,
        )

    def test_stanley_short_path_returns_zero(self):
        path = [(0.0, 0.0)]
        command = stanley_path_control(
            (0.0, 0.0), 0.0, path,
            StanleyControllerConfig(), StanleyState(), dt=0.05,
        )
        self.assertEqual(command["speed"], 0.0)
        self.assertEqual(command["steering"], 0.0)

    def test_preview_curvature_zero_for_straight(self):
        path = self._straight_path()
        augmented = augment_path_for_cte(path)
        self.assertLess(preview_curvature(augmented, 0, 3.0), 1e-9)

    # --- v1.1-yaml-spec-test: algorithm_defaults loading ---

    def test_from_yaml_full_block_via_dict(self):
        source = {
            "algorithm_defaults": {
                "planner": {"horizon_m": 50.0, "safety_margin": 0.5},
                "controller": {"target_speed": 4.0},
                "stanley_controller": {"k_stanley": 1.2, "target_speed": 3.0},
            }
        }
        planner = PlannerConfig.from_yaml(source)
        controller = ControllerConfig.from_yaml(source)
        stanley = StanleyControllerConfig.from_yaml(source)
        self.assertAlmostEqual(planner.horizon_m, 50.0)
        self.assertAlmostEqual(planner.safety_margin, 0.5)
        # Untouched field keeps dataclass default.
        self.assertAlmostEqual(planner.center_weight, 1.0)
        self.assertAlmostEqual(controller.target_speed, 4.0)
        # Stanley inherits + overrides.
        self.assertAlmostEqual(stanley.target_speed, 3.0)
        self.assertAlmostEqual(stanley.k_stanley, 1.2)
        # Untouched Stanley field keeps its default.
        self.assertAlmostEqual(stanley.kd_heading, 0.3)

    def test_from_yaml_missing_file_falls_back_silently(self):
        # File does not exist: must return defaults, not raise.
        planner = PlannerConfig.from_yaml("/nonexistent/path/params.yaml")
        self.assertAlmostEqual(planner.horizon_m, 30.0)
        self.assertAlmostEqual(planner.safety_margin, 0.25)
        controller = ControllerConfig.from_yaml("/nonexistent/path/params.yaml")
        self.assertAlmostEqual(controller.target_speed, 2.5)
        stanley = StanleyControllerConfig.from_yaml("/nonexistent/path/params.yaml")
        self.assertAlmostEqual(stanley.k_stanley, 0.8)
        self.assertTrue(stanley.adaptive_steering)

    def test_from_yaml_overrides_take_precedence(self):
        source = {
            "algorithm_defaults": {
                "planner": {"horizon_m": 50.0},
            }
        }
        # Override beats yaml; untouched yaml field passes through.
        planner = PlannerConfig.from_yaml(source, horizon_m=80.0, clearance_weight=15.0)
        self.assertAlmostEqual(planner.horizon_m, 80.0)
        self.assertAlmostEqual(planner.clearance_weight, 15.0)

    def test_load_algorithm_defaults_with_real_yaml_file(self):
        yaml_text = (
            "# comment line\n"
            "algorithm_defaults:\n"
            "  planner:\n"
            "    horizon_m: 42.0\n"
            "    safety_margin: 0.33\n"
            "  controller:\n"
            "    target_speed: 3.7\n"
            "  stanley_controller:\n"
            "    k_stanley: 1.1\n"
            "    adaptive_steering: false\n"
        )
        with tempfile.NamedTemporaryFile(
            "w", suffix=".yaml", delete=False, encoding="utf-8"
        ) as handle:
            handle.write(yaml_text)
            path = handle.name
        try:
            defaults = load_algorithm_defaults(path)
            self.assertAlmostEqual(defaults["planner"]["horizon_m"], 42.0)
            self.assertAlmostEqual(defaults["planner"]["safety_margin"], 0.33)
            self.assertAlmostEqual(defaults["controller"]["target_speed"], 3.7)
            self.assertAlmostEqual(defaults["stanley_controller"]["k_stanley"], 1.1)
            self.assertFalse(defaults["stanley_controller"]["adaptive_steering"])
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
