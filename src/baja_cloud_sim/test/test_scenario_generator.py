import json
import re
import tempfile
import unittest
from pathlib import Path

from baja_cloud_sim import scenario_generator


class ScenarioGeneratorTests(unittest.TestCase):
    def test_obstacle_sdf_has_no_duplicate_return(self):
        """Regression: _obstacle_sdf previously had two identical return
        statements back-to-back; the second was unreachable dead code."""
        source = Path(scenario_generator.__file__).read_text(encoding="utf-8")
        # Locate the function body and count return statements.
        match = re.search(r"def _obstacle_sdf.*?(?=\ndef |\Z)", source, re.DOTALL)
        self.assertIsNotNone(match, "_obstacle_sdf not found")
        body = match.group(0)
        returns = body.count("    return ")
        self.assertEqual(returns, 1, "_obstacle_sdf must contain exactly one return")

    def test_obstacle_uses_terrain_aware_z(self):
        from baja_cloud_sim.core import generate_centerline, generate_obstacles
        centerline = generate_centerline(100.0, 0.5)
        obstacles = generate_obstacles(centerline, seed=42, count=3)
        for obs in obstacles:
            # Each obstacle must carry a z greater than just height*0.5+0.03
            # because the reference now anchors it to the terrain profile.
            self.assertGreater(obs["z"], obs["height"] * 0.5 + 0.02)

    def test_segmented_collision_ground_present(self):
        """The generated world must retain the segmented collision ground:
        curve-following strips, raised ridges, and the hill ramp."""
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "scenario"
            scenario_generator.generate(
                output=out,
                seed=42,
                obstacle_count=3,
                package_share=out,
            )
            world = (out / "baja_100m.sdf").read_text(encoding="utf-8")
        for name in ("ground_s1", "ground_s2", "ridge_0", "hill_ramp", "ground_s4"):
            self.assertIn(
                f'name="{name}"', world,
                f"segmented collision box {name!r} missing from world SDF",
            )

    def test_obstacles_config_z_reanchored_to_terrain(self):
        """An obstacle loaded from --obstacles-config must have its z
        re-anchored to the terrain profile using its `s`."""
        from baja_cloud_sim.core import generate_centerline
        centerline = generate_centerline(100.0, 0.5)
        s_value = 15.0
        height = 0.60
        ref = next(p for p in centerline if abs(p["s"] - s_value) < 0.5)
        expected_z = ref["z"] + height * 0.5 + 0.03
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "scenario"
            out.mkdir(parents=True, exist_ok=True)
            config = out / "obstacles_config.json"
            config.write_text(json.dumps([
                {"x": ref["x"], "y": ref["y"], "yaw": 0.0, "s": s_value,
                 "length": 1.0, "width": 1.0, "height": height},
            ]), encoding="utf-8")
            scenario_generator.generate(
                output=out,
                seed=42,
                obstacle_count=0,
                package_share=out,
                obstacles_config=config,
            )
            scenario = json.loads((out / "scenario.json").read_text(encoding="utf-8"))
        loaded = next(o for o in scenario["obstacles"] if o.get("s") == s_value)
        self.assertAlmostEqual(loaded["z"], expected_z, places=5)
