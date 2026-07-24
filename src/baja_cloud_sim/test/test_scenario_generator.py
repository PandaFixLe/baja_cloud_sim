import re
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
