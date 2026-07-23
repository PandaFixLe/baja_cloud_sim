"""Generate a deterministic 100 m dirt-road world and random obstacles."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory

from .core import generate_boundaries, generate_centerline, generate_obstacles


def _write_obj(path: Path, centerline: Sequence[Dict[str, float]], seed: int) -> None:
    lateral_samples = [(-4.5 + 0.5 * index) for index in range(19)]
    vertices: List[Tuple[float, float, float]] = []
    for center in centerline:
        for lateral in lateral_samples:
            x = center["x"] - math.sin(center["yaw"]) * lateral
            y = center["y"] + math.cos(center["yaw"]) * lateral
            # Low-amplitude deterministic corrugation: enough to excite the chassis
            # without creating unrealistic wheel impacts.
            z = (
                0.012 * math.sin(0.73 * center["s"] + 0.31 * lateral + seed)
                + 0.008 * math.sin(1.41 * center["s"] - 0.57 * lateral)
                + 0.004 * math.sin(3.2 * lateral + 0.13 * seed)
            )
            vertices.append((x, y, z))
    rows = len(centerline)
    columns = len(lateral_samples)
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("# Generated 100 m dirt-road mesh\n")
        handle.write("o dirt_road\n")
        for x, y, z in vertices:
            handle.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        # DART's ODE collision backend requires a normal for every imported
        # mesh vertex. A smooth upward normal is sufficient for this gently
        # corrugated height field and avoids an Assimp/DART zero-normal crash.
        for _ in vertices:
            handle.write("vn 0.000000 0.000000 1.000000\n")
        handle.write("s 1\n")
        for row in range(rows - 1):
            for column in range(columns - 1):
                a = row * columns + column + 1
                b = a + 1
                c = a + columns
                d = c + 1
                handle.write(f"f {a}//{a} {c}//{c} {b}//{b}\n")
                handle.write(f"f {b}//{b} {c}//{c} {d}//{d}\n")


def _obstacle_sdf(obstacle: Dict[str, float]) -> str:
    return f"""
    <model name="obstacle_{obstacle['id']}">
      <static>true</static>
      <pose>{obstacle['x']:.5f} {obstacle['y']:.5f} {obstacle['height'] * 0.5 + 0.03:.5f} 0 0 {obstacle['yaw']:.5f}</pose>
      <link name="body">
        <collision name="collision">
          <geometry><box><size>{obstacle['length']:.4f} {obstacle['width']:.4f} {obstacle['height']:.4f}</size></box></geometry>
          <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{obstacle['length']:.4f} {obstacle['width']:.4f} {obstacle['height']:.4f}</size></box></geometry>
          <material><ambient>0.42 0.12 0.07 1</ambient><diffuse>0.70 0.22 0.10 1</diffuse><pbr><metal><roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr></material>
        </visual>
      </link>
    </model>"""


def generate(
    output: Path,
    seed: int,
    obstacle_count: int,
    package_share: Path,
    obstacles_config: Optional[Path] = None,
    save_obstacles_config: bool = False,
) -> Dict[str, str]:
    output.mkdir(parents=True, exist_ok=True)
    centerline = generate_centerline(100.0, 0.5)
    left, right = generate_boundaries(centerline)

    if obstacles_config is not None:
        with open(obstacles_config, "r", encoding="utf-8") as handle:
            obstacles = json.load(handle)
        for idx, obs in enumerate(obstacles):
            obs.setdefault("id", idx)
            obs.setdefault("z", 0.45)
        print(f"Loaded {len(obstacles)} obstacle(s) from {obstacles_config}", flush=True)
    else:
        obstacles = generate_obstacles(centerline, seed, obstacle_count)

    mesh_path = output / "dirt_road.obj"
    scenario_path = output / "scenario.json"
    world_path = output / "baja_100m.sdf"
    _write_obj(mesh_path, centerline, seed)

    if save_obstacles_config:
        config_path = output / "obstacles_config.json"
        config_data = [
            {
                "x": obs["x"],
                "y": obs["y"],
                "yaw": obs["yaw"],
                "length": obs["length"],
                "width": obs["width"],
                "height": obs["height"],
            }
            for obs in obstacles
        ]
        config_path.write_text(
            json.dumps(config_data, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(f"Saved obstacle config to {config_path}", flush=True)

    scenario = {
        "seed": seed,
        "length": 100.0,
        "spacing": 0.5,
        "gps_origin": {"latitude": 30.0, "longitude": 114.0, "altitude": 30.0},
        "vehicle": {"length": 3.0, "width": 1.5, "wheelbase": 1.43, "mass": 292.0},
        "centerline": centerline,
        "left_boundary": [{"x": point[0], "y": point[1]} for point in left],
        "right_boundary": [{"x": point[0], "y": point[1]} for point in right],
        "obstacles": obstacles,
    }
    scenario_path.write_text(json.dumps(scenario, ensure_ascii=False, indent=2), encoding="utf-8")

    vehicle_uri = (package_share / "models" / "baja_vehicle").as_uri()
    road_uri = mesh_path.resolve().as_uri()
    obstacle_models = "\n".join(_obstacle_sdf(obstacle) for obstacle in obstacles)
    start = centerline[0]
    world = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="baja_track">
    <physics name="high_rate_dynamics" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.3 0.3 0.35 1</background><shadows>false</shadows></scene>
    <light name="sun" type="directional"><cast_shadows>false</cast_shadows><pose>0 0 20 0 0 0</pose><diffuse>0.9 0.86 0.75 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -0.9</direction></light>
    <model name="base_ground"><static>true</static><pose>50 0 -0.18 0 0 0</pose><link name="ground"><collision name="collision"><geometry><box><size>130 40 0.2</size></box></geometry><surface><friction><ode><mu>0.78</mu><mu2>0.72</mu2><slip1>0.015</slip1><slip2>0.025</slip2></ode></friction><contact><ode><kp>600000</kp><kd>180</kd><max_vel>0.1</max_vel><min_depth>0.001</min_depth></ode></contact></surface></collision><visual name="visual"><geometry><box><size>130 40 0.2</size></box></geometry><material><ambient>0.28 0.18 0.10 1</ambient><diffuse>0.42 0.28 0.15 1</diffuse><pbr><metal><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material></visual></link></model>
    <model name="dirt_road"><static>true</static><pose>0 0 0.005 0 0 0</pose><link name="road"><visual name="visual"><geometry><mesh><uri>{road_uri}</uri></mesh></geometry><material><ambient>0.30 0.20 0.11 1</ambient><diffuse>0.46 0.31 0.17 1</diffuse><pbr><metal><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material></visual></link></model>
    {obstacle_models}
    <include><uri>{vehicle_uri}</uri><name>baja_vehicle</name><pose>{start['x']:.5f} {start['y']:.5f} 0.49 0 0 {start['yaw']:.5f}</pose></include>
  </world>
</sdf>
"""
    world_path.write_text(world, encoding="utf-8")
    return {"world": str(world_path), "scenario": str(scenario_path), "mesh": str(mesh_path)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="Generated scenario directory")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--obstacles", type=int, default=5)
    parser.add_argument(
        "--obstacles-config",
        type=Path,
        default=None,
        help="Path to a JSON file with fixed obstacle positions (skips random generation)",
    )
    parser.add_argument(
        "--save-obstacles-config",
        action="store_true",
        default=False,
        help="Write generated obstacle positions to obstacles_config.json in the output dir",
    )
    args = parser.parse_args()
    package_share = Path(get_package_share_directory("baja_cloud_sim"))
    result = generate(
        Path(args.output).resolve(),
        args.seed,
        args.obstacles,
        package_share,
        obstacles_config=args.obstacles_config,
        save_obstacles_config=args.save_obstacles_config,
    )
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
