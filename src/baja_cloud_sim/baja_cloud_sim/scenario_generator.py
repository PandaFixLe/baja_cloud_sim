"""Generate a deterministic 100 m dirt-road world and random obstacles."""

from __future__ import annotations

import argparse
import json
import math
import os
import random
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory

from .core import generate_boundaries, generate_centerline, generate_obstacles


def _terrain_features(s: float, lateral: float, seed: int) -> float:
    """Return additional Z offset for terrain: trench potholes + hill."""
    z_extra = 0.0
    rng = random.Random(seed + 777)

    # ---- Section 2: 坑洼带  s = 30–55 m ----
    # Lowered base level with cos-profile dips in the gaps between ridges
    if 30.0 <= s <= 55.0:
        z_extra -= 0.12  # base drop matching ground_s2
        # Gaps between ridges (the actual "potholes")
        ridge_centers = [30.5, 34.5, 38.5, 43.0, 47.5, 51.5]
        gap_zones = [
            (32.0, 1.8),   # (s_center, half_length)
            (36.0, 2.0),
            (40.5, 2.2),
            (45.0, 2.5),
            (49.5, 2.0),
        ]
        for gs, g_half in gap_zones:
            ds_gap = (s - gs) / g_half
            if abs(ds_gap) < 1.0:
                # Smooth depression: cos profile (inverted hill shape)
                z_extra -= 0.10 * math.cos(math.pi * 0.5 * abs(ds_gap))

    # ---- Section 3: 小土坡  s = 60–72 m, 8°, cos profile ----
    if 60.0 <= s <= 72.0:
        hill_len = 12.0
        hill_height = hill_len * math.tan(math.radians(8.0))
        phase = (s - 60.0) / hill_len
        z_extra += hill_height * (1.0 - math.cos(math.pi * phase)) * 0.5

    return z_extra


def _write_obj(path: Path, centerline: Sequence[Dict[str, float]], seed: int) -> None:
    lateral_samples = [(-3.5 + 0.5 * index) for index in range(15)]
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
            # Overlay terrain features (potholes, hill)
            z += _terrain_features(center["s"], lateral, seed)
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


def _nearest_center_index(
    centerline: Sequence[Dict[str, float]], s_target: float
) -> int:
    """Return the centerline index closest to *s_target*."""
    return min(
        range(len(centerline)),
        key=lambda i: abs(centerline[i]["s"] - s_target),
    )


def _add_obstacle(
    obstacles: list,
    ref: Dict[str, float],
    lateral: float,
    yaw_offset: float,
    length: float,
    width: float,
    height: float,
) -> None:
    """Append one obstacle to the list at a Frenet position relative to *ref*."""
    x = ref["x"] - math.sin(ref["yaw"]) * lateral
    y = ref["y"] + math.cos(ref["yaw"]) * lateral
    obs_id = max((o.get("id", 0) for o in obstacles), default=-1) + 1
    obstacles.append({
        "id": obs_id,
        "x": x,
        "y": y,
        "z": 0.45,
        "yaw": ref["yaw"] + yaw_offset,
        "length": length,
        "width": width,
        "height": height,
    })


def _segmented_ground_sdf(centerline: Sequence[Dict[str, float]]) -> str:
    """Generate segmented collision ground: flat → pothole trench → hill.

    A thin safety-net base_ground sits at z=-0.40 to prevent the vehicle
    from falling through any gaps between segments.
    """
    rng = random.Random(42)
    FRIC = ('<ode><mu>0.78</mu><mu2>0.72</mu2>'
            '<slip1>0.015</slip1><slip2>0.025</slip2></ode>')
    CONTACT = ('<ode><kp>600000</kp><kd>180</kd>'
               '<max_vel>0.1</max_vel><min_depth>0.001</min_depth></ode>')
    SURF = f'<surface><friction>{FRIC}</friction><contact>{CONTACT}</contact></surface>'

    def _cl(s_val: float) -> Dict[str, float]:
        return min(centerline, key=lambda p: abs(p["s"] - s_val))

    def _box(name: str, s_mid: float, length: float, z_center: float,
             yaw: float, width: float = 7.0) -> str:
        ref = _cl(s_mid)
        return (
            f'<model name="{name}"><static>true</static>\n'
            f'      <pose>{ref["x"]:.5f} {ref["y"]:.5f} {z_center:.5f} 0 0 {yaw:.5f}</pose>\n'
            f'      <link name="body"><collision name="collision">\n'
            f'      <geometry><box><size>{length:.2f} {width:.1f} 0.2</size></box></geometry>\n'
            f'      {SURF}</collision></link></model>'
        )

    parts: List[str] = []

    # ---- Safety net: thin, low base ground ----
    parts.append(
        '<model name="base_ground"><static>true</static>'
        '<pose>50 0 -0.55 0 0 0</pose>'
        '<link name="body"><collision name="collision">'
        '<geometry><box><size>130 40 0.05</size></box></geometry>'
        f'{SURF}</collision></link></model>'
    )

    # ---- Start pad: wide flat box at s=0 to catch the vehicle ----
    parts.append(
        '<model name="start_pad"><static>true</static>'
        '<pose>1.5 0 -0.10 0 0 0</pose>'
        '<link name="body"><collision name="collision">'
        '<geometry><box><size>5 7 0.2</size></box></geometry>'
        f'{SURF}</collision></link></model>'
    )
    # ---- Section 1: Flat (s=2-30), follows road curve ----
    s1y = _cl(15.0)["yaw"]
    parts.append(_box("ground_s1", 15.0, 30, -0.10, s1y, width=9.0))

    # ---- Section 2: 坑洼带 (s=30-55) ----
    # Strategy: ONE lowered ground strip plus raised "ridges" on top.
    # The vehicle rides on the ridges and drops into the lowered gaps.
    hill_len = 12.0
    hill_height = hill_len * math.tan(math.radians(8.0))
    # Lowered continuous ground (z-top ≈ -0.12, same as flat zone top=0.0 minus 0.12)
    s2y = _cl(42.5)["yaw"]
    parts.append(_box("ground_s2", 42.5, 27, -0.22, s2y))

    # 5 raised ridge strips ON TOP of the lowered zone (same height as flat zone)
    ridge_specs = [
        (30.5, 1.5), (34.5, 1.5), (38.5, 1.5),
        (43.0, 2.0), (47.5, 2.0), (51.5, 1.5),
    ]
    for i, (s_mid, r_len) in enumerate(ridge_specs):
        ref = _cl(s_mid)
        parts.append(
            f'<model name="ridge_{i}"><static>true</static>\n'
            f'      <pose>{ref["x"]:.5f} {ref["y"]:.5f} -0.10 0 0 {ref["yaw"]:.5f}</pose>\n'
            f'      <link name="body"><collision name="collision">\n'
            f'      <geometry><box><size>{r_len:.2f} 2.5 0.15</size></box></geometry>\n'
            f'      <surface><friction><ode><mu>0.78</mu><mu2>0.72</mu2></ode></friction></surface>'
            f'</collision></link></model>'
        )

    # ---- Transition flat (s=55-60) ----
    s2by = _cl(57.5)["yaw"]
    parts.append(_box("ground_s2b", 57.5, 6, -0.10, s2by))

    # ---- Section 3: Hill ramp (s=60-72, 8°, cos profile) ----
    slope_rad = math.atan2(math.tan(math.radians(8.0)) * hill_len * 0.5, hill_len * 0.5)
    hx = (_cl(60.0)["x"] + _cl(72.0)["x"]) * 0.5
    hy = (_cl(60.0)["y"] + _cl(72.0)["y"]) * 0.5
    hyaw = _cl(66.0)["yaw"]
    parts.append(
        f'<model name="hill_ramp"><static>true</static>\n'
        f'      <pose>{hx:.5f} {hy:.5f} {hill_height * 0.25:.5f} 0 {slope_rad:.5f} {hyaw:.5f}</pose>\n'
        f'      <link name="body"><collision name="collision">\n'
        f'      <geometry><box><size>{hill_len:.2f} 7 {hill_height * 0.5:.4f}</size></box></geometry>\n'
        f'      {SURF}</collision></link></model>'
    )

    # ---- Flat after hill (s=72-100), at hill-top height ----
    s4y = _cl(86.0)["yaw"]
    parts.append(_box("ground_s4", 86.0, 29, hill_height - 0.10, s4y))

    return "\n".join(parts)


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
        # Drop the two lateral obstacles placed at s ≈ 18 m (now redundant
        # with the new 4 m-spaced pair in the 避障 section).
        obstacles = [o for o in obstacles if not (17.5 < o.get("x", 0) < 19.0)]
        print(f"Loaded {len(obstacles)} obstacle(s) from {obstacles_config} (filtered)", flush=True)
    else:
        obstacles = generate_obstacles(centerline, seed, obstacle_count)

    # ---- Section 1: two obstacles ~4 m apart (避障区, s ≈ 12 / 16 m) ----
    rng_obs = random.Random(seed + 9001)
    ref_a = centerline[_nearest_center_index(centerline, 12.0)]
    ref_b = centerline[_nearest_center_index(centerline, 16.0)]
    _add_obstacle(obstacles, ref_a, rng_obs.uniform(-2.5, 2.5),
                  rng_obs.uniform(-0.3, 0.3), 1.5, 1.2, 0.75)
    _add_obstacle(obstacles, ref_b, rng_obs.uniform(-2.5, 2.5),
                  rng_obs.uniform(-0.3, 0.3), 1.4, 1.1, 0.80)

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
        "vehicle": {"length": 1.70, "width": 1.50, "wheelbase": 1.43, "mass": 252.0},
        "centerline": centerline,
        "left_boundary": [{"x": point[0], "y": point[1]} for point in left],
        "right_boundary": [{"x": point[0], "y": point[1]} for point in right],
        "obstacles": obstacles,
    }
    scenario_path.write_text(json.dumps(scenario, ensure_ascii=False, indent=2), encoding="utf-8")

    vehicle_uri = (package_share / "models" / "baja_vehicle").as_uri()
    road_uri = mesh_path.resolve().as_uri()
    obstacle_models = "\n".join(_obstacle_sdf(obstacle) for obstacle in obstacles)
    segmented_ground = _segmented_ground_sdf(centerline)
    start = centerline[0]
    world = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="baja_track">
    <physics name="high_rate_dynamics" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.3 0.3 0.35 1</background><shadows>false</shadows></scene>
    <light name="sun" type="directional"><cast_shadows>false</cast_shadows><pose>0 0 20 0 0 0</pose><diffuse>0.9 0.86 0.75 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -0.9</direction></light>
    {segmented_ground}
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
