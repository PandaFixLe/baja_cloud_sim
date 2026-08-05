"""Generate a deterministic 100 m dirt road with a 90-degree turn."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

from .core import generate_boundaries, generate_obstacles
from .core.track import generate_centerline as _loop_centerline
from .core.track import generate_straight_centerline


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
                center.get("z", 0.0)
                + 0.012 * math.sin(0.73 * center["s"] + 0.31 * lateral + seed)
                + 0.008 * math.sin(1.41 * center["s"] - 0.57 * lateral)
                + 0.004 * math.sin(3.2 * lateral + 0.13 * seed)
            )
            vertices.append((x, y, z))
    rows = len(centerline)
    columns = len(lateral_samples)
    normals: List[Tuple[float, float, float]] = []
    for row in range(rows):
        previous_row = max(0, row - 1)
        next_row = min(rows - 1, row + 1)
        for column in range(columns):
            previous_column = max(0, column - 1)
            next_column = min(columns - 1, column + 1)
            before = vertices[previous_row * columns + column]
            after = vertices[next_row * columns + column]
            right = vertices[row * columns + previous_column]
            left = vertices[row * columns + next_column]
            tangent = (
                after[0] - before[0],
                after[1] - before[1],
                after[2] - before[2],
            )
            lateral = (
                left[0] - right[0],
                left[1] - right[1],
                left[2] - right[2],
            )
            nx = tangent[1] * lateral[2] - tangent[2] * lateral[1]
            ny = tangent[2] * lateral[0] - tangent[0] * lateral[2]
            nz = tangent[0] * lateral[1] - tangent[1] * lateral[0]
            magnitude = math.sqrt(nx * nx + ny * ny + nz * nz)
            if magnitude <= 1e-12:
                normals.append((0.0, 0.0, 1.0))
            else:
                if nz < 0.0:
                    nx, ny, nz = -nx, -ny, -nz
                normals.append((nx / magnitude, ny / magnitude, nz / magnitude))
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("# Generated 100 m dirt-road mesh with hills and speed bumps\n")
        handle.write("o dirt_road\n")
        for x, y, z in vertices:
            handle.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        # DART requires one imported normal per vertex. Calculated surface
        # normals also make the graded terrain render correctly.
        for nx, ny, nz in normals:
            handle.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")
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
      <pose>{obstacle['x']:.5f} {obstacle['y']:.5f} {obstacle['z']:.5f} 0 0 {obstacle['yaw']:.5f}</pose>
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


def generate_edge_tires(centerline: Sequence[Dict[str, float]], spacing_m: float = 5.0,
                         offset_m: float = 0.4) -> List[Dict[str, float]]:
    """Place vertical cylinder obstacles (standing tires) along both road edges.

    One tire is dropped every ``spacing_m`` metres of arc length, offset just
    outside the road half-width (``half_width + offset_m``) on the left and
    right boundary. This mirrors real Baja cross-country courses where tires
    line the track to delineate the drivable corridor, and gives the perception
    group a dense, regular landmark field to validate detection against.

    Returns a flat list of tire dicts with world pose + geometry so the SDF
    builder can render them.
    """
    tires: List[Dict[str, float]] = []
    tire_id = 0
    # Walk the centerline by arc length; drop a pair of tires whenever we have
    # advanced ``spacing_m`` past the previous drop location.
    next_drop = 0.0
    for point in centerline:
        s = float(point.get("s", 0.0))
        if s + 1e-6 < next_drop:
            continue
        next_drop += spacing_m
        hw = float(point.get("half_width", 4.0))
        z = float(point.get("z", 0.0))
        yaw = float(point.get("yaw", 0.0))
        for sign in (1.0, -1.0):
            lat = sign * (hw + offset_m)
            x = point["x"] - math.sin(yaw) * lat
            y = point["y"] + math.cos(yaw) * lat
            tires.append({
                "id": tire_id,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "side": "left" if sign > 0 else "right",
            })
            tire_id += 1
    return tires


def _tire_sdf(tire: Dict[str, float]) -> str:
    """SDF for one vertical cylinder ('standing tire')."""
    radius = 0.34
    height = 0.7
    return f"""
    <model name="edge_tire_{tire['id']}">
      <static>true</static>
      <pose>{tire['x']:.5f} {tire['y']:.5f} {tire['z'] + height / 2:.5f} 0 0 {tire['yaw']:.5f}</pose>
      <link name="body">
        <collision name="collision">
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder></geometry>
          <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.10 0.10 0.10 1</diffuse><pbr><metal><roughness>0.8</roughness><metalness>0.0</metalness></metal></pbr></material>
        </visual>
      </link>
    </model>"""


def _extend_runout(centerline: Sequence[Dict[str, float]], runout_m: float) -> List[Dict[str, float]]:
    """Append a plain straight stopping runout after the last centerline point.

    The runout continues along the final segment's tangent (no hills / speed
    bumps) so the vehicle has a clear 20 m zone to decelerate and stop in
    after crossing the finish line.
    """
    if runout_m <= 0.0 or not centerline:
        return list(centerline)
    last = centerline[-1]
    yaw = float(last.get("yaw", 0.0))
    dx, dy = math.cos(yaw), math.sin(yaw)
    step = 0.5
    count = int(round(runout_m / step))
    out: List[Dict[str, float]] = list(centerline)
    s0 = float(last.get("s", 0.0))
    half_width = float(last.get("half_width", 4.0))
    z0 = float(last.get("z", 0.0))
    for i in range(1, count + 1):
        s = s0 + i * step
        out.append({
            "s": s,
            "x": last["x"] + dx * i * step,
            "y": last["y"] + dy * i * step,
            "z": z0,
            "yaw": yaw,
            "half_width": half_width,
        })
    return out


def _build_world_sdf(centerline: Sequence[Dict[str, float]], obstacles: Sequence[Dict[str, float]],
                     mesh_path: Path, package_share: Path, start: Dict[str, float],
                     road_surface: Sequence[Dict[str, float]],
                     tires: Sequence[Dict[str, float]] = ()) -> str:
    """Build the Gazebo world SDF string for a given centerline + obstacles."""
    vehicle_uri = (package_share / "models" / "baja_vehicle").as_uri()
    road_uri = mesh_path.resolve().as_uri()
    obstacle_models = "\n".join(_obstacle_sdf(obstacle) for obstacle in obstacles)
    tire_models = "\n".join(_tire_sdf(tire) for tire in tires)
    minimum_x = min(point["x"] for point in road_surface) - 15.0
    maximum_x = max(point["x"] for point in road_surface) + 15.0
    minimum_y = min(point["y"] for point in road_surface) - 15.0
    maximum_y = max(point["y"] for point in road_surface) + 15.0
    ground_x = 0.5 * (minimum_x + maximum_x)
    ground_y = 0.5 * (minimum_y + maximum_y)
    ground_size_x = maximum_x - minimum_x
    ground_size_y = maximum_y - minimum_y
    ground_z = min(point["z"] for point in road_surface) - 0.24
    return f"""<?xml version="1.0"?>
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
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.67 0.80 0.90 1</background><shadows>true</shadows></scene>
    <light name="sun" type="directional"><cast_shadows>true</cast_shadows><pose>0 0 20 0 0 0</pose><diffuse>0.9 0.86 0.75 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -0.9</direction></light>
    <model name="base_ground"><static>true</static><pose>{ground_x:.4f} {ground_y:.4f} {ground_z:.4f} 0 0 0</pose><link name="ground"><collision name="collision"><geometry><box><size>{ground_size_x:.4f} {ground_size_y:.4f} 0.2</size></box></geometry><surface><friction><ode><mu>1.1</mu><mu2>1.0</mu2></ode></friction></surface></collision><visual name="visual"><geometry><box><size>{ground_size_x:.4f} {ground_size_y:.4f} 0.2</size></box></geometry><material><ambient>0.16 0.25 0.12 1</ambient><diffuse>0.23 0.34 0.16 1</diffuse><pbr><metal><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material></visual></link></model>
    <model name="dirt_road"><static>true</static><link name="road"><collision name="collision"><geometry><mesh><uri>{road_uri}</uri></mesh></geometry><surface><friction><ode><mu>1.25</mu><mu2>1.0</mu2></ode></friction><contact><ode><kp>800000</kp><kd>220</kd><max_vel>0.08</max_vel><min_depth>0.0005</min_depth></ode></contact></surface></collision><visual name="visual"><geometry><mesh><uri>{road_uri}</uri></mesh></geometry><material><ambient>0.30 0.20 0.11 1</ambient><diffuse>0.46 0.31 0.17 1</diffuse><pbr><metal><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material></visual></link></model>
    {obstacle_models}
    {tire_models}
    <include><uri>{vehicle_uri}</uri><name>baja_vehicle</name><pose>{start['x']:.5f} {start['y']:.5f} {start['z'] + 0.52:.5f} 0 0 {start['yaw']:.5f}</pose></include>
  </world>
</sdf>
"""


def generate(output: Path, seed: int, obstacle_count: int, package_share: Path, runout_m: float = 0.0) -> Dict[str, str]:
    """Generate the open 100 m straight course (point-to-point, finite finish).

    Used by the line / 直线障碍 / 单圈计时 events. A plain straight ``runout_m``
    stopping zone is appended after the finish so the vehicle can decelerate
    within 20 m of track past the line.
    """
    output.mkdir(parents=True, exist_ok=True)
    centerline = generate_straight_centerline(100.0, 0.5)
    centerline = _extend_runout(centerline, runout_m)
    road_surface = centerline
    left, right = generate_boundaries(centerline)
    obstacles = generate_obstacles(centerline, seed, obstacle_count)
    tires = generate_edge_tires(centerline, spacing_m=5.0, offset_m=0.4)
    mesh_path = output / "dirt_road.obj"
    scenario_path = output / "scenario.json"
    world_path = output / "baja_100m.sdf"
    _write_obj(mesh_path, road_surface, seed)

    scenario = {
        "seed": seed,
        "length": 100.0,
        "spacing": 0.5,
        "gps_origin": {"latitude": 30.0, "longitude": 114.0, "altitude": 30.0},
        "vehicle": {"length": 3.0, "width": 1.5, "wheelbase": 1.43, "mass": 292.0},
        "road_layout": {
            "initial_straight_m": 50.0,
            "turn_direction": "left",
            "turn_angle_deg": 90.0,
            "turn_radius_m": 15.0,
        },
        "terrain_features": [
            {"type": "smooth_hill", "start_s": 16.0, "length": 14.0, "height": 0.55, "max_grade_deg": 7.1},
            {"type": "circular_speed_bump", "center_s": 39.0, "width": 1.20, "height": 0.08},
            {"type": "smooth_hill", "start_s": 76.0, "length": 13.0, "height": 0.45, "max_grade_deg": 6.3},
            {"type": "circular_speed_bump", "center_s": 92.0, "width": 1.00, "height": 0.07},
        ],
        "centerline": centerline,
        "left_boundary": [{"x": point[0], "y": point[1]} for point in left],
        "right_boundary": [{"x": point[0], "y": point[1]} for point in right],
        "obstacles": obstacles,
        "edge_tires": tires,
        "finish_s": 100.0,
        "runout_m": runout_m,
    }
    scenario_path.write_text(json.dumps(scenario, ensure_ascii=False, indent=2), encoding="utf-8")

    world = _build_world_sdf(centerline, obstacles, mesh_path, package_share, centerline[0], road_surface,
                             tires=tires)
    world_path.write_text(world, encoding="utf-8")
    return {"world": str(world_path), "scenario": str(scenario_path), "mesh": str(mesh_path)}


def generate_loop(output: Path, seed: int, obstacle_count: int, package_share: Path) -> Dict[str, str]:
    """Generate the closed-loop endurance track (rectangular loop via track.py).

    The loop's start/finish is the seam at s=0; the 20 m stopping zone is the
    next stretch of track after crossing the line, so no geometry extension is
    needed here.
    """
    output.mkdir(parents=True, exist_ok=True)
    centerline = _loop_centerline(spacing=0.5)
    road_surface = centerline
    left, right = generate_boundaries(centerline)
    obstacles = generate_obstacles(centerline, seed, obstacle_count)
    tires = generate_edge_tires(centerline, spacing_m=5.0, offset_m=0.4)
    mesh_path = output / "loop_road.obj"
    scenario_path = output / "loop_scenario.json"
    world_path = output / "baja_loop.sdf"
    _write_obj(mesh_path, road_surface, seed)
    total = centerline[-1]["s"] if centerline else 0.0
    scenario = {
        "seed": seed,
        "length": total,
        "spacing": 0.5,
        "gps_origin": {"latitude": 30.0, "longitude": 114.0, "altitude": 30.0},
        "vehicle": {"length": 3.0, "width": 1.5, "wheelbase": 1.43, "mass": 292.0},
        "road_layout": {
            "initial_straight_m": 50.0,
            "turn_direction": "left",
            "turn_angle_deg": 90.0,
            "turn_radius_m": 15.0,
        },
        "terrain_features": [],
        "centerline": centerline,
        "left_boundary": [{"x": point[0], "y": point[1]} for point in left],
        "right_boundary": [{"x": point[0], "y": point[1]} for point in right],
        "obstacles": obstacles,
        "edge_tires": tires,
        "finish_s": 0.0,
        "runout_m": 0.0,
    }
    scenario_path.write_text(json.dumps(scenario, ensure_ascii=False, indent=2), encoding="utf-8")
    world = _build_world_sdf(centerline, obstacles, mesh_path, package_share, centerline[0], road_surface,
                             tires=tires)
    world_path.write_text(world, encoding="utf-8")
    return {"world": str(world_path), "scenario": str(scenario_path), "mesh": str(mesh_path)}


def main() -> None:
    from ament_index_python.packages import get_package_share_directory

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="Generated scenario directory")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--obstacles", type=int, default=5)
    parser.add_argument("--runout_m", type=float, default=0.0,
                        help="Length of plain straight stopping runout appended after the finish line")
    args = parser.parse_args()
    package_share = Path(get_package_share_directory("baja_cloud_sim"))
    result = generate(Path(args.output).resolve(), args.seed, args.obstacles, package_share, args.runout_m)
    print(json.dumps(result, ensure_ascii=False))


def main_loop() -> None:
    from ament_index_python.packages import get_package_share_directory

    parser = argparse.ArgumentParser(description="Generate the closed-loop endurance track")
    parser.add_argument("--output", required=True, help="Generated scenario directory")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--obstacles", type=int, default=5)
    args = parser.parse_args()
    package_share = Path(get_package_share_directory("baja_cloud_sim"))
    result = generate_loop(Path(args.output).resolve(), args.seed, args.obstacles, package_share)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
