#!/usr/bin/env python3
"""Run a dependency-free kinematic smoke test before starting Gazebo."""

import argparse
import math
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1] / "src" / "baja_cloud_sim"
sys.path.insert(0, str(PACKAGE))

from baja_cloud_sim.core import (  # noqa: E402
    ControllerConfig, PlannerConfig, clamp, generate_boundaries,
    generate_centerline, generate_obstacles, legacy_path_control,
    nearest_index, plan_frenet_path, point_to_oriented_box_clearance,
    signed_lateral, world_to_base, wrap_angle,
)


def run(seed: int, obstacle_count: int) -> int:
    centerline = generate_centerline(100.0, 0.5)
    left_points, right_points = generate_boundaries(centerline)
    left = [signed_lateral(left_points[i], centerline[i]) for i in range(len(centerline))]
    right = [signed_lateral(right_points[i], centerline[i]) for i in range(len(centerline))]
    obstacles = generate_obstacles(centerline, seed, obstacle_count)
    x, y, yaw = centerline[0]["x"], centerline[0]["y"], centerline[0]["yaw"]
    speed = 0.0
    path = []
    nearest = 0
    collision_count = 0
    collision_active = False
    feasible = False
    dt = 0.05
    steps = int(65.0 / dt)

    for step in range(steps):
        if step % 2 == 0:
            nearest = nearest_index(centerline, x, y, max(0, nearest - 3))
            visible = []
            for obstacle in obstacles:
                local = world_to_base((obstacle["x"], obstacle["y"]), (x, y), yaw)
                if -6.0 <= local[0] <= 34.0:
                    visible.append(obstacle)
            result = plan_frenet_path(centerline, nearest, (x, y), left, right, visible, PlannerConfig())
            feasible, path = result.feasible, result.path
        navigation_yaw = wrap_angle(math.pi * 0.5 - yaw)
        command = legacy_path_control((x, y), navigation_yaw, path, ControllerConfig()) if feasible else {"speed": 0.0, "steering": 0.0}
        speed += clamp(command["speed"] - speed, -2.4 * dt, 2.4 * dt)
        yaw = wrap_angle(yaw + speed * math.tan(command["steering"]) / 1.43 * dt)
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt
        collision = any(point_to_oriented_box_clearance((x, y), obstacle, 1.5, 0.75) <= 0.0 for obstacle in obstacles)
        if collision and not collision_active:
            collision_count += 1
        collision_active = collision
        if nearest >= len(centerline) - 4:
            break

    center_error = abs(signed_lateral((x, y), centerline[nearest]))
    print(f"seed={seed} progress={centerline[nearest]['s']:.1f}/100m collisions={collision_count} center_error={center_error:.2f}m")
    return 0 if centerline[nearest]["s"] >= 97.0 and collision_count == 0 and center_error < 1.0 else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--obstacles", type=int, default=5)
    arguments = parser.parse_args()
    raise SystemExit(run(arguments.seed, arguments.obstacles))
