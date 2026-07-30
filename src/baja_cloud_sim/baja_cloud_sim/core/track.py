"""Track generation — centreline, boundaries, obstacles, B-spline smoothing."""

from __future__ import annotations

import math
import random
from typing import Dict, List, Tuple

from .geometry import Point, frenet_to_world
from .terrain import terrain_height


def generate_centerline(length: float = 100.0, spacing: float = 0.5) -> List[Dict[str, float]]:
    points: List[Dict[str, float]] = []
    count = int(round(length / spacing)) + 1
    straight_length = min(50.0, length)
    turn_radius = 15.0
    turn_angle = math.pi * 0.5
    turn_length = min(turn_radius * turn_angle, max(0.0, length - straight_length))
    turn_end = straight_length + turn_length
    for index in range(count):
        s_coord = min(length, index * spacing)
        if s_coord <= straight_length:
            x, y, yaw = s_coord, 0.0, 0.0
        elif s_coord <= turn_end:
            angle = (s_coord - straight_length) / turn_radius
            x = straight_length + turn_radius * math.sin(angle)
            y = turn_radius * (1.0 - math.cos(angle))
            yaw = angle
        else:
            final_angle = turn_length / turn_radius
            turn_x = straight_length + turn_radius * math.sin(final_angle)
            turn_y = turn_radius * (1.0 - math.cos(final_angle))
            distance_after_turn = s_coord - turn_end
            x = turn_x + distance_after_turn * math.cos(final_angle)
            y = turn_y + distance_after_turn * math.sin(final_angle)
            yaw = final_angle
        half_width = 4.0 - 0.25 * math.exp(-((s_coord - 66.0) / 9.0) ** 2)
        points.append({"s": s_coord, "x": x, "y": y, "z": terrain_height(s_coord),
                        "yaw": yaw, "half_width": half_width})
    return points


def generate_boundaries(centerline) -> Tuple[List[Point], List[Point]]:
    left, right = [], []
    for point in centerline:
        left.append(frenet_to_world(point, point["half_width"]))
        right.append(frenet_to_world(point, -point["half_width"]))
    return left, right


def generate_obstacles(centerline, seed: int, count: int = 5) -> List[Dict[str, float]]:
    rng = random.Random(seed)
    usable_indices = list(range(30, max(31, len(centerline) - 25)))
    chosen: List[int] = []
    attempts = 0
    while len(chosen) < count and attempts < 500:
        attempts += 1
        index = rng.choice(usable_indices)
        if all(abs(index - previous) >= 22 for previous in chosen):
            chosen.append(index)
    chosen.sort()
    obstacles: List[Dict[str, float]] = []
    for obstacle_id, index in enumerate(chosen):
        reference = centerline[index]
        max_offset = max(0.8, reference["half_width"] - 1.5)
        lateral = rng.uniform(-max_offset, max_offset)
        if obstacle_id == 0:
            lateral *= 0.25
        x, y = frenet_to_world(reference, lateral)
        height = rng.uniform(0.65, 1.0)
        obstacles.append({
            "id": obstacle_id, "x": x, "y": y,
            "z": reference.get("z", 0.0) + height * 0.5 + 0.03,
            "yaw": reference["yaw"] + rng.uniform(-0.35, 0.35),
            "length": rng.uniform(1.3, 2.1), "width": rng.uniform(1.0, 1.7),
            "height": height, "s": reference["s"], "lateral": lateral,
        })
    return obstacles


# ── B-Spline smoothing (Phase 1) ──────────────────────────────────────

def smooth_centerline_c2(raw_points, target_spacing=0.1, smoothing_factor=0.5):
    """C² cubic-spline smoothing + arc-length resampling."""
    if len(raw_points) < 4:
        return [dict(p) for p in raw_points]
    try:
        import numpy as np
        from scipy.interpolate import splprep, splev
    except ImportError:
        return [dict(p) for p in raw_points]
    xs = np.array([p["x"] for p in raw_points], dtype=np.float64)
    ys = np.array([p["y"] for p in raw_points], dtype=np.float64)
    s_val = smoothing_factor * len(raw_points)
    tck, u = splprep([xs, ys], s=s_val, k=3)
    total_arc = _spline_arc_length(tck, 0.0, 1.0, steps=500)
    if total_arc < 1e-6:
        return [dict(p) for p in raw_points]
    num_out = max(2, int(round(total_arc / target_spacing)) + 1)
    u_samples = _resample_by_arc_length(tck, total_arc, num_out)
    x_smooth, y_smooth = splev(u_samples, tck, der=0)
    dx, dy = splev(u_samples, tck, der=1)
    ddx, ddy = splev(u_samples, tck, der=2)
    result: List[Dict[str, float]] = []
    cumulative = 0.0
    for i in range(num_out):
        k = _spline_curvature(dx[i], dy[i], ddx[i], ddy[i])
        yaw = math.atan2(dy[i], dx[i])
        if i > 0:
            cumulative += math.hypot(x_smooth[i] - x_smooth[i-1],
                                     y_smooth[i] - y_smooth[i-1])
        result.append({
            "s": cumulative, "x": float(x_smooth[i]), "y": float(y_smooth[i]),
            "z": _spline_z(tck, u_samples[i], raw_points),
            "yaw": yaw, "kappa": k,
            "half_width": raw_points[min(i, len(raw_points) - 1)].get("half_width", 4.0),
        })
    return result


def _spline_curvature(dx, dy, ddx, ddy):
    denom = max((dx * dx + dy * dy) ** 1.5, 1e-12)
    return (dx * ddy - dy * ddx) / denom


def _spline_arc_length(tck, u_start, u_end, steps=500):
    import numpy as np
    from scipy.interpolate import splev
    us = np.linspace(u_start, u_end, steps + 1)
    xd, yd = splev(us, tck, der=1)
    return float(np.sum(np.sqrt(np.diff(xd)**2 + np.diff(yd)**2)))


def _resample_by_arc_length(tck, total_arc, num_points):
    import numpy as np
    from scipy.interpolate import splev
    target_ds = total_arc / max(num_points - 1, 1)
    u_vals, s_accum, u, du = [0.0], 0.0, 0.0, 0.001
    for _ in range(num_points - 1):
        while u < 1.0:
            xd, yd = splev([u, u + du], tck, der=1)
            ds = math.hypot(xd[1] - xd[0], yd[1] - yd[0]) if du > 0 else 0.0
            s_accum += ds
            u += du
            if s_accum >= target_ds:
                u_vals.append(u); s_accum -= target_ds; break
            if ds > 1e-9:
                du = min(0.01, target_ds / ds * du)
        else:
            break
    if len(u_vals) < num_points:
        u_vals.append(1.0)
    return np.array(u_vals[:num_points], dtype=np.float64)


def _spline_z(tck, u, raw_points):
    from scipy.interpolate import splev
    x, y = splev([u], tck, der=0)
    best = min(raw_points, key=lambda p: (p["x"] - x[0])**2 + (p["y"] - y[0])**2)
    return best.get("z", 0.0)
