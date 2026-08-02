"""Track generation — rectangular loop built around the original L-track.

Segment 1 (original L-track, 50 m east + 90 deg left + 50 m north):
  Starts at origin heading east.  Contains the two hills (s=16-30 m,
  s=~88-102 m) and two speed bumps (s=39 m, s=~105 m) from the original
  terrain profile — **the terrain lives here**.

Segments 2-4 (three more straights + three more corners):
  Plain straights (same lengths as original) that close the rectangle
  back to (0,0).  Flat, no obstacles.
"""

from __future__ import annotations

import math
import random
from typing import Dict, List, Tuple

from .geometry import Point, frenet_to_world
from .terrain import terrain_height


# ── Rectangular closed-loop centreline ──────────────────────────────────

def generate_centerline(
    _length: float = 100.0,       # ignored — kept for backward compat
    spacing: float = 0.5,
    straight1: float = 50.0,      # S1 (east)  & S3 (west)
    straight2: float = 50.0,      # S2 (north) & S4 (south)  — original L-track
    turn_radius: float = 15.0,
) -> List[Dict[str, float]]:
    """Closed rectangular loop using the original L-track as segment 1-2.

    Layout:
      S1 (east,  50 m) → T1 (left, 90°) → S2 (north, 50 m)
    → T2 (left, 90°) → S3 (west,  50 m) → T3 (left, 90°)
    → S4 (south, 50 m) → T4 (left, 90°) → back to (0,0) heading east.

    Turn centres (left of each straight):
      C1 = (straight1,  turn_radius)                    = (50, 15)
      C2 = (straight1,  straight2 + turn_radius)        = (50, 65)
      C3 = (0,          straight2 + turn_radius)        = ( 0, 65)
      C4 = (0,          turn_radius)                    = ( 0, 15)
    """
    turn_arc = math.pi * 0.5 * turn_radius
    total = 2.0 * (straight1 + straight2) + 4.0 * turn_arc

    # Segment arc-length boundaries
    s1_end  = straight1
    t1_end  = s1_end + turn_arc
    s2_end  = t1_end + straight2
    t2_end  = s2_end + turn_arc
    s3_end  = t2_end + straight1
    t3_end  = s3_end + turn_arc
    s4_end  = t3_end + straight2
    # t4_end = s4_end + turn_arc  (= total)

    # Turn centres
    C = {
        1: (straight1, turn_radius),                     # after S1
        2: (straight1, straight2 + turn_radius),         # after S2
        3: (0.0,       straight2 + turn_radius),         # after S3
        4: (0.0,       turn_radius),                     # after S4
    }

    # Entry angles (from centre to car at turn entry)
    entry_a = {
        1: -math.pi * 0.5,   # car south of centre  → east→north
        2:  0.0,             # car east  of centre  → north→west
        3:  math.pi * 0.5,   # car north of centre  → west→south
        4:  math.pi,         # car west  of centre  → south→east
    }

    count = int(round(total / spacing)) + 1
    points: List[Dict[str, float]] = []

    for index in range(count):
        s = min(total, index * spacing)

        if s <= s1_end:
            x, y = s, 0.0
            yaw = 0.0
        elif s <= t1_end:
            a = entry_a[1] + (s - s1_end) / turn_radius
            x = C[1][0] + turn_radius * math.cos(a)
            y = C[1][1] + turn_radius * math.sin(a)
            yaw = a + math.pi * 0.5
        elif s <= s2_end:
            d = s - t1_end
            x = C[1][0] + turn_radius
            y = turn_radius + d
            yaw = math.pi * 0.5
        elif s <= t2_end:
            a = entry_a[2] + (s - s2_end) / turn_radius
            x = C[2][0] + turn_radius * math.cos(a)
            y = C[2][1] + turn_radius * math.sin(a)
            yaw = a + math.pi * 0.5
        elif s <= s3_end:
            d = s - t2_end
            x = C[2][0] - d
            y = C[2][1] + turn_radius
            yaw = math.pi
        elif s <= t3_end:
            a = entry_a[3] + (s - s3_end) / turn_radius
            x = C[3][0] + turn_radius * math.cos(a)
            y = C[3][1] + turn_radius * math.sin(a)
            yaw = a + math.pi * 0.5
        elif s <= s4_end:
            d = s - t3_end
            x = C[3][0] - turn_radius
            y = C[3][1] - d
            yaw = math.pi * 1.5
        else:
            a = entry_a[4] + (s - s4_end) / turn_radius
            x = C[4][0] + turn_radius * math.cos(a)
            y = C[4][1] + turn_radius * math.sin(a)
            yaw = a + math.pi * 0.5

        half_width = 4.0
        # Narrower road on original L-track segment (visual only)
        if s <= s2_end:
            half_width -= 0.25 * math.exp(-((s - 66.0) / 9.0) ** 2)

        points.append({"s": s, "x": x, "y": y, "z": terrain_height(s),
                        "yaw": yaw, "half_width": half_width})

    return points


def generate_boundaries(centerline) -> Tuple[List[Point], List[Point]]:
    left, right = [], []
    for point in centerline:
        left.append(frenet_to_world(point, point["half_width"]))
        right.append(frenet_to_world(point, -point["half_width"]))
    return left, right


def generate_obstacles(centerline, seed: int, count: int = 0) -> List[Dict[str, float]]:
    if count <= 0:
        return []
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


# ── B-Spline smoothing ───────────────────────────────────────────────────

def smooth_centerline_c2(raw_points, target_spacing=0.1, smoothing_factor=0.5):
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
