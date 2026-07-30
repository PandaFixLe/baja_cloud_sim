"""Collision detection — oriented-box clearance & segment safety."""

from __future__ import annotations

import math
from typing import Dict, Sequence

from .geometry import Point


def point_to_oriented_box_clearance(
    point: Point,
    obstacle: Dict[str, float],
    inflate_longitudinal: float = 0.0,
    inflate_lateral: float = 0.0,
) -> float:
    """Signed distance from a point to an oriented, optionally inflated box."""
    dx, dy = point[0] - obstacle["x"], point[1] - obstacle["y"]
    c, s = math.cos(obstacle["yaw"]), math.sin(obstacle["yaw"])
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    half_x = obstacle["length"] * 0.5 + inflate_longitudinal
    half_y = obstacle["width"] * 0.5 + inflate_lateral
    qx, qy = abs(local_x) - half_x, abs(local_y) - half_y
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside = min(max(qx, qy), 0.0)
    return outside + inside


def segment_is_safe(
    start: Point,
    end: Point,
    obstacles: Sequence[Dict[str, float]],
    vehicle_half_length: float,
    vehicle_half_width: float,
    samples: int = 2,
) -> bool:
    """Check vehicle-front safety along a segment (3 front-weighted samples)."""
    seg_len = math.hypot(end[0] - start[0], end[1] - start[1])
    if seg_len < 1e-9:
        return all(
            point_to_oriented_box_clearance(start, obs, vehicle_half_length, vehicle_half_width) > 0.0
            for obs in obstacles
        )
    dir_x = (end[0] - start[0]) / seg_len
    dir_y = (end[1] - start[1]) / seg_len
    ratios = [0.0, 0.2, 0.4]
    front_offset = vehicle_half_length
    for ratio in ratios:
        cx = start[0] + dir_x * seg_len * ratio
        cy = start[1] + dir_y * seg_len * ratio
        fx, fy = cx + dir_x * front_offset, cy + dir_y * front_offset
        for obstacle in obstacles:
            if point_to_oriented_box_clearance(
                (fx, fy), obstacle, vehicle_half_length, vehicle_half_width,
            ) <= 0.0:
                return False
    return True


def polyline_distance(point: Point, path: Sequence[Point]) -> float:
    """Minimum perpendicular distance from a point to a polyline."""
    from .geometry import clamp
    if not path:
        return float("inf")
    if len(path) == 1:
        return math.hypot(point[0] - path[0][0], point[1] - path[0][1])
    best = float("inf")
    for idx in range(len(path) - 1):
        ax, ay = path[idx]
        bx, by = path[idx + 1]
        abx, aby = bx - ax, by - ay
        denom = abx * abx + aby * aby
        if denom <= 1e-12:
            d = math.hypot(point[0] - ax, point[1] - ay)
        else:
            t = clamp(((point[0] - ax) * abx + (point[1] - ay) * aby) / denom, 0.0, 1.0)
            d = math.hypot(point[0] - (ax + t * abx), point[1] - (ay + t * aby))
        best = min(best, d)
    return best
