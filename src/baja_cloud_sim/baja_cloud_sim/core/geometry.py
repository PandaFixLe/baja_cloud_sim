"""Geometry helpers — pure functions, no ROS/NumPy dependencies."""

from __future__ import annotations

import math
from typing import Dict, Sequence, Tuple

Point = Tuple[float, float]
_WORLD_YAW_OFFSET = math.pi * 0.5  # world 0=East → nav 90°=North


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi * 0.5, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    return roll, pitch, quaternion_to_yaw(x, y, z, w)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def world_to_base(point: Point, vehicle: Point, vehicle_yaw: float) -> Point:
    dx, dy = point[0] - vehicle[0], point[1] - vehicle[1]
    c, s = math.cos(vehicle_yaw), math.sin(vehicle_yaw)
    return c * dx + s * dy, -s * dx + c * dy


def base_to_world(point: Point, vehicle: Point, vehicle_yaw: float) -> Point:
    c, s = math.cos(vehicle_yaw), math.sin(vehicle_yaw)
    return (vehicle[0] + c * point[0] - s * point[1],
            vehicle[1] + s * point[0] + c * point[1])


def nearest_index(points: Sequence[Dict[str, float]], x: float, y: float,
                  start: int = 0) -> int:
    if not points:
        return 0
    best_index = clamp(int(start), 0, len(points) - 1)
    best_distance = float("inf")
    for index in range(int(best_index), len(points)):
        point = points[index]
        d2 = (point["x"] - x) ** 2 + (point["y"] - y) ** 2
        if d2 < best_distance:
            best_distance, best_index = d2, index
        elif index > best_index + 40 and d2 > best_distance + 100.0:
            break
    return int(best_index)


def signed_lateral(point: Point, reference: Dict[str, float]) -> float:
    dx, dy = point[0] - reference["x"], point[1] - reference["y"]
    return -math.sin(reference["yaw"]) * dx + math.cos(reference["yaw"]) * dy


def frenet_to_world(reference: Dict[str, float], lateral: float) -> Point:
    return (reference["x"] - math.sin(reference["yaw"]) * lateral,
            reference["y"] + math.cos(reference["yaw"]) * lateral)


def nav_to_world_yaw(nav_yaw: float) -> float:
    return wrap_angle(_WORLD_YAW_OFFSET - nav_yaw)


def world_to_nav_yaw(world_yaw: float) -> float:
    return wrap_angle(_WORLD_YAW_OFFSET - world_yaw)


def gps_to_local(lat: float, lon: float, origin_lat: float, origin_lon: float) -> Point:
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(origin_lat))
    return ((lon - origin_lon) * meters_per_deg_lon,
            (lat - origin_lat) * 111320.0)
