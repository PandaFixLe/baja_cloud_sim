"""Pure-Python geometry, planning and control helpers.

This module intentionally depends only on the Python standard library so that
the same planner can run on a cloud workstation and a small vehicle computer.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_to_rpy(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float, float]:
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi * 0.5, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    return roll, pitch, quaternion_to_yaw(x, y, z, w)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def rpy_to_quaternion(
    roll: float,
    pitch: float,
    yaw: float,
) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def world_to_base(point: Point, vehicle: Point, vehicle_yaw: float) -> Point:
    dx = point[0] - vehicle[0]
    dy = point[1] - vehicle[1]
    c = math.cos(vehicle_yaw)
    s = math.sin(vehicle_yaw)
    return c * dx + s * dy, -s * dx + c * dy


def base_to_world(point: Point, vehicle: Point, vehicle_yaw: float) -> Point:
    c = math.cos(vehicle_yaw)
    s = math.sin(vehicle_yaw)
    return (
        vehicle[0] + c * point[0] - s * point[1],
        vehicle[1] + s * point[0] + c * point[1],
    )


def nearest_index(points: Sequence[Dict[str, float]], x: float, y: float, start: int = 0) -> int:
    if not points:
        return 0
    best_index = clamp(int(start), 0, len(points) - 1)
    best_distance = float("inf")
    for index in range(int(best_index), len(points)):
        point = points[index]
        d2 = (point["x"] - x) ** 2 + (point["y"] - y) ** 2
        if d2 < best_distance:
            best_distance = d2
            best_index = index
        elif index > best_index + 40 and d2 > best_distance + 100.0:
            break
    return int(best_index)


def signed_lateral(point: Point, reference: Dict[str, float]) -> float:
    dx = point[0] - reference["x"]
    dy = point[1] - reference["y"]
    return -math.sin(reference["yaw"]) * dx + math.cos(reference["yaw"]) * dy


def frenet_to_world(reference: Dict[str, float], lateral: float) -> Point:
    return (
        reference["x"] - math.sin(reference["yaw"]) * lateral,
        reference["y"] + math.cos(reference["yaw"]) * lateral,
    )


def point_to_oriented_box_clearance(
    point: Point,
    obstacle: Dict[str, float],
    inflate_longitudinal: float = 0.0,
    inflate_lateral: float = 0.0,
) -> float:
    """Signed distance from a point to an oriented, optionally inflated box."""
    dx = point[0] - obstacle["x"]
    dy = point[1] - obstacle["y"]
    c = math.cos(obstacle["yaw"])
    s = math.sin(obstacle["yaw"])
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    half_x = obstacle["length"] * 0.5 + inflate_longitudinal
    half_y = obstacle["width"] * 0.5 + inflate_lateral
    qx = abs(local_x) - half_x
    qy = abs(local_y) - half_y
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside = min(max(qx, qy), 0.0)
    return outside + inside


def segment_is_safe(
    start: Point,
    end: Point,
    obstacles: Sequence[Dict[str, float]],
    vehicle_half_length: float,
    vehicle_half_width: float,
    samples: int = 4,
) -> bool:
    for step in range(samples + 1):
        ratio = step / max(samples, 1)
        point = (
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        )
        for obstacle in obstacles:
            clearance = point_to_oriented_box_clearance(
                point, obstacle, vehicle_half_length, vehicle_half_width
            )
            if clearance <= 0.0:
                return False
    return True


def polyline_distance(point: Point, path: Sequence[Point]) -> float:
    if not path:
        return float("inf")
    if len(path) == 1:
        return distance(point, path[0])
    best = float("inf")
    for index in range(len(path) - 1):
        ax, ay = path[index]
        bx, by = path[index + 1]
        abx, aby = bx - ax, by - ay
        denominator = abx * abx + aby * aby
        if denominator <= 1e-12:
            candidate = math.hypot(point[0] - ax, point[1] - ay)
        else:
            ratio = clamp(((point[0] - ax) * abx + (point[1] - ay) * aby) / denominator, 0.0, 1.0)
            candidate = math.hypot(point[0] - (ax + ratio * abx), point[1] - (ay + ratio * aby))
        best = min(best, candidate)
    return best


def _smooth_hill(
    s_coord: float,
    start: float,
    length: float,
    height: float,
) -> float:
    """Smooth sin-squared hill with zero height and grade at both ends."""
    if s_coord < start or s_coord > start + length:
        return 0.0
    phase = math.pi * (s_coord - start) / length
    return height * math.sin(phase) ** 2


def _circular_speed_bump(
    s_coord: float,
    center: float,
    width: float,
    height: float,
) -> float:
    """Low circular-segment bump with the requested width and crown height."""
    half_width = width * 0.5
    offset = abs(s_coord - center)
    if offset > half_width:
        return 0.0
    radius = (half_width * half_width + height * height) / (2.0 * height)
    baseline = radius - height
    return math.sqrt(max(0.0, radius * radius - offset * offset)) - baseline


def terrain_height(s_coord: float) -> float:
    """Road elevation profile: two sub-10-degree hills and two low bumps."""
    return (
        _smooth_hill(s_coord, start=16.0, length=14.0, height=0.55)
        + _smooth_hill(s_coord, start=76.0, length=13.0, height=0.45)
        + _circular_speed_bump(s_coord, center=39.0, width=1.20, height=0.08)
        + _circular_speed_bump(s_coord, center=92.0, width=1.00, height=0.07)
    )


def generate_centerline(length: float = 100.0, spacing: float = 0.5) -> List[Dict[str, float]]:
    """Generate a long straight, a 90-degree left turn, and a final straight.

    ``s`` is true reference-path distance, so adjacent samples retain the
    requested spacing through the circular bend.
    """
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
            x = s_coord
            y = 0.0
            yaw = 0.0
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
        points.append(
            {
                "s": s_coord,
                "x": x,
                "y": y,
                "z": terrain_height(s_coord),
                "yaw": yaw,
                "half_width": half_width,
            }
        )
    return points


def generate_boundaries(centerline: Sequence[Dict[str, float]]) -> Tuple[List[Point], List[Point]]:
    left: List[Point] = []
    right: List[Point] = []
    for point in centerline:
        left.append(frenet_to_world(point, point["half_width"]))
        right.append(frenet_to_world(point, -point["half_width"]))
    return left, right


def generate_obstacles(
    centerline: Sequence[Dict[str, float]], seed: int, count: int = 5
) -> List[Dict[str, float]]:
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
        obstacles.append(
            {
                "id": obstacle_id,
                "x": x,
                "y": y,
                "z": reference.get("z", 0.0) + height * 0.5 + 0.03,
                "yaw": reference["yaw"] + rng.uniform(-0.35, 0.35),
                "length": rng.uniform(1.3, 2.1),
                "width": rng.uniform(1.0, 1.7),
                "height": height,
                "s": reference["s"],
                "lateral": lateral,
            }
        )
    return obstacles


@dataclass
class PlannerConfig:
    horizon_m: float = 30.0
    layer_spacing_m: float = 1.0
    lateral_spacing_m: float = 0.25
    vehicle_length: float = 2.1
    vehicle_width: float = 1.55
    safety_margin: float = 0.15
    desired_clearance: float = 1.2
    center_weight: float = 1.0
    clearance_weight: float = 12.0
    smooth_weight: float = 7.0
    slope_weight: float = 3.0
    max_lateral_step: float = 0.9


@dataclass
class PlanResult:
    path: List[Point]
    laterals: List[float]
    feasible: bool
    planning_ms: float
    min_clearance: float
    reason: str


def plan_frenet_path(
    centerline: Sequence[Dict[str, float]],
    start_index: int,
    current: Point,
    left_limits: Sequence[float],
    right_limits: Sequence[float],
    obstacles: Sequence[Dict[str, float]],
    config: Optional[PlannerConfig] = None,
) -> PlanResult:
    """Plan a local collision-free path using a layered Frenet lattice."""
    started = time.perf_counter()
    cfg = config or PlannerConfig()
    if not centerline or start_index >= len(centerline) - 2:
        return PlanResult([], [], False, 0.0, 0.0, "centerline unavailable or goal reached")

    center_spacing = max(0.05, centerline[min(start_index + 1, len(centerline) - 1)]["s"] - centerline[start_index]["s"])
    index_step = max(1, int(round(cfg.layer_spacing_m / center_spacing)))
    horizon_points = max(2, int(round(cfg.horizon_m / center_spacing)))
    indices = list(range(start_index + index_step, min(len(centerline), start_index + horizon_points + 1), index_step))
    if not indices:
        return PlanResult([], [], False, 0.0, 0.0, "empty planning horizon")

    start_reference = centerline[start_index]
    start_lateral = signed_lateral(current, start_reference)
    vehicle_half_length = cfg.vehicle_length * 0.5 + cfg.safety_margin
    vehicle_half_width = cfg.vehicle_width * 0.5 + cfg.safety_margin
    layers: List[List[float]] = []
    bounds: List[Tuple[float, float]] = []

    for center_index in indices:
        left = left_limits[center_index] if center_index < len(left_limits) else centerline[center_index]["half_width"]
        right = right_limits[center_index] if center_index < len(right_limits) else -centerline[center_index]["half_width"]
        lower = right + vehicle_half_width
        upper = left - vehicle_half_width
        bounds.append((lower, upper))
        candidates: List[float] = []
        if upper >= lower:
            first = math.ceil(lower / cfg.lateral_spacing_m) * cfg.lateral_spacing_m
            value = first
            while value <= upper + 1e-9:
                candidates.append(round(value, 4))
                value += cfg.lateral_spacing_m
        layers.append(candidates)

    costs: List[List[float]] = []
    parents: List[List[int]] = []
    previous_points: List[Point] = [current]
    previous_laterals: List[float] = [start_lateral]
    previous_costs: List[float] = [0.0]

    for layer_index, candidates in enumerate(layers):
        if not candidates:
            elapsed = (time.perf_counter() - started) * 1000.0
            return PlanResult([], [], False, elapsed, 0.0, "road corridor narrower than vehicle")
        reference = centerline[indices[layer_index]]
        lower, upper = bounds[layer_index]
        current_costs = [float("inf")] * len(candidates)
        current_parents = [-1] * len(candidates)
        current_points = [frenet_to_world(reference, lateral) for lateral in candidates]

        for candidate_index, (lateral, point) in enumerate(zip(candidates, current_points)):
            obstacle_clearance = float("inf")
            collision = False
            for obstacle in obstacles:
                clearance = point_to_oriented_box_clearance(
                    point, obstacle, vehicle_half_length, vehicle_half_width
                )
                obstacle_clearance = min(obstacle_clearance, clearance)
                if clearance <= 0.0:
                    collision = True
                    break
            if collision:
                continue
            boundary_clearance = min(lateral - lower, upper - lateral)
            clearance = min(obstacle_clearance, boundary_clearance)
            clearance_penalty = max(0.0, cfg.desired_clearance - clearance) ** 2
            node_cost = cfg.center_weight * lateral * lateral + cfg.clearance_weight * clearance_penalty

            for previous_index, previous_point in enumerate(previous_points):
                lateral_change = lateral - previous_laterals[previous_index]
                if abs(lateral_change) > cfg.max_lateral_step:
                    continue
                if not segment_is_safe(
                    previous_point,
                    point,
                    obstacles,
                    vehicle_half_length,
                    vehicle_half_width,
                    samples=5,
                ):
                    continue
                transition_cost = cfg.smooth_weight * lateral_change * lateral_change
                if layer_index > 0:
                    transition_cost += cfg.slope_weight * abs(lateral_change)
                total = previous_costs[previous_index] + node_cost + transition_cost
                if total < current_costs[candidate_index]:
                    current_costs[candidate_index] = total
                    current_parents[candidate_index] = previous_index

        if all(not math.isfinite(cost) for cost in current_costs):
            elapsed = (time.perf_counter() - started) * 1000.0
            return PlanResult([], [], False, elapsed, 0.0, "no collision-free lattice connection")
        costs.append(current_costs)
        parents.append(current_parents)
        previous_points = current_points
        previous_laterals = candidates
        previous_costs = current_costs

    last_index = min(range(len(costs[-1])), key=lambda index: costs[-1][index])
    selected: List[int] = [last_index]
    for layer_index in range(len(layers) - 1, 0, -1):
        selected.append(parents[layer_index][selected[-1]])
    selected.reverse()

    laterals = [layers[layer_index][candidate_index] for layer_index, candidate_index in enumerate(selected)]
    path = [current]
    for center_index, lateral in zip(indices, laterals):
        path.append(frenet_to_world(centerline[center_index], lateral))

    minimum = float("inf")
    for point, lateral, bound in zip(path[1:], laterals, bounds):
        minimum = min(minimum, lateral - bound[0], bound[1] - lateral)
        for obstacle in obstacles:
            minimum = min(
                minimum,
                point_to_oriented_box_clearance(point, obstacle, vehicle_half_length, vehicle_half_width),
            )
    elapsed = (time.perf_counter() - started) * 1000.0
    return PlanResult(path, laterals, True, elapsed, max(0.0, minimum), "ok")


@dataclass
class ControllerConfig:
    target_speed: float = 2.5
    lookahead_distance: float = 3.0
    heading_gain: float = 1.2
    max_steering_deg: float = 35.0


def legacy_path_control(
    current: Point,
    yaw_navigation: float,
    path: Sequence[Point],
    config: Optional[ControllerConfig] = None,
) -> Dict[str, float]:
    """Existing heading / lookahead control law using navigation yaw convention."""
    cfg = config or ControllerConfig()
    if len(path) < 2:
        return {"speed": 0.0, "steering": 0.0, "target_x": current[0], "target_y": current[1]}
    nearest = min(range(len(path)), key=lambda index: distance(current, path[index]))
    target_index = len(path) - 1
    for index in range(nearest, len(path)):
        if distance(current, path[index]) >= cfg.lookahead_distance:
            target_index = index
            break
    target = path[target_index]
    east = target[0] - current[0]
    north = target[1] - current[1]
    desired_navigation = math.atan2(east, north)
    error = wrap_angle(desired_navigation - yaw_navigation)
    max_steering = math.radians(cfg.max_steering_deg)
    steering = clamp(-cfg.heading_gain * error, -max_steering, max_steering)
    steering_deg = abs(math.degrees(steering))
    if steering_deg > 30.0:
        speed_factor = 0.50
    elif steering_deg > 20.0:
        speed_factor = 0.70
    elif steering_deg > 12.0:
        speed_factor = 0.85
    elif steering_deg > 6.0:
        speed_factor = 0.95
    else:
        speed_factor = 1.0
    return {
        "speed": cfg.target_speed * speed_factor,
        "steering": steering,
        "target_x": target[0],
        "target_y": target[1],
        "heading_error": error,
    }


# ---------------------------------------------------------------------------
# Stanley + PD + dual-damping + low-pass + rate-limited steering (v1.1)
# ---------------------------------------------------------------------------


@dataclass
class StanleyControllerConfig(ControllerConfig):
    """Stanley controller config; inherits geometry/speed fields from ControllerConfig."""

    kd_heading: float = 0.3
    k_stanley: float = 0.8
    k_cte_dot: float = 0.15
    k_yaw_rate: float = 0.5
    steering_alpha: float = 0.6
    max_steer_rate_deg: float = 8.0
    yaw_rate_alpha: float = 0.7
    yaw_rate_scale: float = 0.3
    min_speed_for_stanley: float = 2.0
    adaptive_steering: bool = True
    adaptive_speed: bool = True
    preview_curv_boost_limit: float = 1.5
    preview_curv_boost_scale: float = 5.0


@dataclass
class StanleyState:
    """Mutable controller state threaded through stanley_path_control."""

    prev_heading_error: float = 0.0
    prev_steering: float = 0.0
    prev_yaw: float = 0.0
    prev_cte: float = 0.0
    yaw_rate_filtered: float = 0.0
    last_nearest: int = 0
    initialized: bool = False


def augment_path_for_cte(path: Sequence[Point]) -> List[Dict[str, float]]:
    """Convert Sequence[Point] to dicts with synthesized math-frame yaw.

    The yaw at each point is the bearing of the outgoing segment (or the previous
    bearing for the last point). Used by stanley_path_control and exposed for tests.
    """
    augmented: List[Dict[str, float]] = []
    for index, point in enumerate(path):
        if index + 1 < len(path):
            dx = path[index + 1][0] - point[0]
            dy = path[index + 1][1] - point[1]
        elif augmented:
            dx = point[0] - augmented[-1]["x"]
            dy = point[1] - augmented[-1]["y"]
        else:
            dx, dy = 1.0, 0.0
        if dx * dx + dy * dy < 1e-12:
            yaw = augmented[-1]["yaw"] if augmented else 0.0
        else:
            yaw = math.atan2(dy, dx)
        augmented.append({"x": point[0], "y": point[1], "yaw": yaw})
    return augmented


def preview_curvature(
    augmented: Sequence[Dict[str, float]],
    start_index: int,
    lookahead_m: float,
) -> float:
    """Return max |Δbearing|/Δs over the next ~lookahead_m of the polyline."""
    if start_index >= len(augmented) - 1:
        return 0.0
    cumulative = 0.0
    max_curv = 0.0
    last_bearing = wrap_angle(augmented[start_index]["yaw"])
    for index in range(start_index + 1, len(augmented)):
        x0 = augmented[index - 1]["x"]
        y0 = augmented[index - 1]["y"]
        x1 = augmented[index]["x"]
        y1 = augmented[index]["y"]
        segment = math.hypot(x1 - x0, y1 - y0)
        if segment <= 1e-6:
            continue
        cumulative += segment
        bearing = wrap_angle(augmented[index]["yaw"])
        delta = abs(wrap_angle(bearing - last_bearing))
        curv = delta / segment
        if curv > max_curv:
            max_curv = curv
        last_bearing = bearing
        if cumulative >= lookahead_m:
            break
    return max_curv


def stanley_path_control(
    current: Point,
    yaw_navigation: float,
    path: Sequence[Point],
    config: Optional[StanleyControllerConfig] = None,
    state: Optional[StanleyState] = None,
    dt: float = 0.05,
) -> Dict[str, object]:
    """Stanley + PD + dual-damping + low-pass + rate-limited steering.

    Returns a dict compatible with legacy_path_control (speed, steering, target_x,
    target_y, heading_error) plus cte, preview_curvature and updated state.
    """
    cfg = config or StanleyControllerConfig()
    if state is None:
        state = StanleyState()

    if len(path) < 2:
        return {
            "speed": 0.0,
            "steering": 0.0,
            "target_x": current[0],
            "target_y": current[1],
            "heading_error": 0.0,
            "cte": 0.0,
            "preview_curvature": 0.0,
            "state": state,
        }

    augmented = augment_path_for_cte(path)
    yaw_math = wrap_angle(math.pi * 0.5 - yaw_navigation)

    nearest = nearest_index(augmented, current[0], current[1], start=state.last_nearest)
    state.last_nearest = max(0, nearest - 4)

    reference = augmented[nearest]
    cte = signed_lateral(current, reference)

    target_index = len(path) - 1
    lookahead = max(0.5, cfg.lookahead_distance)
    for index in range(nearest, len(path)):
        if math.hypot(path[index][0] - current[0], path[index][1] - current[1]) >= lookahead:
            target_index = index
            break
    target = path[target_index]
    east = target[0] - current[0]
    north = target[1] - current[1]
    desired_navigation = math.atan2(east, north)
    heading_error = wrap_angle(desired_navigation - yaw_navigation)

    pc_lookahead = max(3.0, cfg.target_speed * 1.2)
    preview_curv = preview_curvature(augmented, nearest, pc_lookahead)

    if not state.initialized:
        state.prev_heading_error = heading_error
        state.prev_steering = 0.0
        state.prev_yaw = yaw_math
        state.prev_cte = cte
        state.yaw_rate_filtered = 0.0
        state.initialized = True

    d_heading = wrap_angle(heading_error - state.prev_heading_error) / max(dt, 1e-3)
    cte_dot = (cte - state.prev_cte) / max(dt, 1e-3)

    yaw_rate_raw = wrap_angle(yaw_math - state.prev_yaw) / max(dt, 1e-3)
    state.yaw_rate_filtered = (
        cfg.yaw_rate_alpha * state.yaw_rate_filtered
        + (1.0 - cfg.yaw_rate_alpha) * yaw_rate_raw
    )
    yaw_rate_term = cfg.yaw_rate_scale * state.yaw_rate_filtered

    v_safe = max(cfg.target_speed, cfg.min_speed_for_stanley)
    stanley_term = math.atan2(cfg.k_stanley * cte, v_safe)

    if cfg.adaptive_steering:
        scale = 1.0 + min(preview_curv * cfg.preview_curv_boost_scale, cfg.preview_curv_boost_limit)
        max_steer = math.radians(cfg.max_steering_deg) / max(1.0, scale)
    else:
        max_steer = math.radians(cfg.max_steering_deg)

    raw = (
        -cfg.heading_gain * heading_error
        - cfg.kd_heading * d_heading
        - stanley_term
        - cfg.k_cte_dot * cte_dot
        - cfg.k_yaw_rate * yaw_rate_term
    )
    raw_clamped = clamp(raw, -max_steer, max_steer)

    filtered = (
        cfg.steering_alpha * raw_clamped
        + (1.0 - cfg.steering_alpha) * state.prev_steering
    )

    rate_rad = math.radians(cfg.max_steer_rate_deg)
    delta = clamp(filtered - state.prev_steering, -rate_rad, rate_rad)
    steering = state.prev_steering + delta

    state.prev_heading_error = heading_error
    state.prev_steering = steering
    state.prev_yaw = yaw_math
    state.prev_cte = cte

    steering_deg = abs(math.degrees(steering))
    if steering_deg > 30.0:
        speed_factor = 0.50
    elif steering_deg > 20.0:
        speed_factor = 0.70
    elif steering_deg > 12.0:
        speed_factor = 0.85
    elif steering_deg > 6.0:
        speed_factor = 0.95
    else:
        speed_factor = 1.0

    if cfg.adaptive_speed:
        adaptive_scale = 1.0 / (1.0 + preview_curv * 3.0)
        speed_factor *= max(0.30, adaptive_scale)

    return {
        "speed": cfg.target_speed * speed_factor,
        "steering": steering,
        "target_x": target[0],
        "target_y": target[1],
        "heading_error": heading_error,
        "cte": cte,
        "preview_curvature": preview_curv,
        "state": state,
    }
