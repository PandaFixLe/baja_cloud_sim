"""Frenet lattice DP planner."""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Sequence

from .collision import point_to_oriented_box_clearance, segment_is_safe
from .config import PlanResult, PlannerConfig
from .geometry import Point, frenet_to_world, signed_lateral


def plan_frenet_path(
    centerline: Sequence[Dict[str, float]],
    start_index: int,
    current: Point,
    left_limits: Sequence[float],
    right_limits: Sequence[float],
    obstacles: Sequence[Dict[str, float]],
    config: Optional[PlannerConfig] = None,
) -> PlanResult:
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
    bounds: List[tuple] = []

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
                hard_clearance = point_to_oriented_box_clearance(
                    point, obstacle, vehicle_half_length, vehicle_half_width
                )
                if hard_clearance <= 0.0:
                    collision = True
                    break
                soft_clearance = point_to_oriented_box_clearance(
                    point, obstacle, 0.0, vehicle_half_width
                )
                obstacle_clearance = min(obstacle_clearance, soft_clearance)
            if collision:
                continue
            boundary_clearance = min(lateral - lower, upper - lateral)
            clearance = min(obstacle_clearance, boundary_clearance)
            clearance_penalty = 0.5 / max(clearance, 0.02)
            node_cost = cfg.center_weight * lateral * lateral + cfg.clearance_weight * clearance_penalty

            for previous_index, previous_point in enumerate(previous_points):
                lateral_change = lateral - previous_laterals[previous_index]
                hard_limit = cfg.max_lateral_step * 1.67
                if abs(lateral_change) > hard_limit:
                    continue
                overstep = max(0.0, abs(lateral_change) - cfg.max_lateral_step)
                lateral_step_penalty = 20.0 * overstep * overstep
                if not segment_is_safe(
                    previous_point, point, obstacles,
                    vehicle_half_length, vehicle_half_width,
                ):
                    continue
                transition_cost = (
                    cfg.smooth_weight * lateral_change * lateral_change
                    + lateral_step_penalty
                )
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
                point_to_oriented_box_clearance(point, obstacle, 0.0, vehicle_half_width),
            )
    elapsed = (time.perf_counter() - started) * 1000.0
    return PlanResult(path, laterals, True, elapsed, max(0.0, minimum), "ok")
