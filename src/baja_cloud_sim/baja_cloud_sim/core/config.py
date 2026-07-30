"""Dataclass configurations — single source of truth for all parameters."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from .geometry import Point


@dataclass
class PlannerConfig:
    horizon_m: float = 30.0
    layer_spacing_m: float = 1.0
    lateral_spacing_m: float = 0.25
    vehicle_length: float = 3.0
    vehicle_width: float = 1.5
    safety_margin: float = 0.25
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


@dataclass
class SpeedProfileConfig:
    max_speed: float = 5.0
    max_lateral_accel: float = 2.5
    max_accel: float = 2.0
    max_decel: float = -2.5
    min_speed: float = 1.0
    min_speed_obstacle: float = 2.0
    max_jerk: float = 4.0
    curvature_smooth_window: int = 5


@dataclass
class LQRConfig:
    wheelbase: float = 1.43
    mass: float = 292.0
    Iz: float = 190.0
    Cf: float = 35000.0
    Cr: float = 35000.0
    lf: float = 0.715
    lr: float = 0.715
    max_steering: float = 0.6109
    Q: Tuple[float, ...] = (10.0, 0.1, 5.0, 0.05)
    R: float = 1.0
    v_norm: float = 2.5
    understeer_gradient: float = 0.0
    dt: float = 0.02
    lqr_min_velocity: float = 0.5
    dare_solve_interval: int = 10
    velocity_recompute_threshold: float = 0.5


@dataclass
class ControllerConfig:
    target_speed: float = 2.5
    lookahead_distance: float = 3.0
    heading_gain: float = 1.2
    max_steering_deg: float = 35.0


@dataclass
class PlannedTrajectory:
    path: List[Point]
    yaws: List[float]
    curvatures: List[float]
    target_speeds: List[float]
    arc_lengths: List[float]
    feasible: bool
    planning_ms: float = 0.0
    min_clearance: float = 0.0
    reason: str = "ok"

    @staticmethod
    def from_plan_result(result, centerline, speed_cfg=None):
        """Upgrade a legacy PlanResult into a PlannedTrajectory."""
        import math
        if not result.feasible or len(result.path) < 2:
            return PlannedTrajectory(
                path=[], yaws=[], curvatures=[], target_speeds=[], arc_lengths=[],
                feasible=False, planning_ms=result.planning_ms,
                min_clearance=result.min_clearance, reason=result.reason,
            )
        N = len(result.path)
        yaws, curvatures, arc_lengths = [], [], []
        cumulative = 0.0
        for i in range(N):
            if i < N - 1:
                yaw_i = math.atan2(result.path[i+1][1] - result.path[i][1],
                                   result.path[i+1][0] - result.path[i][0])
            elif i > 0:
                yaw_i = yaws[-1]
            else:
                yaw_i = 0.0
            yaws.append(yaw_i)
            if i > 0:
                cumulative += math.hypot(result.path[i][0] - result.path[i-1][0],
                                         result.path[i][1] - result.path[i-1][1])
            arc_lengths.append(cumulative)
        for i in range(N):
            if i > 0 and i < N - 1:
                ds_i = arc_lengths[i] - arc_lengths[i-1]
                k = (yaws[i] - yaws[i-1]) / max(ds_i, 1e-6) if ds_i > 1e-6 else 0.0
            else:
                k = 0.0
            curvatures.append(k)
        win = (speed_cfg.curvature_smooth_window if speed_cfg else 5)
        curvatures = _smooth_list(curvatures, win)
        from .speed_profile import plan_speed_profile
        target_speeds = plan_speed_profile(curvatures, arc_lengths, speed_cfg)
        return PlannedTrajectory(
            path=list(result.path), yaws=yaws, curvatures=curvatures,
            target_speeds=target_speeds, arc_lengths=arc_lengths,
            feasible=True, planning_ms=result.planning_ms,
            min_clearance=result.min_clearance, reason=result.reason,
        )


def _smooth_list(values, window):
    if window <= 1 or len(values) <= 1:
        return list(values)
    half, N, out = window // 2, len(values), []
    for i in range(N):
        lo, hi = max(0, i - half), min(N, i + half + 1)
        out.append(sum(values[lo:hi]) / (hi - lo))
    return out
