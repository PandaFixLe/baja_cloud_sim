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
    curvature_smooth_window: int = 10
    # 曲率前瞻距离(m): 当前点速度受"前方 lookahead 内最大曲率"约束,
    # 避免车还在弯里就因前方直道κ=0而提前加速(导致弯切直蛇形震荡).
    # 0 = 关闭前瞻(用当前点曲率, 原行为).
    curvature_lookahead_m: float = 3.0


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
    # Keep in sync with config/params.yaml (lqr_Q / lqr_R / lqr_v_norm).
    # The offline harness uses these defaults, so any drift means the offline
    # gate is not testing the tuning that actually runs in the simulator.
    Q: Tuple[float, ...] = (0.05, 8.0, 2.0, 4.0, 0.5)  # [∫e_y, e_y, e_y_dot, e_psi, e_psi_dot] — 5‑element ⇒ LQI
    R: float = 3.0
    v_norm: float = 1.5
    understeer_gradient: float = 0.0
    dt: float = 0.05  # must match the control-loop timer period (path_follower_node._control)
    lqr_min_velocity: float = 0.5
    dare_solve_interval: int = 10
    velocity_recompute_threshold: float = 0.5
    # 方案 G: 反馈项速度自适应软化. 高速时小误差不应激进修, 防止放大成蛇形.
    # beta(v) = 1/(1+alpha*(v-v_ref)), 饱和到 [beta_min, 1.0]. alpha=0 关闭.
    fb_speed_soften_alpha: float = 0.6
    fb_speed_ref: float = 2.5           # 低于此速度 beta=1.0(不软化)
    fb_speed_beta_min: float = 0.4      # beta 下限, 高速反馈弱化至 40% 防残差累积爆发
    # K1: e_psi 移动平均窗口大小(控制周期数). 滤除参考点索引跳变引起的 e_psi 离散阶跃,
    # 这是直道/缓弯指令高频抖动的主要来源. 0=关闭(用原始 e_psi). 窗口 5 ≈ 0.25s 滞后.
    e_psi_ma_window: int = 5


@dataclass
class ControllerConfig:
    target_speed: float = 2.5
    lookahead_distance: float = 3.0
    heading_gain: float = 1.2
    max_steering_deg: float = 26.0


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
