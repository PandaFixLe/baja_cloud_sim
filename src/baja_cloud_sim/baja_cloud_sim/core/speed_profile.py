"""Curvature-aware speed profile planner."""

from __future__ import annotations

import math
from typing import List, Optional, Sequence

from .config import SpeedProfileConfig
from .geometry import clamp


def plan_speed_profile(
    curvatures: Sequence[float],
    arc_lengths: Sequence[float],
    cfg: Optional[SpeedProfileConfig] = None,
) -> List[float]:
    if cfg is None:
        cfg = SpeedProfileConfig()
    N = len(curvatures)
    if N == 0:
        return []
    import numpy as np
    v = np.full(N, cfg.max_speed, dtype=np.float64)
    # 曲率前瞻+后视: 当前点速度受 [i-look, i+look] 区间内最大曲率约束.
    # 前瞻(i+look): 提前减速进弯(车还没到弯, 但前方有弯就压速).
    # 后视(i-look): 延后加速出弯(车还在弯里/刚出弯, 后方弯道曲率仍约束速度,
    #   只有车离开弯道 look 距离后才允许速度升高). look 足够小(3m)不影响远处直道.
    # 目的: 防止"车还在弯里(转向δ_ff满弯)就因前方直道κ=0提前加速"导致离心力飙升+LQR跟不上→蛇形.
    look_m = max(0.0, getattr(cfg, "curvature_lookahead_m", 0.0))
    if look_m > 0.0:
        # 预算每个点的前方/后方 lookahead 索引界
        fwd_idx = [0] * N
        bwd_idx = [0] * N
        j_hi = 0
        j_lo = 0
        for i in range(N):
            target_hi = arc_lengths[i] + look_m
            while j_hi < N - 1 and arc_lengths[j_hi] < target_hi:
                j_hi += 1
            fwd_idx[i] = j_hi
            target_lo = arc_lengths[i] - look_m
            while j_lo < i and arc_lengths[j_lo] < target_lo:
                j_lo += 1
            bwd_idx[i] = j_lo
        for i in range(N):
            # 取 [bwd_idx[i], fwd_idx[i]] 区间内最大 |κ|
            k = 0.0
            for jj in range(bwd_idx[i], fwd_idx[i] + 1):
                ak = abs(curvatures[jj])
                if ak > k:
                    k = ak
            k = max(k, 1e-4)
            v[i] = min(v[i], math.sqrt(cfg.max_lateral_accel / k))
    else:
        for i in range(N):
            k = max(abs(curvatures[i]), 1e-4)
            v[i] = min(v[i], math.sqrt(cfg.max_lateral_accel / k))
    # Forward pass — acceleration constraint.
    # Speed can only increase so fast along the path (limited by max_accel).
    # This replaces the old deceleration-forcing forward pass (which used
    # max_decel and caused the speed to collapse toward zero on straights).
    for i in range(1, N):
        ds = arc_lengths[i] - arc_lengths[i-1]
        if ds <= 0.0:
            continue
        v_limit = math.sqrt(max(0.0, v[i-1]**2 + 2.0 * cfg.max_accel * ds))
        v[i] = min(v[i], v_limit)
    # Backward pass — deceleration constraint (in reverse).
    # Low speeds at upcoming curves / bumps propagate rearward so the
    # profile includes the deceleration distance the vehicle actually
    # needs (≈1‑2 m for typical speed drops).
    # Uses the absolute deceleration magnitude so the sqrt term grows
    # (v[i] ≤ sqrt(v[i+1]² + 2·|decel|·ds)).
    for i in range(N - 2, -1, -1):
        ds = arc_lengths[i+1] - arc_lengths[i]
        if ds <= 0.0:
            continue
        v_limit = math.sqrt(max(0.0, v[i+1]**2 + 2.0 * (-cfg.max_decel) * ds))
        v[i] = min(v[i], v_limit)
    return [float(clamp(vi, cfg.min_speed, cfg.max_speed)) for vi in v]
