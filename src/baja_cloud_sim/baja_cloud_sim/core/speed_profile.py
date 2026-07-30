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
    for i in range(N):
        k = max(abs(curvatures[i]), 1e-4)
        v[i] = min(v[i], math.sqrt(cfg.max_lateral_accel / k))
    for i in range(1, N):
        ds = arc_lengths[i] - arc_lengths[i-1]
        if ds <= 0.0:
            continue
        v_limit = math.sqrt(max(0.0, v[i-1]**2 + 2.0 * cfg.max_decel * ds))
        v[i] = min(v[i], v_limit)
    for i in range(N - 2, -1, -1):
        ds = arc_lengths[i+1] - arc_lengths[i]
        if ds <= 0.0:
            continue
        v_limit = math.sqrt(max(0.0, v[i+1]**2 + 2.0 * cfg.max_accel * ds))
        v[i] = min(v[i], v_limit)
    return [float(clamp(vi, cfg.min_speed, cfg.max_speed)) for vi in v]
