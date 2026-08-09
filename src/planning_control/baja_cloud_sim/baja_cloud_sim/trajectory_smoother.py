"""Bézier trajectory smoother: path → C² curve → trajectory table.

Intentionally uses only the Python standard library (math only) to stay
compatible with the core.py no-dependency constraint.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

# Shared type alias with core.py
Point = Tuple[float, float]

# Throttle counter for diagnostic prints (shared across all TrajectorySmoother instances)
_fail_count: int = 0


def _throttled_print(msg: str, interval: int = 100) -> None:
    """Print msg only once every `interval` calls (across all failures)."""
    global _fail_count
    _fail_count += 1
    if _fail_count % interval == 1 or _fail_count == 1:
        print(msg) if _fail_count == 1 else print(f"{msg}  [count={_fail_count}]")


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class TrajectoryPoint:
    """One sample along the smoothed trajectory."""
    s: float            # arc-length from table start (m)
    x: float            # world x (m)
    y: float            # world y (m)
    yaw: float          # tangent direction (rad, math frame: 0=+X, CCW+)
    curvature: float    # signed curvature (rad/m), left-positive
    speed_limit: float  # target speed at this point (m/s)
    steering_ff: float  # feed-forward steering (rad), Ackermann: atan2(L*κ, 1.0)
    t: float = 0.0      # relative time offset from table generation (s)


@dataclass
class TrajectoryTable:
    """Pre-computed lookahead trajectory for feed-forward + LQR feedback."""
    points: List[TrajectoryPoint] = field(default_factory=list)
    total_length: float = 0.0
    generated_at: float = 0.0  # ROS time (seconds) when table was generated

    def __bool__(self) -> bool:
        return len(self.points) >= 2

    def __len__(self) -> int:
        return len(self.points)


# ---------------------------------------------------------------------------
# Cubic Bézier helpers  (pure-math, no numpy)
# ---------------------------------------------------------------------------

def _bezier_point(
    p0: Point, p1: Point, p2: Point, p3: Point, u: float,
) -> Tuple[float, float]:
    """Cubic Bézier B(u) = (1-u)³P0 + 3(1-u)²u·P1 + 3(1-u)u²·P2 + u³P3."""
    u1 = 1.0 - u
    u1_2 = u1 * u1
    u1_3 = u1_2 * u1
    u_2 = u * u
    u_3 = u_2 * u
    x = (u1_3 * p0[0] + 3.0 * u1_2 * u * p1[0]
         + 3.0 * u1 * u_2 * p2[0] + u_3 * p3[0])
    y = (u1_3 * p0[1] + 3.0 * u1_2 * u * p1[1]
         + 3.0 * u1 * u_2 * p2[1] + u_3 * p3[1])
    return x, y


def _bezier_deriv1(
    p0: Point, p1: Point, p2: Point, p3: Point, u: float,
) -> Tuple[float, float]:
    """First derivative B'(u) = 3(1-u)²(P1-P0) + 6(1-u)u(P2-P1) + 3u²(P3-P2)."""
    u1 = 1.0 - u
    a = 3.0 * u1 * u1
    b = 6.0 * u1 * u
    c = 3.0 * u * u
    dx = a * (p1[0] - p0[0]) + b * (p2[0] - p1[0]) + c * (p3[0] - p2[0])
    dy = a * (p1[1] - p0[1]) + b * (p2[1] - p1[1]) + c * (p3[1] - p2[1])
    return dx, dy


def _bezier_deriv2(
    p0: Point, p1: Point, p2: Point, p3: Point, u: float,
) -> Tuple[float, float]:
    """Second derivative B''(u) = 6(1-u)(P2-2P1+P0) + 6u(P3-2P2+P1)."""
    u1 = 1.0 - u
    a = 6.0 * u1
    b = 6.0 * u
    dx = a * (p2[0] - 2.0 * p1[0] + p0[0]) + b * (p3[0] - 2.0 * p2[0] + p1[0])
    dy = a * (p2[1] - 2.0 * p1[1] + p0[1]) + b * (p3[1] - 2.0 * p2[1] + p1[1])
    return dx, dy


def _curvature(p0: Point, p1: Point, p2: Point, p3: Point, u: float) -> float:
    """Signed curvature κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)."""
    dx, dy = _bezier_deriv1(p0, p1, p2, p3, u)
    ddx, ddy = _bezier_deriv2(p0, p1, p2, p3, u)
    denom = (dx * dx + dy * dy) ** 1.5
    if denom < 1e-9:
        return 0.0
    return (dx * ddy - dy * ddx) / denom


# ---------------------------------------------------------------------------
# Arc-length reparameterisation
# ---------------------------------------------------------------------------

def _build_arc_table(
    p0: Point, p1: Point, p2: Point, p3: Point,
    n_samples: int = 100,
) -> Tuple[List[float], List[float]]:
    """Build numerical u→s mapping for one Bézier segment.

    Returns (u_table, s_table) where s_table[i] = cumulative arc-length at u_table[i].
    """
    u_table: List[float] = []
    s_table: List[float] = [0.0]
    du = 1.0 / n_samples
    u_table.append(0.0)
    for i in range(1, n_samples + 1):
        u = i * du
        u_table.append(u)
        dx, dy = _bezier_deriv1(p0, p1, p2, p3, u - du * 0.5)  # midpoint approx
        ds = math.hypot(dx, dy) * du
        s_table.append(s_table[-1] + ds)
    return u_table, s_table


def _s_to_u(s_target: float, u_table: List[float], s_table: List[float]) -> float:
    """Inverse map: given arc-length s, return parameter u via linear interpolation."""
    if s_target <= 0.0:
        return 0.0
    if s_target >= s_table[-1]:
        return 1.0
    # Linear search — s_table is monotonic and small (≤100 entries)
    for i in range(len(s_table) - 1):
        if s_table[i] <= s_target <= s_table[i + 1]:
            if s_table[i + 1] - s_table[i] < 1e-12:
                return u_table[i]
            alpha = (s_target - s_table[i]) / (s_table[i + 1] - s_table[i])
            return u_table[i] + alpha * (u_table[i + 1] - u_table[i])
    return 1.0


# ---------------------------------------------------------------------------
# Segment fitting
# ---------------------------------------------------------------------------

@dataclass
class _BezierSegment:
    """One cubic Bézier segment with pre-computed arc-length table."""
    p0: Point
    p1: Point
    p2: Point
    p3: Point
    u_table: List[float] = field(default_factory=list)
    s_table: List[float] = field(default_factory=list)
    arc_length: float = 0.0

    def __post_init__(self) -> None:
        self.u_table, self.s_table = _build_arc_table(
            self.p0, self.p1, self.p2, self.p3,
        )
        self.arc_length = self.s_table[-1]

    def s_to_u(self, s: float) -> float:
        return _s_to_u(s, self.u_table, self.s_table)

    def point_at_s(self, s: float) -> Tuple[float, float]:
        return _bezier_point(self.p0, self.p1, self.p2, self.p3, self.s_to_u(s))

    def yaw_at_s(self, s: float) -> float:
        u = self.s_to_u(s)
        dx, dy = _bezier_deriv1(self.p0, self.p1, self.p2, self.p3, u)
        return math.atan2(dy, dx)

    def curvature_at_s(self, s: float) -> float:
        u = self.s_to_u(s)
        return _curvature(self.p0, self.p1, self.p2, self.p3, u)


def _fit_bezier_segment(pts: List[Point]) -> _BezierSegment:
    """Fit one cubic Bézier to 4 path points with tangent-aware control points.

    p0 = pts[0]  (anchor — passed through)
    p1 = pts[1] + (pts[1] - pts[0]) * tangent_scale  (tangent direction)
    p2 = pts[2] + (pts[2] - pts[3]) * tangent_scale  (tangent direction)
    p3 = pts[3]  (anchor — passed through)
    """
    tangent_scale = 0.30
    p0 = pts[0]
    p3 = pts[3]
    # p1: pull away from p0 along the p0→p1 direction
    p1 = (pts[1][0] + (pts[1][0] - pts[0][0]) * tangent_scale,
          pts[1][1] + (pts[1][1] - pts[0][1]) * tangent_scale)
    # p2: pull away from p3 along the p3→p2 direction (backward from p3)
    p2 = (pts[2][0] + (pts[2][0] - pts[3][0]) * tangent_scale,
          pts[2][1] + (pts[2][1] - pts[3][1]) * tangent_scale)
    return _BezierSegment(p0, p1, p2, p3)


# ---------------------------------------------------------------------------
# Double-pass speed profile
# ---------------------------------------------------------------------------

def _build_speed_profile(
    curvatures: List[float],
    ds: float,
    target_speed: float,
    a_lat_max: float = 3.0,
    a_accel: float = 2.0,
    a_decel: float = 3.5,
) -> List[float]:
    """Forward-backward speed profile respecting lateral-accel and accel/decel limits.

    Phase 1 — curvature limit:  v_curve[i] = sqrt(a_lat_max / max(|κ|, κ_min))
    Phase 2 — forward pass:     enforce acceleration limit
    Phase 3 — backward pass:    enforce deceleration limit (anticipate curves)
    """
    n = len(curvatures)
    kappa_min = 0.01

    # Phase 1: curvature speed ceiling
    v_curve = [target_speed] * n
    for i in range(n):
        k = max(abs(curvatures[i]), kappa_min)
        v_curve[i] = min(target_speed, math.sqrt(a_lat_max / k))

    # Phase 2: forward pass (acceleration constraint)
    v_fwd = [0.0] * n
    v_fwd[0] = min(0.5, v_curve[0])  # start gently
    for i in range(1, n):
        v_accel = math.sqrt(max(0.0, v_fwd[i - 1] ** 2 + 2.0 * a_accel * ds))
        v_fwd[i] = min(v_curve[i], v_accel)

    # Phase 3: backward pass (deceleration anticipation)
    v_bwd = [0.0] * n
    v_bwd[-1] = min(v_fwd[-1], target_speed)
    for i in range(n - 2, -1, -1):
        v_decel = math.sqrt(max(0.0, v_bwd[i + 1] ** 2 + 2.0 * a_decel * ds))
        v_bwd[i] = min(v_fwd[i], v_decel)

    return v_bwd


# ---------------------------------------------------------------------------
# Main public API
# ---------------------------------------------------------------------------

class TrajectorySmoother:
    """Convert a raw path polyline into a smoothed trajectory table.

    Usage::

        smoother = TrajectorySmoother(wheelbase=1.43)
        table = smoother.generate(path_points, target_speed=6.0)
    """

    def __init__(self, wheelbase: float = 1.43) -> None:
        self.wheelbase = wheelbase

    def generate(
        self,
        path: Sequence[Point],
        target_speed: float = 6.0,
        num_lookahead_pts: int = 12,
        segments: int = 3,
        ds_resample: float = 0.15,
        generated_at: float = 0.0,
    ) -> Optional[TrajectoryTable]:
        """Generate a smoothed trajectory table from raw path points.

        Args:
            path: Raw polyline as list of (x, y) tuples.
            target_speed: Desired cruise speed (m/s).
            num_lookahead_pts: Number of leading path points to fit (default 12).
            segments: Number of cubic Bézier segments to fit (default 3).
            ds_resample: Arc-length spacing for output samples (m).

        Returns:
            TrajectoryTable, or None if path is too short.
        """
        n_pts = len(path)
        if n_pts < 4:
            _throttled_print(f"[Smoother] return None: path too short ({n_pts} pts)")
            return None

        # ---- select lookahead window ----
        window = list(path[:min(num_lookahead_pts, n_pts)])
        if len(window) < 4:
            _throttled_print(f"[Smoother] return None: window too short ({len(window)} pts, "
                             f"num_lookahead_pts={num_lookahead_pts})")
            return None

        # ---- partition into overlapping segments ----
        # Example: 12 pts, 3 segs → each seg covers 4 pts, overlap 1
        pts_per_seg = max(4, len(window) // segments)
        overlap = min(1, pts_per_seg // 2)

        bezier_segs: List[_BezierSegment] = []
        seg_start = 0
        while seg_start + 3 < len(window):
            seg_end = min(seg_start + pts_per_seg, len(window))
            if seg_end - seg_start < 4:
                seg_end = min(seg_start + 4, len(window))
            seg_pts = [window[i] for i in range(seg_start, seg_end)]
            if len(seg_pts) >= 4:
                bezier_segs.append(_fit_bezier_segment(seg_pts))
            seg_start = seg_end - overlap
            if seg_start >= len(window) - 3:
                break

        if not bezier_segs:
            _throttled_print(f"[Smoother] return None: no Bézier segments "
                             f"(window={len(window)}, segs={segments})")
            return None

        # ---- resample uniformly in arc-length ----
        total_arc = sum(seg.arc_length for seg in bezier_segs)
        if total_arc < ds_resample:
            _throttled_print(f"[Smoother] return None: total_arc={total_arc:.4f} < "
                             f"ds_resample={ds_resample}")
            return None

        # Build cumulative arc-length boundaries for each segment
        seg_boundaries: List[float] = [0.0]
        for seg in bezier_segs:
            seg_boundaries.append(seg_boundaries[-1] + seg.arc_length)

        table_points: List[TrajectoryPoint] = []
        curvatures: List[float] = []

        s = 0.0
        while s <= total_arc + 1e-9:
            # Find which segment this s belongs to
            seg_idx = 0
            local_s = s
            for i in range(len(bezier_segs)):
                if s < seg_boundaries[i + 1] or i == len(bezier_segs) - 1:
                    seg_idx = i
                    local_s = s - seg_boundaries[i]
                    break

            seg = bezier_segs[seg_idx]
            x, y = seg.point_at_s(local_s)
            yaw = seg.yaw_at_s(local_s)
            kappa = seg.curvature_at_s(local_s)
            steering_ff = math.atan2(self.wheelbase * kappa, 1.0)

            table_points.append(TrajectoryPoint(
                s=s,
                x=x, y=y,
                yaw=yaw,
                curvature=kappa,
                speed_limit=0.0,   # filled by speed profile below
                steering_ff=steering_ff,
            ))
            curvatures.append(kappa)
            s += ds_resample

        # ---- speed profile ----
        speeds = _build_speed_profile(curvatures, ds_resample, target_speed)
        for i, pt in enumerate(table_points):
            pt.speed_limit = speeds[i]

        # ---- time stamps:  t[i] = t[i-1] + Δs / v_avg  ----
        t = 0.0
        for i, pt in enumerate(table_points):
            pt.t = t
            if i < len(table_points) - 1:
                v_avg = max((speeds[i] + speeds[i + 1]) * 0.5, 0.1)
                t += ds_resample / v_avg

        return TrajectoryTable(
            points=table_points,
            total_length=total_arc,
            generated_at=generated_at,
        )
