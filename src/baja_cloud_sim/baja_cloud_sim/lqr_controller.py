"""LQR lateral controller with bicycle-model gain scheduling.

Offline:  build a speed-indexed gain table via iterative DARE.
Online:   interpolate K, compute δ_fb = -K·x, add Bézier feed-forward.

Requires numpy (matrix ops).  Does NOT require scipy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from .trajectory_smoother import TrajectoryPoint, TrajectoryTable


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class LQRConfig:
    """Vehicle parameters and Q/R weights for the LQR controller."""

    # ---- Vehicle (Baja SAE) ----
    wheelbase: float = 1.43      # m
    mass: float = 200.0          # kg
    Iz: float = 50.0             # kg·m²  yaw inertia (estimate)
    lf: float = 0.79             # m  CoG → front axle  (~0.55 × wheelbase)
    lr: float = 0.64             # m  CoG → rear axle   (~0.45 × wheelbase)
    Cf: float = 80000.0          # N/rad  front cornering stiffness (ATV tyre)
    Cr: float = 80000.0          # N/rad  rear cornering stiffness

    # ---- Discrete-time ----
    dt: float = 0.05             # s  control period

    # ---- LQR weights (Q diagonal, R scalar) ----
    q_cte: float = 10.0          # cross-track error
    q_cte_dot: float = 1.0       # lateral speed
    q_heading: float = 5.0       # heading error
    q_yaw_rate: float = 0.5      # yaw-rate error
    r_steer: float = 10.0        # steering effort penalty

    # ---- Feedback clamp ----
    fb_limit_deg: float = 3.0   # cap feedback steering to ± this many degrees

    # ---- Gain-scheduling grid ----
    speed_min: float = 0.5       # m/s
    speed_max: float = 10.0      # m/s
    speed_step: float = 0.5      # m/s

    @property
    def Q_diag(self) -> List[float]:
        return [self.q_cte, self.q_cte_dot, self.q_heading, self.q_yaw_rate]

    @property
    def R_scalar(self) -> float:
        return self.r_steer


# ---------------------------------------------------------------------------
# Discrete Algebraic Riccati Equation solver (no scipy)
# ---------------------------------------------------------------------------

def _solve_dare(
    A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray,
    max_iter: int = 1000, tol: float = 1e-9,
) -> np.ndarray:
    """Iterative DARE: P_{k+1} = Q + AᵀP_kA - AᵀP_kB(R + BᵀP_kB)⁻¹BᵀP_kA.

    Returns the optimal gain K = (R + BᵀPB)⁻¹ BᵀPA.
    """
    P = Q.copy()
    I = np.eye(A.shape[0])
    for _ in range(max_iter):
        S = R + B.T @ P @ B
        # Solve S·K = BᵀPA  via S⁻¹,  S is 1×1 (scalar) here — cheap & stable
        P_next = Q + A.T @ P @ A - A.T @ P @ B @ np.linalg.solve(S, B.T @ P @ A)
        if np.max(np.abs(P_next - P)) < tol:
            P = P_next
            break
        P = P_next
    S = R + B.T @ P @ B
    K = np.linalg.solve(S, B.T @ P @ A)
    return K  # shape (1, 4) for single-input


# ---------------------------------------------------------------------------
# Bicycle model linearisation
# ---------------------------------------------------------------------------

def _linearise_bicycle(v: float, cfg: LQRConfig) -> tuple:
    """Continuous-time lateral-error bicycle model at longitudinal speed v.

    State  x = [e₁  ė₁  e₂  ė₂]ᵀ
      e₁ : cross-track error (m)
      e₂ : heading error (rad)

    Returns (A, B) — 4×4 and 4×1 numpy arrays.
    """
    Cf, Cr = cfg.Cf, cfg.Cr
    m, Iz = cfg.mass, cfg.Iz
    lf, lr = cfg.lf, cfg.lr

    # guard against v → 0  (model degenerates)
    v_safe = max(v, 0.5)

    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, -(Cf + Cr) / (m * v_safe), (Cf + Cr) / m,
         (lr * Cr - lf * Cf) / (m * v_safe)],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, (lr * Cr - lf * Cf) / (Iz * v_safe),
         (lf * Cf - lr * Cr) / Iz,
         -(lf**2 * Cf + lr**2 * Cr) / (Iz * v_safe)],
    ], dtype=np.float64)
    B = np.array([[0.0], [Cf / m], [0.0], [lf * Cf / Iz]], dtype=np.float64)
    return A, B


def _discretise(A: np.ndarray, B: np.ndarray, dt: float) -> tuple:
    """Zero-order-hold discretisation:  A_d = exp(A·dt),  B_d = ∫exp(A·τ)B dτ."""
    n = A.shape[0]
    # Augmented matrix
    M = np.zeros((n + 1, n + 1), dtype=np.float64)
    M[:n, :n] = A * dt
    M[:n, n] = (B * dt).flatten()
    # Matrix exponential
    E = _expm_pade(M, order=6)
    A_d = E[:n, :n]
    B_d = E[:n, n].reshape(-1, 1)
    return A_d, B_d


def _expm_pade(M: np.ndarray, order: int = 6) -> np.ndarray:
    """Pade-approximant matrix exponential using scaling-and-squaring (pure NumPy).

    Avoids scipy.linalg.expm to bypass NumPy 2.x / SciPy 1.x ABI incompatibility.
    Sufficient for the 5×5 augmented discretisation matrix in _discretise().
    Called only at LQR init time (once), not in the online control loop.
    """
    n = M.shape[0]
    # --- scale so that ||M||_1 < 1 ---
    norm = np.linalg.norm(M, ord=1)
    s = max(0, int(np.ceil(np.log2(norm + 1e-15))))
    A = M / (2.0 ** s)

    # --- order-6 Pade coefficients ---
    # R_66(z) = N_6(z) / D_6(z)  where both are cubic polynomials
    # N_6 coefficients:  1/2 + (3/28)z + (1/84)z^2 + (1/1680)z^3
    # D_6(z) = N_6(-z)   (diagonal Pade)
    c = [1.0, 3.0 / 28.0, 1.0 / 84.0, 1.0 / 1680.0]

    # Build numerator N = c0*I + c1*A + c2*A^2 + c3*A^3
    I = np.eye(n, dtype=np.float64)
    A2 = A @ A
    A3 = A2 @ A
    N = c[0] * I + c[1] * A + c[2] * A2 + c[3] * A3
    D = c[0] * I - c[1] * A + c[2] * A2 - c[3] * A3  # N(-A)

    # R = D^{-1} * N   (solving D*R = N is more stable than explicit inverse)
    R = np.linalg.solve(D, N)

    # --- un-scale by repeated squaring ---
    for _ in range(s):
        R = R @ R

    return R


# ---------------------------------------------------------------------------
# Gain table
# ---------------------------------------------------------------------------

class LQRGainTable:
    """Speed-indexed optimal LQR gain lookup."""

    def __init__(self, cfg: LQRConfig) -> None:
        self.cfg = cfg
        self._table: Dict[float, np.ndarray] = {}  # speed → K (1×4)
        self._speeds: List[float] = []
        self._build()

    def _build(self) -> None:
        Q = np.diag(self.cfg.Q_diag)
        R = np.array([[self.cfg.R_scalar]])
        v = self.cfg.speed_min
        while v <= self.cfg.speed_max + 1e-9:
            A, B = _linearise_bicycle(v, self.cfg)
            A_d, B_d = _discretise(A, B, self.cfg.dt)
            K = _solve_dare(A_d, B_d, Q, R)
            key = round(v, 3)
            self._table[key] = K
            self._speeds.append(key)
            v += self.cfg.speed_step

    def lookup(self, v: float) -> np.ndarray:
        """Interpolate gain for a given speed.  Clamps at edges."""
        if not self._speeds:
            raise RuntimeError("Gain table is empty")
        if v <= self._speeds[0]:
            return self._table[self._speeds[0]]
        if v >= self._speeds[-1]:
            return self._table[self._speeds[-1]]
        for i in range(len(self._speeds) - 1):
            if self._speeds[i] <= v <= self._speeds[i + 1]:
                lo, hi = self._speeds[i], self._speeds[i + 1]
                alpha = (v - lo) / (hi - lo) if hi > lo else 0.0
                return (1.0 - alpha) * self._table[lo] + alpha * self._table[hi]
        return self._table[self._speeds[-1]]


# ---------------------------------------------------------------------------
# Online LQR controller
# ---------------------------------------------------------------------------

# Lightweight vehicle state passed from the node
@dataclass
class VehicleState:
    speed: float          # m/s  (best available: commanded speed from prev tick)
    cte: float            # m    cross-track error
    heading_error: float  # rad  heading error (wrapped)
    yaw_rate: float       # rad/s  filtered yaw rate
    x: float = 0.0        # m    world x position (for nearest-index lookup)
    y: float = 0.0        # m    world y position (for nearest-index lookup)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class LQRController:
    """Online LQR lateral controller.

    Usage (each 50 ms tick)::

        v_cmd, delta_cmd, delta_ff, delta_fb = lqr.compute(
            table, vehicle_state, target_speed=6.0,
        )
    """

    def __init__(self, cfg: Optional[LQRConfig] = None) -> None:
        self.cfg = cfg or LQRConfig()
        self.gains = LQRGainTable(self.cfg)
        self._fb_limit_rad = math.radians(self.cfg.fb_limit_deg)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute(
        self,
        table: TrajectoryTable,
        vehicle: VehicleState,
        target_speed: float = 6.0,
    ) -> tuple:
        """Compute one control step.

        Returns:
            (v_cmd, delta_cmd, delta_ff, delta_fb)  — all floats
              v_cmd:      target speed (m/s)
              delta_cmd:  total steering (rad)
              delta_ff:   feed-forward component (rad)
              delta_fb:   LQR feedback component (rad)
        """
        if not table:
            # Fallback — no feed-forward, no feedback
            return target_speed, 0.0, 0.0, 0.0

        # 1. spatial lookup — find nearest point on table
        idx = self._nearest_index(table, vehicle.x, vehicle.y)

        # 2. preview slice
        preview = self._preview_slice(table, idx, vehicle.speed)

        # 3. feed-forward from preview curvature
        delta_ff = self._compute_feedforward(preview)

        # 4. LQR feedback
        ref = table.points[idx]
        delta_fb = self._compute_feedback(vehicle, ref)

        # 5. speed — most conservative limit in preview window
        v_cmd = min((p.speed_limit for p in preview), default=target_speed)
        v_cmd = _clamp(v_cmd, 0.0, target_speed)

        # 6. total steering
        delta_cmd = delta_ff + delta_fb

        return v_cmd, delta_cmd, delta_ff, delta_fb

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    @staticmethod
    def _nearest_index(table: TrajectoryTable, x: float = 0.0, y: float = 0.0) -> int:
        """Return index of the trajectory point closest to (x, y).

        Searches the entire table for the point with minimum Euclidean distance
        to the given world position.  Falls back to index 0 when position is
        unavailable (x=y=0) — this preserves backward compatibility for callers
        that don't pass position info.
        """
        if not table.points:
            return 0
        if x == 0.0 and y == 0.0:
            return 0  # position not provided, fall back to table start
        best_idx = 0
        best_d2 = float("inf")
        for i, pt in enumerate(table.points):
            d2 = (pt.x - x) ** 2 + (pt.y - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i
        return best_idx

    @staticmethod
    def _preview_slice(
        table: TrajectoryTable, idx: int, speed: float,
    ) -> List[TrajectoryPoint]:
        """Slice H points ahead of index, covering ~1-2 s of lookahead."""
        lookahead_dist = max(2.0, speed * 0.8)
        lookahead_s = table.points[idx].s + lookahead_dist
        end_idx = idx
        for i in range(idx, len(table.points)):
            if table.points[i].s >= lookahead_s:
                end_idx = i
                break
        else:
            end_idx = len(table.points) - 1
        # At least 5 points for meaningful preview
        if end_idx - idx < 5:
            end_idx = min(idx + 5, len(table.points) - 1)
        return table.points[idx:end_idx + 1]

    def _compute_feedforward(
        self, preview: List[TrajectoryPoint],
    ) -> float:
        """Feed-forward steering based on curvature trend in preview slice."""
        if len(preview) < 3:
            return preview[0].steering_ff if preview else 0.0

        one_third = len(preview) // 3
        two_thirds = min(2 * one_third, len(preview) - 1)

        k_near = preview[one_third].curvature
        k_far = preview[two_thirds].curvature
        k_trend = k_far - k_near
        # Blend near curvature with a fraction of the trend for anticipation
        k_eff = k_near + 0.3 * k_trend

        return math.atan2(self.cfg.wheelbase * k_eff, 1.0)

    def _compute_feedback(
        self, vehicle: VehicleState, ref: TrajectoryPoint,
    ) -> float:
        """LQR state feedback δ_fb = -K·x."""
        # ---- build state vector ----
        e1 = vehicle.cte
        e2 = vehicle.heading_error
        e1_dot = vehicle.speed * math.sin(e2)
        # yaw-rate error: actual − reference
        ref_yaw_rate = vehicle.speed * ref.curvature
        e2_dot = vehicle.yaw_rate - ref_yaw_rate

        # ---- gain scheduling ----
        v = max(0.5, vehicle.speed)
        K = self.gains.lookup(v)

        # ---- feedback ----
        x = np.array([e1, e1_dot, e2, e2_dot])
        delta_fb = -float(K @ x)  # K is 1×4, x is 4×1
        delta_fb = _clamp(delta_fb, -self._fb_limit_rad, self._fb_limit_rad)

        return delta_fb
