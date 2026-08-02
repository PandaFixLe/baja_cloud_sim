"""LQR 4×4 + Pure-Pursuit fallback controller."""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .config import ControllerConfig, LQRConfig
from .geometry import Point, clamp, distance, wrap_angle


# ── Pure Pursuit ──────────────────────────────────────────────────────

def legacy_path_control(
    current: Point,
    yaw_navigation: float,
    path: Sequence[Point],
    config: Optional[ControllerConfig] = None,
    current_speed: float = 2.5,
) -> Dict[str, float]:
    cfg = config or ControllerConfig()
    if len(path) < 2:
        return {"speed": 0.0, "steering": 0.0, "target_x": current[0], "target_y": current[1]}
    nearest = min(range(len(path)), key=lambda index: distance(current, path[index]))
    adaptive_lookahead = max(1.0, min(cfg.lookahead_distance, current_speed * 0.6))
    target_index = len(path) - 1
    for index in range(nearest, len(path)):
        if distance(current, path[index]) >= adaptive_lookahead:
            target_index = index
            break
    target = path[target_index]
    east, north = target[0] - current[0], target[1] - current[1]
    desired_navigation = math.atan2(east, north)
    error = wrap_angle(desired_navigation - yaw_navigation)
    max_steering = math.radians(cfg.max_steering_deg)
    steering = clamp(-cfg.heading_gain * error, -max_steering, max_steering)
    ratio = abs(steering) / max_steering
    speed_factor = 0.5 + 0.5 * math.cos(ratio * math.pi * 0.5)
    return {
        "speed": cfg.target_speed * speed_factor,
        "steering": steering,
        "target_x": target[0],
        "target_y": target[1],
        "heading_error": error,
    }


# ── LQR 4×4 ───────────────────────────────────────────────────────────

def estimate_lqr_state(
    position: Point,
    yaw: float,
    odom_velocity: Tuple[float, float],
    yaw_rate: float,
    reference: Dict[str, float],
    e_y_int: float = 0.0,
) -> np.ndarray:
    """Augmented 5‑state vector: [∫e_y, e_y, e_y_dot, e_psi, e_psi_dot].

    The integral term ``e_y_int`` is accumulated externally (one sample per
    control cycle) so the LQR gain includes integral action that eliminates
    steady‑state lateral offset.
    """
    dx = position[0] - reference["x"]
    dy = position[1] - reference["y"]
    e_y = -math.sin(reference["yaw"]) * dx + math.cos(reference["yaw"]) * dy
    e_psi = wrap_angle(yaw - reference["yaw"])
    vx, vy = odom_velocity
    e_y_dot = -math.sin(reference["yaw"]) * vx + math.cos(reference["yaw"]) * vy
    e_psi_dot = yaw_rate - vx * reference.get("kappa", 0.0)
    return np.array([e_y_int, e_y, e_y_dot, e_psi, e_psi_dot], dtype=np.float64)


def build_lqr_matrices(velocity: float, cfg: LQRConfig) -> Tuple[np.ndarray, np.ndarray]:
    """Build the continuous-time bicycle-model matrices.

    When ``cfg.Q`` has 5 elements the system is augmented with an
    integrator on lateral error (LQI) so the controller eliminates
    steady-state offset.
    """
    v = max(velocity, 0.1)
    m, Iz = cfg.mass, cfg.Iz
    Cf, Cr = cfg.Cf, cfg.Cr
    lf, lr = cfg.lf, cfg.lr
    a22 = -(Cf + Cr) / (m * v)
    a23 = (Cf + Cr) / m
    a24 = (lf * Cf - lr * Cr) / (m * v)
    a42 = -(lf * Cf - lr * Cr) / (Iz * v)
    a43 = (lf * Cf - lr * Cr) / Iz
    a44 = -(lf * lf * Cf + lr * lr * Cr) / (Iz * v)
    b21 = Cf / (m * v) if v > 0.5 else Cf / (m * 0.5)
    b41 = (lf * Cf) / (Iz * v) if v > 0.5 else (lf * Cf) / (Iz * 0.5)

    use_integral = len(cfg.Q) == 5
    if use_integral:
        # 5-state augmented system: [∫e_y, e_y, e_y_dot, e_psi, e_psi_dot]
        A = np.array([
            [0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, a22, a23, a24],
            [0.0, 0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, a42, a43, a44],
        ], dtype=np.float64)
        B = np.array([[0.0], [0.0], [b21], [0.0], [b41]], dtype=np.float64)
    else:
        # legacy 4-state system
        A = np.array([
            [0.0, 1.0, 0.0, 0.0],
            [0.0, a22, a23, a24],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, a42, a43, a44],
        ], dtype=np.float64)
        B = np.array([[0.0], [b21], [0.0], [b41]], dtype=np.float64)
    return A, B


def _solve_dare(velocity: float, cfg: LQRConfig) -> Optional[np.ndarray]:
    try:
        from scipy.linalg import expm, solve_discrete_are
    except ImportError:
        return None
    A, B = build_lqr_matrices(velocity, cfg)
    n = A.shape[0]
    # Zero-order-hold discretisation via Van Loan (matrix exponential).
    # Forward Euler (I + A·dt) is unconditionally unstable for this plant
    # because |a₂₂·dt| > 1 at all speeds below ≈4.8 m/s.
    dt = cfg.dt
    M = np.zeros((n + 1, n + 1), dtype=np.float64)
    M[:n, :n] = A * dt
    M[:n, n] = B.squeeze() * dt
    expM = expm(M)
    A_d = expM[:n, :n]
    B_d = expM[:n, n].reshape(-1, 1)
    R_eff = cfg.R * (1.0 + (velocity / cfg.v_norm) ** 2)
    try:
        P = solve_discrete_are(A_d, B_d, np.diag(cfg.Q), np.array([[R_eff]], dtype=np.float64))
        return (1.0 / R_eff) * B_d.T @ P
    except (np.linalg.LinAlgError, ValueError):
        return None


class LQRController:
    def __init__(self):
        self._last_K: Optional[np.ndarray] = None
        self._last_v: float = -1.0
        self._solve_counter: int = 0

    def get_gain(self, velocity: float, cfg: LQRConfig) -> Optional[np.ndarray]:
        self._solve_counter += 1
        if (self._last_K is None
                or self._solve_counter >= cfg.dare_solve_interval
                or abs(velocity - self._last_v) > cfg.velocity_recompute_threshold):
            K = _solve_dare(velocity, cfg)
            if K is not None:
                self._last_K, self._last_v = K, velocity
            self._solve_counter = 0
        return self._last_K


def compute_feedforward(curvature: float, velocity: float, cfg: LQRConfig) -> float:
    kin = cfg.wheelbase * curvature
    k_us = cfg.understeer_gradient
    if k_us == 0.0 and cfg.Cf > 0 and cfg.Cr > 0:
        k_us = (cfg.mass * (cfg.lf * cfg.Cf - cfg.lr * cfg.Cr)
                / (2.0 * cfg.wheelbase * cfg.Cf * cfg.Cr))
    return kin + k_us * velocity * velocity * curvature


def _lateral_thresholds(velocity: float) -> tuple:
    """Return (enter_m, exit_m) — speed‑adaptive lateral hysteresis.

    At low speed LQR gains are higher and the vehicle is more
    manoeuvrable → tighter thresholds.  At high speed give more
    room to glide.
    """
    enter = 0.10 + 0.03 * velocity   # v=1.0→0.13  v=2.5→0.175
    exit_ = enter * 0.5               # 50 % hysteresis
    return (enter, exit_)


def compute_lqr_control(
    position, yaw, odom_velocity, yaw_rate,
    reference, speed_profile, s_index, actual_velocity,
    lqr_controller, lqr_cfg, path, ctrl_cfg, yaw_navigation,
    lqr_e_y_int: float = 0.0,
) -> Dict[str, float]:
    v_op = actual_velocity
    if speed_profile and 0 <= s_index < len(speed_profile):
        v_op = max(speed_profile[s_index], 0.5)
    if actual_velocity >= lqr_cfg.lqr_min_velocity and reference.get("kappa") is not None:
        K = lqr_controller.get_gain(v_op, lqr_cfg)
        if K is not None:
            use_int = len(lqr_cfg.Q) == 5
            state = estimate_lqr_state(position, yaw, odom_velocity, yaw_rate,
                                       reference, e_y_int=lqr_e_y_int)
            if not use_int:
                state = state[1:]  # drop ∫e_y → 4-state [e_y, e_y_dot, e_psi, e_psi_dot]
            # State layout: [e_y_int?, e_y, e_y_dot, e_psi, e_psi_dot]
            i_ey = 1 if use_int else 0
            i_eydot = 2 if use_int else 1
            i_epsi = 3 if use_int else 2
            i_epsidot = 4 if use_int else 3
            e_y = state[i_ey]
            e_y_dot = state[i_eydot]
            e_psi = state[i_epsi]
            e_psi_dot = state[i_epsidot]

            # ── Continuous lateral gain (smoothstep) ──
            # Instead of a hard on/off trigger, the lateral gain ramps
            # linearly from 0 at th_exit to 1 at th_enter.  Yaw gain
            # ramps from 25 % to 100 % over the same band.  This
            # eliminates the discontinuity that caused steering jolts
            # at the threshold boundary.
            th_enter, th_exit = _lateral_thresholds(v_op)
            cross_track = abs(e_y)

            if cross_track < th_exit:
                lat_gain = 0.0
            elif cross_track > th_enter:
                lat_gain = 1.0
            else:
                lat_gain = (cross_track - th_exit) / (th_enter - th_exit)

            yaw_gain = 0.3 + 0.7 * lat_gain

            K_scaled = K.copy()
            K_scaled[0, i_ey] *= lat_gain
            K_scaled[0, i_eydot] *= lat_gain
            K_scaled[0, i_epsi] *= yaw_gain
            K_scaled[0, i_epsidot] *= yaw_gain
            # integral: freeze (gain→0) in deep glide, active otherwise
            if use_int:
                K_scaled[0, 0] *= lat_gain

            delta_fb = -float(K_scaled @ state)
            # Hard feedback cap: LQR only provides fine trimming (±3°)
            delta_fb = clamp(delta_fb, -math.radians(3.0), math.radians(3.0))
            delta_ff = compute_feedforward(reference.get("kappa", 0.0), v_op, lqr_cfg)
            steering = clamp(delta_fb + delta_ff, -lqr_cfg.max_steering, lqr_cfg.max_steering)
            return {"speed": float(v_op), "steering": steering,
                    "target_x": reference["x"], "target_y": reference["y"],
                    "heading_error": e_psi, "e_y": float(e_y)}
    return legacy_path_control((position[0], position[1]), yaw_navigation, path, ctrl_cfg)


# ── Utility ───────────────────────────────────────────────────────────

def smooth_velocity(target: float, current: float, max_accel: float,
                    max_decel: float, dt: float, prev_accel: float = 0.0,
                    max_jerk: float = 4.0) -> Tuple[float, float]:
    delta = target - current
    accel = clamp(delta / max(dt, 1e-6), -abs(max_decel), max_accel)
    jerk = clamp((accel - prev_accel) / dt, -max_jerk, max_jerk)
    accel = prev_accel + jerk * dt
    return current + accel * dt, accel
