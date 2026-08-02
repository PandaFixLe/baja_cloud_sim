#!/usr/bin/env python3
"""Offline closed-loop smoke test — runs BEFORE Gazebo to validate P&C.

Supports:
- Scenario JSON loading (with programmatic centreline)
- Controller switching (pure_pursuit; lqr placeholder)
- Speed profile on/off toggle
- Matplotlib trajectory / error / speed / curvature plots
- CI gate: --exit-code returns non-zero on metric failure
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")  # headless-safe
import matplotlib.pyplot as plt  # noqa: E402

PACKAGE = Path(__file__).resolve().parents[1] / "src" / "baja_cloud_sim"
sys.path.insert(0, str(PACKAGE))

from baja_cloud_sim.core import (  # noqa: E402
    ControllerConfig,
    LQRConfig,
    LQRController,
    PlannerConfig,
    clamp,
    compute_feedforward,
    estimate_lqr_state,
    generate_boundaries,
    generate_centerline,
    generate_obstacles,
    gps_to_local,
    legacy_path_control,
    nearest_index,
    nav_to_world_yaw,
    plan_frenet_path,
    point_to_oriented_box_clearance,
    signed_lateral,
    smooth_velocity,
    world_to_base,
    wrap_angle,
)

Point = Tuple[float, float]
DEG = 180.0 / math.pi


# ── helpers ───────────────────────────────────────────────────────────
def _load_scenario(scenario_path: str) -> dict:
    """Load a scenario JSON file (or None to use procedural centreline)."""
    if scenario_path is None:
        return {}
    return json.loads(Path(scenario_path).read_text(encoding="utf-8"))


def _get_centreline(scenario: dict) -> List[Dict[str, float]]:
    """Return centreline from scenario or generate procedurally."""
    if scenario and "centerline" in scenario:
        return scenario["centerline"]
    return generate_centerline(100.0, 0.5)


def _speed_factor_continuous(steering: float, max_steering: float) -> float:
    """Cos-based continuous speed derating (matching core.py Phase 0.7)."""
    ratio = abs(steering) / max_steering
    return 0.5 + 0.5 * math.cos(ratio * math.pi * 0.5)


def _closest_index_tuples(path: List[Point], position: Point) -> int:
    """Nearest index in a list of (x, y) tuples to a given position."""
    best, best_d2 = 0, float("inf")
    for i, (px, py) in enumerate(path):
        d2 = (px - position[0]) ** 2 + (py - position[1]) ** 2
        if d2 < best_d2:
            best, best_d2 = i, d2
    return best


# ── main loop ─────────────────────────────────────────────────────────
def run(
    seed: int = 42,
    obstacle_count: int = 5,
    scenario_path: Optional[str] = None,
    controller: str = "pure_pursuit",
    target_speed: float = 2.5,
    use_speed_profile: bool = False,
    plot_dir: Optional[str] = None,
) -> Tuple[int, dict]:
    """Run offline closed-loop simulation.  Returns (exit_code, summary_dict)."""
    scenario = _load_scenario(scenario_path)
    centerline = _get_centreline(scenario)
    left_pts, right_pts = generate_boundaries(centerline)
    left = [signed_lateral(left_pts[i], centerline[i]) for i in range(len(centerline))]
    right = [signed_lateral(right_pts[i], centerline[i]) for i in range(len(centerline))]
    obstacles = generate_obstacles(centerline, seed, obstacle_count)

    planner_cfg = PlannerConfig()
    ctrl_cfg = ControllerConfig(target_speed=target_speed)
    max_steering = math.radians(ctrl_cfg.max_steering_deg)

    x = centerline[0]["x"]
    y = centerline[0]["y"]
    yaw = centerline[0]["yaw"]
    speed = 0.0
    prev_accel = 0.0
    path: List[Point] = []
    nearest = 0
    collision_count = 0
    collision_active = False
    feasible = False
    infeasible_count = 0
    dt = 0.05  # 20 Hz
    prev_steering = 0.0
    e_y_int = 0.0
    lqr_ctrl = None
    lqr_cfg = None
    if controller == "lqr_apollo":
        lqr_ctrl = LQRController()
        lqr_cfg = LQRConfig()

    # logging
    log: List[Dict[str, float]] = []

    steps = int(80.0 / dt)  # 80 s should be more than enough for 100 m
    for step in range(steps):
        # ── planner (every-other cycle, ~10 Hz) ──
        if step % 2 == 0:
            nearest = nearest_index(centerline, x, y, max(0, nearest - 3))
            visible = []
            for obs in obstacles:
                local = world_to_base((obs["x"], obs["y"]), (x, y), yaw)
                if -6.0 <= local[0] <= 34.0:
                    visible.append(obs)
            result = plan_frenet_path(
                centerline, nearest, (x, y), left, right, visible, planner_cfg,
            )
            feasible = result.feasible
            if feasible and len(result.path) >= 2:
                path = result.path
                infeasible_count = 0
            else:
                infeasible_count += 1

        # ── controller ──
        navigation_yaw = wrap_angle(math.pi * 0.5 - yaw)
        effective_path = path
        if not feasible or len(path) < 2:
            if infeasible_count > 6:  # freewheel 300 ms → stop
                effective_path = []
        if not effective_path or len(effective_path) < 2:
            command = {"speed": 0.0, "steering": 0.0, "target_x": x, "target_y": y}
        elif controller == "lqr_apollo" and lqr_ctrl is not None:
            # Reference = planner path (mirrors the node using /planned_path).
            # NOTE: this index addresses `effective_path`, NOT `centerline`.
            # Keep it in its own variable — `nearest` is the centerline index
            # used by the planner seed and the metrics below.
            p_idx = _closest_index_tuples(effective_path, (x, y))
            # Clamp to an *interior* node so p0/p1/p2 are three distinct
            # points.  At p_idx == 0 the old code aliased p0 == p1, making
            # atan2(0, 0) == 0 and inflating kappa by more than an order of
            # magnitude — the feed-forward term then demanded absurd steering.
            p_idx = min(max(p_idx, 1), max(1, len(effective_path) - 2))
            p0 = effective_path[p_idx - 1]
            p1 = effective_path[p_idx]
            p2 = effective_path[p_idx + 1]
            yaw_prev = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            yaw_next = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            # Node-centred curvature, normalised by the mean adjacent
            # segment length (matches path_follower_node._compute_speed_profile)
            ds = 0.5 * (math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                        + math.hypot(p2[0] - p1[0], p2[1] - p1[1]))
            kappa = wrap_angle(yaw_next - yaw_prev) / ds if ds > 1e-6 else 0.0
            # Node yaw = bisector of the two adjacent segments
            ref_yaw = wrap_angle(yaw_prev + 0.5 * wrap_angle(yaw_next - yaw_prev))
            ref = {"x": p1[0], "y": p1[1], "yaw": ref_yaw, "kappa": kappa}
            yaw_rate = speed * math.tan(prev_steering) / 1.43 if abs(speed) > 0.01 else 0.0
            odom_vel = (speed * math.cos(yaw), speed * math.sin(yaw))
            state = estimate_lqr_state((x, y), yaw, odom_vel, yaw_rate, ref, e_y_int)
            K = lqr_ctrl.get_gain(max(speed, 0.5), lqr_cfg)
            if K is not None:
                delta_fb = -float(K @ state)
                delta_ff = compute_feedforward(kappa, max(speed, 0.5), lqr_cfg)
                steering = clamp(delta_ff + delta_fb, -max_steering, max_steering)
                e_y_int = clamp(e_y_int + float(state[1]) * dt, -0.5, 0.5)
            else:
                steering = 0.0
            v_ref = target_speed
            if use_speed_profile:
                v_ref = min(target_speed, math.sqrt(1.8 / max(abs(kappa), 1e-4)))
            command = {"speed": v_ref, "steering": steering,
                       "target_x": p2[0], "target_y": p2[1]}
        else:
            command = legacy_path_control((x, y), navigation_yaw, effective_path, ctrl_cfg)
            # continuous derating (Phase 0.7)
            if not use_speed_profile:
                sf = _speed_factor_continuous(command["steering"], max_steering)
                command["speed"] = target_speed * sf

        # ── accel + jerk smoothing (Phase 0.6) ──
        smoothed, prev_accel = smooth_velocity(
            command["speed"], speed, 2.0, -2.5, dt, prev_accel=prev_accel, max_jerk=4.0,
        )
        speed = smoothed
        prev_steering = command["steering"]

        # ── dynamics (bicycle model) ──
        yaw = wrap_angle(yaw + speed * math.tan(command["steering"]) / 1.43 * dt)
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt

        # ── collision check ──
        half_l = planner_cfg.vehicle_length * 0.5
        half_w = planner_cfg.vehicle_width * 0.5
        collision = any(
            point_to_oriented_box_clearance((x, y), obs, half_l, half_w) <= 0.0
            for obs in obstacles
        )
        if collision and not collision_active:
            collision_count += 1
        collision_active = collision

        # ── log ──
        ref = centerline[nearest]
        log.append({
            "t": step * dt,
            "x": x, "y": y, "yaw_deg": yaw * DEG,
            "speed": speed,
            "steering_deg": math.degrees(command["steering"]),
            "s": ref["s"],
            "center_error": abs(signed_lateral((x, y), ref)),
            "feasible": 1.0 if feasible else 0.0,
        })

        if nearest >= len(centerline) - 4 or ref["s"] >= 99.0:
            break

    # ── summary ──
    ref = centerline[min(nearest, len(centerline) - 1)]
    center_error = abs(signed_lateral((x, y), ref))
    progress = ref["s"]
    passed = progress >= 97.0 and collision_count == 0 and center_error < 1.0

    # Steering smoothness: a controller can track well on paper while
    # slamming the rack lock-to-lock.  Saturation percentage and the
    # step-to-step rate are the metrics that correlate with the limit
    # cycle seen in Gazebo, so surface them next to the tracking error.
    steer_series = [r["steering_deg"] for r in log] or [0.0]
    sat_limit = math.degrees(max_steering) * 0.98
    steer_sat_pct = 100.0 * sum(1 for s in steer_series if abs(s) >= sat_limit) / len(steer_series)
    steer_mean = sum(steer_series) / len(steer_series)
    steer_std = math.sqrt(sum((s - steer_mean) ** 2 for s in steer_series) / len(steer_series))
    steer_rate = (
        sum(abs(steer_series[i] - steer_series[i - 1]) for i in range(1, len(steer_series)))
        / (max(1, len(steer_series) - 1) * dt)
    )
    err_series = [r["center_error"] for r in log] or [0.0]
    center_error_mean = sum(err_series) / len(err_series)

    summary = {
        "seed": seed,
        "progress_m": progress,
        "collisions": collision_count,
        "center_error_m": center_error,
        "center_error_mean_m": center_error_mean,
        "final_speed": speed,
        "steer_std_deg": steer_std,
        "steer_sat_pct": steer_sat_pct,
        "steer_rate_dps": steer_rate,
        "steps": len(log),
        "passed": passed,
    }
    print(
        f"seed={seed}  progress={progress:.1f}/100 m  "
        f"collisions={collision_count}  center_error={center_error:.2f} m "
        f"(mean {center_error_mean:.2f})  "
        f"final_speed={speed:.2f} m/s  "
        f"steer[std={steer_std:.1f}° sat={steer_sat_pct:.0f}% rate={steer_rate:.0f}°/s]  "
        f"{'PASS' if passed else 'FAIL'}"
    )

    # ── plots ──
    if plot_dir:
        _make_plots(log, centerline, obstacles, Path(plot_dir), seed)

    return (0 if passed else 1), summary


# ── matplotlib plots ──────────────────────────────────────────────────
def _make_plots(
    log: List[dict],
    centerline: List[dict],
    obstacles: List[dict],
    out_dir: Path,
    seed: int,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    t = [r["t"] for r in log]
    xs = [r["x"] for r in log]
    ys = [r["y"] for r in log]
    speeds = [r["speed"] for r in log]
    steer = [r["steering_deg"] for r in log]
    c_err = [r["center_error"] for r in log]
    s_vals = [r["s"] for r in log]

    # trajectory
    fig, ax = plt.subplots(figsize=(8, 6))
    cx = [p["x"] for p in centerline]
    cy = [p["y"] for p in centerline]
    ax.plot(cx, cy, "g-", lw=1.5, alpha=0.6, label="centreline")
    ax.plot(xs, ys, "r-", lw=1.2, label="actual")
    for obs in obstacles:
        rect = plt.Rectangle(
            (obs["x"] - obs["length"] / 2, obs["y"] - obs["width"] / 2),
            obs["length"], obs["width"],
            angle=math.degrees(obs["yaw"]), fc="orange", alpha=0.35, ec="darkorange",
        )
        ax.add_patch(rect)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(f"Offline Closed-Loop — seed={seed}")
    ax.legend(); fig.tight_layout()
    fig.savefig(out_dir / f"trajectory_seed{seed}.png", dpi=150)
    plt.close(fig)

    # time-series
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t, speeds, "b-", lw=1.2)
    axes[0].set_ylabel("speed [m/s]"); axes[0].grid(True)
    axes[1].plot(t, steer, "m-", lw=1.2)
    axes[1].set_ylabel("steering [deg]"); axes[1].grid(True)
    axes[2].plot(t, c_err, "r-", lw=1.2, label="center error")
    axes[2].set_ylabel("error [m]"); axes[2].set_xlabel("time [s]"); axes[2].grid(True)
    axes[2].legend()
    fig.suptitle(f"Time-Series — seed={seed}")
    fig.tight_layout()
    fig.savefig(out_dir / f"timeseries_seed{seed}.png", dpi=150)
    plt.close(fig)

    print(f"Plots saved to {out_dir}/trajectory_seed{seed}.png, timeseries_seed{seed}.png")


# ── CLI ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--obstacles", type=int, default=5)
    parser.add_argument("--scenario", type=str, default=None,
                        help="Path to scenario JSON (default: procedural centreline)")
    parser.add_argument("--controller", choices=["pure_pursuit", "lqr_apollo"],
                        default="pure_pursuit")
    parser.add_argument("--speed", type=float, default=2.5,
                        help="Target speed [m/s]")
    parser.add_argument("--speed-profile", action="store_true",
                        help="Enable curvature-based speed profile")
    parser.add_argument("--plot-dir", type=str, default=None,
                        help="Output directory for matplotlib plots")
    parser.add_argument("--exit-code", action="store_true",
                        help="Return non-zero exit code on metric failure (CI gate)")
    args = parser.parse_args()

    code, summary = run(
        seed=args.seed,
        obstacle_count=args.obstacles,
        scenario_path=args.scenario,
        controller=args.controller,
        target_speed=args.speed,
        use_speed_profile=args.speed_profile,
        plot_dir=args.plot_dir,
    )
    if args.exit_code:
        raise SystemExit(code)
