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
    PlannerConfig,
    clamp,
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

    summary = {
        "seed": seed,
        "progress_m": progress,
        "collisions": collision_count,
        "center_error_m": center_error,
        "final_speed": speed,
        "steps": len(log),
        "passed": passed,
    }
    print(
        f"seed={seed}  progress={progress:.1f}/100 m  "
        f"collisions={collision_count}  center_error={center_error:.2f} m  "
        f"final_speed={speed:.2f} m/s  {'PASS' if passed else 'FAIL'}"
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
    parser.add_argument("--controller", choices=["pure_pursuit", "lqr"],
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
