#!/usr/bin/env python3
"""Post-process evaluator CSV → diagnostic plots.

Usage:
    python3 tools/plot_tracking.py results/seed_0/tracking_20260731_120000.csv

Produces two PNG files in the same directory as the CSV:
    tracking_path.png   — expected vs actual path (2D, time heatmap)
    tracking_time.png   — speed & steering vs time
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _load_csv(csv_path: Path) -> dict:
    """Read evaluator CSV → dict of numpy arrays."""
    rows = []
    with csv_path.open("r", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            rows.append(row)
    if not rows:
        raise SystemExit(f"CSV is empty: {csv_path}")

    data = {}
    for key in rows[0].keys():
        try:
            data[key] = np.array([float(r[key]) for r in rows])
        except (ValueError, KeyError):
            pass  # skip non-numeric columns
    return data


def _load_centerline(scenario_path: Path) -> tuple:
    """Return (x, y, kappa) arrays from scenario centreline."""
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    cl = scenario["centerline"]
    xs = np.array([p["x"] for p in cl])
    ys = np.array([p["y"] for p in cl])
    yaws = np.array([p["yaw"] for p in cl])

    # Compute arc-length and curvature
    curvatures = [0.0]
    arc = [0.0]
    for i in range(1, len(cl)):
        ds = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        arc.append(arc[-1] + ds)
        k = (yaws[i] - yaws[i - 1]) / max(ds, 1e-6)
        curvatures.append(k)
    curvatures = np.array(curvatures)
    arc = np.array(arc)
    return xs, ys, yaws, curvatures, arc


def _estimate_expected_steering(curvature: float, wheelbase: float = 1.43) -> float:
    """Kinematic Ackermann steering angle for a given curvature."""
    return math.atan2(wheelbase * curvature, 1.0)


def _compute_speed_profile(
    curvatures: np.ndarray, arc: np.ndarray,
    max_lat_accel: float = 1.8, max_speed: float = 5.0,
    max_accel: float = 2.0, max_decel: float = -2.5,
) -> np.ndarray:
    """Replicate the online speed-profile computation for plotting."""
    N = len(curvatures)
    v = np.full(N, max_speed, dtype=np.float64)
    # curvature ceiling
    for i in range(N):
        k = max(abs(curvatures[i]), 1e-4)
        v[i] = min(v[i], math.sqrt(max_lat_accel / k))
    # forward pass (acceleration constraint)
    for i in range(1, N):
        ds = arc[i] - arc[i - 1]
        if ds <= 0:
            continue
        v_limit = math.sqrt(max(0.0, v[i - 1] ** 2 + 2.0 * max_accel * ds))
        v[i] = min(v[i], v_limit)
    # backward pass (deceleration constraint)
    for i in range(N - 2, -1, -1):
        ds = arc[i + 1] - arc[i]
        if ds <= 0:
            continue
        v_limit = math.sqrt(max(0.0, v[i + 1] ** 2 + 2.0 * max_decel * ds))
        v[i] = min(v[i], v_limit)
    return v


# ---------------------------------------------------------------------------
# Plot 1 — Path comparison (2D, time heatmap)
# ---------------------------------------------------------------------------

def _plot_path(data: dict, cl_x: np.ndarray, cl_y: np.ndarray,
               out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 8))

    t = data["time_s"]
    x = data["x"]
    y = data["y"]

    # Actual path coloured by time
    points = ax.scatter(x, y, c=t, s=2, cmap="plasma", alpha=0.7, label="actual")
    cbar = fig.colorbar(points, ax=ax, label="time (s)")

    # Centerline (expected path)
    ax.plot(cl_x, cl_y, "k--", linewidth=0.8, alpha=0.5, label="centreline (expected)")

    # Start / end markers
    ax.scatter(x[0], y[0], c="green", s=60, marker="o", zorder=5, label="start")
    ax.scatter(x[-1], y[-1], c="red", s=60, marker="x", zorder=5, label="end")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Expected vs Actual Path (time heatmap)")
    ax.legend(loc="best")
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  → {out_path}")


# ---------------------------------------------------------------------------
# Plot 2 — Speed & steering vs time
# ---------------------------------------------------------------------------

def _plot_time_series(data: dict, cl_curvatures: np.ndarray,
                      cl_arc: np.ndarray, out_path: Path) -> None:
    t = data["time_s"]
    actual_speed = data["speed_mps"]
    cmd_speed = data.get("command_speed_mps", None)
    actual_steering = np.degrees(data["steering_rad"])

    # Estimate expected steering from centreline curvature
    expected_steering = np.zeros_like(t)
    expected_speed = np.zeros_like(t)
    for i in range(len(t)):
        # Find nearest centreline point by progress
        # (use time as proxy for s — approximate)
        cl_idx = min(int(t[i] * 2.0), len(cl_curvatures) - 1)  # ~2 m/s → 2 pts/s
        expected_steering[i] = np.degrees(
            _estimate_expected_steering(cl_curvatures[cl_idx]))

    # Speed profile (offline recompute)
    v_profile = _compute_speed_profile(cl_curvatures, cl_arc)
    for i in range(len(t)):
        cl_idx = min(int(t[i] * 2.0), len(v_profile) - 1)
        expected_speed[i] = v_profile[cl_idx]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # --- Speed ---
    ax1.plot(t, actual_speed, "b-", linewidth=0.8, alpha=0.8, label="actual speed")
    if cmd_speed is not None:
        ax1.plot(t, cmd_speed, "b--", linewidth=0.6, alpha=0.4, label="cmd speed")
    ax1.plot(t, expected_speed, "r-", linewidth=0.7, alpha=0.6, label="expected (profile)")
    ax1.set_ylabel("speed (m/s)")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_title("Speed & Steering vs Time")

    # --- Steering ---
    ax2.plot(t, actual_steering, "b-", linewidth=0.8, alpha=0.8, label="actual steering")
    ax2.plot(t, expected_steering, "r-", linewidth=0.7, alpha=0.6, label="expected (kinematic)")
    ax2.axhline(y=0, color="gray", linewidth=0.5)
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("steering (deg)")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"  → {out_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Plot evaluator tracking CSV")
    parser.add_argument("csv", type=Path, help="Path to tracking_*.csv")
    parser.add_argument("--scenario", type=Path, default=None,
                        help="Path to scenario.json (auto-detected if omitted)")
    args = parser.parse_args()

    csv_path = args.csv.resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    # Auto-detect scenario
    if args.scenario:
        scenario_path = args.scenario.resolve()
    else:
        # results/seed_0/tracking.csv → runtime/scenario_0/scenario.json
        results_dir = csv_path.parent
        seed_name = results_dir.name  # e.g. "seed_0" or "flat_seed_0"
        scenario_dir = seed_name.replace("flat_", "").replace("seed_", "scenario_")
        scenario_path = csv_path.parents[2] / "runtime" / scenario_dir / "scenario.json"
        if not scenario_path.exists():
            # fallback: look for scenario_0
            scenario_path = csv_path.parents[2] / "runtime" / "scenario_0" / "scenario.json"

    if not scenario_path.exists():
        print(f"Warning: scenario not found at {scenario_path}, using empty centreline.",
              file=sys.stderr)
        cl_x, cl_y = np.array([]), np.array([])
        cl_curvatures, cl_arc = np.array([0.0]), np.array([0.0])
    else:
        print(f"Scenario: {scenario_path}")
        cl_x, cl_y, _, cl_curvatures, cl_arc = _load_centerline(scenario_path)

    print(f"CSV: {csv_path}  ({csv_path.stat().st_size / 1024:.0f} KB)")
    data = _load_csv(csv_path)
    print(f"  {len(data['time_s'])} rows, {data['time_s'][-1]:.1f} s elapsed")

    out_dir = csv_path.parent
    _plot_path(data, cl_x, cl_y, out_dir / "tracking_path.png")
    _plot_time_series(data, cl_curvatures, cl_arc, out_dir / "tracking_time.png")

    print("Done.")


if __name__ == "__main__":
    main()
