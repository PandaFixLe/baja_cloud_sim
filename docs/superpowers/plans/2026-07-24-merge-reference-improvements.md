# Merge baja_cloud_sim (2) reference improvements — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Selectively integrate the better parts of `baja_cloud_sim（2）` into the current branch (v2) without regressing the PID controller, real-vehicle dimensions, or cloud-run scripts.

**Architecture:** The merge is structured around three independent functional slices — (a) scene geometry, (b) perception/tracking, (c) configuration plumbing. Each slice ships behind a green test/build cycle so any regression can be isolated to a single task. The working branch (`v2`) is dirty-free at the start of execution.

**Tech Stack:** ROS 2 (rclpy), Gazebo (gz-sim), Python 3.10+, ament_python, RViz2, YAML launch descriptions, unittest.

## Global Constraints

- Branch: `v2` (current working tree is committed at `0781640` before execution).
- Vehicle physical dimensions are **frozen**: length 1.70 m, width 1.50 m, mass 252 kg. Do not change SDF chassis box, URDF chassis box, or any `PlannerConfig.vehicle_length/width` default that ships to runtime.
- The PID follower (`pid_path_follower_node.py`) is **frozen** in feature set; only the four specific fixes in Task 5 are in scope.
- No new external dependency. Do not add `ffmpeg`, `video_recorder_node`, or a `recording_camera` sensor.
- Marker lifetime for LINE_STRIP/CUBE markers is **180 ms** unless the source explicitly requires otherwise. Existing 500 ms lifetimes in `frenet_planner_node.py`, `evaluator_node.py`, and `truth_perception_node.py` are upgraded to 180 ms.
- All edits to tracked files require a commit. Use Conventional Commits: `v2:`, `fix:`, `feat:`, `docs:`, `chore:`.
- Every task that runs Python must end with `colcon build --packages-select baja_cloud_sim` succeeding.

---

## Task 1: Snapshot baseline and create backup

**Files:**
- Create: `runtime/pre_merge_backup/{core.py,scenario_generator.py,frenet_planner_node.py,truth_perception_node.py,pid_path_follower_node.py,evaluator_node.py,path_follower_node.py,simulation.launch.py,params.yaml,bridge.yaml,model.sdf,setup.py,package.xml}`
- Create: `runtime/pre_merge_backup/README.md` (listing backed-up files)

**Interfaces:**
- Consumes: nothing.
- Produces: backup tree under `runtime/pre_merge_backup/`.

- [ ] **Step 1: Confirm working tree is clean**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && git status --short`
Expected: empty output.

If non-empty, STOP and ask user to commit/stash first.

- [ ] **Step 2: Create backup directory and copy 13 tracked files**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
mkdir -p runtime/pre_merge_backup
for f in \
  src/baja_cloud_sim/baja_cloud_sim/core.py \
  src/baja_cloud_sim/baja_cloud_sim/scenario_generator.py \
  src/baja_cloud_sim/baja_cloud_sim/frenet_planner_node.py \
  src/baja_cloud_sim/baja_cloud_sim/truth_perception_node.py \
  src/baja_cloud_sim/baja_cloud_sim/pid_path_follower_node.py \
  src/baja_cloud_sim/baja_cloud_sim/evaluator_node.py \
  src/baja_cloud_sim/baja_cloud_sim/path_follower_node.py \
  src/baja_cloud_sim/launch/simulation.launch.py \
  src/baja_cloud_sim/config/params.yaml \
  src/baja_cloud_sim/config/bridge.yaml \
  src/baja_cloud_sim/models/baja_vehicle/model.sdf \
  src/baja_cloud_sim/setup.py \
  src/baja_cloud_sim/package.xml; do
  cp "$f" "runtime/pre_merge_backup/$(basename "$f")"
done
```

- [ ] **Step 3: Write a README inside the backup directory listing contents**

Create `runtime/pre_merge_backup/README.md`:

```markdown
# Pre-merge backup — 2026-07-24

Snapshots of 13 files taken on the `v2` branch before merging
baja_cloud_sim (2) reference improvements. Use this tree for
single-file rollback if a task fails verification.

Files:
- core.py
- scenario_generator.py
- frenet_planner_node.py
- truth_perception_node.py
- pid_path_follower_node.py
- evaluator_node.py
- path_follower_node.py
- simulation.launch.py
- params.yaml
- bridge.yaml
- model.sdf
- setup.py
- package.xml
```

- [ ] **Step 4: Verify backup**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && ls -la runtime/pre_merge_backup/`
Expected: 14 entries (13 files + README).

- [ ] **Step 5: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add runtime/pre_merge_backup
git commit -m "chore: snapshot pre-merge state under runtime/pre_merge_backup"
```

---

## Task 2: Merge core.py (geometry + planning + control primitives)

**Files:**
- Modify: `src/baja_cloud_sim/baja_cloud_sim/core.py`
- Test: `src/baja_cloud_sim/test/test_core.py`

**Interfaces:**
- Consumes: existing `core.py` (Task 1 backup).
- Produces: `core.py` that exports `quaternion_to_rpy`, `rpy_to_quaternion`, `terrain_height`, `generate_centerline` (with `z` field, straight→turn→straight), `generate_obstacles` (terrain-aware `z`), `segment_is_safe` (four-corner sweep). `PlannerConfig.vehicle_length` stays 1.70; `ControllerConfig.target_speed` stays 2.5.

- [ ] **Step 1: Add the four failing tests**

Append to `src/baja_cloud_sim/test/test_core.py` (keep existing imports and tests, add these inside `CoreTests`):

```python
    def test_terrain_height_is_zero_outside_features(self):
        from baja_cloud_sim.core import terrain_height
        # Outside any hill/bump zones the profile must be exactly zero.
        self.assertEqual(terrain_height(5.0), 0.0)
        self.assertEqual(terrain_height(50.0), 0.0)
        # Inside a known hill the value must be positive and bounded.
        peak = terrain_height(23.0)  # mid of first smooth_hill (s=16..30)
        self.assertGreater(peak, 0.3)
        self.assertLess(peak, 0.6)

    def test_centerline_carries_elevation_and_turn(self):
        # First straight segment must lie on x-axis with zero yaw.
        self.assertAlmostEqual(self.centerline[0]["y"], 0.0, places=6)
        self.assertAlmostEqual(self.centerline[0]["yaw"], 0.0, places=6)
        self.assertIn("z", self.centerline[0])
        # After s = 60 m (well past the turn that ends near s ≈ 65 m)
        # the centerline must have non-zero y and yaw.
        late = next(p for p in self.centerline if p["s"] >= 60.0)
        self.assertGreater(late["y"], 5.0)
        self.assertGreater(abs(late["yaw"]), 0.3)

    def test_segment_is_safe_catches_rear_corner_graze(self):
        """Regression: reference regressed this to a single-point test,
        allowing a swept vehicle rectangle to clip an obstacle's rear
        corner even when the centre line clears. The four-corner sweep
        must reject this path."""
        from baja_cloud_sim.core import segment_is_safe
        # Path that arcs around an obstacle: start in front, end behind.
        obstacles = [{
            "id": 0, "x": 5.0, "y": 0.0, "yaw": 0.0,
            "length": 1.0, "width": 1.0, "height": 0.5,
        }]
        # End point sits behind the obstacle but offset laterally so a
        # *point* test passes; the rear corner of the vehicle does not.
        unsafe = segment_is_safe(
            start=(4.0, -1.5), end=(6.0, 1.5),
            obstacles=obstacles,
            vehicle_half_length=1.0, vehicle_half_width=0.9,
            samples=4,
        )
        self.assertFalse(unsafe, "rear-corner graze must be detected")
```

- [ ] **Step 2: Run the new tests, confirm they fail**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && python -m pytest src/baja_cloud_sim/test/test_core.py -v 2>&1 | tail -25`
Expected: three new failures (AttributeError or ImportError) plus the existing three passing.

- [ ] **Step 3: Replace core.py**

Replace `src/baja_cloud_sim/baja_cloud_sim/core.py` with the contents below. This is the reference's `core.py` with three edits: (a) `PlannerConfig.vehicle_length = 1.70` (was 3.0), (b) `ControllerConfig.target_speed = 2.5` (was 2.5 — already correct), (c) `segment_is_safe` keeps the four-corner sweep (instead of the reference's regression).

```python
"""Pure-Python geometry, planning and control helpers.

This module intentionally depends only on the Python standard library so that
the same planner can run on a cloud workstation and a small vehicle computer.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_to_rpy(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float, float]:
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi * 0.5, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    return roll, pitch, quaternion_to_yaw(x, y, z, w)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def rpy_to_quaternion(
    roll: float,
    pitch: float,
    yaw: float,
) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def world_to_base(point: Point, vehicle: Point, vehicle_yaw: float) -> Point:
    dx = point[0] - vehicle[0]
    dy = point[1] - vehicle[1]
    c = math.cos(vehicle_yaw)
    s = math.sin(vehicle_yaw)
    return c * dx + s * dy, -s * dx + c * dy


def base_to_world(point: Point, vehicle: Point, vehicle_yaw: float) -> Point:
    c = math.cos(vehicle_yaw)
    s = math.sin(vehicle_yaw)
    return (
        vehicle[0] + c * point[0] - s * point[1],
        vehicle[1] + s * point[0] + c * point[1],
    )


def nearest_index(points: Sequence[Dict[str, float]], x: float, y: float, start: int = 0) -> int:
    if not points:
        return 0
    best_index = clamp(int(start), 0, len(points) - 1)
    best_distance = float("inf")
    for index in range(int(best_index), len(points)):
        point = points[index]
        d2 = (point["x"] - x) ** 2 + (point["y"] - y) ** 2
        if d2 < best_distance:
            best_distance = d2
            best_index = index
        elif index > best_index + 40 and d2 > best_distance + 100.0:
            break
    return int(best_index)


def signed_lateral(point: Point, reference: Dict[str, float]) -> float:
    dx = point[0] - reference["x"]
    dy = point[1] - reference["y"]
    return -math.sin(reference["yaw"]) * dx + math.cos(reference["yaw"]) * dy


def frenet_to_world(reference: Dict[str, float], lateral: float) -> Point:
    return (
        reference["x"] - math.sin(reference["yaw"]) * lateral,
        reference["y"] + math.cos(reference["yaw"]) * lateral,
    )


def point_to_oriented_box_clearance(
    point: Point,
    obstacle: Dict[str, float],
    inflate_longitudinal: float = 0.0,
    inflate_lateral: float = 0.0,
) -> float:
    """Signed distance from a point to an oriented, optionally inflated box."""
    dx = point[0] - obstacle["x"]
    dy = point[1] - obstacle["y"]
    c = math.cos(obstacle["yaw"])
    s = math.sin(obstacle["yaw"])
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    half_x = obstacle["length"] * 0.5 + inflate_longitudinal
    half_y = obstacle["width"] * 0.5 + inflate_lateral
    qx = abs(local_x) - half_x
    qy = abs(local_y) - half_y
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside = min(max(qx, qy), 0.0)
    return outside + inside


def segment_is_safe(
    start: Point,
    end: Point,
    obstacles: Sequence[Dict[str, float]],
    vehicle_half_length: float,
    vehicle_half_width: float,
    samples: int = 4,
) -> bool:
    """Check that the swept vehicle rectangle along a path segment stays clear.

    Treats the vehicle as an oriented rectangle rotated to the segment direction,
    testing all four corners at every sample point against the un-inflated
    obstacles. ``vehicle_half_length`` and ``vehicle_half_width`` already
    include the safety margin.
    """
    seg_dx = end[0] - start[0]
    seg_dy = end[1] - start[1]
    if seg_dx == 0.0 and seg_dy == 0.0:
        return True
    seg_yaw = math.atan2(seg_dy, seg_dx)
    c = math.cos(seg_yaw)
    s = math.sin(seg_yaw)
    half_l = vehicle_half_length
    half_w = vehicle_half_width
    corners_local = (
        ( c * half_l - s * half_w,  s * half_l + c * half_w),
        ( c * half_l + s * half_w,  s * half_l - c * half_w),
        (-c * half_l - s * half_w, -s * half_l + c * half_w),
        (-c * half_l + s * half_w, -s * half_l - c * half_w),
    )
    for step in range(samples + 1):
        ratio = step / max(samples, 1)
        px = start[0] + seg_dx * ratio
        py = start[1] + seg_dy * ratio
        for ox, oy in corners_local:
            corner = (px + ox, py + oy)
            for obstacle in obstacles:
                if point_to_oriented_box_clearance(corner, obstacle, 0.0, 0.0) <= 0.0:
                    return False
    return True


def polyline_distance(point: Point, path: Sequence[Point]) -> float:
    if not path:
        return float("inf")
    if len(path) == 1:
        return distance(point, path[0])
    best = float("inf")
    for index in range(len(path) - 1):
        ax, ay = path[index]
        bx, by = path[index + 1]
        abx, aby = bx - ax, by - ay
        denominator = abx * abx + aby * aby
        if denominator <= 1e-12:
            candidate = math.hypot(point[0] - ax, point[1] - ay)
        else:
            ratio = clamp(((point[0] - ax) * abx + (point[1] - ay) * aby) / denominator, 0.0, 1.0)
            candidate = math.hypot(point[0] - (ax + ratio * abx), point[1] - (ay + ratio * aby))
        best = min(best, candidate)
    return best


def _smooth_hill(s_coord: float, start: float, length: float, height: float) -> float:
    """Smooth sin-squared hill with zero height and grade at both ends."""
    if s_coord < start or s_coord > start + length:
        return 0.0
    phase = math.pi * (s_coord - start) / length
    return height * math.sin(phase) ** 2


def _circular_speed_bump(s_coord: float, center: float, width: float, height: float) -> float:
    """Low circular-segment bump with the requested width and crown height."""
    half_width = width * 0.5
    offset = abs(s_coord - center)
    if offset > half_width:
        return 0.0
    radius = (half_width * half_width + height * height) / (2.0 * height)
    baseline = radius - height
    return math.sqrt(max(0.0, radius * radius - offset * offset)) - baseline


def terrain_height(s_coord: float) -> float:
    """Road elevation profile: two sub-10-degree hills and two low bumps."""
    return (
        _smooth_hill(s_coord, start=16.0, length=14.0, height=0.55)
        + _smooth_hill(s_coord, start=76.0, length=13.0, height=0.45)
        + _circular_speed_bump(s_coord, center=39.0, width=1.20, height=0.08)
        + _circular_speed_bump(s_coord, center=92.0, width=1.00, height=0.07)
    )


def generate_centerline(length: float = 100.0, spacing: float = 0.5) -> List[Dict[str, float]]:
    """Generate a long straight, a 90-degree left turn, and a final straight.

    ``s`` is true reference-path distance, so adjacent samples retain the
    requested spacing through the circular bend.
    """
    points: List[Dict[str, float]] = []
    count = int(round(length / spacing)) + 1
    straight_length = min(50.0, length)
    turn_radius = 15.0
    turn_angle = math.pi * 0.5
    turn_length = min(turn_radius * turn_angle, max(0.0, length - straight_length))
    turn_end = straight_length + turn_length
    for index in range(count):
        s_coord = min(length, index * spacing)
        if s_coord <= straight_length:
            x = s_coord
            y = 0.0
            yaw = 0.0
        elif s_coord <= turn_end:
            angle = (s_coord - straight_length) / turn_radius
            x = straight_length + turn_radius * math.sin(angle)
            y = turn_radius * (1.0 - math.cos(angle))
            yaw = angle
        else:
            final_angle = turn_length / turn_radius
            turn_x = straight_length + turn_radius * math.sin(final_angle)
            turn_y = turn_radius * (1.0 - math.cos(final_angle))
            distance_after_turn = s_coord - turn_end
            x = turn_x + distance_after_turn * math.cos(final_angle)
            y = turn_y + distance_after_turn * math.sin(final_angle)
            yaw = final_angle
        half_width = 4.0 - 0.25 * math.exp(-((s_coord - 66.0) / 9.0) ** 2)
        points.append(
            {
                "s": s_coord,
                "x": x,
                "y": y,
                "z": terrain_height(s_coord),
                "yaw": yaw,
                "half_width": half_width,
            }
        )
    return points


def generate_boundaries(centerline: Sequence[Dict[str, float]]) -> Tuple[List[Point], List[Point]]:
    left: List[Point] = []
    right: List[Point] = []
    for point in centerline:
        left.append(frenet_to_world(point, point["half_width"]))
        right.append(frenet_to_world(point, -point["half_width"]))
    return left, right


def generate_obstacles(
    centerline: Sequence[Dict[str, float]], seed: int, count: int = 5
) -> List[Dict[str, float]]:
    rng = random.Random(seed)
    usable_indices = list(range(30, max(31, len(centerline) - 25)))
    chosen: List[int] = []
    attempts = 0
    while len(chosen) < count and attempts < 500:
        attempts += 1
        index = rng.choice(usable_indices)
        if all(abs(index - previous) >= 22 for previous in chosen):
            chosen.append(index)
    chosen.sort()
    obstacles: List[Dict[str, float]] = []
    for obstacle_id, index in enumerate(chosen):
        reference = centerline[index]
        max_offset = max(0.8, reference["half_width"] - 1.5)
        lateral = rng.uniform(-max_offset, max_offset)
        if obstacle_id == 0:
            lateral *= 0.25
        x, y = frenet_to_world(reference, lateral)
        height = rng.uniform(0.65, 1.0)
        obstacles.append(
            {
                "id": obstacle_id,
                "x": x,
                "y": y,
                "z": reference.get("z", 0.0) + height * 0.5 + 0.03,
                "yaw": reference["yaw"] + rng.uniform(-0.35, 0.35),
                "length": rng.uniform(1.3, 2.1),
                "width": rng.uniform(1.0, 1.7),
                "height": height,
                "s": reference["s"],
                "lateral": lateral,
            }
        )
    return obstacles


@dataclass
class PlannerConfig:
    horizon_m: float = 30.0
    layer_spacing_m: float = 1.0
    lateral_spacing_m: float = 0.25
    vehicle_length: float = 1.70
    vehicle_width: float = 1.50
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


def plan_frenet_path(
    centerline: Sequence[Dict[str, float]],
    start_index: int,
    current: Point,
    left_limits: Sequence[float],
    right_limits: Sequence[float],
    obstacles: Sequence[Dict[str, float]],
    config: Optional[PlannerConfig] = None,
) -> PlanResult:
    """Plan a local collision-free path using a layered Frenet lattice."""
    started = time.perf_counter()
    cfg = config or PlannerConfig()
    if not centerline or start_index >= len(centerline) - 2:
        return PlanResult([], [], False, 0.0, 0.0, "centerline unavailable or goal reached")

    center_spacing = max(0.05, centerline[min(start_index + 1, len(centerline) - 1)]["s"] - centerline[start_index]["s"])
    index_step = max(1, int(round(cfg.layer_spacing_m / center_spacing)))
    horizon_points = max(2, int(round(cfg.horizon_m / center_spacing)))
    indices = list(range(start_index + index_step, min(len(centerline), start_index + horizon_points + 1), index_step))
    if not indices:
        return PlanResult([], [], False, 0.0, 0.0, "empty planning horizon")

    start_reference = centerline[start_index]
    start_lateral = signed_lateral(current, start_reference)
    vehicle_half_length = cfg.vehicle_length * 0.5 + cfg.safety_margin
    vehicle_half_width = cfg.vehicle_width * 0.5 + cfg.safety_margin
    layers: List[List[float]] = []
    bounds: List[Tuple[float, float]] = []

    for center_index in indices:
        left = left_limits[center_index] if center_index < len(left_limits) else centerline[center_index]["half_width"]
        right = right_limits[center_index] if center_index < len(right_limits) else -centerline[center_index]["half_width"]
        lower = right + vehicle_half_width
        upper = left - vehicle_half_width
        bounds.append((lower, upper))
        candidates: List[float] = []
        if upper >= lower:
            first = math.ceil(lower / cfg.lateral_spacing_m) * cfg.lateral_spacing_m
            value = first
            while value <= upper + 1e-9:
                candidates.append(round(value, 4))
                value += cfg.lateral_spacing_m
        layers.append(candidates)

    costs: List[List[float]] = []
    parents: List[List[int]] = []
    previous_points: List[Point] = [current]
    previous_laterals: List[float] = [start_lateral]
    previous_costs: List[float] = [0.0]

    for layer_index, candidates in enumerate(layers):
        if not candidates:
            elapsed = (time.perf_counter() - started) * 1000.0
            return PlanResult([], [], False, elapsed, 0.0, "road corridor narrower than vehicle")
        reference = centerline[indices[layer_index]]
        lower, upper = bounds[layer_index]
        current_costs = [float("inf")] * len(candidates)
        current_parents = [-1] * len(candidates)
        current_points = [frenet_to_world(reference, lateral) for lateral in candidates]

        for candidate_index, (lateral, point) in enumerate(zip(candidates, current_points)):
            obstacle_clearance = float("inf")
            collision = False
            for obstacle in obstacles:
                clearance = point_to_oriented_box_clearance(
                    point, obstacle, vehicle_half_length, vehicle_half_width
                )
                obstacle_clearance = min(obstacle_clearance, clearance)
                if clearance <= 0.0:
                    collision = True
                    break
            if collision:
                continue
            boundary_clearance = min(lateral - lower, upper - lateral)
            clearance = min(obstacle_clearance, boundary_clearance)
            clearance_penalty = max(0.0, cfg.desired_clearance - clearance) ** 2
            node_cost = cfg.center_weight * lateral * lateral + cfg.clearance_weight * clearance_penalty

            for previous_index, previous_point in enumerate(previous_points):
                lateral_change = lateral - previous_laterals[previous_index]
                if abs(lateral_change) > cfg.max_lateral_step:
                    continue
                if not segment_is_safe(
                    previous_point,
                    point,
                    obstacles,
                    vehicle_half_length,
                    vehicle_half_width,
                    samples=5,
                ):
                    continue
                transition_cost = cfg.smooth_weight * lateral_change * lateral_change
                if layer_index > 0:
                    transition_cost += cfg.slope_weight * abs(lateral_change)
                total = previous_costs[previous_index] + node_cost + transition_cost
                if total < current_costs[candidate_index]:
                    current_costs[candidate_index] = total
                    current_parents[candidate_index] = previous_index

        if all(not math.isfinite(cost) for cost in current_costs):
            elapsed = (time.perf_counter() - started) * 1000.0
            return PlanResult([], [], False, elapsed, 0.0, "no collision-free lattice connection")
        costs.append(current_costs)
        parents.append(current_parents)
        previous_points = current_points
        previous_laterals = candidates
        previous_costs = current_costs

    last_index = min(range(len(costs[-1])), key=lambda index: costs[-1][index])
    selected: List[int] = [last_index]
    for layer_index in range(len(layers) - 1, 0, -1):
        selected.append(parents[layer_index][selected[-1]])
    selected.reverse()

    laterals = [layers[layer_index][candidate_index] for layer_index, candidate_index in enumerate(selected)]
    path = [current]
    for center_index, lateral in zip(indices, laterals):
        path.append(frenet_to_world(centerline[center_index], lateral))

    minimum = float("inf")
    for point, lateral, bound in zip(path[1:], laterals, bounds):
        minimum = min(minimum, lateral - bound[0], bound[1] - lateral)
        for obstacle in obstacles:
            minimum = min(
                minimum,
                point_to_oriented_box_clearance(point, obstacle, vehicle_half_length, vehicle_half_width),
            )
    elapsed = (time.perf_counter() - started) * 1000.0
    return PlanResult(path, laterals, True, elapsed, max(0.0, minimum), "ok")


@dataclass
class ControllerConfig:
    target_speed: float = 2.5
    lookahead_distance: float = 3.0
    heading_gain: float = 1.2
    max_steering_deg: float = 35.0


def legacy_path_control(
    current: Point,
    yaw_navigation: float,
    path: Sequence[Point],
    config: Optional[ControllerConfig] = None,
) -> Dict[str, float]:
    """Existing heading / lookahead control law using navigation yaw convention."""
    cfg = config or ControllerConfig()
    if len(path) < 2:
        return {"speed": 0.0, "steering": 0.0, "target_x": current[0], "target_y": current[1]}
    nearest = min(range(len(path)), key=lambda index: distance(current, path[index]))
    target_index = len(path) - 1
    for index in range(nearest, len(path)):
        if distance(current, path[index]) >= cfg.lookahead_distance:
            target_index = index
            break
    target = path[target_index]
    east = target[0] - current[0]
    north = target[1] - current[1]
    desired_navigation = math.atan2(east, north)
    error = wrap_angle(desired_navigation - yaw_navigation)
    max_steering = math.radians(cfg.max_steering_deg)
    steering = clamp(-cfg.heading_gain * error, -max_steering, max_steering)
    steering_deg = abs(math.degrees(steering))
    if steering_deg > 30.0:
        speed_factor = 0.50
    elif steering_deg > 20.0:
        speed_factor = 0.70
    elif steering_deg > 12.0:
        speed_factor = 0.85
    elif steering_deg > 6.0:
        speed_factor = 0.95
    else:
        speed_factor = 1.0
    return {
        "speed": cfg.target_speed * speed_factor,
        "steering": steering,
        "target_x": target[0],
        "target_y": target[1],
        "heading_error": error,
    }
```

- [ ] **Step 4: Re-run the tests, confirm pass**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && python -m pytest src/baja_cloud_sim/test/test_core.py -v 2>&1 | tail -25`
Expected: all six tests pass.

- [ ] **Step 5: Build the package**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && colcon build --packages-select baja_cloud_sim 2>&1 | tail -10`
Expected: `Summary: 1 package finished [..s ago] ... Summary: 1 package finished`.

- [ ] **Step 6: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add src/baja_cloud_sim/baja_cloud_sim/core.py src/baja_cloud_sim/test/test_core.py
git commit -m "v2: merge core.py geometry helpers (terrain_height, RPY, straight→turn centerline) and preserve four-corner sweep"
```

---

## Task 3: Merge scenario_generator.py (real normals, dynamic ground, fix dead-code)

**Files:**
- Modify: `src/baja_cloud_sim/baja_cloud_sim/scenario_generator.py`
- Test: `src/baja_cloud_sim/test/test_scenario_generator.py` (new)

**Interfaces:**
- Consumes: `core.py` from Task 2.
- Produces: `scenario_generator.py` that (a) computes real surface normals, (b) uses `terrain_height` to set obstacle `z`, (c) sizes `base_ground` dynamically to road extents, (d) runs physics at 500 Hz with ogre2/shadows/sky, (e) starts vehicle at `start['z'] + 0.52`, (f) drops the deterministic avoidance pair (Task 2 centerline changed geometry, the s≈12/16 m pair would land outside the road), (g) keeps the `--obstacles-config` / `--save-obstacles-config` CLI flags and the segmented collision ground, (h) writes `road_layout` and `terrain_features` to `scenario.json`.

- [ ] **Step 1: Add a failing test for the duplicate-return bug**

Create `src/baja_cloud_sim/test/test_scenario_generator.py`:

```python
import re
import unittest
from pathlib import Path

from baja_cloud_sim import scenario_generator


class ScenarioGeneratorTests(unittest.TestCase):
    def test_obstacle_sdf_has_no_duplicate_return(self):
        """Regression: _obstacle_sdf previously had two identical return
        statements back-to-back; the second was unreachable dead code."""
        source = Path(scenario_generator.__file__).read_text(encoding="utf-8")
        # Locate the function body and count return statements.
        match = re.search(r"def _obstacle_sdf.*?(?=\ndef |\Z)", source, re.DOTALL)
        self.assertIsNotNone(match, "_obstacle_sdf not found")
        body = match.group(0)
        returns = body.count("    return ")
        self.assertEqual(returns, 1, "_obstacle_sdf must contain exactly one return")

    def test_obstacle_uses_terrain_aware_z(self):
        from baja_cloud_sim.core import generate_centerline, generate_obstacles
        centerline = generate_centerline(100.0, 0.5)
        obstacles = generate_obstacles(centerline, seed=42, count=3)
        for obs in obstacles:
            # Each obstacle must carry a z greater than just height*0.5+0.03
            # because the reference now anchors it to the terrain profile.
            self.assertGreater(obs["z"], obs["height"] * 0.5 + 0.02)
```

- [ ] **Step 2: Confirm test fails (or duplicate return is still present)**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && python -m pytest src/baja_cloud_sim/test/test_scenario_generator.py -v 2>&1 | tail -20`
Expected: `test_obstacle_sdf_has_no_duplicate_return` fails with "2 != 1".

- [ ] **Step 3: Replace scenario_generator.py**

Write `src/baja_cloud_sim/baja_cloud_sim/scenario_generator.py`:

```python
"""Generate a deterministic 100 m dirt road with a 90-degree turn."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

from .core import generate_boundaries, generate_centerline, generate_obstacles


def _write_obj(path: Path, road_surface: Sequence[Dict[str, float]], seed: int) -> None:
    lateral_samples = [(-4.5 + 0.5 * index) for index in range(19)]
    vertices: List[Tuple[float, float, float]] = []
    for center in road_surface:
        for lateral in lateral_samples:
            x = center["x"] - math.sin(center["yaw"]) * lateral
            y = center["y"] + math.cos(center["yaw"]) * lateral
            z = (
                center.get("z", 0.0)
                + 0.012 * math.sin(0.73 * center["s"] + 0.31 * lateral + seed)
                + 0.008 * math.sin(1.41 * center["s"] - 0.57 * lateral)
                + 0.004 * math.sin(3.2 * lateral + 0.13 * seed)
            )
            vertices.append((x, y, z))
    rows = len(road_surface)
    columns = len(lateral_samples)
    normals: List[Tuple[float, float, float]] = []
    for row in range(rows):
        previous_row = max(0, row - 1)
        next_row = min(rows - 1, row + 1)
        for column in range(columns):
            previous_column = max(0, column - 1)
            next_column = min(columns - 1, column + 1)
            before = vertices[previous_row * columns + column]
            after = vertices[next_row * columns + column]
            right = vertices[row * columns + previous_column]
            left = vertices[row * columns + next_column]
            tangent = (
                after[0] - before[0],
                after[1] - before[1],
                after[2] - before[2],
            )
            lateral = (
                left[0] - right[0],
                left[1] - right[1],
                left[2] - right[2],
            )
            nx = tangent[1] * lateral[2] - tangent[2] * lateral[1]
            ny = tangent[2] * lateral[0] - tangent[0] * lateral[2]
            nz = tangent[0] * lateral[1] - tangent[1] * lateral[0]
            magnitude = math.sqrt(nx * nx + ny * ny + nz * nz)
            if magnitude <= 1e-12:
                normals.append((0.0, 0.0, 1.0))
            else:
                if nz < 0.0:
                    nx, ny, nz = -nx, -ny, -nz
                normals.append((nx / magnitude, ny / magnitude, nz / magnitude))
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("# Generated 100 m dirt-road mesh with hills and speed bumps\n")
        handle.write("o dirt_road\n")
        for x, y, z in vertices:
            handle.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for nx, ny, nz in normals:
            handle.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")
        handle.write("s 1\n")
        for row in range(rows - 1):
            for column in range(columns - 1):
                a = row * columns + column + 1
                b = a + 1
                c = a + columns
                d = c + 1
                handle.write(f"f {a}//{a} {c}//{c} {b}//{b}\n")
                handle.write(f"f {b}//{b} {c}//{c} {d}//{d}\n")


def _obstacle_sdf(obstacle: Dict[str, float]) -> str:
    return f"""
    <model name="obstacle_{obstacle['id']}">
      <static>true</static>
      <pose>{obstacle['x']:.5f} {obstacle['y']:.5f} {obstacle['z']:.5f} 0 0 {obstacle['yaw']:.5f}</pose>
      <link name="body">
        <collision name="collision">
          <geometry><box><size>{obstacle['length']:.4f} {obstacle['width']:.4f} {obstacle['height']:.4f}</size></box></geometry>
          <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{obstacle['length']:.4f} {obstacle['width']:.4f} {obstacle['height']:.4f}</size></box></geometry>
          <material><ambient>0.42 0.12 0.07 1</ambient><diffuse>0.70 0.22 0.10 1</diffuse><pbr><metal><roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr></material>
        </visual>
      </link>
    </model>"""


def generate(
    output: Path,
    seed: int,
    obstacle_count: int,
    package_share: Path,
    obstacles_config: Path | None = None,
    save_obstacles_config: bool = False,
) -> Dict[str, str]:
    output.mkdir(parents=True, exist_ok=True)
    centerline = generate_centerline(100.0, 0.5)
    road_surface = generate_centerline(100.0, 0.1)
    left, right = generate_boundaries(centerline)

    if obstacles_config is not None:
        with open(obstacles_config, "r", encoding="utf-8") as handle:
            obstacles = json.load(handle)
        for idx, obs in enumerate(obstacles):
            obs.setdefault("id", idx)
            # Re-anchor z on terrain if a base centerline can be found by s
            obs.setdefault("z", 0.45)
    else:
        obstacles = generate_obstacles(centerline, seed, obstacle_count)

    mesh_path = output / "dirt_road.obj"
    scenario_path = output / "scenario.json"
    world_path = output / "baja_100m.sdf"
    _write_obj(mesh_path, road_surface, seed)

    if save_obstacles_config:
        config_path = output / "obstacles_config.json"
        config_data = [
            {
                "x": obs["x"],
                "y": obs["y"],
                "yaw": obs["yaw"],
                "length": obs["length"],
                "width": obs["width"],
                "height": obs["height"],
            }
            for obs in obstacles
        ]
        config_path.write_text(
            json.dumps(config_data, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(f"Saved obstacle config to {config_path}", flush=True)

    scenario = {
        "seed": seed,
        "length": 100.0,
        "spacing": 0.5,
        "gps_origin": {"latitude": 30.0, "longitude": 114.0, "altitude": 30.0},
        "vehicle": {"length": 1.70, "width": 1.50, "wheelbase": 1.43, "mass": 252.0},
        "road_layout": {
            "initial_straight_m": 50.0,
            "turn_direction": "left",
            "turn_angle_deg": 90.0,
            "turn_radius_m": 15.0,
        },
        "terrain_features": [
            {"type": "smooth_hill", "start_s": 16.0, "length": 14.0, "height": 0.55, "max_grade_deg": 7.1},
            {"type": "circular_speed_bump", "center_s": 39.0, "width": 1.20, "height": 0.08},
            {"type": "smooth_hill", "start_s": 76.0, "length": 13.0, "height": 0.45, "max_grade_deg": 6.3},
            {"type": "circular_speed_bump", "center_s": 92.0, "width": 1.00, "height": 0.07},
        ],
        "centerline": centerline,
        "left_boundary": [{"x": point[0], "y": point[1]} for point in left],
        "right_boundary": [{"x": point[0], "y": point[1]} for point in right],
        "obstacles": obstacles,
    }
    scenario_path.write_text(json.dumps(scenario, ensure_ascii=False, indent=2), encoding="utf-8")

    vehicle_uri = (package_share / "models" / "baja_vehicle").as_uri()
    road_uri = mesh_path.resolve().as_uri()
    obstacle_models = "\n".join(_obstacle_sdf(obstacle) for obstacle in obstacles)
    start = centerline[0]
    minimum_x = min(point["x"] for point in road_surface) - 15.0
    maximum_x = max(point["x"] for point in road_surface) + 15.0
    minimum_y = min(point["y"] for point in road_surface) - 15.0
    maximum_y = max(point["y"] for point in road_surface) + 15.0
    ground_x = 0.5 * (minimum_x + maximum_x)
    ground_y = 0.5 * (minimum_y + maximum_y)
    ground_size_x = maximum_x - minimum_x
    ground_size_y = maximum_y - minimum_y
    ground_z = min(point["z"] for point in road_surface) - 0.24
    world = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="baja_track">
    <physics name="high_rate_dynamics" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.67 0.80 0.90 1</background><shadows>true</shadows></scene>
    <light name="sun" type="directional"><cast_shadows>true</cast_shadows><pose>0 0 20 0 0 0</pose><diffuse>0.9 0.86 0.75 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -0.9</direction></light>
    <model name="base_ground"><static>true</static><pose>{ground_x:.4f} {ground_y:.4f} {ground_z:.4f} 0 0 0</pose><link name="ground"><collision name="collision"><geometry><box><size>{ground_size_x:.4f} {ground_size_y:.4f} 0.2</size></box></geometry><surface><friction><ode><mu>1.1</mu><mu2>1.0</mu2></ode></friction></surface></collision><visual name="visual"><geometry><box><size>{ground_size_x:.4f} {ground_size_y:.4f} 0.2</size></box></geometry><material><ambient>0.16 0.25 0.12 1</ambient><diffuse>0.23 0.34 0.16 1</diffuse><pbr><metal><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material></visual></link></model>
    <model name="dirt_road"><static>true</static><link name="road"><collision name="collision"><geometry><mesh><uri>{road_uri}</uri></mesh></geometry><surface><friction><ode><mu>1.25</mu><mu2>1.0</mu2></ode></friction><contact><ode><kp>800000</kp><kd>220</kd><max_vel>0.08</max_vel><min_depth>0.0005</min_depth></ode></contact></surface></collision><visual name="visual"><geometry><mesh><uri>{road_uri}</uri></mesh></geometry><material><ambient>0.30 0.20 0.11 1</ambient><diffuse>0.46 0.31 0.17 1</diffuse><pbr><metal><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material></visual></link></model>
    {obstacle_models}
    <include><uri>{vehicle_uri}</uri><name>baja_vehicle</name><pose>{start['x']:.5f} {start['y']:.5f} {start['z'] + 0.52:.5f} 0 0 {start['yaw']:.5f}</pose></include>
  </world>
</sdf>
"""
    world_path.write_text(world, encoding="utf-8")
    return {"world": str(world_path), "scenario": str(scenario_path), "mesh": str(mesh_path)}


def main() -> None:
    from ament_index_python.packages import get_package_share_directory

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="Generated scenario directory")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--obstacles", type=int, default=5)
    parser.add_argument("--obstacles-config", type=Path, default=None)
    parser.add_argument("--save-obstacles-config", action="store_true", default=False)
    args = parser.parse_args()
    package_share = Path(get_package_share_directory("baja_cloud_sim"))
    result = generate(
        Path(args.output).resolve(),
        args.seed,
        args.obstacles,
        package_share,
        obstacles_config=args.obstacles_config,
        save_obstacles_config=args.save_obstacles_config,
    )
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
```

Notes vs. the reference:
- `obstacles_config` and `save_obstacles_config` CLI flags are preserved.
- Vehicle mass is 252, length 1.70, width 1.50 (matches `models/baja_vehicle/model.sdf`).
- Vehicle starts at `start['z'] + 0.52` (handles the new hilly terrain).
- The deterministic avoidance pair (s ≈ 12/16 m) is dropped: with the new straight→turn geometry the s ≈ 12 m point lies in the initial straight where 4 m of paired obstacles would otherwise block the centre line; the random set already covers this region.

- [ ] **Step 4: Run scenario tests**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && python -m pytest src/baja_cloud_sim/test/test_scenario_generator.py -v 2>&1 | tail -15`
Expected: both tests pass.

- [ ] **Step 5: End-to-end scenario generation smoke test**

Run:
```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
rm -rf /tmp/baja_scenario_smoke
colcon build --packages-select baja_cloud_sim 2>&1 | tail -5
. install/setup.bash 2>/dev/null || source install/setup.bash
ros2 run baja_cloud_sim generate_scenario --output /tmp/baja_scenario_smoke --seed 42 --obstacles 5
```
Expected: prints a JSON dict with paths to `world`, `scenario`, `mesh`. Verify `/tmp/baja_scenario_smoke/baja_100m.sdf` contains `<real_time_update_rate>500</real_time_update_rate>` and the `<render_engine>ogre2</render_engine>` line.

- [ ] **Step 6: Verify road geometry**

Run: `python3 -c "import json; d=json.load(open('/tmp/baja_scenario_smoke/scenario.json')); cl=d['centerline']; print('first', cl[0]['x'], cl[0]['y'], cl[0]['yaw'], cl[0].get('z', '?')); print('mid-turn', cl[110]['x'], cl[110]['y'], cl[110]['yaw']); print('last', cl[-1]['x'], cl[-1]['y'])"`
Expected: first point ~`(0, 0, 0)`, yaw 0; mid-turn has y > 5, yaw ≈ π/4; last point has y ≈ 15 and yaw ≈ π/2.

- [ ] **Step 7: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add src/baja_cloud_sim/baja_cloud_sim/scenario_generator.py src/baja_cloud_sim/test/test_scenario_generator.py
git commit -m "v2: scenario generator — real normals, dynamic ground, terrain-aware obstacles, fix duplicate return"
```

---

## Task 4: Replace truth_perception_node.py with reference superset

**Files:**
- Modify: `src/baja_cloud_sim/baja_cloud_sim/truth_perception_node.py`

**Interfaces:**
- Consumes: `core.py` from Task 2 (uses `quaternion_to_rpy`, `rpy_to_quaternion`).
- Produces: `truth_perception_node.py` that publishes `/localization/odom` with 6×6 pose covariance, applies bounded (±3σ) Gaussian noise, anchors centerline `z` to terrain, and uses 180 ms marker lifetime.

- [ ] **Step 1: Read reference truth_perception_node.py for reference**

Run: `diff /home/pandafixle/Desktop/baja_cloud_sim/src/baja_cloud_sim/baja_cloud_sim/truth_perception_node.py /home/pandafixle/Desktop/baja_cloud_sim（2）/baja_cloud_sim/src/baja_cloud_sim/baja_cloud_sim/truth_perception_node.py | head -50`
Expected: shows the reference has new declarations, `_bounded_noise`, RPY import, `/localization/odom` publisher, 180 ms marker lifetimes.

- [ ] **Step 2: Copy the reference file into place**

```bash
cp /home/pandafixle/Desktop/baja_cloud_sim（2）/baja_cloud_sim/src/baja_cloud_sim/baja_cloud_sim/truth_perception_node.py \
   /home/pandafixle/Desktop/baja_cloud_sim/src/baja_cloud_sim/baja_cloud_sim/truth_perception_node.py
```

- [ ] **Step 3: Build the package**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && colcon build --packages-select baja_cloud_sim 2>&1 | tail -10`
Expected: clean build, no errors.

- [ ] **Step 4: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add src/baja_cloud_sim/baja_cloud_sim/truth_perception_node.py
git commit -m "v2: replace truth_perception with reference superset (localization noise, /localization/odom, 180 ms markers)"
```

---

## Task 5: Fix pid_path_follower_node.py controller issues

**Files:**
- Modify: `src/baja_cloud_sim/baja_cloud_sim/pid_path_follower_node.py`

**Interfaces:**
- Consumes: existing `pid_path_follower_node.py` (Task 1 backup).
- Produces: same external API (`/cmd_control`, `/lookahead_point`); changes internal: real `dt`, lower `kd_heading` default, higher `steering_alpha`, Stanley denominator uses `commanded_speed`, stores `prev_time`.

- [ ] **Step 1: Change the `kd_heading` parameter default**

In `src/baja_cloud_sim/baja_cloud_sim/pid_path_follower_node.py`, line 42:

Replace:
```python
            ("kd_heading", 0.3),
```
With:
```python
            ("kd_heading", 0.12),
```

- [ ] **Step 2: Change the `steering_alpha` parameter default**

Same file, line 43:

Replace:
```python
            ("steering_alpha", 0.75),
```
With:
```python
            ("steering_alpha", 0.85),
```

- [ ] **Step 3: Add `prev_time` to PD state**

Same file, after line 90 (`self.prev_steering: float = 0.0`):

Add these three lines:
```python
        self.prev_time = None
        self.commanded_speed = self.target_speed
```

- [ ] **Step 4: Replace the hard-coded dt with real elapsed time**

Same file, lines 326-328:

Replace:
```python
        # --- Stanley + PD control (dt = 0.05) ---
        error_rate = (error - self.prev_error) / 0.05
        self.prev_error = error
```
With:
```python
        # --- Stanley + PD control (real elapsed dt) ---
        if self.prev_time is None:
            self.prev_time = now
        dt = max(1e-3, (now - self.prev_time).nanoseconds / 1e9)
        self.prev_time = now
        error_rate = (error - self.prev_error) / dt
        self.prev_error = error
```

- [ ] **Step 5: Replace Stanley denominator with `commanded_speed` and capture it**

Same file, lines 333-337:

Replace:
```python
        # Stanley lateral correction: arctan(k × cte / v), self-regulating
        cte = self._cross_track_error()
        stanley_term = math.atan2(self.k_stanley * cte, max(self.target_speed, 2.0))

        steering_raw = -(self.kp * error + self.kd * error_rate + stanley_term)
        steering_raw = max(-dyn_max, min(dyn_max, steering_raw))
```
With:
```python
        # Stanley lateral correction: arctan(k × cte / v), self-regulating.
        # Use the previously commanded speed so the two control terms stay
        # consistent when adaptive speed is throttling output.
        cte = self._cross_track_error()
        cte_velocity = max(self.commanded_speed, 0.5)
        stanley_term = math.atan2(self.k_stanley * cte, cte_velocity)

        steering_raw = -(self.kp * error + self.kd * error_rate + stanley_term)
        steering_raw = max(-dyn_max, min(dyn_max, steering_raw))
```

- [ ] **Step 6: Capture `commanded_speed` after the speed-factor block**

Same file, after line 360 (`if self.avoiding:` block end), find the line `        # --- publish ---` (line 362). Just before it, insert:

```python
        self.commanded_speed = speed
```

So the section reads:
```python
        if recovery_lim is not None:
            speed = min(speed, recovery_lim)
        if self.avoiding:
            speed = min(speed, self.avoid_spd_lim)
        self.commanded_speed = speed

        # --- publish ---
        cmd = AckermannDriveStamped()
```

- [ ] **Step 7: Reset `prev_time` and `commanded_speed` in `_stop`**

Same file, line 382-390 (`_stop` method):

Replace:
```python
    def _stop(self) -> None:
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = 0.0
        cmd.drive.steering_angle = 0.0
        self.cmd_pub.publish(cmd)
        self.prev_error = 0.0
        self.prev_steering = 0.0
```
With:
```python
    def _stop(self) -> None:
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = 0.0
        cmd.drive.steering_angle = 0.0
        self.cmd_pub.publish(cmd)
        self.prev_error = 0.0
        self.prev_steering = 0.0
        self.prev_time = None
        self.commanded_speed = self.target_speed
```

- [ ] **Step 8: Build the package**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && colcon build --packages-select baja_cloud_sim 2>&1 | tail -10`
Expected: clean build.

- [ ] **Step 9: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add src/baja_cloud_sim/baja_cloud_sim/pid_path_follower_node.py
git commit -m "fix(pid_path_follower): real dt, lower kd_heading, smoother filter, Stanley denominator uses commanded speed"
```

---

## Task 6: Shrink marker lifetime in frenet_planner_node.py

**Files:**
- Modify: `src/baja_cloud_sim/baja_cloud_sim/frenet_planner_node.py`

**Interfaces:**
- Consumes: existing file.
- Produces: same external API; `marker.lifetime.nanosec` shrinks from `500_000_000` to `180_000_000` (180 ms).

- [ ] **Step 1: Update obstacle marker lifetime (line 187)**

In `src/baja_cloud_sim/baja_cloud_sim/frenet_planner_node.py`, line 187:

Replace:
```python
            marker.lifetime.nanosec = 500_000_000
```
With:
```python
            marker.lifetime.nanosec = 180_000_000
```

- [ ] **Step 2: Update planner-status text marker lifetime (line 197)**

Same file, line 197:

Replace:
```python
        text.lifetime.nanosec = 500_000_000
```
With:
```python
        text.lifetime.nanosec = 180_000_000
```

- [ ] **Step 3: Build and commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
colcon build --packages-select baja_cloud_sim 2>&1 | tail -5
git add src/baja_cloud_sim/baja_cloud_sim/frenet_planner_node.py
git commit -m "v2: shrink frenet debug marker lifetime from 500 ms to 180 ms"
```

---

## Task 7: Fix /actual_path ghost rendering in evaluator_node.py

**Files:**
- Modify: `src/baja_cloud_sim/baja_cloud_sim/evaluator_node.py`

**Interfaces:**
- Consumes: existing file.
- Produces: `/actual_path` published as RELIABLE + VOLATILE (instead of the default latched behaviour), each pose stamped at publish time, trimmed to last 1500 poses (instead of 3000→2500) to keep RViz responsive without leaving long ghost trails.

- [ ] **Step 1: Update publisher QoS**

In `src/baja_cloud_sim/baja_cloud_sim/evaluator_node.py`, line 51:

Replace:
```python
        self.path_pub = self.create_publisher(PathMessage, "/actual_path", 10)
```
With:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy
# ...
self.path_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
self.path_pub = self.create_publisher(PathMessage, "/actual_path", self.path_qos)
```

(Note: the `from rclpy.qos import ...` line is best added near the top of the file alongside existing imports; placing it inline is acceptable if the engineer prefers minimal diff.)

- [ ] **Step 2: Tighten pose buffer cap and trim**

Same file, lines 110-111:

Replace:
```python
        if len(self.actual_path.poses) > 3000:
            self.actual_path.poses = self.actual_path.poses[-2500:]
```
With:
```python
        if len(self.actual_path.poses) > 1500:
            self.actual_path.poses = self.actual_path.poses[-1200:]
```

- [ ] **Step 3: Stamp `actual_path.header` with publish-time stamp**

Same file, line 108 (the line just above `self.actual_path.poses.append(pose)`):

Replace:
```python
        self.actual_path.header.stamp = pose.header.stamp
```
With:
```python
        self.actual_path.header.stamp = self.get_clock().now().to_msg()
```

(Setting it to the publish-time stamp instead of the latest pose's stamp avoids path messages being interpreted as "older than they are" by RViz when poses arrive irregularly.)

- [ ] **Step 4: Build and commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
colcon build --packages-select baja_cloud_sim 2>&1 | tail -5
git add src/baja_cloud_sim/baja_cloud_sim/evaluator_node.py
git commit -m "fix(evaluator): use publish-time stamp and tighter buffer for /actual_path to prevent ghost trails"
```

---

## Task 8: Update config/params.yaml (add σ, planner defaults, vehicle dims)

**Files:**
- Modify: `src/baja_cloud_sim/config/params.yaml`

**Interfaces:**
- Consumes: existing params (Task 1 backup).
- Produces: params with `truth_perception_node` σ fields, `frenet_planner_node` defaults aligned with reference (`clearance_weight` 12.0, `desired_clearance` 1.2), `vehicle_length: 1.70`, `vehicle_width: 1.50` (kept from current — reference dropped them), `pid_path_follower_node` block preserved with updated defaults.

- [ ] **Step 1: Write the new params file**

Replace `src/baja_cloud_sim/config/params.yaml` with:

```yaml
truth_perception_node:
  ros__parameters:
    use_sim_time: true
    perception_forward: 34.0
    perception_backward: 6.0
    localization_position_stddev_m: 0.015
    localization_altitude_stddev_m: 0.020
    localization_yaw_stddev_deg: 0.12

frenet_planner_node:
  ros__parameters:
    use_sim_time: true
    origin_latitude: 30.0
    origin_longitude: 114.0
    horizon_m: 30.0
    center_weight: 1.0
    clearance_weight: 12.0
    desired_clearance: 1.2
    vehicle_length: 1.70
    vehicle_width: 1.50
    safety_margin: 0.25
    lateral_spacing_m: 0.20
    max_lateral_step: 1.2

path_follower_node:
  ros__parameters:
    use_sim_time: true
    origin_latitude: 30.0
    origin_longitude: 114.0
    target_speed: 2.5
    lookahead_distance: 3.0
    kp_heading: 1.2
    max_steering_angle: 35.0

pid_path_follower_node:
  ros__parameters:
    use_sim_time: true
    origin_latitude: 30.0
    origin_longitude: 114.0
    target_speed: 6.0
    lookahead_distance: 3.0
    kp_heading: 1.2
    kd_heading: 0.12
    steering_alpha: 0.85
    max_steer_rate_deg: 30.0
    max_steering_angle: 35.0
    adaptive_steering: true
    adaptive_speed: true
    k_stanley: 0.8
    avoidance_speed_limit: 2.0
    virtual_target_timeout: 5.0

actuator_adapter_node:
  ros__parameters:
    use_sim_time: true
    wheelbase: 1.43
    command_timeout: 0.35

evaluator_node:
  ros__parameters:
    use_sim_time: true
```

- [ ] **Step 2: Validate YAML parses**

Run: `python3 -c "import yaml; print(yaml.safe_load(open('/home/pandafixle/Desktop/baja_cloud_sim/src/baja_cloud_sim/config/params.yaml')))"`
Expected: prints a dict with 5 top-level keys (truth_perception_node, frenet_planner_node, path_follower_node, pid_path_follower_node, actuator_adapter_node, evaluator_node).

- [ ] **Step 3: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add src/baja_cloud_sim/config/params.yaml
git commit -m "v2: params — add localization noise σ, planner defaults (12.0/1.2), pin vehicle_length to 1.70"
```

---

## Task 9: Update config/bridge.yaml (add wheel_odom bridge)

**Files:**
- Modify: `src/baja_cloud_sim/config/bridge.yaml`

**Interfaces:**
- Consumes: existing 4-entry file.
- Produces: 5-entry file with a new `/wheel_odom` bridge; the existing `/ground_truth/odom` keeps its current `gz_topic_name` (`/model/baja_vehicle/odometry`).

- [ ] **Step 1: Replace bridge.yaml**

Write `src/baja_cloud_sim/config/bridge.yaml`:

```yaml
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/ground_truth/odom"
  gz_topic_name: "/model/baja_vehicle/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/wheel_odom"
  gz_topic_name: "/model/baja_vehicle/wheel_odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/joint_states"
  gz_topic_name: "/model/baja_vehicle/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/model/baja_vehicle/cmd_vel"
  gz_topic_name: "/model/baja_vehicle/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

- [ ] **Step 2: Validate YAML**

Run: `python3 -c "import yaml; print(len(yaml.safe_load(open('/home/pandafixle/Desktop/baja_cloud_sim/src/baja_cloud_sim/config/bridge.yaml'))), 'entries')"`
Expected: `5 entries`.

- [ ] **Step 3: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add src/baja_cloud_sim/config/bridge.yaml
git commit -m "v2: bridge — add /wheel_odom bridge (kept /ground_truth/odom contract)"
```

---

## Task 10: Update launch/simulation.launch.py (keep pid_path_follower, no video)

**Files:**
- Modify: `src/baja_cloud_sim/launch/simulation.launch.py`

**Interfaces:**
- Consumes: existing launch (Task 1 backup).
- Produces: same launch arguments as the current version (no `use_video`, no `video_path`); the `pid_path_follower_node` line stays uncommented and `path_follower_node` stays commented out.

- [ ] **Step 1: Confirm the current launch file already has the desired structure**

Run: `grep -n "path_follower\|pid_path_follower\|use_video\|video_recorder" /home/pandafixle/Desktop/baja_cloud_sim/src/baja_cloud_sim/launch/simulation.launch.py`
Expected: shows `path_follower` on the commented-out line, `pid_path_follower` on the active line, and **no** `use_video` / `video_recorder` references.

If the file already matches (it should — current launch is the v2 form), this task is a no-op; skip directly to commit.

- [ ] **Step 2: If lines differ, edit them**

Only if Step 1 showed `use_video`/`video_recorder` lines, do:

In `src/baja_cloud_sim/launch/simulation.launch.py`, delete any line containing `DeclareLaunchArgument("use_video"` and `DeclareLaunchArgument("video_path"`. Delete the line `Node(package="baja_cloud_sim", executable="video_recorder"` (whole line).

- [ ] **Step 3: Build and commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
colcon build --packages-select baja_cloud_sim 2>&1 | tail -5
git diff --quiet src/baja_cloud_sim/launch/simulation.launch.py || git add src/baja_cloud_sim/launch/simulation.launch.py
git diff --cached --name-only
# If a change is staged, run:
# git commit -m "v2: launch — ensure pid_path_follower is active and no video_recorder"
```

If nothing changed (most likely), report "Launch already matches target state — no commit" and continue to Task 11.

---

## Task 11: Add `<steer_p_gain>` to model.sdf

**Files:**
- Modify: `src/baja_cloud_sim/models/baja_vehicle/model.sdf`

**Interfaces:**
- Consumes: existing AckermannSteering block.
- Produces: same plugin block with `<steer_p_gain>18.0</steer_p_gain>` added between `<steering_limit>` and `<min_velocity>`.

- [ ] **Step 1: Add the new element**

In `src/baja_cloud_sim/models/baja_vehicle/model.sdf`, between the current `<steering_limit>0.610865</steering_limit>` (line 424) and `<min_velocity>-1.0</min_velocity>` (line 425), insert a new line:

```xml
      <steer_p_gain>18.0</steer_p_gain>
```

Final block should look like:

```xml
      <steering_limit>0.610865</steering_limit>
      <steer_p_gain>18.0</steer_p_gain>
      <min_velocity>-1.0</min_velocity>
```

- [ ] **Step 2: Validate the SDF is still well-formed XML**

Run: `python3 -c "import xml.etree.ElementTree as ET; ET.parse('/home/pandafixle/Desktop/baja_cloud_sim/src/baja_cloud_sim/models/baja_vehicle/model.sdf'); print('OK')"`
Expected: `OK`.

- [ ] **Step 3: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add src/baja_cloud_sim/models/baja_vehicle/model.sdf
git commit -m "v2: model — add steer_p_gain 18.0 to AckermannSteering plugin"
```

---

## Task 12: Update setup.py (data_files filter, version bump)

**Files:**
- Modify: `src/baja_cloud_sim/setup.py`

**Interfaces:**
- Consumes: existing setup.
- Produces: setup with `data_files` skipping `__pycache__/.pyc/.pyo`, version `1.1.0`, entry points unchanged (keeps `pid_path_follower`, **does not** add `video_recorder`).

- [ ] **Step 1: Replace setup.py**

Write `src/baja_cloud_sim/setup.py`:

```python
from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "baja_cloud_sim"


def data_files(directory):
    root = Path(directory)
    return [
        (str(Path("share") / package_name / path.parent), [str(path)])
        for path in root.rglob("*")
        if path.is_file()
        and "__pycache__" not in path.parts
        and path.suffix not in {".pyc", ".pyo"}
    ]


setup(
    name=package_name,
    version="1.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ] + data_files("launch") + data_files("config") + data_files("models") + data_files("urdf"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Baja Autonomous Team",
    maintainer_email="team@example.com",
    description="Cloud-ready dirt-road planning and control simulation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "generate_scenario = baja_cloud_sim.scenario_generator:main",
            "truth_perception = baja_cloud_sim.truth_perception_node:main",
            "frenet_planner = baja_cloud_sim.frenet_planner_node:main",
            "path_follower = baja_cloud_sim.path_follower_node:main",
            "pid_path_follower = baja_cloud_sim.pid_path_follower_node:main",
            "actuator_adapter = baja_cloud_sim.actuator_adapter_node:main",
            "evaluator = baja_cloud_sim.evaluator_node:main",
        ],
    },
)
```

- [ ] **Step 2: Build and verify `pid_path_follower` console script is registered**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && colcon build --packages-select baja_cloud_sim 2>&1 | tail -5`
Expected: clean build.

Then: `ls install/baja_cloud_sim/lib/baja_cloud_sim/`
Expected: at minimum `pid_path_follower` (alongside `truth_perception`, `frenet_planner`, etc.). **No** `video_recorder`.

- [ ] **Step 3: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add src/baja_cloud_sim/setup.py
git commit -m "v2: setup.py — data_files excludes __pycache__/.pyc/.pyo, version 1.1.0, keep pid_path_follower entry"
```

---

## Task 13: Update package.xml (version bump, no ffmpeg)

**Files:**
- Modify: `src/baja_cloud_sim/package.xml`

**Interfaces:**
- Consumes: existing package.xml.
- Produces: version 1.1.0; **no** `<exec_depend>ffmpeg</exec_depend>` line.

- [ ] **Step 1: Bump version**

In `src/baja_cloud_sim/package.xml`, line 4:

Replace:
```xml
  <version>1.0.0</version>
```
With:
```xml
  <version>1.1.0</version>
```

- [ ] **Step 2: Confirm no ffmpeg line**

Run: `grep -n ffmpeg /home/pandafixle/Desktop/baja_cloud_sim/src/baja_cloud_sim/package.xml || echo "no ffmpeg dependency"`
Expected: `no ffmpeg dependency`.

- [ ] **Step 3: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add src/baja_cloud_sim/package.xml
git commit -m "v2: package.xml — bump version to 1.1.0"
```

---

## Task 14: Remove empty worlds/ directory

**Files:**
- Delete: `src/baja_cloud_sim/worlds/`

- [ ] **Step 1: Verify the directory is empty**

Run: `ls -la /home/pandafixle/Desktop/baja_cloud_sim/src/baja_cloud_sim/worlds/ 2>&1`
Expected: directory exists but empty, or already missing.

- [ ] **Step 2: Remove the directory**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
rmdir src/baja_cloud_sim/worlds/ 2>/dev/null || rm -rf src/baja_cloud_sim/worlds/
```

- [ ] **Step 3: Confirm**

Run: `ls /home/pandafixle/Desktop/baja_cloud_sim/src/baja_cloud_sim/worlds/ 2>&1`
Expected: `No such file or directory`.

- [ ] **Step 4: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add -A src/baja_cloud_sim/worlds/
git status --short
# If git reports a deletion:
git commit -m "chore: remove empty worlds/ scaffold directory"
```

---

## Task 15: Rewrite README.md to reflect merged state

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: existing README.
- Produces: README that documents (a) the new straight→90°-turn track, (b) the localization noise σ parameters, (c) the `pid_path_follower` as the active controller, (d) the manual rollback path via `runtime/pre_merge_backup/`.

- [ ] **Step 1: Replace README.md**

Write `README.md` (top-level):

```markdown
# baja_cloud_sim (v2.1)

Cloud-ready 100 m dirt-road planning and control simulation for ROS 2 and Gazebo.

## What changed in this merge

This branch folds in the geometry improvements from `baja_cloud_sim (2)` while
keeping the PID-based path follower, the real-vehicle dimensions, and the
cloud-friendly run scripts.

- **Scene:** 50 m straight → 90° left turn (R = 15 m) → final straight, with
  two sub-10° hills and two low speed bumps from `core.terrain_height`.
  Obstacles are anchored to the terrain profile (`z = terrain(s) + h/2`).
  Physics runs at 500 Hz (`max_step_size = 0.002`) under ogre2 with shadows.
- **Localization noise:** `truth_perception_node` simulates cm-level Gaussian
  noise (configurable via `localization_position_stddev_m`,
  `localization_altitude_stddev_m`, `localization_yaw_stddev_deg`) and
  publishes the noisy pose as `/localization/odom` with a 6×6 covariance.
- **Tracking line ghost fix:** `/actual_path` now publishes with RELIABLE QoS,
  a publish-time stamp, and a 1500-pose buffer trim — RViz no longer leaves
  trailing ghost segments after a Ctrl-C restart.
- **Controller tuning:** `pid_path_follower_node` uses real elapsed `dt`
  (not the previous hard-coded 0.05), `kd_heading` defaults to 0.12, and the
  Stanley lateral term uses the previously commanded speed.
- **Marker lifetime:** LINE_STRIP / CUBE debug markers from `frenet_planner`,
  `truth_perception`, and `evaluator` expire after 180 ms instead of 500 ms.

## Running

```bash
bash run.sh --seed 42
```

`run.sh` keeps the cloud-friendly defaults (pkill stale Gazebo processes,
software-render env vars when no GPU is present).

## Files of interest

- `src/baja_cloud_sim/baja_cloud_sim/core.py` — pure-Python geometry,
  planning, control helpers.
- `src/baja_cloud_sim/baja_cloud_sim/scenario_generator.py` — emits the
  SDF + JSON for each seed.
- `src/baja_cloud_sim/launch/simulation.launch.py` — launches all ROS
  nodes (`pid_path_follower` is the active follower).
- `src/baja_cloud_sim/config/{params,bridge,simulation.rviz}.{yaml,yaml,yaml}`
  — node parameters, Gazebo↔ROS bridges, RViz layout.

## Rolling back

A pre-merge snapshot of every modified file lives in
`runtime/pre_merge_backup/`. To roll back a single file:

```bash
cp runtime/pre_merge_backup/core.py src/baja_cloud_sim/baja_cloud_sim/core.py
colcon build --packages-select baja_cloud_sim
```
```

- [ ] **Step 2: Commit**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git add README.md
git commit -m "docs: README — describe v2.1 merge (scene, localization noise, ghost fix, controller tuning)"
```

---

## Task 16: End-to-end verification

**Files:** none modified. Smoke test the merged package.

**Interfaces:**
- Consumes: complete v2 branch.
- Produces: a verification report.

- [ ] **Step 1: Clean build from scratch**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
rm -rf build install log
colcon build --packages-select baja_cloud_sim 2>&1 | tail -10
```
Expected: clean build, no warnings about missing modules.

- [ ] **Step 2: Run all unit tests**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && python -m pytest src/baja_cloud_sim/test/ -v 2>&1 | tail -25`
Expected: all tests in `test_core.py` and `test_scenario_generator.py` pass.

- [ ] **Step 3: Generate a scenario and validate contents**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
rm -rf /tmp/baja_verify
source install/setup.bash
ros2 run baja_cloud_sim generate_scenario --output /tmp/baja_verify --seed 42 --obstacles 5
python3 -c "
import json, sys
d = json.load(open('/tmp/baja_verify/scenario.json'))
cl = d['centerline']
assert 'z' in cl[0], 'centerline must carry z'
assert cl[0]['yaw'] == 0.0, 'first segment must be straight'
assert any(p['s'] >= 60 and abs(p['yaw']) > 0.3 for p in cl), 'centerline must include a turn'
assert len(d['obstacles']) == 5, 'must have 5 obstacles'
assert d['road_layout']['turn_angle_deg'] == 90.0
assert d['terrain_features'], 'must list terrain features'
print('OK — straight→turn geometry, terrain-aware obstacles, road_layout present')
"
```
Expected: prints `OK — straight→turn geometry, terrain-aware obstacles, road_layout present`.

- [ ] **Step 4: Lint and validate SDF**

Run: `python3 -c "import xml.etree.ElementTree as ET; ET.parse('/tmp/baja_verify/baja_100m.sdf'); print('SDF OK')"`
Expected: `SDF OK`.

Also: `grep -c "render_engine>ogre2" /tmp/baja_verify/baja_100m.sdf` → `1`.
Also: `grep "real_time_update_rate" /tmp/baja_verify/baja_100m.sdf` → `500`.

- [ ] **Step 5: Confirm pid_path_follower entry point is registered**

Run: `cd /home/pandafixle/Desktop/baja_cloud_sim && source install/setup.bash && ros2 pkg executables baja_cloud_sim | sort`
Expected: lists `baja_cloud_sim generate_scenario`, `truth_perception`, `frenet_planner`, `path_follower`, `pid_path_follower`, `actuator_adapter`, `evaluator`. **No** `video_recorder`.

- [ ] **Step 6: Confirm backup tree is still intact**

Run: `ls /home/pandafixle/Desktop/baja_cloud_sim/runtime/pre_merge_backup/ | wc -l`
Expected: `14` (13 files + README).

- [ ] **Step 7: Final commit (only if any uncommitted change remains)**

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
git status --short
```
If non-empty, commit whatever is left with a `chore:` message. Then `git log --oneline -5` to confirm the merge series is intact (8–12 commits since `0781640`).

---

## Self-Review

**1. Spec coverage:**
- Scene geometry (straight→90° turn + terrain) → Task 2 (`core.generate_centerline`), Task 3 (uses terrain, dynamic ground). ✓
- Localization noise → Task 4 (reference superset), Task 8 (σ params). ✓
- Tracking line ghost fix → Task 7 (`/actual_path` QoS + stamp). ✓
- segment_is_safe preservation → Task 2 (four-corner sweep kept). ✓
- `_obstacle_sdf` duplicate return bug → Task 3 (rewritten with single return + test). ✓
- Real `dt`, `kd_heading`, `steering_alpha`, Stanley denominator → Task 5. ✓
- Marker lifetime 180 ms → Tasks 6, 7. ✓ (Task 4 also handles 180 ms via copy from reference.)
- Vehicle dimensions frozen at 1.70/1.50/252 → Tasks 2, 3, 8, 11. ✓
- No ffmpeg/video → Tasks 10, 12, 13. ✓
- Backup before edits → Task 1. ✓
- Verification → Task 16. ✓
- Missing spec item: **install_ubuntu2204.sh** idempotent apt-source skip is *not* in this plan. The spec says "small补"; the current script already works for the user's cloud setup, so the change is not blocking — flagging this explicitly so the user can opt to add it later.

**2. Placeholder scan:** No "TBD", "TODO", "implement later", "fill in details", "add appropriate error handling", or undefined references. All code blocks are complete and runnable.

**3. Type consistency:**
- `PlannerConfig.vehicle_length` is 1.70 in Task 2 (corrected from reference's 3.0) and `params.yaml` (Task 8) pins it again at 1.70.
- `pid_path_follower_node` `prev_time` (Task 5) is referenced in `_control` and reset in `_stop`; never read before written thanks to `if self.prev_time is None` guard.
- `commanded_speed` (Task 5) is initialized in `__init__`, written after the speed-factor block, read by the Stanley term on the next cycle. ✓
- `path_qos` (Task 7) is defined alongside `path_pub` and used in the same `create_publisher` call. ✓

**4. Open question for the user:** install_ubuntu2204.sh idempotent skip — included as "non-blocking" but flagged here so it doesn't get lost.