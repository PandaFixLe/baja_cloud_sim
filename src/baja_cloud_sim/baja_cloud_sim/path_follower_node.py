"""Online-path version of the existing lookahead / heading controller."""

from __future__ import annotations

import math
from enum import Enum

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from visualization_msgs.msg import MarkerArray

from .core import (
    ControllerConfig,
    LQRConfig,
    LQRController,
    PlannedTrajectory,
    SpeedProfileConfig,
    base_to_world,
    compute_lqr_control,
    gps_to_local,
    legacy_path_control,
    nav_to_world_yaw,
    nearest_index,
    plan_speed_profile,
    quaternion_to_yaw,
    signed_lateral,
    point_to_oriented_box_clearance,
    wrap_angle,
)


# ── Phase 3 state machine ─────────────────────────────────────────────
class _ControlState(Enum):
    NORMAL = 0
    SLOWDOWN = 1
    EMERGENCY = 2


def _evaluate_state(
    consecutive_infeasible: int,
    consecutive_feasible: int,
    tracking_error: float,
    min_clearance: float,
    current: _ControlState,
) -> _ControlState:
    """3-level state machine with hysteresis (Phase 3.3)."""
    if current == _ControlState.NORMAL:
        if (min_clearance < 0.05 or consecutive_infeasible >= 30):
            return _ControlState.EMERGENCY
        if (consecutive_infeasible >= 3
                or tracking_error > 0.8
                or min_clearance < 0.3):
            return _ControlState.SLOWDOWN
        return _ControlState.NORMAL
    elif current == _ControlState.SLOWDOWN:
        if (min_clearance < 0.05 or consecutive_infeasible >= 30):
            return _ControlState.EMERGENCY
        if (consecutive_feasible >= 5
                and tracking_error < 0.3
                and min_clearance > 0.5):
            return _ControlState.NORMAL
        return _ControlState.SLOWDOWN
    else:  # EMERGENCY
        if (consecutive_feasible >= 50
                and tracking_error < 0.2
                and min_clearance > 1.0):
            return _ControlState.NORMAL
        return _ControlState.EMERGENCY


class PathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("path_follower_node")
        for name, default in (
            ("origin_latitude", 30.0), ("origin_longitude", 114.0),
            ("target_speed", 2.5), ("lookahead_distance", 3.0),
            ("kp_heading", 1.2), ("max_steering_angle", 35.0),
        ):
            self.declare_parameter(name, default)
        self.origin_lat = float(self.get_parameter("origin_latitude").value)
        self.origin_lon = float(self.get_parameter("origin_longitude").value)
        self.config = ControllerConfig(
            target_speed=float(self.get_parameter("target_speed").value),
            lookahead_distance=float(self.get_parameter("lookahead_distance").value),
            heading_gain=float(self.get_parameter("kp_heading").value),
            max_steering_deg=float(self.get_parameter("max_steering_angle").value),
        )
        self.position = None
        self.yaw_navigation = 0.0
        self.yaw_world = 0.0
        self.path = []
        # LQR state (Phase 2)
        self._odom_velocity = (0.0, 0.0)  # (vx, vy) world-frame
        self._yaw_rate = 0.0
        # LQR params exposed to params.yaml (Phase: oscillation damping)
        self.declare_parameter("lqr_R", 1.0)
        self.declare_parameter("lqr_v_norm", 2.5)
        self.declare_parameter("lqr_Q", [5.0, 2.0, 2.0, 1.0])
        self._lqr = LQRController()
        self._lqr_cfg = LQRConfig(
            R=float(self.get_parameter("lqr_R").value),
            v_norm=float(self.get_parameter("lqr_v_norm").value),
            Q=tuple(float(v) for v in self.get_parameter("lqr_Q").value),
        )
        self._current_speed = 0.0
        self._prev_steering = 0.0  # low-pass filter state (Phase 3.2)
        self.declare_parameter("enable_lqr", True)
        self._enable_lqr = bool(self.get_parameter("enable_lqr").value)
        self.planner_feasible = False
        self.last_path_time = None
        self._last_valid_path = []  # freewheel buffer
        self._infeasible_count = 0
        self._consecutive_feasible = 0  # Phase 3.5
        self._ctrl_state = _ControlState.NORMAL  # Phase 3.3
        self._speed_profile: List[float] = []  # target speeds along path
        self._path_curvatures: List[float] = []  # curvature per path point
        self._path_yaws: List[float] = []  # yaw per path point
        self._path_arc_lengths: List[float] = []  # arc-length per path point
        self._path_nearest: int = 0  # nearest index in path for speed lookup

        # speed profile toggle
        self.declare_parameter("use_speed_profile", True)
        self._use_speed_profile = bool(
            self.get_parameter("use_speed_profile").value
        )
        self.declare_parameter("desired_clearance", 1.2)
        self._desired_clearance = float(self.get_parameter("desired_clearance").value)
        self.declare_parameter("min_speed_obstacle", 2.0)
        self._speed_cfg = SpeedProfileConfig(
            min_speed_obstacle=float(self.get_parameter("min_speed_obstacle").value),
        )
        # terrain-aware speed derating
        self.declare_parameter("terrain_slope_threshold", 0.06)
        self._terrain_slope_threshold = float(self.get_parameter("terrain_slope_threshold").value)
        self.declare_parameter("terrain_min_speed", 1.0)
        self._terrain_min_speed = float(self.get_parameter("terrain_min_speed").value)
        self._centerline_z: dict = {}  # s → z lookup

        self.command_pub = self.create_publisher(AckermannDriveStamped, "/cmd_control", 10)
        self.lookahead_pub = self.create_publisher(PointStamped, "/lookahead_point", 10)
        self.create_subscription(NavSatFix, "/gps/fix", self._gps_callback, 20)
        self.create_subscription(Float32, "/imu/yaw", self._yaw_callback, 20)
        self.create_subscription(PathMessage, "/planned_path", self._path_callback, 10)
        self.create_subscription(String, "/planner/status", self._status_callback, 10)
        self.create_subscription(Odometry, "/ground_truth/odom", self._odom_callback, 20)
        self.create_subscription(MarkerArray, "/obstacle_markers", self._obstacle_callback, 10)
        self.create_subscription(PathMessage, "/reference_centerline", self._centerline_callback, 10)
        self._obstacles: list = []  # world-frame obstacle dicts
        self.create_timer(0.05, self._control)
        mode = "LQR 4x4" if self._enable_lqr else "Pure Pursuit"
        self.get_logger().info(f"Path follower ready: {mode} control with speed profile")

    def _gps_callback(self, message: NavSatFix) -> None:
        self.position = gps_to_local(
            message.latitude, message.longitude, self.origin_lat, self.origin_lon
        )

    def _yaw_callback(self, message: Float32) -> None:
        self.yaw_navigation = float(message.data)
        self.yaw_world = nav_to_world_yaw(self.yaw_navigation)

    def _odom_callback(self, message: Odometry) -> None:
        self._odom_velocity = (
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
        )
        self._yaw_rate = message.twist.twist.angular.z
        self._current_speed = math.hypot(
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
        )

    def _obstacle_callback(self, message: MarkerArray) -> None:
        """Store obstacles in world frame (convert from base_link)."""
        if self.position is None:
            return
        obstacles = []
        for marker in message.markers:
            x, y = base_to_world(
                (marker.pose.position.x, marker.pose.position.y),
                self.position, self.yaw_world,
            )
            q = marker.pose.orientation
            relative_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            obstacles.append({
                "id": marker.id,
                "x": x, "y": y,
                "yaw": wrap_angle(self.yaw_world + relative_yaw),
                "length": marker.scale.x,
                "width": marker.scale.y,
                "height": marker.scale.z,
            })
        self._obstacles = obstacles

    def _centerline_callback(self, message: PathMessage) -> None:
        """Build s→z lookup from reference centreline for terrain derating."""
        z_map = {}
        cumulative = 0.0
        prev = None
        for pose in message.poses:
            x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            if prev is not None:
                cumulative += math.hypot(x - prev[0], y - prev[1])
            z_map[cumulative] = z
            prev = (x, y)
        if z_map:
            self._centerline_z = z_map

    def _path_callback(self, message: PathMessage) -> None:
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        if len(self.path) >= 2:
            self._last_valid_path = self.path
            if self._use_speed_profile:
                self._compute_speed_profile(self.path)
        self.last_path_time = self.get_clock().now()

    def _compute_speed_profile(self, path: List[tuple]) -> None:
        """Derive curvatures from path points → plan speed profile."""
        N = len(path)
        if N < 3:
            self._speed_profile = [self.config.target_speed] * N
            return

        # arc lengths and yaws
        arc = [0.0]
        yaws = [0.0]
        for i in range(N):
            if i > 0:
                ds = math.hypot(path[i][0] - path[i-1][0],
                                path[i][1] - path[i-1][1])
                arc.append(arc[-1] + ds)
            if i < N - 1:
                yaw = math.atan2(path[i+1][1] - path[i][1],
                                 path[i+1][0] - path[i][0])
                yaws.append(yaw)
            elif i > 0:
                yaws.append(yaws[-1])

        # curvature from yaw difference
        curvatures = [0.0]
        for i in range(1, N):
            ds_i = arc[i] - arc[i-1]
            k = (yaws[i] - yaws[i-1]) / max(ds_i, 1e-6) if ds_i > 1e-6 else 0.0
            curvatures.append(k)

        # smooth curvature
        win_size = self._speed_cfg.curvature_smooth_window
        curvatures = _moving_average(curvatures, win_size)
        self._path_curvatures = curvatures
        self._path_yaws = yaws

        self._speed_profile = plan_speed_profile(curvatures, arc, self._speed_cfg)
        # ── Clearance-aware derating (Phase 4+) ──
        # Reduce speed near obstacles regardless of path curvature.
        if self._obstacles:
            self._speed_profile = _derate_by_clearance(
                path, self._speed_profile, self._obstacles,
                self._speed_cfg, self._desired_clearance,
            )
        # ── Terrain-aware derating ──
        if self._centerline_z:
            self._speed_profile = _derate_by_terrain(
                arc, self._speed_profile, self._centerline_z,
                self._terrain_slope_threshold, self._terrain_min_speed,
            )
        self._path_arc_lengths = arc

    def _status_callback(self, message: String) -> None:
        self.planner_feasible = message.data == "FEASIBLE"

    def _publish_stop(self) -> None:
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed = 0.0
        message.drive.steering_angle = 0.0
        self.command_pub.publish(message)

    def _control(self) -> None:
        if self.position is None:
            self._publish_stop()
            return

        # Freewheel: if planner reports infeasible, keep using the last
        # valid path for up to ~300 ms (6 cycles @ 20 Hz) before stopping.
        if not self.planner_feasible or len(self.path) < 2:
            self._infeasible_count += 1
        else:
            self._infeasible_count = 0
            self._consecutive_feasible += 1

        # Phase 3.3: evaluate control state
        nearest = _closest_index(self.path if len(self.path) >= 2 else [[0.0, 0.0]], self.position)
        center_ref = self.path[nearest] if nearest < len(self.path) else (0.0, 0.0)
        track_err = abs(signed_lateral(self.position,
                        {"x": center_ref[0], "y": center_ref[1], "yaw": self.yaw_world}))
        self._ctrl_state = _evaluate_state(
            self._infeasible_count, self._consecutive_feasible,
            track_err, 1.0,  # clearance not available in node
            self._ctrl_state,
        )

        effective_path = self.path
        if self._infeasible_count > 0:
            if self._infeasible_count <= 6 and len(self._last_valid_path) >= 2:
                effective_path = self._last_valid_path
            else:
                self._publish_stop()
                return

        if self.last_path_time is not None:
            age_ns = (self.get_clock().now() - self.last_path_time).nanoseconds
            if age_ns > 400_000_000:
                self._publish_stop()
                return

        if len(effective_path) < 2:
            self._publish_stop()
            return

        # ── LQR control (Phase 2) ──
        if self._enable_lqr and len(self._path_curvatures) > 0:
            nearest = _closest_index(effective_path, self.position)
            idx = min(nearest, len(effective_path) - 1)
            reference = {
                "x": effective_path[idx][0],
                "y": effective_path[idx][1],
                "yaw": self._path_yaws[idx] if idx < len(self._path_yaws) else self.yaw_world,
                "kappa": self._path_curvatures[idx] if idx < len(self._path_curvatures) else 0.0,
            }
            command = compute_lqr_control(
                self.position, self.yaw_world,
                self._odom_velocity, self._yaw_rate,
                reference,
                self._speed_profile if self._use_speed_profile else None,
                nearest,
                self._current_speed,
                self._lqr, self._lqr_cfg,
                effective_path, self.config,
                self.yaw_navigation,
            )
        else:
            command = legacy_path_control(
                self.position, self.yaw_navigation, effective_path,
                self.config, current_speed=self._current_speed,
            )

        # Phase 1.5: replace steering-based derating with curvature-aware speed profile
        target_speed = float(command["speed"])
        if self._use_speed_profile and len(self._speed_profile) > 0:
            nearest = _closest_index(effective_path, self.position)
            if nearest < len(self._speed_profile):
                target_speed = self._speed_profile[nearest]

        # Phase 3.4: state-based speed override
        if self._ctrl_state == _ControlState.EMERGENCY:
            target_speed = 0.0
            self._prev_steering = 0.0  # also zero steering on stop
        elif self._ctrl_state == _ControlState.SLOWDOWN:
            target_speed = min(target_speed, 1.0)

        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed = float(target_speed)
        # Phase 3.2: EMA low-pass on steering to avoid aggressive corrections
        raw_steer = float(command["steering"])
        self._prev_steering = 0.7 * raw_steer + 0.3 * self._prev_steering
        message.drive.steering_angle = self._prev_steering
        self.command_pub.publish(message)

        lookahead = PointStamped()
        lookahead.header.stamp = message.header.stamp
        lookahead.header.frame_id = "map"
        lookahead.point.x = command["target_x"]
        lookahead.point.y = command["target_y"]
        lookahead.point.z = 0.18
        self.lookahead_pub.publish(lookahead)


def _closest_index(path, position) -> int:
    """Return index of the path point closest to the given position."""
    if not path:
        return 0
    best = 0
    best_d2 = float("inf")
    px, py = position
    for i, (x, y) in enumerate(path):
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = i
    return best


def _moving_average(values, window: int):
    """Simple sliding-window moving average."""
    if window <= 1 or len(values) <= 1:
        return list(values)
    half = window // 2
    N = len(values)
    out = []
    for i in range(N):
        lo = max(0, i - half)
        hi = min(N, i + half + 1)
        out.append(sum(values[lo:hi]) / (hi - lo))
    return out


def _derate_by_clearance(
    path, speeds, obstacles, cfg: SpeedProfileConfig, desired_clearance: float,
) -> List[float]:
    """Scale target speed down proportionally to obstacle proximity.

    v_out = min_speed_obstacle + (c / desired_clearance) * (v_in - min_speed_obstacle)
    """
    if not obstacles or desired_clearance <= 0.0:
        return speeds
    from .core import point_to_oriented_box_clearance
    derated = list(speeds)
    half_w = cfg.max_lateral_accel  # not used — safety_margin handled by inflate
    for i, (px, py) in enumerate(path):
        min_c = float("inf")
        for obs in obstacles:
            c = point_to_oriented_box_clearance((px, py), obs, 0.0, 0.75)
            if c < min_c:
                min_c = c
        if min_c < desired_clearance:
            ratio = max(0.0, min_c / desired_clearance)
            derated[i] = max(cfg.min_speed_obstacle,
                             cfg.min_speed_obstacle + ratio * (speeds[i] - cfg.min_speed_obstacle))
    return derated


def _derate_by_terrain(
    arc: list, speeds: list, z_map: dict,
    slope_threshold: float, min_speed: float,
) -> list:
    """Scale target speed down when terrain gradient exceeds threshold."""
    if not z_map or len(arc) < 2:
        return speeds
    z_sorted = sorted(z_map.items())  # [(s, z), ...]
    if len(z_sorted) < 2:
        return speeds
    derated = list(speeds)
    for i, s_val in enumerate(arc):
        # find bracketing s-values in z_map
        lo, hi = 0, len(z_sorted) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if z_sorted[mid][0] <= s_val:
                lo = mid
            else:
                hi = mid
        ds = z_sorted[hi][0] - z_sorted[lo][0]
        if ds > 0.01:
            slope = abs(z_sorted[hi][1] - z_sorted[lo][1]) / ds
            if slope > slope_threshold:
                derated[i] = min(derated[i], min_speed)
    return derated


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
