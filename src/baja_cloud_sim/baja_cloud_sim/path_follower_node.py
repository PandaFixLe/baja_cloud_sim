"""Online-path version of the existing lookahead / heading controller."""

from __future__ import annotations

import math
from enum import Enum

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from .core import (
    ControllerConfig,
    LQRConfig,
    LQRController,
    PlannedTrajectory,
    SpeedProfileConfig,
    base_to_world,
    clamp,
    compute_lqr_steering,
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
            ("kp_heading", 1.2), ("max_steering_angle", 26.0),
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
        self.declare_parameter("lqr_max_steering", 0.4538)
        self.declare_parameter("lqr_velocity_recompute_threshold", 0.5)
        self.declare_parameter("dare_solve_interval", 50)
        # 方案 G: 反馈项速度自适应软化参数
        self.declare_parameter("fb_speed_soften_alpha", 0.3)
        self.declare_parameter("fb_speed_ref", 2.5)
        self.declare_parameter("fb_speed_beta_min", 0.5)
        self._lqr = LQRController()
        self._lqr_cfg = LQRConfig(
            R=float(self.get_parameter("lqr_R").value),
            v_norm=float(self.get_parameter("lqr_v_norm").value),
            Q=tuple(float(v) for v in self.get_parameter("lqr_Q").value),
            max_steering=float(self.get_parameter("lqr_max_steering").value),
            velocity_recompute_threshold=float(
                self.get_parameter("lqr_velocity_recompute_threshold").value),
            dare_solve_interval=int(
                self.get_parameter("dare_solve_interval").value),
            fb_speed_soften_alpha=float(
                self.get_parameter("fb_speed_soften_alpha").value),
            fb_speed_ref=float(self.get_parameter("fb_speed_ref").value),
            fb_speed_beta_min=float(self.get_parameter("fb_speed_beta_min").value),
        )
        self._current_speed = 0.0
        self._prev_steering = 0.0  # low-pass filter state (Phase 3.2)
        self._prev_target_speed = 0.0  # speed rate-limiter state
        # 转向输出平滑参数 (实车轮胎友好)
        self.declare_parameter("max_steer_rate", 0.25)
        self.declare_parameter("steer_lowpass_alpha", 0.22)
        self.declare_parameter("state_lowpass_alpha", 0.5)
        self._max_steer_rate = float(self.get_parameter("max_steer_rate").value)
        self._steer_alpha = float(self.get_parameter("steer_lowpass_alpha").value)
        self._state_alpha = float(self.get_parameter("state_lowpass_alpha").value)
        self.declare_parameter("enable_lqr", True)
        self._enable_lqr = bool(self.get_parameter("enable_lqr").value)
        self.planner_feasible = False
        self._planner_status_received = False  # guards startup state-machine
        self._planned_clearance = float("inf")  # last planner-reported clearance (m)
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
        # terrain-aware speed derating (disabled by default post perception-port;
        # special terrain is handled via flat_ground obstacle class instead)
        self.declare_parameter("terrain_slope_threshold", 0.06)
        self._terrain_slope_threshold = float(self.get_parameter("terrain_slope_threshold").value)
        self.declare_parameter("terrain_min_speed", 1.0)
        self._terrain_min_speed = float(self.get_parameter("terrain_min_speed").value)
        self.declare_parameter("use_terrain_profile", False)
        self._use_terrain_profile = bool(self.get_parameter("use_terrain_profile").value)
        # obstacle classification (ns="tall" / ns="flat_ground")
        self.declare_parameter("obstacle_classes.tall.desired_clearance", 1.5)
        self.declare_parameter("obstacle_classes.tall.min_speed", 1.5)
        self.declare_parameter("obstacle_classes.flat_ground.approach_distance", 5.0)
        self.declare_parameter("obstacle_classes.flat_ground.slow_speed", 2.0)
        self.declare_parameter("obstacle_classes.flat_ground.default_half_width", 0.9)
        self._tall_clearance = float(self.get_parameter("obstacle_classes.tall.desired_clearance").value)
        self._tall_min_speed = float(self.get_parameter("obstacle_classes.tall.min_speed").value)
        self._flat_approach = float(self.get_parameter("obstacle_classes.flat_ground.approach_distance").value)
        self._flat_slow = float(self.get_parameter("obstacle_classes.flat_ground.slow_speed").value)
        self._flat_half_width = float(self.get_parameter("obstacle_classes.flat_ground.default_half_width").value)
        # Detections within this base_link radius of the sensor origin are
        # treated as spurious self/clutter (radar reporting the vehicle chassis
        # or ground clang at startup) and ignored.
        self.declare_parameter("min_obstacle_range", 0.5)
        self._min_obstacle_range = float(self.get_parameter("min_obstacle_range").value)
        self._centerline_pts: list = []  # [(x,y,z), ...] for nearest-neighbour lookup

        # s-projection reference (replaces Euclidean nearest-neighbour)
        self.declare_parameter("s_proj_lookahead", 0.8)  # m ahead along path
        self._s_proj_lookahead = float(self.get_parameter("s_proj_lookahead").value)
        self._last_proj_idx: int = 0

        # LQI integral state (lateral-error accumulator)
        self._lqr_e_y_int: float = 0.0
        self._recovery_hold: int = 0  # countdown timer for RECOVERY exit delay

        # curvature-aware pre-deceleration
        self.declare_parameter("max_lateral_accel", 1.8)
        self._speed_cfg.max_lateral_accel = float(
            self.get_parameter("max_lateral_accel").value)
        # 曲率前瞻距离: 出弯加速延后, 防止弯切直蛇形(0=关闭)
        self.declare_parameter("curvature_lookahead_m", 3.0)
        self._speed_cfg.curvature_lookahead_m = float(
            self.get_parameter("curvature_lookahead_m").value)

        self._diag_tick = 0  # speed-profile diagnostic counter

        # ── Longitudinal cascaded-PID params (Apollo-style) ──
        self.declare_parameter("lon_kp", 2.0)
        self.declare_parameter("lon_ki", 0.5)
        self.declare_parameter("lon_kd", 0.2)
        self.declare_parameter("lon_i_limit", 2.0)
        self.declare_parameter("idle_speed", 1.0)
        self.declare_parameter("slope_comp_gain", 1.0)
        self.declare_parameter("speed_profile_max", 3.5)
        self.declare_parameter("speed_tier", "normal")
        self.declare_parameter("tier_slow_speed", 2.8)
        self.declare_parameter("tier_normal_speed", 3.5)
        self.declare_parameter("tier_fast_speed", 5.5)
        self.declare_parameter("lon_accel_step", 0.20)   # m/s per 50 ms ≈ 4.0 m/s²
        self.declare_parameter("lon_decel_step", 0.25)   # ≈ 5.0 m/s²
        self._lon_kp = float(self.get_parameter("lon_kp").value)
        self._lon_ki = float(self.get_parameter("lon_ki").value)
        self._lon_kd = float(self.get_parameter("lon_kd").value)
        self._lon_i_limit = float(self.get_parameter("lon_i_limit").value)
        self._idle_speed = float(self.get_parameter("idle_speed").value)
        self._slope_comp_gain = float(self.get_parameter("slope_comp_gain").value)
        self._lon_accel_step = float(self.get_parameter("lon_accel_step").value)
        self._lon_decel_step = float(self.get_parameter("lon_decel_step").value)
        self._dt = 0.05
        self._lon_i = 0.0
        self._lon_e_prev = 0.0
        # Lift speed-profile ceiling to a tracking-comfortable value.
        # speed_tier 选择直线段最大速度 (slow/normal/fast)；tier 非法时
        # 退回 speed_profile_max 作为 fallback，保证历史兼容。
        tier = str(self.get_parameter("speed_tier").value).strip().lower()
        tier_speeds = {
            "slow": float(self.get_parameter("tier_slow_speed").value),
            "normal": float(self.get_parameter("tier_normal_speed").value),
            "fast": float(self.get_parameter("tier_fast_speed").value),
        }
        if tier in tier_speeds:
            self._speed_cfg.max_speed = tier_speeds[tier]
        else:
            self.get_logger().warn(
                f"Unknown speed_tier '{tier}', falling back to speed_profile_max")
            self._speed_cfg.max_speed = float(
                self.get_parameter("speed_profile_max").value)

        # ── Finish / 终点逻辑参数 ──
        self.declare_parameter("finish_mode", "none")
        self.declare_parameter("finish_runout_m", 20.0)
        self.declare_parameter("finish_decel", 1.5)
        self.declare_parameter("finish_s", -1.0)
        self.declare_parameter("finish_target_lap", 1)
        self.declare_parameter("finish_time_limit_s", 1200.0)
        self.declare_parameter("finish_max_duration_s", 600.0)
        self._finish_mode = str(self.get_parameter("finish_mode").value)
        self._finish_runout_m = float(self.get_parameter("finish_runout_m").value)
        self._finish_decel = float(self.get_parameter("finish_decel").value)
        self._finish_s_param = float(self.get_parameter("finish_s").value)
        self._finish_target_lap = int(self.get_parameter("finish_target_lap").value)
        self._finish_time_limit = float(self.get_parameter("finish_time_limit_s").value)
        self._finish_max_duration = float(self.get_parameter("finish_max_duration_s").value)
        # Centreline progress / lap state (populated by _centerline_callback)
        self._cl_xy = []
        self._cl_arc = []
        self._cl_length = 0.0
        self._cl_loop = False
        self._cl_last_idx = 0
        self._cl_last_s = 0.0
        self._cl_s = 0.0  # current arc-length progress along the centreline
        self._finish_s = 0.0
        self._stop_s = 0.0
        self._lap_count = 0
        self._lap_idx_progress = 0  # signed index progress for loop lap counting
        self._finish_armed = False
        self._has_moved = False
        self._finish_t0_set = False
        self._finish_t0 = self.get_clock().now()
        self._finished = False
        self._finish_watchdog_t0 = self.get_clock().now()
        self._finish_pub = self.create_publisher(String, "/finish/status", 10)
        self._finish_pub.publish(String(data="RUNNING"))
        # Red finish-line marker (rviz).  Latched so it shows even before the
        # follower starts moving.  Not used in time mode.
        self._finish_line_pub = self.create_publisher(
            Marker,
            "/finish_line_marker",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.command_pub = self.create_publisher(AckermannDriveStamped, "/cmd_control", 10)
        self.lookahead_pub = self.create_publisher(PointStamped, "/lookahead_point", 10)
        self.create_subscription(NavSatFix, "/gps/fix", self._gps_callback, 20)
        self.create_subscription(Float32, "/imu/yaw", self._yaw_callback, 20)
        self.create_subscription(PathMessage, "/planned_path", self._path_callback, 10)
        self.create_subscription(String, "/planner/status", self._status_callback, 10)
        self.create_subscription(Odometry, "/ground_truth/odom", self._odom_callback, 20)
        self.create_subscription(MarkerArray, "/obstacle_markers", self._obstacle_callback, 10)
        # Planner-reported minimum clearance to the lateral corridor (metres).
        # Feeds the safety state machine's clearance guards (EMERGENCY when the
        # corridor collapses below 5 cm, SLOWDOWN below 30 cm). Without this the
        # state machine would only react to lateral tracking error.
        self.create_subscription(Float32, "/metrics/planned_clearance", self._clearance_callback, 10)
        _latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PathMessage, "/reference_centerline",
                                 self._centerline_callback, _latched)
        self._obstacles: list = []  # world-frame obstacle dicts (ns="tall")
        self._ground_anomalies: list = []  # base_link segments (ns="flat_ground")
        self.create_timer(0.05, self._control)
        mode = "LQI 5x5" if (self._enable_lqr and len(self._lqr_cfg.Q) == 5) else \
               "LQR 4x4" if self._enable_lqr else "Pure Pursuit"
        self.get_logger().info(f"Path follower ready: {mode} control with speed profile")

    def _gps_callback(self, message: NavSatFix) -> None:
        self.position = gps_to_local(
            message.latitude, message.longitude, self.origin_lat, self.origin_lon
        )

    def _yaw_callback(self, message: Float32) -> None:
        self.yaw_navigation = float(message.data)
        self.yaw_world = nav_to_world_yaw(self.yaw_navigation)

    def _odom_callback(self, message: Odometry) -> None:
        # Gazebo's OdometryPublisher reports twist in the *body* frame
        # (robot_base_frame), while estimate_lqr_state() projects the
        # velocity onto the reference normal in the *world* frame.
        # Rotate here using the odometry orientation.
        vx_b = message.twist.twist.linear.x
        vy_b = message.twist.twist.linear.y
        q = message.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        vx_w = vx_b * cos_y - vy_b * sin_y
        vy_w = vx_b * sin_y + vy_b * cos_y
        # 反馈状态低通 (方案B)：降低 yaw_rate / 速度噪声对 LQR 反馈项的激励，
        # 从源头抑制转向锯齿。alpha 越小越平滑(相位滞后越大)。
        a = self._state_alpha
        self._odom_velocity = (
            a * vx_w + (1.0 - a) * self._odom_velocity[0],
            a * vy_w + (1.0 - a) * self._odom_velocity[1],
        )
        self._yaw_rate = a * message.twist.twist.angular.z + (1.0 - a) * self._yaw_rate
        self._current_speed = math.hypot(vx_b, vy_b)

    def _obstacle_callback(self, message: MarkerArray) -> None:
        """Split obstacles by class (ns) into lateral-avoiding vs ground-derating.

        - "tall":        high obstacle → world-frame obstacle for clearance (lateral avoid)
        - "flat_ground": special terrain (bump/speed bump) → relative-frame segment for
                         longitudinal smoothing only (no lateral avoidance)
        Markers with any other / missing ns are ignored.
        """
        if self.position is None:
            return
        obstacles = []
        ground = []
        for marker in message.markers:
            cls = marker.ns
            # Ignore spurious self/clutter detections at the sensor origin.
            if math.hypot(marker.pose.position.x, marker.pose.position.y) < self._min_obstacle_range:
                continue
            if cls == "tall":
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
            elif cls == "flat_ground":
                # store in base_link frame (relative to vehicle) for derating math
                ground.append({
                    "id": marker.id,
                    "x": float(marker.pose.position.x),
                    "y": float(marker.pose.position.y),
                    "length": float(marker.scale.x),   # forward extent of the terrain patch
                })
        self._obstacles = obstacles
        self._ground_anomalies = ground

    def _ground_derate(self, current_speed: float) -> float:
        """Longitudinal derating for flat_ground (special terrain) anomalies.

        Anomalies are base_link segments {x: centre (forward +), length: forward
        extent}. The vehicle (x=0) should slow to `slow_speed` while the segment
        overlaps the vehicle, and recover smoothly over `approach_distance` after
        the segment's end passes behind the vehicle (x_end < 0).

        Picks the *nearest* anomaly (smallest x_start) — tracks are assumed to
        contain at most one special-terrain patch at a time. Result is clamped to
        idle_speed so the vehicle never stalls to a halt on flat ground.
        """
        if not self._ground_anomalies:
            return float("inf")
        # nearest anomaly ahead of (or overlapping) the vehicle
        cand = [g for g in self._ground_anomalies if g["x"] > -g["length"]]
        if not cand:
            return float("inf")
        g = min(cand, key=lambda a: a["x"])
        # Fall back to the configured default half-width when the perception
        # group does not provide a forward extent (length), so a zero/empty
        # length cannot collapse the anomaly to a point and break the
        # ramp math below.
        length = g["length"] if g["length"] > 1e-3 else 2.0 * self._flat_half_width
        x_start = g["x"] - length / 2.0
        x_end = g["x"] + length / 2.0
        approach = max(0.5, self._flat_approach)

        if x_end < 0.0:
            # fully behind the vehicle → recovering
            dist_past = -x_end
            ramp = min(1.0, dist_past / approach)
            return max(self._idle_speed, self._flat_slow + ramp * (current_speed - self._flat_slow))
        if x_start <= 0.0:
            # overlapping the vehicle now → hold slow speed
            return max(self._idle_speed, self._flat_slow)
        # ahead → smooth ramp-down as it approaches
        ramp = min(1.0, (x_start) / approach)
        return max(self._idle_speed, self._flat_slow + (1.0 - ramp) * (current_speed - self._flat_slow))

    def _centerline_callback(self, message: PathMessage) -> None:
        """Store centreline for terrain derating and finish-progress tracking."""
        pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
               for p in message.poses]
        if not pts:
            return
        self._centerline_pts = pts
        cl_xy = [(x, y) for (x, y, _z) in pts]
        arc = [0.0]
        for i in range(1, len(cl_xy)):
            arc.append(arc[-1] + math.hypot(cl_xy[i][0] - cl_xy[i - 1][0],
                                           cl_xy[i][1] - cl_xy[i - 1][1]))
        self._cl_xy = cl_xy
        self._cl_arc = arc
        self._cl_length = arc[-1] if arc else 0.0
        # Closed loop if the first and last centreline points coincide.
        self._cl_loop = math.hypot(cl_xy[0][0] - cl_xy[-1][0],
                                   cl_xy[0][1] - cl_xy[-1][1]) < 3.0
        # Resolve the finish-line arc position.
        if self._finish_s_param >= 0.0:
            self._finish_s = self._finish_s_param
        elif self._cl_loop:
            self._finish_s = 0.0
        else:
            self._finish_s = max(0.0, self._cl_length - self._finish_runout_m)
        # Stop target = finish line + fixed runout (cyclic on a loop).
        self._stop_s = self._finish_s + self._finish_runout_m
        # Only initialise the tracked index/s on the FIRST centreline message.
        # truth_perception re-publishes the centreline every 1 Hz with latched
        # QoS, so resetting _cl_last_idx here every time would yank the windowed
        # progress search back to the start, freezing cl_s and breaking both the
        # finish speed-cap (line mode) and lap counting (circle mode).
        if not getattr(self, "_cl_initialized", False):
            self._cl_last_idx = 0
            self._cl_last_s = 0.0
            self._lap_idx_progress = 0
            self._cl_initialized = True
        # Draw the red finish line at finish_s (skipped in time mode).
        if self._finish_mode != "time":
            self._publish_finish_line()

    def _publish_finish_line(self) -> None:
        """Draw a red finish line across the track at finish_s (rviz marker).

        The line is centred on the centreline at arc-length finish_s and
        oriented perpendicular to the local track heading, so it visually
        spans the track width.  Published once per centreline update (latched
        QoS).  Time mode does not use a spatial finish line.
        """
        if self._finish_mode == "time" or not self._cl_xy or not self._cl_arc:
            return
        arc = self._cl_arc
        L = self._cl_length
        fs = self._finish_s
        # Target arc-length, wrapped into [0, L) for closed loops.
        if self._cl_loop:
            fs = fs % L
        # Find the centreline segment containing fs and interpolate the point.
        n = len(self._cl_xy)
        # Locate index i with arc[i] <= fs < arc[i+1].
        i = 0
        for k in range(n - 1):
            if arc[k] <= fs <= arc[k + 1]:
                i = k
                break
        else:
            i = n - 2 if n >= 2 else 0
        ax, ay = self._cl_xy[i]
        bx, by = self._cl_xy[(i + 1) % n] if self._cl_loop else self._cl_xy[min(i + 1, n - 1)]
        seg_len = math.hypot(bx - ax, by - ay)
        if seg_len < 1e-6:
            tx, ty = 1.0, 0.0
            cx, cy = ax, ay
        else:
            t = (fs - arc[i]) / seg_len if arc[i] != arc[min(i + 1, n - 1)] else 0.0
            t = max(0.0, min(1.0, t))
            cx, cy = ax + t * (bx - ax), ay + t * (by - ay)
            tx, ty = (bx - ax) / seg_len, (by - ay) / seg_len
        # Perpendicular (across track) unit vector.
        nx, ny = -ty, tx
        # Half-width of the visible line (spans the track).
        half_w = 3.0
        # Z height (slightly above ground).
        cz = (self._centerline_pts[i][2] if hasattr(self, "_centerline_pts")
              and i < len(self._centerline_pts) else 0.0) + 0.05
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "finish_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        # Two end points across the track, perpendicular to heading.
        p1 = Point(x=cx + nx * half_w, y=cy + ny * half_w, z=cz)
        p2 = Point(x=cx - nx * half_w, y=cy - ny * half_w, z=cz)
        marker.points = [p1, p2]
        marker.scale.x = 0.35  # line thickness
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        self._finish_line_pub.publish(marker)

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

        # Arc length at each node, and segment headings/lengths.
        # seg_yaw[i] is the heading of segment i → i+1 (N-1 entries).
        arc = [0.0]
        seg_yaw: List[float] = []
        seg_len: List[float] = []
        for i in range(N - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            ds = math.hypot(dx, dy)
            seg_len.append(ds)
            seg_yaw.append(math.atan2(dy, dx) if ds > 1e-9 else
                           (seg_yaw[-1] if seg_yaw else 0.0))
            arc.append(arc[-1] + ds)

        # Node yaw: interior nodes average the two adjacent segments;
        # the endpoints copy their single segment.  Never fabricate a
        # zero heading for node 0 — that injects a false curvature
        # spike at the path start, which the moving average then smears
        # several metres downstream into the feed-forward term.
        yaws = [seg_yaw[0]]
        for i in range(1, N - 1):
            yaws.append(wrap_angle(
                seg_yaw[i-1] + 0.5 * wrap_angle(seg_yaw[i] - seg_yaw[i-1])))
        yaws.append(seg_yaw[-1])

        # Curvature at node i from the turn between its adjacent segments,
        # normalised by the mean of their lengths.  Angles are wrapped so a
        # ±π rollover cannot masquerade as an enormous curvature.
        curvatures = [0.0] * N
        for i in range(1, N - 1):
            ds_i = 0.5 * (seg_len[i-1] + seg_len[i])
            if ds_i > 1e-6:
                curvatures[i] = wrap_angle(seg_yaw[i] - seg_yaw[i-1]) / ds_i
        if N > 2:
            curvatures[0] = curvatures[1]
            curvatures[-1] = curvatures[-2]

        # smooth curvature
        win_size = self._speed_cfg.curvature_smooth_window
        curvatures = _moving_average(curvatures, win_size)
        self._path_curvatures = curvatures
        self._path_yaws = yaws

        self._speed_profile = plan_speed_profile(curvatures, arc, self._speed_cfg)
        # ── Merged derating (clearance + terrain, single pass) ──
        self._speed_profile = _derate_speed_profile(
            path, arc, self._speed_profile, self._speed_cfg,
            self._obstacles, self._desired_clearance,
            self._centerline_pts,
            self._terrain_slope_threshold if self._use_terrain_profile else -1.0,
            self._terrain_min_speed,
        )
        self._path_arc_lengths = arc

    def _status_callback(self, message: String) -> None:
        self.planner_feasible = message.data == "FEASIBLE"
        self._planner_status_received = True

    def _clearance_callback(self, message: Float32) -> None:
        # Cleared once per planner cycle; NaN/garbage guarded by the state machine.
        if math.isfinite(message.data):
            self._planned_clearance = float(message.data)

    def _publish_stop(self) -> None:
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed = 0.0
        message.drive.steering_angle = 0.0
        self.command_pub.publish(message)

    def _control(self) -> None:
        """Control-loop orchestrator.

        Delegates to focused helpers for each responsibility:
        finish/infeasible guarding → lateral command → longitudinal target
        → actuation publishing.  Keeping this method thin prevents the
        timing-coupled finish/braking exemptions from tangling with the
        compute steps below.
        """
        if self.position is None:
            self._publish_stop()
            return

        should_stop, effective_path, braking_to_stop, track_err = \
            self._guard_finish_and_infeasible()
        if should_stop:
            return

        command = self._compute_lateral_command(effective_path)
        target_speed = self._compute_longitudinal_target(
            command, braking_to_stop, track_err)
        self._publish_actuation(command, target_speed)

    def _guard_finish_and_infeasible(self):
        """Handle finish watchdog, finish pre-check, planner-infeasible
        bookkeeping and all hard-stop guards.

        Returns ``(should_stop, effective_path, braking_to_stop, track_err)``.
        ``should_stop`` is True when a guard already published a stop command.
        """
        # Global finish watchdog (safety net if the trigger never fires).
        if (not self._finished and self._finish_mode != "none"
                and (self.get_clock().now() - self._finish_watchdog_t0).nanoseconds / 1e9
                > self._finish_max_duration):
            self._finished = True
            self._finish_pub.publish(String(data="FINISHED"))
            self.get_logger().warn("Finish watchdog triggered (max duration exceeded).")
            self._publish_stop()
            return True, self.path, False, 0.0

        # ── Finish pre-check (runs BEFORE the infeasible-stop guards) ──
        # When the graceful finish is actively braking the car to a stop, the
        # planner deliberately returns INFEASIBLE near the end of the track
        # ("goal reached").  If we let the infeasible guards call _publish_stop()
        # the car is yanked to a halt by an emergency stop instead of coasting
        # to stop_s — and the FINISHED latch (which only runs further down) is
        # skipped entirely, so `fin` stays False forever.  So compute the
        # finish state up front and exempt the braking-to-stop case from every
        # hard-stop guard in this routine.
        if self._finish_mode != "none":
            self._update_progress()
            self._update_finish_arm()
            if self._current_speed > 0.5:
                self._has_moved = True
        # Treat the car as "braking to stop" once it crosses the finish line
        # (cl_s >= finish_s) and the finish logic is armed.  This is earlier
        # than waiting for the speed cap to drop below idle, which is critical:
        # the planner returns INFEASIBLE near end-of-track, and without this
        # early exemption the infeasible-stop guard yanks the car to a halt
        # instead of letting it coast smoothly under the finish profile.
        braking_to_stop = (
            self._finish_mode != "none"
            and self._finish_armed
            and self._cl_s >= self._finish_s
        )

        if self._planner_status_received and (
                not self.planner_feasible or len(self.path) < 2):
            self._infeasible_count += 1
        elif self.planner_feasible and len(self.path) >= 2:
            self._infeasible_count = 0
            self._consecutive_feasible += 1

        # Phase 3.3: evaluate control state
        nearest = _closest_index(self.path if len(self.path) >= 2 else [[0.0, 0.0]], self.position)
        center_ref = self.path[nearest] if nearest < len(self.path) else (0.0, 0.0)
        track_err = abs(signed_lateral(self.position,
                        {"x": center_ref[0], "y": center_ref[1], "yaw": self.yaw_world}))
        self._ctrl_state = _evaluate_state(
            self._infeasible_count, self._consecutive_feasible,
            track_err, self._planned_clearance,
            self._ctrl_state,
        )

        effective_path = self.path
        if self._infeasible_count > 0:
            if braking_to_stop and len(self._last_valid_path) >= 2:
                # Finishing: keep tracking the last valid path so the car can
                # coast to stop_s under the finish speed cap (no emergency stop).
                effective_path = self._last_valid_path
            elif self._infeasible_count <= 6 and len(self._last_valid_path) >= 2:
                effective_path = self._last_valid_path
            else:
                self._publish_stop()
                return True, effective_path, braking_to_stop, track_err

        if self.last_path_time is not None:
            age_ns = (self.get_clock().now() - self.last_path_time).nanoseconds
            if age_ns > 400_000_000 and not braking_to_stop:
                self._publish_stop()
                return True, effective_path, braking_to_stop, track_err

        if len(effective_path) < 2:
            if braking_to_stop and len(self._last_valid_path) >= 2:
                effective_path = self._last_valid_path
            else:
                self._publish_stop()
                return True, effective_path, braking_to_stop, track_err

        return False, effective_path, braking_to_stop, track_err

    def _compute_lateral_command(self, effective_path):
        """Lateral control: Apollo-style LQR + feed-forward (primary), pure
        pursuit fallback.  Returns the command dict (steering + diagnostics).
        """
        # ── Lateral control: Apollo-style LQR + feed-forward (primary) ──
        # Pure pursuit is used ONLY as a fallback when LQR is unavailable
        # (disabled, vehicle too slow, or no reference curvature).
        command = {
            "speed": float(self.config.target_speed),
            "target_x": self.position[0], "target_y": self.position[1],
            "e_y": 0.0, "heading_error": 0.0, "steering": 0.0,
        }
        lqr_primary = (
            self._enable_lqr
            and len(self._path_curvatures) > 0
            and len(self._path_yaws) > 0
            and self._current_speed >= self._lqr_cfg.lqr_min_velocity
        )
        if lqr_primary:
            idx = _projected_index(effective_path, self._path_arc_lengths,
                                   self.position, self._last_proj_idx,
                                   self._s_proj_lookahead)
            # Guard against a length mismatch between the path and its cached
            # arc-lengths (e.g. during freewheel after planner infeasibility),
            # which would otherwise raise IndexError and kill the node.
            if not effective_path:
                lqr_primary = False
            else:
                idx = min(max(idx, 0), len(effective_path) - 1)
                self._last_proj_idx = max(0, idx - 3)
                ref_x, ref_y = effective_path[idx]
            ref_yaw = self._path_yaws[idx] if idx < len(self._path_yaws) else self.yaw_world
            kappa = self._path_curvatures[idx] if idx < len(self._path_curvatures) else 0.0
            ref = {"x": ref_x, "y": ref_y, "yaw": ref_yaw, "kappa": kappa}
            lqr_out = compute_lqr_steering(
                self.position, self.yaw_world,
                self._odom_velocity, self._yaw_rate,
                ref, self._current_speed,
                self._lqr, self._lqr_cfg,
                e_y_int=self._lqr_e_y_int,
            )
            if lqr_out is not None:
                command["steering"] = float(lqr_out["steering"])
                command["e_y"] = float(lqr_out["e_y"])
                command["heading_error"] = float(lqr_out["heading_error"])
                command["target_x"], command["target_y"] = ref_x, ref_y
                # Anti-windup: freeze integral when steering saturates
                saturated = abs(command["steering"]) >= self._lqr_cfg.max_steering * 0.98
                if not saturated:
                    self._lqr_e_y_int += command["e_y"] * self._dt
                    self._lqr_e_y_int = clamp(self._lqr_e_y_int, -0.5, 0.5)
            else:
                lqr_primary = False  # fall through to pure pursuit

        if not lqr_primary:
            pp_command = legacy_path_control(
                self.position, self.yaw_navigation, effective_path,
                self.config, current_speed=self._current_speed,
            )
            command = pp_command

        # Clamp steering to actuator limit (rate/low-pass applied in publishing step)
        command["steering"] = clamp(
            float(command["steering"]),
            -self._lqr_cfg.max_steering, self._lqr_cfg.max_steering,
        )
        return command

    def _compute_longitudinal_target(self, command, braking_to_stop, track_err) -> float:
        """Longitudinal target speed: speed-profile reference → finish cap →
        flat_ground derate → PID → slope comp → idle floor → safety state
        machine → accel/jerk limiter.  Returns the final target speed (m/s).
        """
        # ── Longitudinal control: cascaded PID tracking curvature profile ──
        # Replaces the old non-hysteretic e_y/yaw speed-cap (limit-cycle source).
        v_ref = float(command["speed"])
        if self._use_speed_profile and len(self._speed_profile) > 0:
            nearest = _closest_index(self.path, self.position)
            if nearest < len(self._speed_profile):
                v_ref = self._speed_profile[nearest]
                if self._diag_tick < 20:
                    self._diag_tick += 1
                    self.get_logger().info(
                        f'[SPEED-DIAG #{self._diag_tick}] v_ref={v_ref:.1f} '
                        f'actual={self._current_speed:.1f}')

        # ── Finish / 终点逻辑：锚定停车目标，平滑减速至停止 ──
        # NOTE: _update_progress / _update_finish_arm already ran early in
        # _guard_finish_and_infeasible() so the infeasible-stop guards could
        # exempt the braking-to-stop case.  We only apply the cap + FINISHED
        # latch here.
        if self._finish_mode != "none":
            if self._finished:
                v_ref = 0.0  # hold once stopped (prevents re-accel on loop / early stop)
            else:
                v_ref = min(v_ref, self._finish_speed_cap())
            self._maybe_publish_finished()

        # ── flat_ground (special terrain) longitudinal derating ──
        # Straight-line pass-through with smooth speed reduction; no lateral
        # avoidance (handled separately from tall-obstacle clearance above).
        if self._ground_anomalies:
            v_ground = self._ground_derate(self._current_speed)
            v_ref = min(v_ref, v_ground)

        # Speed-loop PID → desired acceleration
        e_v = v_ref - self._current_speed
        self._lon_i = clamp(self._lon_i + e_v * self._dt,
                            -self._lon_i_limit, self._lon_i_limit)
        a_cmd = (self._lon_kp * e_v
                 + self._lon_ki * self._lon_i
                 + self._lon_kd * (e_v - self._lon_e_prev) / self._dt)
        self._lon_e_prev = e_v

        # Slope (gravity) compensation from reference centreline z-gradient
        a_cmd += 9.81 * self._terrain_slope_at(self.position) * self._slope_comp_gain

        # Integrate to a speed target, then idle/creep compensation
        v_target = self._current_speed + a_cmd * self._dt
        # While finishing, allow the speed to fall to 0 so the vehicle can
        # actually stop. But keep the idle floor *until* we are genuinely
        # braking to a stop (finish cap below idle_speed) — otherwise arming
        # a finish mode at standstill (line / time) removes the creep that
        # gets the vehicle moving in the first place and it never starts.
        if self._finished:
            idle = 0.0
        elif self._finish_armed and self._finish_speed_cap() < self._idle_speed:
            idle = 0.0
        else:
            idle = self._idle_speed
        v_target = max(v_target, idle)

        # While the finish logic is actively braking the vehicle to a stop
        # (cap below idle speed), the safety state machine and off-track floor
        # MUST be suppressed.  Otherwise, in the runout / stop zone the lateral
        # tracking error grows (no more centreline guidance, car drifting off the
        # dirt), which throws the car into SLOWDOWN (caps v at 1.0 m/s) or trips
        # the off-track floor (forces v >= 2.0 m/s) — both prevent the speed from
        # ever reaching ~0, so FINISHED never latches and the car rolls off the
        # track.  When braking to stop we let the finish cap drive v down to 0.
        if not braking_to_stop:
            # Safety: state machine (gentle, decoupled from small lateral error)
            if self._ctrl_state == _ControlState.EMERGENCY:
                v_target = 0.0
                self._prev_steering = 0.0
            elif self._ctrl_state == _ControlState.SLOWDOWN:
                v_target = min(v_target, 1.0)
            # Off-track floor keeps steering authority. Suppressed once finished
            # so the vehicle can actually hold at zero.
            if abs(track_err) > 0.5 and not self._finished:
                v_target = max(v_target, 2.0)

        # Acceleration / jerk rate limiter
        delta_spd = v_target - self._prev_target_speed
        clamped_spd = clamp(delta_spd, -self._lon_decel_step, self._lon_accel_step)
        self._prev_target_speed += clamped_spd
        return self._prev_target_speed

    def _publish_actuation(self, command, target_speed) -> None:
        """Apply steering smoothing (low-pass + rate limit + spike guard),
        publish the Ackermann command and the lookahead marker."""
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed = float(target_speed)
        # 转向输出平滑 (方案A, 实车轮胎友好):
        # 1) 一阶低通 — 滤除 LQR 反馈项的高频抖动, 产出平滑渐变而非锯齿.
        # 2) 速率限幅 — 限制每步最大变化, 即使低通残差超限也不允许猛打方向.
        # 旧逻辑在大航向误差时放宽到 8°/步(≈160°/s), 是照片里高频反向跳变的
        # 来源之一, 这里移除放宽, 统一用参数化的 max_steer_rate.
        raw_steer = float(command["steering"])
        # 1) 低通
        filtered = self._steer_alpha * raw_steer + (1.0 - self._steer_alpha) * self._prev_steering
        # 2) 速率限幅 (基于实际控制周期 dt)
        step = self._max_steer_rate * max(self._dt, 1e-3)
        d = filtered - self._prev_steering
        # 3) 突变保护：DARE 重解/投影跳变等造成的阶跃式脉冲，
        #    低通+常规速率限幅可能压不住（如 20° 跳变经低通后仍 >10°）。
        #    当阶跃幅度超过正常步长的 2 倍时，用更紧的步长截断。
        max_jump = step * 2.0
        if abs(d) > max_jump:
            tight_step = step * 0.5
            d = max(-tight_step, min(tight_step, d))
        d = max(-step, min(step, d))
        self._prev_steering += d
        message.drive.steering_angle = float(self._prev_steering)
        self.command_pub.publish(message)

        lookahead = PointStamped()
        lookahead.header.stamp = message.header.stamp
        lookahead.header.frame_id = "map"
        lookahead.point.x = command["target_x"]
        lookahead.point.y = command["target_y"]
        lookahead.point.z = 0.18
        self.lookahead_pub.publish(lookahead)


    def _terrain_slope_at(self, position) -> float:
        """Local road grade (dz/ds) at the vehicle, from reference centreline z."""
        pts = self._centerline_pts
        if not pts or len(pts) < 2:
            return 0.0
        best_d2, best_k = float("inf"), 0
        for k, (cx, cy, _cz) in enumerate(pts):
            d2 = (position[0] - cx) ** 2 + (position[1] - cy) ** 2
            if d2 < best_d2:
                best_d2, best_k = d2, k
        if best_k < len(pts) - 1:
            x0, y0, z0 = pts[best_k]
            x1, y1, z1 = pts[best_k + 1]
            ds = math.hypot(x1 - x0, y1 - y0)
            if ds > 1e-6:
                return (z1 - z0) / ds
        return 0.0


    def _update_progress(self) -> None:
        """Forward-tracked projection of the vehicle onto the centreline.

        Updates self._cl_s and, on a closed loop, counts laps via s-wrap.

        The search window must respect the track topology:
          * Closed loop  → scan a *continuous window* around the last index
            (with modulo wrap).  A global nearest-point search snaps to the
            geometrically closest point, which on a symmetric rectangular loop
            is the identical point on the *opposite* straight — so ``best_i``
            teleports across the loop and the lap counter runs wild (cl_s=117
            already read as lap 10).  A windowed search keeps ``best_i`` anchored
            to the vehicle's real forward position, so lap counting is exact and
            the index advances one point at a time.
          * Open track   → scan the *entire* centreline for the nearest point.
            A fixed forward window (or the 1 Hz centreline re-publish resetting
            _cl_last_idx) froze cl_s near ~90 m, so the stop target at 120 m
            was never reached and the car blew past the finish without
            decelerating.  O(n) over a few-hundred-point centreline is
            negligible, and a global search is robust to the index being reset.
        """
        if not self._cl_xy or self.position is None:
            return
        px, py = self.position
        n = len(self._cl_xy)
        if self._cl_loop and n > 1:
            # Continuous windowed search: only consider points near the last
            # known index.  This prevents the global nearest-point search from
            # snapping to the symmetric point on the opposite side of the loop.
            best_i = self._cl_last_idx
            best_d2 = float("inf")
            W = 80  # generous arc-window (points), covers one lap with margin
            for k in range(-W, W + 1):
                i = (self._cl_last_idx + k) % n
                dx = self._cl_xy[i][0] - px
                dy = self._cl_xy[i][1] - py
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2, best_i = d2, i
            s = self._cl_arc[best_i] if best_i < len(self._cl_arc) else 0.0
            # Count laps from the *signed* forward progress of the centreline
            # index.  The raw s wraps at the seam, so a one-frame nearest-point
            # flicker there read as a backward wrap and cancelled the lap
            # increment.  The modulo delta below is remapped to a signed step,
            # so crossing the seam reads as a small forward step, never a
            # spurious backwards one.
            ds = (best_i - self._cl_last_idx) % n
            if ds > n / 2:
                ds -= n
            if not hasattr(self, "_lap_idx_progress"):
                self._lap_idx_progress = 0
            self._lap_idx_progress += ds
            self._lap_count = self._lap_idx_progress // n
        else:
            # Open track: global nearest-point search is robust and correct.
            best_i = 0
            best_d2 = float("inf")
            for i in range(n):
                dx = self._cl_xy[i][0] - px
                dy = self._cl_xy[i][1] - py
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2, best_i = d2, i
            s = self._cl_arc[best_i] if best_i < len(self._cl_arc) else 0.0
        self._cl_last_idx = best_i
        self._cl_last_s = s
        self._cl_s = s

    def _update_finish_arm(self) -> None:
        """Decide whether the graceful finish deceleration is active."""
        if self._finish_mode == "none":
            self._finish_armed = False
            return
        if self._finish_mode == "time":
            now = self.get_clock().now()
            # Simulation clock reset (re-run / seek) → re-baseline.
            if self._finish_t0_set and now < self._finish_t0:
                self._finish_t0 = now
                self._lap_count = 0
            if not self._finish_t0_set:
                self._finish_t0 = now
                self._finish_t0_set = True
            self._finish_armed = True
            return
        if self._finish_mode == "line":
            # Cap is inherently safe: it only binds after the finish line.
            self._finish_armed = True
            return
        if self._finish_mode == "circle":
            self._finish_armed = self._lap_count >= self._finish_target_lap
            return

    def _finish_speed_cap(self) -> float:
        """Max speed allowed so the vehicle stops exactly at the stop target.

        Space modes anchor at stop_s = finish_s + runout. Braking begins the
        instant the stopping distance v^2/(2a) reaches (dist to finish)+runout.
        Time mode anchors at the deadline instead.
        """
        if self._finish_mode == "none":
            return float("inf")
        a = self._finish_decel
        if self._finish_mode == "time":
            if not self._finish_t0_set:
                return float("inf")
            remaining = self._finish_time_limit - (
                self.get_clock().now() - self._finish_t0).nanoseconds / 1e9
            return max(0.0, a * remaining)
        if not self._finish_armed:
            return float("inf")
        if not self._cl_xy:  # centreline not received yet → don't brake
            return float("inf")
        if self._cl_loop:
            L = self._cl_length
            # On a loop, use the *shortest* cyclic distance to the stop target.
            # The raw modulo (stop_s - cl_s) % L flips to ~L once the vehicle
            # passes stop_s, which would let the car re-accelerate to full speed
            # instead of stopping.  min(d, L-d) keeps the cap engaged on BOTH
            # sides of the stop target, so the car actually halts at stop_s.
            d = (self._stop_s - self._cl_s) % L
            if d < 0:
                d += L
            dist = min(d, L - d)
        else:
            dist = self._stop_s - self._cl_s
        if dist <= 0.0:
            return 0.0
        brake_cap = math.sqrt(2.0 * a * dist)
        # "Known finish line" profile: the finish position is fixed and known
        # from the centreline (a lookup table), so once the vehicle has crossed
        # the finish line (cl_s >= finish_s) we glide down from cruise speed to
        # 0 across the whole runout.  This makes the deceleration *visible and
        # smooth* (starting right at the red line) instead of a late, hard slam
        # only a few metres before stop_s.  The physics brake_cap is kept as a
        # safety floor so we can always stop in time even if the profile is too
        # aggressive for the current grip.
        if self._cl_s >= self._finish_s:
            runout = max(1e-3, self._stop_s - self._finish_s)
            profile = self._speed_cfg.max_speed * max(
                0.0, (self._stop_s - self._cl_s) / runout)
            return min(brake_cap, profile)
        return min(self._speed_cfg.max_speed, brake_cap)

    def _maybe_publish_finished(self) -> None:
        """Publish FINISHED once the vehicle has stopped at the target.

        After FINISHED the caller holds the vehicle at zero speed, so an
        overshoot on the loop (modulo distance flips large) cannot re-accelerate
        it, and an early stop in time mode is latched instead of re-launched.
        """
        if self._finished or not self._has_moved:
            return
        if self._finish_mode == "time":
            remaining = self._finish_time_limit - (
                self.get_clock().now() - self._finish_t0).nanoseconds / 1e9
            reached = remaining <= 0.0 or self._current_speed < 0.1
        else:
            if self._cl_loop:
                d = (self._stop_s - self._cl_s) % self._cl_length
                reached = min(d, self._cl_length - d) <= 0.5
            else:
                reached = abs(self._stop_s - self._cl_s) <= 0.5
        if reached and self._current_speed < 0.1:
            self._finished = True
            self._finish_pub.publish(String(data="FINISHED"))
            self.get_logger().info("Finish reached: vehicle stopped.")


def _projected_index(path, arc_lengths, position, last_idx=0, lookahead_s=0.8) -> int:
    """s‑coordinate projection: find the closest point on the polyline
    (clamped perpendicular projection onto each segment), then look ahead
    by *lookahead_s* metres of arc length.

    This correctly handles curves where the unclamped perpendicular
    projection falls outside every segment.
    """
    if not path or len(path) < 2:
        return 0
    px, py = position
    start = max(0, last_idx)
    best_d2 = float("inf")
    best_s_proj = 0.0
    # Scan forward from last known index; find the segment that is
    # closest to the vehicle (clamped projection).
    for i in range(start, len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        abx, aby = bx - ax, by - ay
        seg_len2 = abx * abx + aby * aby
        if seg_len2 < 1e-12:
            continue
        t = ((px - ax) * abx + (py - ay) * aby) / seg_len2
        t = max(0.0, min(1.0, t))  # clamp to segment
        cx = ax + t * abx
        cy = ay + t * aby
        d2 = (px - cx) ** 2 + (py - cy) ** 2
        if d2 < best_d2:
            best_d2 = d2
            s_i = arc_lengths[i] if i < len(arc_lengths) else 0.0
            s_next = arc_lengths[i + 1] if i + 1 < len(arc_lengths) else s_i
            best_s_proj = s_i + t * (s_next - s_i)
    # Look ahead by lookahead_s metres of arc length.
    target_s = best_s_proj + lookahead_s
    for j in range(0, len(arc_lengths)):
        if arc_lengths[j] >= target_s:
            return j
    return len(path) - 1


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


def _derate_speed_profile(
    path, arc, speeds, cfg: SpeedProfileConfig,
    obstacles, desired_clearance: float,
    centerline_pts, slope_threshold: float, terrain_min_speed: float,
) -> List[float]:
    """Single-pass speed derating: clearance + terrain → min() of all constraints.

    Clearance: v → min_speed_obstacle + (c/desired) * (v_in - min_speed_obstacle)
    Terrain:   v → terrain_min_speed when |dz/ds| > slope_threshold
    """
    from .core import point_to_oriented_box_clearance
    derated = list(speeds)

    # ── clearance derating ──
    if obstacles and desired_clearance > 0.0:
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

    # ── terrain derating (nearest-neighbour z lookup) ──
    if centerline_pts and len(centerline_pts) > 1 and len(arc) >= 2:
        # pre-compute dz per metre along centreline
        cl_dz = [0.0]
        for k in range(1, len(centerline_pts)):
            ds = math.hypot(centerline_pts[k][0] - centerline_pts[k-1][0],
                            centerline_pts[k][1] - centerline_pts[k-1][1])
            dz = abs(centerline_pts[k][2] - centerline_pts[k-1][2]) / max(ds, 0.01)
            cl_dz.append(dz)
        for i, (px, py) in enumerate(path):
            # find nearest centreline point by xy distance
            best_d2, best_k = float("inf"), 0
            for k, (cx, cy, _cz) in enumerate(centerline_pts):
                d2 = (px - cx) ** 2 + (py - cy) ** 2
                if d2 < best_d2:
                    best_d2, best_k = d2, k
            slope = cl_dz[min(best_k, len(cl_dz) - 1)]
            # Check adjacent centreline indices so the derating zone is
            # at least 3 samples wide — prevents a single-point speed dip
            # that the vehicle cannot physically follow.
            if best_k > 0:
                slope = max(slope, cl_dz[best_k - 1])
            if best_k < len(cl_dz) - 1:
                slope = max(slope, cl_dz[best_k + 1])
            # Proportional derating: ramp smoothly from full speed at
            # slope=0 down to terrain_min_speed at slope_threshold+.
            if slope > 0.0:
                ratio = min(1.0, slope / slope_threshold)
                derated[i] = min(derated[i],
                                 terrain_min_speed + (1.0 - ratio) * (derated[i] - terrain_min_speed))

    # ── Re-run backward feasibility pass after derating ──
    # Derating clamps individual points (terrain bumps, obstacles).
    # The backward pass propagates those clamps rearward through the
    # acceleration constraint so the speed profile includes a physically
    # realisable deceleration zone before each low-speed feature.
    # (The forward decel pass is NOT re-run — it would force an
    #  artificial deceleration from the first point's max_speed,
    #  dropping the target below cruise after a few metres.)
    for i in range(len(derated) - 2, -1, -1):
        ds = arc[i + 1] - arc[i]
        if ds <= 0.0:
            continue
        v_limit = math.sqrt(max(0.0, derated[i + 1] ** 2 + 2.0 * (-cfg.max_decel) * ds))
        derated[i] = min(derated[i], v_limit)

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
