"""Online-path version of the existing lookahead / heading controller."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String

from .core import (
    ControllerConfig,
    StanleyControllerConfig,
    StanleyState,
    clamp,
    legacy_path_control,
    stanley_path_control,
    lqr_path_control,
    signed_lateral,
    polyline_distance,
    wrap_angle,
    augment_path_for_cte,
    nearest_index,
)
from .trajectory_smoother import TrajectorySmoother, TrajectoryTable
from .lqr_controller import LQRController, LQRConfig


class PathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("path_follower_node")
        for name, default in (
            ("origin_latitude", 30.0), ("origin_longitude", 114.0),
            ("target_speed", 2.5), ("lookahead_distance", 3.0),
            ("kp_heading", 1.2), ("max_steering_angle", 35.0),
            # v1.1 Stanley controller additions
            ("controller_mode", "stanley"),
            ("kd_heading", 0.3),
            ("k_stanley", 0.8),
            ("k_cte_dot", 0.15),
            ("k_yaw_rate", 0.5),
            ("steering_alpha", 0.6),
            ("max_steer_rate_deg", 8.0),
            ("adaptive_steering", True),
            ("adaptive_speed", True),
            # LQR controller parameters
            ("wheelbase", 1.43),
            ("lqr_q_cte", 10.0),
            ("lqr_q_cte_dot", 1.0),
            ("lqr_q_heading", 5.0),
            ("lqr_q_yaw_rate", 0.5),
            ("lqr_r_steer", 10.0),
            ("lqr_fb_limit_deg", 3.0),
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
        # Note: v1.0 path_follower_node already consumes online /planned_path;
        # no CSV-loading, 4WS, or virtual_target state-machine remains here.
        mode = str(self.get_parameter("controller_mode").value).strip().lower()
        if mode not in ("legacy", "stanley", "lqr"):
            self.get_logger().warning(
                f"Unknown controller_mode '{mode}', falling back to 'stanley'"
            )
            mode = "stanley"
        self.controller_mode = mode
        if mode in ("stanley", "lqr"):
            self.stanley_config = StanleyControllerConfig(
                target_speed=self.config.target_speed,
                lookahead_distance=self.config.lookahead_distance,
                heading_gain=self.config.heading_gain,
                max_steering_deg=self.config.max_steering_deg,
                kd_heading=float(self.get_parameter("kd_heading").value),
                k_stanley=float(self.get_parameter("k_stanley").value),
                k_cte_dot=float(self.get_parameter("k_cte_dot").value),
                k_yaw_rate=float(self.get_parameter("k_yaw_rate").value),
                steering_alpha=float(self.get_parameter("steering_alpha").value),
                max_steer_rate_deg=float(self.get_parameter("max_steer_rate_deg").value),
                adaptive_steering=bool(self.get_parameter("adaptive_steering").value),
                adaptive_speed=bool(self.get_parameter("adaptive_speed").value),
            )
            self.stanley_state = StanleyState()
        else:
            self.stanley_config = None
            self.stanley_state = None
        self.position = None
        self.prev_position = None   # for GPS speed estimation
        self.yaw_navigation = 0.0
        self.path = []
        self.planner_feasible = False
        self.last_path_time = None

        self.command_pub = self.create_publisher(AckermannDriveStamped, "/cmd_control", 10)
        self.lookahead_pub = self.create_publisher(PointStamped, "/lookahead_point", 10)

        # LQR + Bézier trajectory smoother (used when controller_mode == "lqr")
        self.trajectory_table: Optional[TrajectoryTable] = None
        self.lqr_ctrl: Optional[LQRController] = None
        if mode == "lqr":
            self.smoother = TrajectorySmoother(wheelbase=1.43)
            self.smoothed_pub = self.create_publisher(PathMessage, "/smoothed_path", 10)
            self.lqr_config = LQRConfig(
                wheelbase=float(self.get_parameter("wheelbase").value),
                q_cte=float(self.get_parameter("lqr_q_cte").value),
                q_cte_dot=float(self.get_parameter("lqr_q_cte_dot").value),
                q_heading=float(self.get_parameter("lqr_q_heading").value),
                q_yaw_rate=float(self.get_parameter("lqr_q_yaw_rate").value),
                r_steer=float(self.get_parameter("lqr_r_steer").value),
                fb_limit_deg=float(self.get_parameter("lqr_fb_limit_deg").value),
            )
            self.get_logger().info(
                "path_follower: mode=lqr (Bézier feed-forward + LQR feedback)"
            )
        else:
            self.smoother = None
            self.smoothed_pub = None
            self.lqr_config = None

        # Predicted trajectory (kinematic bicycle forward-sim in RViz)
        self.predicted_pub = self.create_publisher(PathMessage, "/predicted_trajectory", 10)

        self.create_subscription(NavSatFix, "/gps/fix", self._gps_callback, 20)
        self.create_subscription(Float32, "/imu/yaw", self._yaw_callback, 20)
        self.create_subscription(PathMessage, "/planned_path", self._path_callback, 10)
        self.create_subscription(String, "/planner/status", self._status_callback, 10)
        self.create_timer(0.05, self._control)
        if self.controller_mode == "stanley":
            self.get_logger().info(
                "path_follower: mode=stanley (Stanley + PD + dual-damping + low-pass + rate-limit)"
            )
        else:
            self.get_logger().info(
                "path_follower: mode=legacy (fallback)"
            )

    def _gps_callback(self, message: NavSatFix) -> None:
        x = (message.longitude - self.origin_lon) * 111320.0 * math.cos(math.radians(self.origin_lat))
        y = (message.latitude - self.origin_lat) * 111320.0
        if self.position is not None:
            self.prev_position = self.position
        self.position = (x, y)

    def _yaw_callback(self, message: Float32) -> None:
        self.yaw_navigation = float(message.data)

    def _path_callback(self, message: PathMessage) -> None:
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        self.last_path_time = self.get_clock().now()

        # LQR mode: regenerate trajectory table on each new path
        if self.controller_mode == "lqr" and self.smoother is not None:
            target_speed = float(self.get_parameter("target_speed").value)
            generated_at = self.get_clock().now().nanoseconds * 1e-9
            self.trajectory_table = self.smoother.generate(
                self.path, target_speed=target_speed,
                generated_at=generated_at,
            )
            if self.lqr_ctrl is None and self.trajectory_table is not None:
                self.lqr_ctrl = LQRController(self.lqr_config)
            elif self.trajectory_table is None:
                self.get_logger().warn(
                    "Trajectory smoother returned None, falling back to Stanley",
                    throttle_duration_sec=2.0,
                )
            # Publish smoothed path for RViz visualisation
            if self.smoothed_pub is not None and self.trajectory_table is not None:
                self._publish_smoothed_path()

    def _status_callback(self, message: String) -> None:
        self.planner_feasible = message.data == "FEASIBLE"

    def _publish_smoothed_path(self) -> None:
        """Publish the Bézier-smoothed trajectory as a nav_msgs/Path for RViz."""
        if self.trajectory_table is None or self.smoothed_pub is None:
            return
        msg = PathMessage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        from geometry_msgs.msg import PoseStamped
        for pt in self.trajectory_table.points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = pt.x
            ps.pose.position.y = pt.y
            ps.pose.position.z = 0.12
            # Encode yaw in orientation quaternion
            half_yaw = pt.yaw * 0.5
            ps.pose.orientation.z = math.sin(half_yaw)
            ps.pose.orientation.w = math.cos(half_yaw)
            msg.poses.append(ps)
        self.smoothed_pub.publish(msg)

    def _compute_safety_speed(self, speed_cmd: float, steering: float) -> float:
        """Two-level safety monitor: forward-sim predicted trajectory vs planned path.

        Level 1 (max_dev ≥ 0.8 m) → speed = 0.8 m/s
        Level 2 (max_dev ≥ 1.5 m) → speed = 0.4 m/s
        """
        if self.position is None or len(self.path) < 4:
            return speed_cmd

        wheelbase = 1.43
        dt = 0.05
        n_steps = 20
        yaw = math.pi * 0.5 - self.yaw_navigation
        px, py = self.position
        v = max(speed_cmd, 0.1)

        max_dev = 0.0
        for _ in range(n_steps):
            px += v * math.cos(yaw) * dt
            py += v * math.sin(yaw) * dt
            yaw += (v / wheelbase) * math.tan(steering) * dt
            d = polyline_distance((px, py), self.path)
            if d > max_dev:
                max_dev = d

        if max_dev >= 1.5:
            self.get_logger().warn(
                f"SAFETY L2: max_dev={max_dev:.2f}m → speed=0.4",
                throttle_duration_sec=0.5,
            )
            return 0.4
        if max_dev >= 0.8:
            self.get_logger().warn(
                f"SAFETY L1: max_dev={max_dev:.2f}m → speed=0.8",
                throttle_duration_sec=0.5,
            )
            return 0.8
        return speed_cmd

    def _publish_stop(self) -> None:
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed = 0.0
        message.drive.steering_angle = 0.0
        self.command_pub.publish(message)

    def _publish_predicted_trajectory(self, speed: float, steering: float) -> None:
        """Forward-sim kinematic bicycle for 1 second and publish as Path."""
        if self.position is None:
            return
        wheelbase = 1.43
        dt = 0.05
        steps = 20
        # Convert navigation yaw (0=North, CW+) to math yaw (0=+X, CCW+)
        yaw = math.pi * 0.5 - self.yaw_navigation
        x, y = self.position
        v = max(speed, 0.1)

        msg = PathMessage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        from geometry_msgs.msg import PoseStamped

        for _ in range(steps):
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += (v / wheelbase) * math.tan(steering) * dt
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.12
            msg.poses.append(ps)

        self.predicted_pub.publish(msg)

    @staticmethod
    def _interp_table_by_time(points, elapsed):
        """Linear interpolation on trajectory table by time offset."""
        if elapsed <= points[0].t:
            return points[0]
        for i in range(len(points) - 1):
            if points[i].t <= elapsed <= points[i + 1].t:
                dt = points[i + 1].t - points[i].t
                if dt < 1e-9:
                    return points[i]
                alpha = (elapsed - points[i].t) / dt
                # Simple interpolation of key fields via a lightweight dict
                class _Interp:
                    pass
                ref = _Interp()
                ref.x = points[i].x + alpha * (points[i + 1].x - points[i].x)
                ref.y = points[i].y + alpha * (points[i + 1].y - points[i].y)
                ref.yaw = points[i].yaw + alpha * (points[i + 1].yaw - points[i].yaw)
                ref.speed_limit = points[i].speed_limit + alpha * (points[i + 1].speed_limit - points[i].speed_limit)
                ref.steering_ff = points[i].steering_ff + alpha * (points[i + 1].steering_ff - points[i].steering_ff)
                return ref
        return points[-1]

    def _estimate_gps_speed(self) -> float:
        """Estimate actual speed from consecutive GPS positions."""
        if self.position is None or self.prev_position is None:
            return 0.0
        dx = self.position[0] - self.prev_position[0]
        dy = self.position[1] - self.prev_position[1]
        return math.hypot(dx, dy) / 0.05  # GPS at ~20 Hz, ~50 ms between samples

    def _lqr_control_step(self):
        """Time-aligned lookup + Frenet(s,l) dual-axis feedback + rate-limit + lowpass."""
        cfg = self.stanley_config
        state = self.stanley_state
        table = self.trajectory_table

        # fallback to Stanley if table is missing
        if table is None or len(table.points) < 2 or self.lqr_ctrl is None:
            cmd = stanley_path_control(
                self.position, self.yaw_navigation, self.path,
                cfg, state, dt=0.05,
            )
            self.stanley_state = cmd["state"]
            return cmd["speed"], cmd["steering"], cmd["target_x"], cmd["target_y"]

        # --- time-aligned lookup ---
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - table.generated_at
        ref = self._interp_table_by_time(table.points, elapsed)

        # --- init state on first tick ---
        yaw_math = math.pi * 0.5 - self.yaw_navigation
        if not state.initialized:
            state.prev_steering = 0.0
            state.prev_yaw = yaw_math
            state.prev_speed = 0.5  # start from low speed, not target (avoids LQR gain mismatch)
            state.yaw_rate_filtered = 0.0
            state.last_nearest = 0
            state.initialized = True

        # --- Frenet (s, l) ---
        augmented = augment_path_for_cte(self.path)
        nearest = nearest_index(
            augmented,
            self.position[0], self.position[1],
            start=getattr(state, "last_nearest", 0),
        )
        state.last_nearest = max(0, nearest - 4)
        l = signed_lateral(self.position, augmented[nearest])

        # s: project position error onto reference tangent direction
        dx = self.position[0] - ref.x
        dy = self.position[1] - ref.y
        s = dx * math.cos(ref.yaw) + dy * math.sin(ref.yaw)

        # --- s-axis feedback → v_correction ±0.3 m/s ---
        k_s = 1.0
        v_fb = clamp(-k_s * s, -0.3, 0.3)  # s>0 (ahead/overspeed) → decelerate

        # s-axis D-term: derivative of s-deviation catches downhill acceleration early
        if state.initialized:
            ds_dt = (s - state.prev_s) / 0.05  # m/s, how fast car is pulling ahead
            if ds_dt > 0.3:                     # threshold: pulling ahead > 0.3 m/s
                extra_cut = min(0.5, 2.0 * (ds_dt - 0.3))
                v_fb -= extra_cut
        state.prev_s = s

        v_cmd = clamp(ref.speed_limit + v_fb, 0.1, cfg.target_speed)

        # GPS speed guard: if actual speed far exceeds command, force deceleration
        actual_v = self._estimate_gps_speed()
        if actual_v > 0.0 and actual_v > v_cmd * 1.3 and actual_v > 1.5:
            v_cmd *= 0.5

        # --- l-axis feedback → δ_fb ±3° ---
        heading_error = wrap_angle(ref.yaw - yaw_math)
        yaw_rate_raw = wrap_angle(yaw_math - state.prev_yaw) / 0.05
        state.yaw_rate_filtered = 0.7 * state.yaw_rate_filtered + 0.3 * yaw_rate_raw

        from .lqr_controller import VehicleState
        vehicle = VehicleState(
            speed=max(state.prev_speed, 0.5),
            cte=l,
            heading_error=heading_error,
            yaw_rate=state.yaw_rate_filtered,
            x=self.position[0],
            y=self.position[1],
        )
        _, _, _, δ_fb = self.lqr_ctrl.compute(table, vehicle, cfg.target_speed)
        fb_limit = float(self.get_parameter("lqr_fb_limit_deg").value)
        δ_fb = clamp(δ_fb, -math.radians(fb_limit), math.radians(fb_limit))
        δ_cmd = ref.steering_ff + δ_fb

        # --- rate-limit + lowpass on steering ---
        max_steer = math.radians(cfg.max_steering_deg)
        δ_clamped = clamp(δ_cmd, -max_steer, max_steer)
        filtered = cfg.steering_alpha * δ_clamped + (1.0 - cfg.steering_alpha) * state.prev_steering
        rate_rad = math.radians(cfg.max_steer_rate_deg)
        delta_step = clamp(filtered - state.prev_steering, -rate_rad * 0.05, rate_rad * 0.05)
        steering = state.prev_steering + delta_step
        state.prev_steering = steering

        # --- speed lowpass ---
        state.prev_speed += cfg.speed_alpha * (v_cmd - state.prev_speed)
        speed = state.prev_speed

        state.prev_yaw = yaw_math

        return speed, steering, ref.x, ref.y

    def _control(self) -> None:
        if self.position is None or not self.planner_feasible or len(self.path) < 2:
            self._publish_stop()
            return
        if self.last_path_time is None or (self.get_clock().now() - self.last_path_time).nanoseconds > 400_000_000:
            self._publish_stop()
            return

        if self.controller_mode == "legacy":
            command = legacy_path_control(self.position, self.yaw_navigation, self.path, self.config)
            speed = float(command["speed"])
            steering = float(command["steering"])
            target_x, target_y = float(command["target_x"]), float(command["target_y"])
        elif self.controller_mode == "lqr":
            speed, steering, target_x, target_y = self._lqr_control_step()
        else:
            command = stanley_path_control(
                self.position, self.yaw_navigation, self.path,
                self.stanley_config, self.stanley_state, dt=0.05,
            )
            self.stanley_state = command["state"]
            speed = float(command["speed"])
            steering = float(command["steering"])
            target_x, target_y = float(command["target_x"]), float(command["target_y"])

        # ---- safety overlay: two-level warning based on predicted deviation ----
        speed = self._compute_safety_speed(speed, steering)

        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed = speed
        message.drive.steering_angle = steering
        self.command_pub.publish(message)

        lookahead = PointStamped()
        lookahead.header.stamp = message.header.stamp
        lookahead.header.frame_id = "map"
        lookahead.point.x = target_x
        lookahead.point.y = target_y
        lookahead.point.z = 0.18
        self.lookahead_pub.publish(lookahead)

        # ---- predicted trajectory (kinematic bicycle, 1 s / 20 steps) ----
        self._publish_predicted_trajectory(speed, steering)


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
