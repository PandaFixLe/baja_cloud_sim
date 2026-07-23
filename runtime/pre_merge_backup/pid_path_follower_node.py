"""PD path follower with steering filter, rate limiting, adaptive speed,
and virtual-target obstacle-avoidance support.

Follows the online /planned_path from frenet_planner.  When a Point message
arrives on /virtual_target the node temporarily steers toward that vehicle-
local waypoint (e.g. from an external obstacle-avoider), then resumes the
planned path once the waypoint is reached or times out.
"""

from __future__ import annotations

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String

from .core import wrap_angle


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class PidPathFollowerNode(Node):
    """PD-controlled path follower with virtual-target obstacle avoidance."""

    def __init__(self) -> None:
        super().__init__("pid_path_follower_node")

        # ---- parameter declarations ----
        for name, default in (
            ("origin_latitude", 30.0),
            ("origin_longitude", 114.0),
            ("target_speed", 2.5),
            ("lookahead_distance", 3.0),
            ("kp_heading", 1.2),
            ("kd_heading", 0.3),
            ("steering_alpha", 0.75),
            ("max_steer_rate_deg", 30.0),
            ("max_steering_angle", 35.0),
            ("adaptive_steering", True),
            ("adaptive_speed", True),
            ("k_stanley", 0.8),
            ("avoidance_speed_limit", 2.0),
            ("virtual_target_timeout", 5.0),
        ):
            self.declare_parameter(name, default)

        # ---- load parameters ----
        self.origin_lat = float(self.get_parameter("origin_latitude").value)
        self.origin_lon = float(self.get_parameter("origin_longitude").value)
        self.target_speed = float(self.get_parameter("target_speed").value)
        self.lookahead = float(self.get_parameter("lookahead_distance").value)
        self.kp = float(self.get_parameter("kp_heading").value)
        self.kd = float(self.get_parameter("kd_heading").value)
        self.steering_alpha = float(self.get_parameter("steering_alpha").value)
        self.max_steer_rate = math.radians(
            float(self.get_parameter("max_steer_rate_deg").value)
        )
        self.max_steering_deg = float(self.get_parameter("max_steering_angle").value)
        self.max_steering_rad = math.radians(self.max_steering_deg)
        self.adaptive_steering = bool(self.get_parameter("adaptive_steering").value)
        self.adaptive_speed = bool(self.get_parameter("adaptive_speed").value)
        self.k_stanley = float(self.get_parameter("k_stanley").value)
        self.avoid_spd_lim = float(self.get_parameter("avoidance_speed_limit").value)
        self.virtual_timeout = float(
            self.get_parameter("virtual_target_timeout").value
        )

        # ---- state ----
        self.position: tuple[float, float] | None = None  # global XY
        self.yaw_nav: float = 0.0  # navigation convention (0=N, CW+)
        self.path: list[tuple[float, float]] = []
        self.planner_feasible: bool = False
        self.last_path_time = None

        # Virtual target (obstacle-avoidance override)
        self.vt_local: tuple[float, float] | None = None  # vehicle-local
        self.vt_time = None
        self.avoiding: bool = False
        self.avoid_start_time = None

        # PD / filter state
        self.prev_error: float = 0.0
        self.prev_steering: float = 0.0

        # ---- publishers ----
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, "/cmd_control", 10)
        self.lookahead_pub = self.create_publisher(PointStamped, "/lookahead_point", 10)

        # ---- subscribers ----
        self.create_subscription(NavSatFix, "/gps/fix", self._cb_gps, 20)
        self.create_subscription(Float32, "/imu/yaw", self._cb_yaw, 20)
        self.create_subscription(PathMessage, "/planned_path", self._cb_path, 10)
        self.create_subscription(String, "/planner/status", self._cb_status, 10)
        self.create_subscription(Point, "/virtual_target", self._cb_virtual_target, 10)

        # ---- timer (20 Hz) ----
        self.create_timer(0.05, self._control)

        self.get_logger().info(
            f"PID path follower ready | Kp={self.kp} Kd={self.kd} "
            f"alpha={self.steering_alpha} max_steer={self.max_steering_deg}° "
            f"adaptive_steer={self.adaptive_steering} "
            f"adaptive_speed={self.adaptive_speed}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_gps(self, msg: NavSatFix) -> None:
        x = (msg.longitude - self.origin_lon) * 111320.0 * math.cos(
            math.radians(self.origin_lat)
        )
        y = (msg.latitude - self.origin_lat) * 111320.0
        self.position = (x, y)

    def _cb_yaw(self, msg: Float32) -> None:
        self.yaw_nav = float(msg.data)

    def _cb_path(self, msg: PathMessage) -> None:
        self.path = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]
        self.last_path_time = self.get_clock().now()

    def _cb_status(self, msg: String) -> None:
        self.planner_feasible = msg.data == "FEASIBLE"

    def _cb_virtual_target(self, msg: Point) -> None:
        if math.isinf(msg.x) or math.isinf(msg.y):
            self.get_logger().info("Virtual target cleared — resuming planned path")
            self.vt_local = None
            self.vt_time = None
            self.avoiding = False
        else:
            self.vt_local = (float(msg.x), float(msg.y))
            self.vt_time = self.get_clock().now()
            self.avoiding = True
            self.avoid_start_time = self.get_clock().now()
            self.get_logger().info(
                f"Virtual target set: forward={msg.x:.2f} lateral={msg.y:.2f}"
            )

    # ------------------------------------------------------------------
    # Control helpers
    # ------------------------------------------------------------------

    def _preview_curvature(self) -> float:
        """Max angular-change rate along the planned path ahead (rad/m).

        Used to proactively slow down before sharp bends.
        """
        if len(self.path) < 3:
            return 0.0
        preview_dist = max(3.0, self.target_speed * 1.2)
        accumulated = 0.0
        max_rate = 0.0
        prev_bear: float | None = None
        idx = 0
        while idx < len(self.path) - 1:
            seg = _dist(self.path[idx], self.path[idx + 1])
            if seg < 0.01:
                idx += 1
                continue
            accumulated += seg
            if accumulated > preview_dist:
                break
            dx = self.path[idx + 1][0] - self.path[idx][0]
            dy = self.path[idx + 1][1] - self.path[idx][1]
            bearing = math.atan2(dx, dy)
            if prev_bear is not None:
                diff = bearing - prev_bear
                while diff > math.pi:
                    diff -= 2.0 * math.pi
                while diff < -math.pi:
                    diff += 2.0 * math.pi
                rate = abs(diff) / seg
                if rate > max_rate:
                    max_rate = rate
            prev_bear = bearing
            idx += 1
        return max_rate

    def _dynamic_max_steering(self, curvature: float) -> float:
        if not self.adaptive_steering:
            return self.max_steering_rad
        if curvature > 0.5:
            return self.max_steering_rad
        if curvature > 0.3:
            return self.max_steering_rad * 0.85
        if curvature > 0.15:
            return self.max_steering_rad * 0.70
        return self.max_steering_rad * 0.50

    def _speed_factor(self, steering_deg: float, preview_curv: float) -> float:
        if not self.adaptive_speed:
            return 1.0
        sa = abs(steering_deg)
        if sa > 25:
            factor = 0.35
        elif sa > 18:
            factor = 0.50
        elif sa > 12:
            factor = 0.70
        elif sa > 6:
            factor = 0.85
        else:
            factor = 1.0
        if preview_curv > 0.15:
            factor = min(factor, 0.30)
        elif preview_curv > 0.08:
            factor = min(factor, 0.50)
        elif preview_curv > 0.04:
            factor = min(factor, 0.70)
        return factor

    # ------------------------------------------------------------------
    # Main control loop (20 Hz)
    # ------------------------------------------------------------------

    def _cross_track_error(self) -> float:
        """Signed lateral distance from vehicle to the nearest path segment (m).

        Positive = vehicle is to the right of the path.
        """
        if self.position is None or len(self.path) < 2:
            return 0.0
        vx, vy = self.position
        best_idx = 0
        best_d2 = float("inf")
        for i in range(len(self.path)):
            d2 = (self.path[i][0] - vx) ** 2 + (self.path[i][1] - vy) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i
        i = min(best_idx, len(self.path) - 2)
        x1, y1 = self.path[i]
        x2, y2 = self.path[i + 1]
        dx, dy = x2 - x1, y2 - y1
        l2 = dx * dx + dy * dy
        if l2 < 1e-12:
            return math.hypot(vx - x1, vy - y1)
        t = max(0.0, min(1.0, ((vx - x1) * dx + (vy - y1) * dy) / l2))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        cross = (vx - proj_x) * dy - (vy - proj_y) * dx
        return cross / math.sqrt(l2)

    def _control(self) -> None:
        if self.position is None:
            return
        if not self.planner_feasible:
            self._stop()
            return
        if self.last_path_time is None or (
            self.get_clock().now() - self.last_path_time
        ).nanoseconds > 500_000_000:
            self._stop()
            return

        # --- expire stale virtual target ---
        now = self.get_clock().now()
        if self.vt_time is not None:
            if (now - self.vt_time).nanoseconds / 1e9 > self.virtual_timeout:
                self.get_logger().warn("Virtual target timed out, resuming planned path")
                self.vt_local = None
                self.vt_time = None
                self.avoiding = False
        if self.avoiding and self.avoid_start_time is not None:
            if (now - self.avoid_start_time).nanoseconds / 1e9 > 30.0:
                self.get_logger().error("Avoidance exceeded 30 s — forcing exit")
                self.vt_local = None
                self.vt_time = None
                self.avoiding = False

        # --- select target point ---
        if self.vt_local is not None:
            # Convert vehicle-local virtual target → global XY.
            # Navigation yaw (0=N, CW+) → math yaw (0=E, CCW+) for rotation.
            yaw_m = math.radians(90.0 - math.degrees(self.yaw_nav))
            tx_rel, ty_rel = self.vt_local  # forward, left
            target = (
                self.position[0]
                + tx_rel * math.cos(yaw_m)
                - ty_rel * math.sin(yaw_m),
                self.position[1]
                + tx_rel * math.sin(yaw_m)
                + ty_rel * math.cos(yaw_m),
            )
            if _dist(self.position, target) < 0.5:
                self.get_logger().info("Reached virtual target, resuming planned path")
                self.vt_local = None
                self.vt_time = None
                self.avoiding = False
                return
        else:
            if len(self.path) < 2:
                self._stop()
                return
            # Dynamic lookahead: faster speed → look further ahead
            dyn_look = max(self.lookahead, min(self.target_speed * 1.0, 5.0))
            target = self.path[-1]
            for pt in self.path:
                if _dist(self.position, pt) >= dyn_look:
                    target = pt
                    break

        # --- heading error (navigation convention) ---
        east = target[0] - self.position[0]
        north = target[1] - self.position[1]
        desired_nav = math.atan2(east, north)
        error = wrap_angle(desired_nav - self.yaw_nav)

        # Sharp recovery: extreme misalignment → limit speed, don't stop
        recovery_lim = None
        if abs(error) > math.pi * 0.5:
            recovery_lim = 0.5

        # --- Stanley + PD control (dt = 0.05) ---
        error_rate = (error - self.prev_error) / 0.05
        self.prev_error = error

        curvature_est = abs(2.0 * math.sin(error) / max(self.lookahead, 0.1))
        dyn_max = self._dynamic_max_steering(curvature_est)

        # Stanley lateral correction: arctan(k × cte / v), self-regulating
        cte = self._cross_track_error()
        stanley_term = math.atan2(self.k_stanley * cte, max(self.target_speed, 2.0))

        steering_raw = -(self.kp * error + self.kd * error_rate + stanley_term)
        steering_raw = max(-dyn_max, min(dyn_max, steering_raw))

        # --- low-pass filter ---
        steering_filt = (
            self.steering_alpha * steering_raw
            + (1.0 - self.steering_alpha) * self.prev_steering
        )

        # --- rate limit ---
        delta = steering_filt - self.prev_steering
        delta = max(-self.max_steer_rate, min(self.max_steer_rate, delta))
        steering = self.prev_steering + delta
        steering = max(-dyn_max, min(dyn_max, steering))
        self.prev_steering = steering

        # --- adaptive speed ---
        preview_curv = self._preview_curvature()
        factor = self._speed_factor(math.degrees(steering), preview_curv)
        speed = self.target_speed * factor
        if recovery_lim is not None:
            speed = min(speed, recovery_lim)
        if self.avoiding:
            speed = min(speed, self.avoid_spd_lim)

        # --- publish ---
        cmd = AckermannDriveStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = float(speed)
        cmd.drive.steering_angle = float(steering)
        self.cmd_pub.publish(cmd)

        lp = PointStamped()
        lp.header.stamp = cmd.header.stamp
        lp.header.frame_id = "map"
        lp.point.x = target[0]
        lp.point.y = target[1]
        lp.point.z = 0.18
        self.lookahead_pub.publish(lp)

    # ------------------------------------------------------------------
    # Stop
    # ------------------------------------------------------------------

    def _stop(self) -> None:
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = 0.0
        cmd.drive.steering_angle = 0.0
        self.cmd_pub.publish(cmd)
        self.prev_error = 0.0
        self.prev_steering = 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PidPathFollowerNode()
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
