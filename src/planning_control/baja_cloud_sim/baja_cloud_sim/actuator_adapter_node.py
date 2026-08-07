"""Adapt existing AckermannDriveStamped commands to Gazebo cmd_vel."""

from __future__ import annotations

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

from .core import smooth_velocity


class ActuatorAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("actuator_adapter_node")
        self.declare_parameter("wheelbase", 1.43)
        self.declare_parameter("command_timeout", 0.35)
        # Platform-specific topic remaps (phase 0 of real-car port):
        # in simulation this points at the Gazebo vehicle; on the Orin real car
        # it points at the chassis driver (e.g. /chassis/cmd).
        self.declare_parameter("cmd_vel_topic", "/model/baja_vehicle/cmd_vel")
        # Input odometry used only to echo vehicle status. In simulation this is
        # the Gazebo ground-truth odometry; on the real car it would be the
        # fused localization odometry instead.
        self.declare_parameter("odom_topic", "/ground_truth/odom")
        # EPS (electric power steering) actuator model.  Mirrors the real-car
        # rack: a first-order lag with time constant eps_tau, a slew-rate limit
        # eps_max_rate_deg (deg/s) and a small deadband eps_deadband_deg so the
        # controller is exercised against realistic steering dynamics before
        # porting to the Orin platform (see real_car_params_regulation.md).
        # 默认值与 config/params.yaml 对齐; 即使不带参数文件单独启动也是稳定配置
        # (eps_tau=0 依赖 Gazebo AckermannSteering 插件自带的物理转向响应, 不与之一阶
        # 双重建模; eps_max_rate_deg=15 为拟合真实电动助力齿条的转速上限)
        self.declare_parameter("eps_tau", 0.0)
        self.declare_parameter("eps_max_rate_deg", 15.0)
        self.declare_parameter("eps_deadband_deg", 1.0)
        # 输出端绝对死区: 最终齿条角绝对值小于此阈值时强制归零.
        # 治理直道稳态高频抖动 — LQR 在 0° 附近残差被反馈放大成 ±2° 抖动指令,
        # 现有 eps_deadband 是"增量死区"(基于 |target-current|), 抖动时每次增量都>1°
        # 故不触发; 此处加"绝对死区"让齿条在小角度区间完全不动, 弯道(≥2°)正常突破.
        self.declare_parameter("eps_output_deadband_deg", 0.5)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.timeout = float(self.get_parameter("command_timeout").value)
        self.eps_tau = float(self.get_parameter("eps_tau").value)
        self.eps_max_rate = math.radians(float(self.get_parameter("eps_max_rate_deg").value))
        self.eps_deadband = math.radians(float(self.get_parameter("eps_deadband_deg").value))
        self.eps_output_deadband = math.radians(float(self.get_parameter("eps_output_deadband_deg").value))
        self.last_command = None
        self.last_stamp = None
        self._current_speed = 0.0
        self._prev_accel = 0.0
        self._eps_angle = 0.0
        self._last_pub_stamp = None
        self.command_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.status_pub = self.create_publisher(AckermannDriveStamped, "/vehicle_status", 10)
        # EPS diagnostics: ideal command vs actual rack angle (after EPS model).
        self.cmd_steer_pub = self.create_publisher(Float32, "/metrics/cmd_steer", 10)
        self.eps_steer_pub = self.create_publisher(Float32, "/metrics/eps_actual_steer", 10)
        self.create_subscription(AckermannDriveStamped, "/cmd_control", self._command_callback, 20)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._odom_callback, 20)
        self.create_timer(0.05, self._publish)

    def _command_callback(self, message: AckermannDriveStamped) -> None:
        self.last_command = message
        self.last_stamp = self.get_clock().now()

    def _odom_callback(self, message: Odometry) -> None:
        status = AckermannDriveStamped()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "base_link"
        status.drive.speed = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        status.drive.steering_angle = self._eps_angle
        self.status_pub.publish(status)

    def _apply_eps(self, target_steering: float, dt: float) -> float:
        """First-order lag + rate limit + deadband, emulating the EPS rack.

        Returns the **effective** rack angle after the output absolute deadband
        (internal ``self._eps_angle`` keeps the true rack state so incremental
        deadband logic stays consistent). Callers should use the return value.
        """
        # Incremental deadband: ignore tiny command moves (rack freeplay / sensor noise).
        if abs(target_steering - self._eps_angle) < self.eps_deadband:
            pass  # keep self._eps_angle unchanged, still apply output deadband below
        else:
            # First-order lag toward the commanded angle.
            alpha = 1.0 - math.exp(-dt / max(self.eps_tau, 1e-4))
            lagged = self._eps_angle + (target_steering - self._eps_angle) * alpha
            # Slew-rate limit (deg/s -> rad over dt).
            max_step = self.eps_max_rate * dt
            delta = max(-max_step, min(max_step, lagged - self._eps_angle))
            self._eps_angle += delta
        # Output absolute deadband: small rack angles forced to zero.
        # Suppresses steady-state high-frequency jitter around 0° (LQR residual
        # amplification) without affecting curve commands (>=2°).
        if abs(self._eps_angle) < self.eps_output_deadband:
            return 0.0
        return self._eps_angle

    def _publish(self) -> None:
        output = Twist()
        now = self.get_clock().now()
        dt = 0.05
        if self._last_pub_stamp is not None:
            dt = (now - self._last_pub_stamp).nanoseconds / 1e9
        dt = max(min(dt, 0.2), 1e-4)
        self._last_pub_stamp = now
        if self.last_command is not None and self.last_stamp is not None:
            age = (now - self.last_stamp).nanoseconds / 1e9
            if age <= self.timeout:
                target_speed = float(self.last_command.drive.speed)
                steering = float(self.last_command.drive.steering_angle)
                # accel + jerk smoothing (Phase 0.6)
                smoothed, self._prev_accel = smooth_velocity(
                    target_speed, self._current_speed, 2.0, -2.5, 0.05,
                    prev_accel=self._prev_accel, max_jerk=4.0,
                )
                self._current_speed = smoothed
                # EPS actuator model: realistic steering rack dynamics.
                eps_out = self._apply_eps(steering, dt)
                output.linear.x = smoothed
                output.angular.z = smoothed * math.tan(eps_out) / self.wheelbase if abs(smoothed) > 0.01 else 0.0
        else:
            self._current_speed = 0.0
            self._prev_accel = 0.0
            self._eps_angle = 0.0
        self.command_pub.publish(output)
        # Diagnostics: ideal command vs actual rack angle (post-EPS).
        if self.last_command is not None and self.last_stamp is not None:
            age = (now - self.last_stamp).nanoseconds / 1e9
            if age <= self.timeout:
                self.cmd_steer_pub.publish(Float32(data=float(self.last_command.drive.steering_angle)))
                self.eps_steer_pub.publish(Float32(data=float(self._eps_angle)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActuatorAdapterNode()
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
