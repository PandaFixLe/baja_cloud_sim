import math
import socket
import struct
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class RemoteControlNode(Node):
    def __init__(self) -> None:
        super().__init__("remote_control_node")
        self.declare_parameter("listen_address", "127.0.0.1")
        self.declare_parameter("listen_port", 5005)
        self.declare_parameter("max_speed", 10.0)
        self.declare_parameter("max_steering_angle_deg", 35.0)
        self.declare_parameter("steering_input_in_degrees", False)

        listen_address = str(
            self.get_parameter("listen_address").value
        )
        listen_port = int(
            self.get_parameter("listen_port").value
        )
        self.max_speed = float(
            self.get_parameter("max_speed").value
        )
        max_steering_deg = float(
            self.get_parameter("max_steering_angle_deg").value
        )
        self.max_steering_angle = math.radians(
            max_steering_deg
        )
        self.steering_input_in_degrees = bool(
            self.get_parameter("steering_input_in_degrees").value
        )

        self.udp_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )
        self.udp_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )
        self.udp_socket.bind((listen_address, listen_port))
        self.udp_socket.setblocking(False)

        self.packet_without_timestamp = struct.Struct("<dddbb")
        # 34字节：3个double、2个byte、1个double
        self.packet_with_timestamp = struct.Struct("<dddbbd")

        # 上一次实际发布的期望速度，供刹车指令递减使用。
        self.last_speed = 0.0
        # 单独保存上一次收到的油门期望速度，仅用于日志变化判断。
        self.last_input_speed = None
        self.last_steer = None
        self.last_brake = None
        self.last_mode = None
        self.last_extra = None
        self.last_timestamp = None
        self.last_recv_time = None

        self.get_logger().info(
            f"Listening for UDP packets on "
            f"{listen_address}:{listen_port}"
        )

        self.command_publisher = self.create_publisher(
            AckermannDriveStamped,
            "/cmd_control",
            10,
        )

        self.receive_timer = self.create_timer(
            0.02,
            self._receive_udp,
        )

    def _receive_udp(self) -> None:
        while True:
            try:
                data, sender_address = self.udp_socket.recvfrom(1024)
            except BlockingIOError:
                return
            except OSError as error:
                self.get_logger().error(
                    f"UDP receive failed: {error}"
                )
                return

            packet_length = len(data)

            try:
                if packet_length == 26:
                    (
                        steer,
                        speed,
                        brake,
                        mode,
                        extra,
                    ) = self.packet_without_timestamp.unpack(data)

                    timestamp = None

                elif packet_length in (34, 37, 45):
                    (
                        steer,
                        speed,
                        brake,
                        mode,
                        extra,
                        timestamp,
                    ) = self.packet_with_timestamp.unpack_from(data, 0)

                else:
                    self.get_logger().warning(
                        f"Unsupported UDP packet length: "
                        f"{packet_length} bytes"
                    )
                    continue

            except struct.error as error:
                self.get_logger().warning(
                    f"UDP unpack failed: {error}"
                )
                continue

            validated = self._validate_control(
                steer,
                speed,
                brake,
                mode,
                extra,
                timestamp,
            )

            if validated is None:
                continue

            (
                steer,
                speed,
                brake,
                mode,
                extra,
            ) = validated

            self._publish_control(
                steer,
                speed,
                brake,
            )

            current_time = time.monotonic()

            if self.last_recv_time is None:
                interval_ms = 0.0
            else:
                interval_ms = (
                    current_time - self.last_recv_time
                ) * 1000.0

            self.last_recv_time = current_time

            control_changed = (
                speed != self.last_input_speed
                or steer != self.last_steer
                or brake != self.last_brake
                or mode != self.last_mode
                or extra != self.last_extra
            )

            if control_changed:
                self.last_input_speed = speed
                self.last_steer = steer
                self.last_brake = brake
                self.last_mode = mode
                self.last_extra = extra
                self.last_timestamp = timestamp

                sender_ip, sender_port = sender_address

                self.get_logger().info(
                    "\n"
                    f"UDP packet from {sender_ip}:{sender_port}\n"
                    f"Steer   : {steer:.2f}\n"
                    f"Speed   : {speed:.2f}\n"
                    f"Brake   : {brake:.2f}\n"
                    f"Mode    : {bool(mode)}\n"
                    f"Extra   : {bool(extra)}\n"
                    f"Timestamp: {timestamp}\n"
                    f"Raw     : {packet_length} bytes\n"
                    f"Interval: {interval_ms:.2f} ms"
                )

    def _publish_control(
        self,
        steer: float,
        speed: float,
        brake: float,
    ) -> None:
        message = AckermannDriveStamped()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"

        # 刹车优先：有明确刹车输入时，不采用本包中的油门期望速度，
        # 而是在上一次发布速度的基础上递减。
        if brake > 0.05:
            target_speed = self.last_speed * (1.0 - brake)
            target_steer = steer
        else:
            target_speed = speed
            target_steer = steer

        message.drive.speed = float(target_speed)
        message.drive.steering_angle = float(target_steer)

        self.command_publisher.publish(message)
        self.last_speed = target_speed

    def _validate_control(
        self,
        steer: float,
        speed: float,
        brake: float,
        mode: int,
        extra: int,
        timestamp,
    ):
        float_values = [steer, speed, brake]

        if timestamp is not None:
            float_values.append(timestamp)

        if not all(math.isfinite(value) for value in float_values):
            self.get_logger().warning(
                "Rejected UDP packet containing NaN or infinity"
            )
            return None

        if mode not in (0, 1):
            self.get_logger().warning(
                f"Rejected invalid mode value: {mode}"
            )
            return None

        if extra not in (0, 1):
            self.get_logger().warning(
                f"Rejected invalid extra value: {extra}"
            )
            return None

        if not 0.0 <= brake <= 1.0:
            self.get_logger().warning(
                f"Rejected invalid brake value: {brake}"
            )
            return None

        if self.steering_input_in_degrees:
            steer = math.radians(steer)

        limited_speed = max(
            0.0,
            min(speed, self.max_speed),
        )

        limited_steer = max(
            -self.max_steering_angle,
            min(steer, self.max_steering_angle),
        )

        if limited_speed != speed:
            self.get_logger().warning(
                f"Speed limited from {speed:.2f} "
                f"to {limited_speed:.2f} m/s"
            )

        if limited_steer != steer:
            self.get_logger().warning(
                f"Steering limited from {steer:.3f} "
                f"to {limited_steer:.3f} rad"
            )

        return (
            limited_steer,
            limited_speed,
            brake,
            mode,
            extra,
        )

    def destroy_node(self):
        self.udp_socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = RemoteControlNode()

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
