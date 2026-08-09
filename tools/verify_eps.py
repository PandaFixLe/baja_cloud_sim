#!/usr/bin/env python3
"""Sample /vehicle_status and /cmd_control to verify steering/speed stability
after the real-car parameter regulation (mass 210kg, wheel radius 0.279m,
max_steering 26deg, EPS 12deg/s rack model).

Writes a CSV for offline inspection.  Run while the simulation is live.
"""
import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class Sampler(Node):
    def __init__(self, duration: float):
        super().__init__("verify_eps_sampler")
        self._duration = duration
        self._start = self.get_clock().now()
        self._rows = []
        self.create_subscription(
            AckermannDriveStamped, "/vehicle_status", self._status_cb, 20)
        self.create_subscription(
            AckermannDriveStamped, "/cmd_control", self._cmd_cb, 20)
        self._cmd_steer = 0.0
        self._timer = self.create_timer(1.0, self._tick)

    def _cmd_cb(self, msg: AckermannDriveStamped) -> None:
        self._cmd_steer = msg.drive.steering_angle

    def _status_cb(self, msg: AckermannDriveStamped) -> None:
        t = (self.get_clock().now() - self._start).nanoseconds / 1e9
        eps = msg.drive.steering_angle
        spd = msg.drive.speed
        self._rows.append((t, self._cmd_steer, eps, spd))

    def _tick(self) -> None:
        if (self.get_clock().now() - self._start).nanoseconds / 1e9 >= self._duration:
            self._dump()
            rclpy.shutdown()

    def _dump(self) -> None:
        import csv
        with open("results/eps_verify.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "cmd_steer_deg", "eps_steer_deg", "speed"])
            for t, cs, es, sp in self._rows:
                w.writerow([f"{t:.2f}", math.degrees(cs), math.degrees(es), f"{sp:.3f}"])
        # quick summary
        if self._rows:
            eps_vals = [math.degrees(es) for _, _, es, _ in self._rows]
            spd_vals = [sp for _, _, _, sp in self._rows]
            max_abs = max(abs(v) for v in eps_vals)
            nan_cnt = sum(1 for v in eps_vals if math.isnan(v))
            print(f"rows={len(self._rows)}")
            print(f"eps_steer max|deg|={max_abs:.2f} (limit=26.0)")
            print(f"eps_steer NaN={nan_cnt}")
            if spd_vals:
                print(f"speed min={min(spd_vals):.3f} max={max(spd_vals):.3f}")
        print("wrote results/eps_verify.csv")


def main() -> None:
    rclpy.init()
    node = Sampler(float(__import__("sys").argv[1] if len(__import__("sys").argv) > 1 else 75))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
