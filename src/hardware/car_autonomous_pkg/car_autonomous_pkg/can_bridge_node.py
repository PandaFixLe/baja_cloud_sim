#!/usr/bin/env python3
# car_autonomous_pkg/can_bridge_node.py
import rclpy
import math
import functools
import os
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32
from ament_index_python.packages import get_package_share_directory
from .can_manager import CanManager
from .message_handler import CanMessageHandler
from .vehicle_params import VehicleParams


class CanBridgeNode(Node):
    def __init__(self):
        super().__init__('can_bridge_node')

        # ========== 参数 ==========
        self.declare_parameter('vcu_can_channel', 'vcan1')
        self.declare_parameter('can_bustype', 'socketcan')
        self.declare_parameter('control_period_ms', 20)
        self.declare_parameter('target_mode', 1)
        self.declare_parameter('cmd_topic', '/cmd_control')
        self.declare_parameter('vcu_dbc_file', 'autonomous.dbc')
        self.declare_parameter('steering_ratio', 12.0)

        vcu_channel = self.get_parameter('vcu_can_channel').value
        bustype = self.get_parameter('can_bustype').value
        period_sec = self.get_parameter('control_period_ms').value / 1000.0
        self.target_mode = self.get_parameter('target_mode').value
        cmd_topic = self.get_parameter('cmd_topic').value
        self.steering_ratio = self.get_parameter('steering_ratio').value

        # DBC文件路径
        pkg_dir = get_package_share_directory('car_autonomous_pkg')
        vcu_dbc = os.path.join(pkg_dir, self.get_parameter('vcu_dbc_file').value)

        # ========== 初始化 CAN 管理器 ==========
        self.vcu_can_mgr = CanManager([vcu_dbc], vcu_channel, bustype)

        # ========== 发布器 ==========
        self.vehicle_pub = self.create_publisher(AckermannDriveStamped, '/vehicle_status', 10)

        # ========== 消息处理器 ==========
        self.handler = CanMessageHandler(self)
        self.vcu_can_mgr.register_receiver(
            functools.partial(self.handler.on_message_received, db=self.vcu_can_mgr.db)
        )

        # ========== IPC_CMD 控制命令（物理值） ==========
        self.ipc_cmd = {
            'IPC_Exit_Mode3': 0,
            'IPC_Motor_Mode': 1,
            'IPC_Target_Torque': 0,
            'IPC_BrkPress': 0.0,
            'IPC_Target_Steering_En': 1,
            'IPC_Target_Speed': 0.0,
            'IPC_Target_Mode': 0,              # 初始为 0，等待初始化
            'IPC_Target_Gear': 0,
            'IPC_Target_Angle': 0.0,
            'IPC_Life_Signal': 0,
            'IPC_Motor_En': 1,
        }

        self.life_cnt = 0
        self.mode_initialized = False

        # ========== 周期发送定时器 ==========
        self.control_timer = self.create_timer(period_sec, self._send_control_requests)

        # ========== 模式初始化：先发 0，延迟后切到目标 ==========
        self.init_timer = self.create_timer(0.5, self._initialize_mode)

        # ========== 订阅 ==========
        self.cmd_sub = self.create_subscription(
            AckermannDriveStamped, cmd_topic, self.cmd_callback, 10
        )
        self.mode_sub = self.create_subscription(
            Int32, '/target_mode', self.mode_callback, 10
        )

        # 性能监控
        self.cmd_count = 0
        self.performance_timer = self.create_timer(5.0, self.print_performance)

        self.get_logger().info("🚗 CAN桥接节点已启动（仅 VCU 控制）")
        self.print_status()

    # ========== 模式初始化回调 ==========
    def _initialize_mode(self):
        self.mode_initialized = True
        self.ipc_cmd['IPC_Target_Mode'] = self.target_mode
        self.get_logger().info(f"✅ 模式已初始化为 {self.target_mode}")
        self.init_timer.cancel()

    # ========== 周期发送 ==========
    def _send_control_requests(self):
        self.life_cnt = (self.life_cnt + 1) % 16
        self.ipc_cmd['IPC_Life_Signal'] = self.life_cnt

        if not self.mode_initialized:
            self.ipc_cmd['IPC_Target_Mode'] = 0
        else:
            self.ipc_cmd['IPC_Target_Mode'] = self.target_mode

        self.vcu_can_mgr.send_message('IPC_CMD', self.ipc_cmd)

    # ========== 控制指令回调 ==========
    def cmd_callback(self, msg):
        self.cmd_count += 1

        speed_ms = max(VehicleParams.MAX_REVERSE_SPEED_MS,
                       min(VehicleParams.MAX_SPEED_MS, msg.drive.speed))

        # 前轮转角 → 方向盘转角
        front_wheel_rad = max(-0.61, min(0.61, msg.drive.steering_angle))
        front_wheel_deg = math.degrees(front_wheel_rad)
        steering_wheel_deg = front_wheel_deg * self.steering_ratio
        steering_wheel_deg = max(-70.0, min(70.0, steering_wheel_deg))

        # 档位
        if speed_ms > 0.05:
            gear = 1
        elif speed_ms < -0.05:
            gear = 2
        else:
            gear = 0

        # 制动压力 (MPa)
        brake = 0.5 if abs(speed_ms) < 0.01 else 0.0

        self.ipc_cmd['IPC_Target_Speed'] = abs(speed_ms)
        self.ipc_cmd['IPC_Target_Angle'] = steering_wheel_deg
        self.ipc_cmd['IPC_Target_Gear'] = gear
        self.ipc_cmd['IPC_BrkPress'] = brake

        if self.cmd_count % 50 == 0:
            self.get_logger().debug(
                f"前轮: {front_wheel_deg:.1f}° → 方向盘: {steering_wheel_deg:.1f}°, "
                f"速度: {speed_ms:.2f} m/s"
            )

    # ========== 模式切换回调 ==========
    def mode_callback(self, msg):
        self.target_mode = msg.data
        if self.mode_initialized:
            self.ipc_cmd['IPC_Target_Mode'] = self.target_mode
        mode_str = {1: "自主模式", 2: "视驾模式", 3: "紧急模式", 0: "无效"}.get(self.target_mode, "未知")
        self.get_logger().warn(f"🔄 切换到: {mode_str}")

    # ========== 发布方法（由 message_handler 调用） ==========
    def publish_vehicle_status(self, speed, steering_angle_deg):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = math.radians(float(steering_angle_deg))
        self.vehicle_pub.publish(msg)

    # ========== 辅助 ==========
    def print_status(self):
        self.get_logger().info("=" * 55)
        self.get_logger().info(f"   VCU CAN通道: {self.get_parameter('vcu_can_channel').value}")
        self.get_logger().info(f"   转向传动比: {self.steering_ratio:.1f}")
        self.get_logger().info(f"   目标模式: {self.target_mode}")
        self.get_logger().info(f"   控制周期: {self.get_parameter('control_period_ms').value}ms")
        self.get_logger().info("=" * 55)

    def print_performance(self):
        self.get_logger().info(f"📊 性能: 控制指令={self.cmd_count} | 生命={self.life_cnt}")
        self.cmd_count = 0

    def emergency_stop(self):
        self.get_logger().warn("🛑 紧急停车!")
        self.ipc_cmd['IPC_Target_Speed'] = 0.0
        self.ipc_cmd['IPC_Target_Angle'] = 0.0
        self.ipc_cmd['IPC_BrkPress'] = 12.0
        self.ipc_cmd['IPC_Target_Gear'] = 0
        self.ipc_cmd['IPC_Motor_En'] = 1
        self.vcu_can_mgr.send_message('IPC_CMD', self.ipc_cmd)

    def destroy_node(self):
        self.emergency_stop()
        self.vcu_can_mgr.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()