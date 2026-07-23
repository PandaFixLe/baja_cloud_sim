#!/usr/bin/env python3
import rclpy
import math
import csv
import os
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
import subprocess
import os
from ament_index_python.packages import get_package_share_directory


class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        
        # ========== 参数声明 ==========
        self.declare_parameter('path_file', 'recorded_path.csv')
        self.declare_parameter('target_speed', 0.5)
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('kp_heading', 1.0)
        self.declare_parameter('kd_heading', 0.3)            # D增益（阻尼振荡）
        self.declare_parameter('steering_alpha', 0.6)         # 低通滤波系数(0-1, 越小越平滑)
        self.declare_parameter('max_steer_rate_deg', 8.0)    # 最大转向角速率(度/周期, 防突变)
        self.declare_parameter('auto_find_nearest', True)
        self.declare_parameter('max_start_distance', 5.0)
        self.declare_parameter('stop_distance', 1.0)
        self.declare_parameter('max_steering_angle', 40.0)
        self.declare_parameter('adaptive_steering', True)
        self.declare_parameter('adaptive_speed', True)
        self.declare_parameter('cmd_topic', '/cmd_control')
        
        # ========== 新增：四轮转向参数 ==========
        self.declare_parameter('four_wheel_steering_enabled', True)      # 启用四轮转向功能
        self.declare_parameter('rear_steering_ratio', 0.7)               # 后轮转向比例 (0-1)
        self.declare_parameter('rear_steering_max_deg', 12.0)            # 后轮最大转角(度)
        self.declare_parameter('min_speed_for_4ws', 1.0)                 # 低于此速度启用四轮转向(m/s)
        self.declare_parameter('min_steering_for_4ws', 8.0)              # 前轮转角大于此值才启用后轮(度)
        
        # ========== 获取参数 ==========
        path_file = self.get_parameter('path_file').value
        self.target_speed = self.get_parameter('target_speed').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.kp = self.get_parameter('kp_heading').value
        self.kd = self.get_parameter('kd_heading').value
        self.steering_alpha = self.get_parameter('steering_alpha').value
        self.max_steer_rate = math.radians(self.get_parameter('max_steer_rate_deg').value)
        self.auto_find_nearest = self.get_parameter('auto_find_nearest').value
        self.max_start_distance = self.get_parameter('max_start_distance').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.max_steering_deg = self.get_parameter('max_steering_angle').value
        self.max_steering_rad = math.radians(self.max_steering_deg)
        self.adaptive_steering = self.get_parameter('adaptive_steering').value
        self.adaptive_speed = self.get_parameter('adaptive_speed').value
        cmd_topic = self.get_parameter('cmd_topic').value
        
        # 四轮转向参数
        self.four_wheel_steering_enabled = self.get_parameter('four_wheel_steering_enabled').value
        self.rear_steering_ratio = self.get_parameter('rear_steering_ratio').value
        self.rear_steering_max_deg = self.get_parameter('rear_steering_max_deg').value
        self.min_speed_for_4ws = self.get_parameter('min_speed_for_4ws').value
        self.min_steering_for_4ws = self.get_parameter('min_steering_for_4ws').value
        
        # ========== 加载轨迹 ==========
        self.path = self._load_path(path_file)
        
        # ========== 订阅/发布 ==========
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self._gps_cb, 10)
        self.yaw_sub = self.create_subscription(Float32, '/imu/yaw', self._yaw_cb, 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, cmd_topic, 10)
        
        # ========== 新增：发布后轮转向指令 ==========
        self.rear_steering_pub = self.create_publisher(Float32, '/rear_steering_cmd', 10)
        self.four_wheel_steering_status_pub = self.create_publisher(Bool, '/four_wheel_steering_active', 10)
        
        # ========== 虚拟目标点（来自避障节点）==========
        self.virtual_target = None          # (x, y)
        self.virtual_target_time = None
        self.virtual_target_timeout = 5.0   # 5秒超时
        self.avoiding = False
        self.virtual_target_sub = self.create_subscription(
            Point, '/virtual_target', self._virtual_target_cb, 10
        )
        
        # ========== 状态变量 ==========
        self.lat = 0.0
        self.lon = 0.0
        self.yaw = 0.0
        self.idx = 0
        self.nearest_found = False
        self.target_idx = 0
        self.stop_requested = False
        
        # PD控制 + 滤波状态
        self.prev_error = 0.0
        self.prev_steering = 0.0
        
        # GPS转XY相关
        self.origin_lat = None
        self.origin_lon = None
        self.current_x = 0.0
        self.current_y = 0.0
        
        # 调试计数器
        self.debug_count = 0
        self.virtual_debug_count = 0
        
        # 后轮转向状态
        self.last_rear_steering = 0.0
        self.four_ws_active = False
        
        # ========== 定时器 ==========
        self.create_timer(0.05, self._control)
        
        # ========== 启动信息 ==========
        self.get_logger().info("=" * 60)
        self.get_logger().info("循迹节点启动")
        self.get_logger().info(f"  轨迹文件: {path_file}")
        self.get_logger().info(f"  轨迹点数: {len(self.path)}")
        self.get_logger().info(f"  目标速度: {self.target_speed} m/s")
        self.get_logger().info(f"  预瞄距离: {self.lookahead} m")
        self.get_logger().info(f"  航向系数: Kp={self.kp}, Kd={self.kd}")
        self.get_logger().info(f"  转向滤波: alpha={self.steering_alpha}, max_rate={math.degrees(self.max_steer_rate):.1f}°/周期")
        self.get_logger().info(f"  最大转向角: {self.max_steering_deg}°")
        self.get_logger().info(f"  自适应转向: {'开启' if self.adaptive_steering else '关闭'}")
        self.get_logger().info(f"  自适应速度: {'开启' if self.adaptive_speed else '关闭'}")
        self.get_logger().info(f"  发布话题: {cmd_topic}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔧 四轮转向配置:")
        self.get_logger().info(f"  四轮转向: {'启用' if self.four_wheel_steering_enabled else '禁用'}")
        self.get_logger().info(f"  后轮转向比例: {self.rear_steering_ratio}")
        self.get_logger().info(f"  后轮最大转角: {self.rear_steering_max_deg}°")
        self.get_logger().info(f"  启用速度阈值: < {self.min_speed_for_4ws} m/s")
        self.get_logger().info(f"  最小前轮转角: > {self.min_steering_for_4ws}°")
        self.get_logger().info("=" * 60)

    def _gps_to_xy(self, lat, lon):
        """将GPS坐标转换为局部平面坐标(米)，X=东，Y=北"""
        if self.origin_lat is None:
            self.origin_lat = lat
            self.origin_lon = lon
            return 0.0, 0.0
        
        dx = (lon - self.origin_lon) * 111320 * math.cos(math.radians(lat))
        dy = (lat - self.origin_lat) * 111320
        return dx, dy

    def _load_path(self, path_file):
        """加载轨迹文件"""
        path = []
        possible_paths = [
            path_file,
            os.path.join(os.path.expanduser('~'), path_file),
            os.path.join('/home/sunrise/ros2_ws', path_file),
            os.path.join('/home/sunrise/ros2_ws/src/car_autonomous_pkg', path_file),
            os.path.join(os.getcwd(), path_file),
            os.path.join(os.getcwd(), 'src/car_autonomous_pkg', path_file),
        ]
        
        # 如果 ROS2 包已安装，优先使用 share 目录下的轨迹文件
        try:
            pkg_dir = get_package_share_directory('car_autonomous_pkg')
            possible_paths.append(os.path.join(pkg_dir, path_file))
        except Exception:
            pass
        
        file_found = None
        for p in possible_paths:
            if os.path.exists(p):
                file_found = p
                break
        
        if not file_found:
            self.get_logger().error(f"找不到轨迹文件: {path_file}")
            self.get_logger().error(f"搜索路径: {possible_paths}")
            return path
        
        self.get_logger().info(f"✅ 找到轨迹文件: {file_found}")
        
        try:
            with open(file_found, 'r') as f:
                reader = csv.reader(f)
                next(reader)
                for row in reader:
                    if len(row) >= 2:
                        path.append((float(row[0]), float(row[1])))
            self.get_logger().info(f"✅ 加载轨迹: {len(path)} 个航点")
        except Exception as e:
            self.get_logger().error(f"加载失败: {e}")
        
        return path

    def _gps_cb(self, msg):
        """GPS回调"""
        self.lat = msg.latitude
        self.lon = msg.longitude
        
        # 更新当前XY坐标
        self.current_x, self.current_y = self._gps_to_xy(self.lat, self.lon)
        
        if (self.auto_find_nearest and not self.nearest_found and 
            self.path and self.lat != 0.0):
            self._find_nearest_point()

    def _yaw_cb(self, msg):
        """航向角回调"""
        self.yaw = msg.data

    def _virtual_target_cb(self, msg):
        """虚拟目标点回调（来自避障节点）"""
        self.get_logger().info(f"🔔 收到虚拟目标点回调: x={msg.x}, y={msg.y}")
        
        if math.isinf(msg.x) or math.isinf(msg.y):
            # 清除虚拟目标点
            self.get_logger().info("🚫 清除虚拟目标点，恢复正常循迹")
            self.virtual_target = None
            self.virtual_target_time = None
            self.avoiding = False
            # 退出避障时禁用四轮转向
            self._set_four_wheel_steering(False)
        else:
            # 设置虚拟目标点
            self.virtual_target = (msg.x, msg.y)
            self.virtual_target_time = self.get_clock().now()
            self.avoiding = True
            self.get_logger().info(f"🎯 设置虚拟目标点: ({msg.x:.2f}, {msg.y:.2f})")
            # 进入避障模式，启用四轮转向
            self._set_four_wheel_steering(True)

    def _dist(self, lat1, lon1, lat2, lon2):
        """计算两点距离（米）"""
        dlat = (lat2 - lat1) * 111320
        dlon = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
        return math.hypot(dlat, dlon)
    
    def _dist_xy(self, x1, y1, x2, y2):
        """计算XY坐标两点距离（米）"""
        return math.hypot(x1 - x2, y1 - y2)
    
    def _dist_to_end(self):
        """计算到终点的距离"""
        if not self.path:
            return float('inf')
        end_lat, end_lon = self.path[-1]
        return self._dist(self.lat, self.lon, end_lat, end_lon)

    def _find_nearest_point(self):
        """找到离当前位置最近的轨迹点作为起点"""
        if not self.path:
            return
        
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, (lat, lon) in enumerate(self.path):
            dist = self._dist(self.lat, self.lon, lat, lon)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        self.idx = nearest_idx
        self.nearest_found = True
        self.get_logger().info(f"✅ 找到最近点: 索引={nearest_idx}, 距离={min_dist:.2f}m")

    def _calculate_curvature(self, error):
        """计算路径曲率"""
        if self.lookahead > 0:
            curvature = abs(2.0 * math.sin(error) / self.lookahead)
        else:
            curvature = 0.0
        return curvature

    def _get_preview_curvature(self):
        """预判前方路径曲率，用于提前减速
        
        扫描前方航点，计算每米航向变化率，
        返回最大值（越大=弯越急）
        """
        if self.idx >= len(self.path) - 2:
            return 0.0
        
        # 预判距离：基础3m + 速度*1.2秒
        preview_dist = max(3.0, self.target_speed * 1.2)
        
        max_rate = 0.0
        accumulated = 0.0
        prev_bearing = None
        
        for i in range(self.idx, min(self.idx + 40, len(self.path) - 1)):
            lat1, lon1 = self.path[i]
            lat2, lon2 = self.path[i + 1]
            seg_dist = self._dist(lat1, lon1, lat2, lon2)
            if seg_dist < 0.01:
                continue
            accumulated += seg_dist
            if accumulated > preview_dist:
                break
            
            dx = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
            dy = (lat2 - lat1) * 111320
            bearing = math.atan2(dx, dy)
            
            if prev_bearing is not None:
                diff = bearing - prev_bearing
                while diff > math.pi:
                    diff -= 2 * math.pi
                while diff < -math.pi:
                    diff += 2 * math.pi
                rate = abs(diff) / seg_dist
                if rate > max_rate:
                    max_rate = rate
            prev_bearing = bearing
        
        return max_rate

    def _get_dynamic_max_steering(self, curvature):
        """根据曲率计算动态最大转向角"""
        if not self.adaptive_steering:
            return self.max_steering_rad
        
        if curvature > 0.5:
            return self.max_steering_rad
        elif curvature > 0.3:
            return self.max_steering_rad * 0.85
        elif curvature > 0.15:
            return self.max_steering_rad * 0.7
        else:
            return self.max_steering_rad * 0.5
            
    def _get_speed_factor(self, steering_deg, dist_to_end, preview_curvature=0.0):
        """根据转向角和前方曲率计算速度因子"""
        if not self.adaptive_speed:
            return 1.0
        
        steering_abs = abs(steering_deg)
        
        # 根据当前转向角减速（反应式）
        if steering_abs > 25:
            speed_factor = 0.35
        elif steering_abs > 18:
            speed_factor = 0.5
        elif steering_abs > 12:
            speed_factor = 0.7
        elif steering_abs > 6:
            speed_factor = 0.85
        else:
            speed_factor = 1.0
        
        # 前方曲率预判减速（主动式，弯道前就减速）
        if preview_curvature > 0.15:       # 急弯（>8.6°/m）
            speed_factor = min(speed_factor, 0.3)
        elif preview_curvature > 0.08:     # 中弯（>4.6°/m）
            speed_factor = min(speed_factor, 0.5)
        elif preview_curvature > 0.04:     # 缓弯（>2.3°/m）
            speed_factor = min(speed_factor, 0.7)
        
        # 终点减速保持
        if dist_to_end < 3.0:
            speed_factor = min(speed_factor, dist_to_end / 3.0)
        
        return speed_factor

    def _calculate_rear_steering(self, front_steering_deg, current_speed, is_avoiding):
        """计算后轮转向角度"""
        if not self.four_wheel_steering_enabled:
            return 0.0
        
        should_enable_4ws = is_avoiding or (current_speed < self.min_speed_for_4ws and 
                                            abs(front_steering_deg) > self.min_steering_for_4ws)
        
        if not should_enable_4ws:
            if self.four_ws_active:
                self._set_four_wheel_steering(False)
            return 0.0
        
        if not self.four_ws_active:
            self._set_four_wheel_steering(True)
        
        if is_avoiding:
            ratio = self.rear_steering_ratio * 1.2
        else:
            ratio = self.rear_steering_ratio
        
        # 关键修改：改成 + (同向) 试试
        # 原来是反向：rear_angle = -front_steering_deg * ratio
        rear_angle = front_steering_deg * ratio   # 改为同向
        
        rear_angle = max(-self.rear_steering_max_deg, min(self.rear_steering_max_deg, rear_angle))
        
        if abs(rear_angle) < 1.0:
            rear_angle = 0.0
        
        return rear_angle
    
    def _set_four_wheel_steering(self, active):
        """设置四轮转向状态"""
        self.four_ws_active = active
        
        # 发布状态消息
        status_msg = Bool()
        status_msg.data = active
        self.four_wheel_steering_status_pub.publish(status_msg)
        
        if active:
            self.get_logger().info("🔧 四轮转向已启用")
        else:
            self.get_logger().info("🔧 四轮转向已禁用")
            # 发送后轮回零指令
            rear_msg = Float32()
            rear_msg.data = 0.0
            self.rear_steering_pub.publish(rear_msg)
    
    def _publish_rear_steering(self, rear_angle_deg):
        """发布后轮转向指令"""
        # 只有角度变化超过0.5度时才发布，减少CAN负载
        if abs(rear_angle_deg - self.last_rear_steering) > 0.5:
            rear_msg = Float32()
            rear_msg.data = math.radians(rear_angle_deg)
            self.rear_steering_pub.publish(rear_msg)
            self.last_rear_steering = rear_angle_deg
            
            if abs(rear_angle_deg) > 1.0:
                self.get_logger().debug(f"🔧 后轮转向: {rear_angle_deg:.1f}°")

    def _control(self):
        """主控制循环"""
        self.debug_count += 1
        
        # 检查轨迹
        if not self.path:
            return
        
        # 等待GPS数据
        if self.lat == 0.0:
            if self.debug_count % 100 == 0:
                self.get_logger().info("等待GPS数据...")
            return
        
        if self.auto_find_nearest and not self.nearest_found:
            if self.debug_count % 100 == 0:
                self.get_logger().info(f"等待定位最近点...")
            return
        
        # 检查是否到达终点
        dist_to_end = self._dist_to_end()
        if dist_to_end <= self.stop_distance:
            if not self.stop_requested:
                self._stop()
                self.stop_requested = True
                self.get_logger().info(f"🏁 到达终点! 距离: {dist_to_end:.2f}m")
                # 到达终点时禁用四轮转向
                self._set_four_wheel_steering(False)
            return
        
        # ========== 选择目标点 ==========
        tx = None
        ty = None
        use_virtual = False
        
        # 检查虚拟目标点超时
        if self.virtual_target_time is not None:
            elapsed = (self.get_clock().now() - self.virtual_target_time).nanoseconds / 1e9
            if elapsed > self.virtual_target_timeout:
                self.get_logger().warn(f"⏰ 虚拟目标点超时 ({elapsed:.1f}s > {self.virtual_target_timeout}s)，清除")
                self.virtual_target = None
                self.virtual_target_time = None
                self.avoiding = False
                self._set_four_wheel_steering(False)
        
        # 存储虚拟目标点的全局坐标（用于距离检测）
        virtual_target_global = None
        
        if self.virtual_target is not None:
            use_virtual = True
            tx_rel, ty_rel = self.virtual_target
            
            self.get_logger().info(f"🚧 [避障模式] 使用虚拟目标点: rel=({tx_rel:.2f}, {ty_rel:.2f})")
            self.get_logger().info(f"   当前坐标: cur=({self.current_x:.2f}, {self.current_y:.2f}), yaw={math.degrees(self.yaw):.1f}°")
            
            # 坐标系转换（与之前相同）
            yaw_rad_math = math.radians(90 - math.degrees(self.yaw))
            
            # 计算虚拟目标点的全局坐标
            target_global_x = self.current_x + tx_rel * math.cos(yaw_rad_math) - ty_rel * math.sin(yaw_rad_math)
            target_global_y = self.current_y + tx_rel * math.sin(yaw_rad_math) + ty_rel * math.cos(yaw_rad_math)
            virtual_target_global = (target_global_x, target_global_y)
            
            # 计算从当前位置到虚拟目标点的距离
            dist_to_virtual_target = math.hypot(
                target_global_x - self.current_x, 
                target_global_y - self.current_y
            )
            
            self.get_logger().info(f"   虚拟目标点全局坐标: ({target_global_x:.2f}, {target_global_y:.2f})")
            self.get_logger().info(f"   到达虚拟目标点距离: {dist_to_virtual_target:.2f}m")
            
            if dist_to_virtual_target < 0.5:
                self.get_logger().info(f"✅ 车辆已到达虚拟目标点，退出避障模式，恢复循迹")
                self.virtual_target = None
                self.virtual_target_time = None
                self.avoiding = False
                self._set_four_wheel_steering(False)
                use_virtual = False
                return
            else:
                self.get_logger().info(f"🚧 继续避障模式，还需行驶 {dist_to_virtual_target:.2f}m 到达目标点")
                target_lat, target_lon = self._xy_to_gps(target_global_x, target_global_y)
                tx, ty = target_lat, target_lon
                
                if hasattr(self, '_avoidance_start_time') is False:
                    self._avoidance_start_time = self.get_clock().now()
                else:
                    avoidance_elapsed = (self.get_clock().now() - self._avoidance_start_time).nanoseconds / 1e9
                    if avoidance_elapsed > 30.0:
                        self.get_logger().error(f"❌ 避障模式超时 ({avoidance_elapsed:.1f}s > 30s)，强制退出")
                        self.virtual_target = None
                        self.virtual_target_time = None
                        self.avoiding = False
                        self._set_four_wheel_steering(False)
                        delattr(self, '_avoidance_start_time')
                        return
            
            self.get_logger().info(f"   转换为GPS: lat={tx:.7f}, lon={ty:.7f}")
        
        if not use_virtual:
            if hasattr(self, '_avoidance_start_time'):
                delattr(self, '_avoidance_start_time')
                # 退出避障时，延迟禁用四轮转向（给车辆时间回正）
                self._set_four_wheel_steering(False)
            
            # 速度自适应预瞄距离：高速看更远
            dynamic_lookahead = max(self.lookahead, self.target_speed * 0.5)
            
            for i in range(self.idx, len(self.path)):
                dist_to_waypoint = self._dist(self.lat, self.lon, self.path[i][0], self.path[i][1])
                if dist_to_waypoint >= dynamic_lookahead:
                    self.target_idx = i
                    break
            else:
                self.target_idx = len(self.path) - 1
            
            tx, ty = self.path[self.target_idx]
            
            if self.target_idx > self.idx:
                self.idx = self.target_idx
        
        if tx is None or ty is None:
            self.get_logger().warn("⚠️ 目标点为None，跳过本帧")
            return
        
        # 计算期望航向
        dx = (ty - self.lon) * 111320 * math.cos(math.radians(self.lat))
        dy = (tx - self.lat) * 111320
        desired = math.atan2(dx, dy)
        
        error = desired - self.yaw
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        error_deg = math.degrees(error)
        
        target_bearing = math.degrees(desired)
        if target_bearing < 0:
            target_bearing += 360
        current_bearing = math.degrees(self.yaw)
        if current_bearing < 0:
            current_bearing += 360
        
        self.get_logger().info(f"📍 方位角: 目标={target_bearing:.1f}°, 当前={current_bearing:.1f}°, 误差={error_deg:.1f}°")
        
        if abs(error_deg) > 90:
            self.get_logger().warn(f"⚠️ 航向偏差过大: {error_deg:.1f}°，停车等待")
            self._stop()
            return
        
        # 计算曲率
        curvature = self._calculate_curvature(error)
        
        # 计算动态最大转向角
        dynamic_max_steering = self._get_dynamic_max_steering(curvature)
        
        # ========== PD控制 + 低通滤波 + 速率限制 ==========
        # D项：误差变化率（阻尼振荡）
        dt = 0.05  # 定时器周期20Hz
        error_rate = (error - self.prev_error) / dt
        self.prev_error = error
        
        # PD控制
        steering_raw = -(self.kp * error + self.kd * error_rate)
        steering_raw = max(-dynamic_max_steering, min(dynamic_max_steering, steering_raw))
        
        # 低通滤波（平滑转向输出，出弯后渐进回归）
        steering_filtered = self.steering_alpha * steering_raw + (1.0 - self.steering_alpha) * self.prev_steering
        
        # 速率限制（防止转向突变）
        max_delta = self.max_steer_rate
        delta = steering_filtered - self.prev_steering
        delta = max(-max_delta, min(max_delta, delta))
        steering = self.prev_steering + delta
        steering = max(-dynamic_max_steering, min(dynamic_max_steering, steering))
        self.prev_steering = steering
        
        steering_deg = math.degrees(steering)
        
        # 前方路径曲率预判（提前减速）
        preview_curv = self._get_preview_curvature()
        
        # 计算速度因子和当前速度
        speed_factor = self._get_speed_factor(steering_deg, dist_to_end, preview_curv)
        current_speed = self.target_speed * speed_factor
        
        # ========== 新增：计算并发布后轮转向指令 ==========
        if self.four_wheel_steering_enabled:
            rear_angle = self._calculate_rear_steering(steering_deg, current_speed, use_virtual)
            self._publish_rear_steering(rear_angle)
        
        # 调试输出（每5帧打印一次）
        if self.debug_count % 5 == 0:
            yaw_deg = math.degrees(self.yaw)
            desired_deg = math.degrees(desired)
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"📍 位置: lat={self.lat:.7f}, lon={self.lon:.7f}")
            self.get_logger().info(f"   XY: ({self.current_x:.2f}, {self.current_y:.2f})")
            self.get_logger().info(f"🧭 当前航向: {yaw_deg:.1f}°")
            self.get_logger().info(f"🎯 目标航向: {desired_deg:.1f}°")
            self.get_logger().info(f"📐 航向误差: {error_deg:.1f}° (rate={math.degrees(error_rate):.1f}°/s)")
            self.get_logger().info(f"🔧 输出前轮转向: {steering_deg:.1f}° (raw={math.degrees(steering_raw):.1f}°, filt={math.degrees(steering_filtered):.1f}°)")
            if self.four_wheel_steering_enabled and self.four_ws_active:
                self.get_logger().info(f"🔧 后轮转向: {self.last_rear_steering:.1f}°")
            self.get_logger().info(f"⚡ 速度: {current_speed:.2f} m/s (factor={speed_factor:.2f}, preview_curv={preview_curv:.3f})")
            self.get_logger().info(f"📏 到终点距离: {dist_to_end:.2f}m")
            if use_virtual and virtual_target_global:
                self.get_logger().info(f"🚧 避障模式: 虚拟目标点 rel=({self.virtual_target[0]:.2f}, {self.virtual_target[1]:.2f})")
            elif not use_virtual:
                self.get_logger().info(f"📍 正常模式: 预瞄点 ({tx:.7f}, {ty:.7f})")
            self.get_logger().info("=" * 60)
        
        # 发布控制指令
        msg = AckermannDriveStamped()
        msg.drive.speed = float(current_speed)
        msg.drive.steering_angle = steering
        self.cmd_pub.publish(msg)

    def _xy_to_gps(self, x, y):
        """将全局XY坐标转换回GPS坐标"""
        if self.origin_lat is None or self.origin_lon is None:
            self.get_logger().warn("_xy_to_gps: origin未设置")
            return 0.0, 0.0
        
        lat = self.origin_lat + y / 111320
        lon = self.origin_lon + x / (111320 * math.cos(math.radians(lat)))
        return lat, lon

    def _stop(self):
        """停止车辆"""
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.cmd_pub.publish(msg)
        # 停止时也禁用后轮转向
        self._set_four_wheel_steering(False)
        # 重置PD状态，防止累积误差导致重启后突变
        self.prev_error = 0.0
        self.prev_steering = 0.0
        self.get_logger().info("🛑 发送停止指令")


        # 防止重复生成报告
        if hasattr(self, '_report_generated'):
            return
        self._report_generated = True
        
        pkg_dir = os.path.join(os.getcwd(), 'src/car_autonomous_pkg/')
        ref_file = self.get_parameter('path_file').value
        if not os.path.isabs(ref_file):
            ref_file = os.path.join(pkg_dir, ref_file)
        log_file = os.path.join(pkg_dir, 'logs', 'latest_drive_log.csv')
        report_img = os.path.join(pkg_dir, 'report/control_report.png')
        script = os.path.join(pkg_dir, 'scripts', 'plot_control_comparison.py')
        
        # 检查文件是否存在
        if not os.path.exists(log_file):
            self.get_logger().warn("⚠️ 日志文件不存在，跳过报告生成（请启动 control_logger）")
            return
        if not os.path.exists(ref_file) or not os.path.exists(script):
            self.get_logger().error("❌ 参考轨迹或绘图脚本缺失，无法生成报告")
            return
        
        self.get_logger().info("📊 正在生成控制效果对比报告...")
        try:
            subprocess.run(
                ['python3', script, ref_file, log_file, report_img],
                capture_output=True,
                text=True,
                timeout=300,
                check=True
            )
            self.get_logger().info(f"✅ 报告已保存: {report_img}")
        except subprocess.TimeoutExpired:
            self.get_logger().error("⏰ 报告生成超时")
        except Exception as e:
            self.get_logger().error(f"❌ 报告生成失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()