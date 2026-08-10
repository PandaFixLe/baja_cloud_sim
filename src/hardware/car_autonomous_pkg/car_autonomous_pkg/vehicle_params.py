import math

class VehicleParams:
    # 新底盘几何参数（仅供参考，实际控制使用方向盘角度）
    LENGTH = 3.176
    WIDTH = 1.500
    WHEELBASE = 1.430        # 轴距 1430 mm
    TRACK_WIDTH = 1.320      # 前轮轮距 1320 mm
    WHEEL_RADIUS = 0.292
    
    # DBC 中方向盘转角范围为 ±70.0° (物理值 ±700, 分辨率0.1)
    MAX_STEERING_ANGLE_DEG = 70.0
    MAX_STEERING_ANGLE_RAD = math.radians(MAX_STEERING_ANGLE_DEG)
    
    # 速度限制
    MAX_SPEED_MS = 5.0
    MAX_REVERSE_SPEED_MS = -2.0
    
    # 轮胎相关（保留）
    TIRE_CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS
    MS_TO_RPM = 60.0 / TIRE_CIRCUMFERENCE
    
    @classmethod
    def speed_to_rpm(cls, speed_ms):
        return int(speed_ms * cls.MS_TO_RPM)
    
    @classmethod
    def print_info(cls, logger):
        logger.info(f"轴距:{cls.WHEELBASE}m 轮距:{cls.TRACK_WIDTH}m 车轮半径:{cls.WHEEL_RADIUS}m")
        logger.info(f"最大速度:{cls.MAX_SPEED_MS}m/s 最大转向:{cls.MAX_STEERING_ANGLE_DEG:.1f}°")   