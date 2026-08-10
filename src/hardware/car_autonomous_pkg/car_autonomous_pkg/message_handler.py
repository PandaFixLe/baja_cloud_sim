# car_autonomous_pkg/message_handler.py
import math


class CanMessageHandler:
    def __init__(self, ros_node):
        self.node = ros_node

    def on_message_received(self, msg, db):
        """接收 CAN 消息并解析 VCU 反馈"""
        if db is None:
            return

        try:
            decoded = db.decode_message(msg.arbitration_id, msg.data)
            if not decoded:
                return
        except Exception:
            return

        try:
            msg_name = db.get_message_by_frame_id(msg.arbitration_id).name
        except KeyError:
            return

        # 只处理 VCU_Info1（速度和转角反馈）
        if msg_name == 'VCU_Info1':
            speed = decoded.get('Current_Speed', 0.0) * 0.1      # 分辨率 0.1 m/s
            angle_deg = decoded.get('Current_Angle', 0.0) * 0.1  # 分辨率 0.1°
            self.node.publish_vehicle_status(speed, angle_deg)

        # 其他 VCU 消息（Info2/3/4）暂时忽略，可后续扩展
        elif msg_name in ('VCU_Info2', 'VCU_Info3', 'VCU_Info4'):
            pass  # 暂不处理