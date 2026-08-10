# car_autonomous_pkg/can_manager.py
import can
import cantools
import threading
import time
import subprocess
from rclpy.logging import get_logger

logger = get_logger('can_manager')


class CanManager:
    def __init__(self, dbc_paths, channel='can0', bustype='socketcan', bitrate=500000):
        """
        支持多个DBC文件
        
        Args:
            dbc_paths: DBC文件路径列表或单个路径
            channel: CAN通道
            bustype: CAN总线类型
            bitrate: 波特率
        """
        # 加载多个DBC文件
        self.db = None
        if isinstance(dbc_paths, list):
            for dbc_path in dbc_paths:
                if self.db is None:
                    self.db = cantools.database.load_file(dbc_path)
                else:
                    other_db = cantools.database.load_file(dbc_path)
                    for msg in other_db.messages:
                        self.db.messages.append(msg)
                    for node in other_db.nodes:
                        if node not in self.db.nodes:
                            self.db.nodes.append(node)
            self.db.refresh()
            logger.info(f"✅ 加载 {len(dbc_paths)} 个DBC文件")
        else:
            self.db = cantools.database.load_file(dbc_paths)
            logger.info("✅ 加载DBC文件")
        
        # 初始化CAN总线
        self.bus = None
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        
        self._init_can_bus()
        
        self.send_lock = threading.Lock()
        self.notifier = None
        
        if self.bus:
            self.notifier = can.Notifier(self.bus, [])
            logger.info(f"📡 CAN总线已就绪: {channel}")
        else:
            logger.warn("⚠️ CAN总线未就绪")
    
    def _init_can_bus(self):
        """初始化CAN总线"""
        if self.bustype == 'socketcan':
            try:
                subprocess.run(['sudo', 'ip', 'link', 'set', self.channel, 'up'], 
                             capture_output=True)
                self.bus = can.interface.Bus(
                    channel=self.channel, 
                    bustype='socketcan',
                    bitrate=self.bitrate
                )
                logger.info(f"✅ SocketCAN: {self.channel}")
                return
            except Exception as e:
                logger.warn(f"SocketCAN失败: {e}")
        
        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                bustype='pcan',
                bitrate=self.bitrate
            )
            logger.info(f"✅ PCAN: {self.channel}")
            return
        except Exception as e:
            logger.warn(f"PCAN失败: {e}")
        
        try:
            self.bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
            logger.warn("⚠️ 使用虚拟CAN模式")
            return
        except:
            pass
        
        logger.error("❌ 无法初始化CAN总线")
    
    def register_receiver(self, callback):
        if self.notifier:
            self.notifier.add_listener(callback)
    
    def send_message(self, message_name, signals_dict, is_extended=False):
        """
        发送CAN消息
        Args:
            message_name: DBC中的消息名称
            signals_dict: 信号字典
            is_extended: 是否使用扩展帧（默认False）
        """
        if not self.bus:
            return False
        try:
            data_bytes = self.db.encode_message(message_name, signals_dict)
            msg_id = self.db.get_message_by_name(message_name).frame_id
            msg = can.Message(arbitration_id=msg_id, data=data_bytes, is_extended_id=is_extended)
            with self.send_lock:
                self.bus.send(msg)
            return True
        except Exception as e:
            logger.error(f"发送失败 {message_name}: {e}")
            return False
    
    def shutdown(self):
        if self.notifier:
            self.notifier.stop()
        if self.bus:
            self.bus.shutdown()
        logger.info("🛑 CAN总线已关闭")