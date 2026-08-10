# 超声波雷达 ROS2 驱动

基于 SocketCAN 的超声波雷达 ROS2 Humble 驱动，支持 vcan3 虚拟CAN接口。

## 硬件协议

- CAN ID:
  - `0x18D` - 结果/温度
  - `0x3A1` - 后雷达距离1
  - `0x3A2` - 混合距离 (前左 + 后左)
  - `0x3A3` - 前雷达距离
- 距离精度: 2cm, 量程: 0~510cm
- 共12路自发自收距离数据 (前6 + 后6)
- DBC格式: Motorola (大端, @0+)

## 环境依赖

- Ubuntu 22.04
- ROS2 Humble
- SocketCAN (Linux内核自带)

## 编译安装

```bash
cd ~/tiamo
colcon build --packages-select ultrasonic_radar_driver
source install/setup.bash