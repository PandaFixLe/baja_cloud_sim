# Baja 云端规划控制仿真

面向 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Harmonic 的规划控制闭环工程。算法节点只使用
Python 标准库和 ROS 2 消息，不依赖 Torch、NumPy、SciPy、OpenCV。

## 场景与车辆

- 约 100 m 路线：50 m 长直道、半径 15 m 的 90°左转和末段直道；
- 航路点按真实参考线弧长每 0.5 m 采样；
- 两个最大坡度小于 10°的平滑土坡；
- 两个低矮圆弧截面的减速带；
- 可复现的随机障碍物边界框；
- 292 kg 后驱、前轮 Ackermann 转向四轮刚体模型；
- 质量、转动惯量、轮胎接触摩擦、转向限位、速度/加速度/jerk 限制；
- Gazebo 世界真实位姿 `/ground_truth/odom`；
- 1.5 cm 标准差、3σ有界的位置噪声，输出 `/localization/odom`、`/gps/fix` 和 `/imu/yaw`。

当前模型属于基础刚体接触动力学，不包含悬架连杆、轮胎形变、动力电机转矩曲线、制动液压
和可变土壤沉陷，因此不能视为经过实车标定的完整整车动力学模型。

## 安装与构建

```bash
cd /mnt/data/baja_cloud_sim
sudo apt-get install -y ffmpeg
./build.sh
```

如果 ROS 2 Humble、Gazebo Harmonic 等环境尚未安装，再使用：

```bash
./install_ubuntu2204.sh
```

## 运行

```bash
./run.sh --seed 42 --obstacles 5
```

无 Gazebo 图形窗口：

```bash
./run.sh --seed 42 --obstacles 5 --headless-gazebo
```

完全无桌面但仍录制 Gazebo 相机视频：

```bash
./run.sh --seed 42 --obstacles 5 --headless-gazebo --no-rviz
```

调试时可用 `--no-video` 关闭录像。正常运行时按 `Ctrl+C`，录像节点会关闭 ffmpeg 并完成
MP4 文件封装，不要直接使用 `kill -9`。

## 数据流

```text
Gazebo OdometryPublisher（世界真实位姿）
  └─ /ground_truth/odom
       ├─ truth_perception
       │    ├─ /localization/odom
       │    ├─ /gps/fix
       │    ├─ /imu/yaw
       │    ├─ /reference_centerline
       │    ├─ /road_boundary_markers
       │    └─ /obstacle_markers
       └─ evaluator

frenet_planner
  └─ /planned_path
       └─ path_follower
            └─ /cmd_control（期望速度 + 期望转角）
                 └─ actuator_adapter
                      └─ /model/baja_vehicle/cmd_vel
                           └─ Gazebo AckermannSteering
```

`/wheel_odom` 是 Ackermann 插件根据轮速和转向角积分得到的轮式里程计；它与世界真值分开，
不用于闭环评价。

## 已知问题：Frenet 规划器的时间戳与时间同步

`frenet_planner_node.py` 目前对各传感器话题采用"最新值快照"策略，回调里完全没有使用消息头
`header.stamp`，存在以下时间戳相关问题：

1. **多源数据无时间同步**：`/gps/fix`、`/imu/yaw`、`/road_boundary_markers`、
   `/obstacle_markers` 分别在各自回调里直接覆盖 `self.position`、`self.yaw_world`、
   `self.left_world` / `self.right_world`、`self.obstacles`。定时器 `_plan`（10 Hz）触发时
   把当前所有最新值直接融合，不校验它们是否来自同一时刻。各话题频率不同（GPS/IMU 约 20 Hz、
   边界/障碍约 10 Hz），因此每次规划使用的其实是一组时间上并不对齐的数据。此外 `/imu/yaw`
   使用 `std_msgs/Float32`，消息本身没有 header，从类型上就无法携带时间戳。

2. **坐标变换用的是回调时刻位姿，而非传感器采样时刻位姿**：`_boundary_callback`、
   `_obstacle_callback` 用 `base_to_world(..., self.position, self.yaw_world)` 把车体系点
   转到世界系，使用的是"当前最新"的位姿，而不是该边界/障碍消息被观测时的位姿。当车辆以速度
   v 行驶、位姿与感知数据存在 Δt 的时间差时，会引入约 v·Δt 的位置误差（例如
   5 m/s × 50 ms ≈ 0.25 m），障碍膨胀框和可行走廊会随之偏移。

3. **规划路径盖的是"发布时刻"时间戳**：`_plan` 中
   `message.header.stamp = self.get_clock().now().to_msg()`，输出的 `/planned_path` 打的是
   发布瞬间的时间，而不是其所依据的里程计/感知数据的时间。下游（`path_follower`、
   `evaluator`）无法据此判断规划结果对应的真实时刻与延迟，容易产生时间错位（类似评价节点
   历史上出现过的 ghost trail 问题）。

4. **缺少数据新鲜度检查**：`_plan` 仅检查数据是否存在（`self.position is None`、
   `len(self.centerline) < 3` 等），不检查数据是否过期。一旦 GPS/IMU 掉线，规划器会无限期
   沿用陈旧位姿，而不会降级或安全停车。

**改进方向**：

- 订阅时读取并保存每条消息的 `header.stamp`，在 `_plan` 中对齐到共同时间基准，或使用
  `message_filters` 做近似时间同步；对 `/imu/yaw` 换用带 header 的消息类型（如
  `sensor_msgs/Imu`）。
- 坐标变换使用与感知消息时间戳最接近的位姿，对位姿做时间插值/外推后再做 `base_to_world`。
- `/planned_path` 使用输入数据的时间戳而非发布时刻，保证与下游的时间一致性。
- 增加数据超时判断，超时则发布 `INFEASIBLE` 或触发安全停车。

## 输出

```text
runtime/scenario_<seed>/baja_100m.sdf
runtime/scenario_<seed>/dirt_road.obj
runtime/scenario_<seed>/scenario.json
results/seed_<seed>/tracking_YYYYMMDD_HHMMSS.csv
results/seed_<seed>/gazebo_YYYYMMDD_HHMMSS.mp4
```

MP4 来自附着在车辆上的 Gazebo 1280×720、30 FPS 追踪相机，在 GUI 和服务器无界面模式下
都可录制。

## 测试

```bash
cd /mnt/data/baja_cloud_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover -s src/baja_cloud_sim/test -v
```
