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
