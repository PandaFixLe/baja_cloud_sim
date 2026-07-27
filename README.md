# Baja 云端规划控制仿真

面向 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Harmonic 的规划控制闭环工程。
Frenet 规划器 (`core.py`) 仅依赖 Python 标准库；LQR 控制器需 `numpy`、`scipy`。

## 文档索引

| 文档 | 用途 |
| --- | --- |
| [`docs/frenet_planner.md`](docs/frenet_planner.md) | Frenet 局部规划器的坐标基础、Apollo EM Planner 关联、简化形式与运行时结构 |
| [`docs/releases/v1.0.md`](docs/releases/v1.0.md) | v1.0 基线（分层架构定型版）的发布说明 |
| [`docs/planning/v1.1-backlog.md`](docs/planning/v1.1-backlog.md) | v1.1 待办清单 |
| [`docs/planning/post-v1.1-backlog.md`](docs/planning/post-v1.1-backlog.md) | v1.1 后的结构性改进项 |
| [`docs/planning/background-pan-code.md`](docs/planning/background-pan-code.md) | 项目背景与架构演进 |

## 分支与版本

| 分支 | 版本 | 控制模式 | 说明 |
|------|------|---------|------|
| `main` | v1.0 | Legacy | 纯航向 P + 预瞄 |
| `Baja-resource-origin-7.23-v1.1` | v1.1 | Stanley | Stanley + PD + 双阻尼 + 自适应速度/转向 |
| `v1.2-LQR` | v1.2 | LQR | Bézier 平滑 + LQR 切片预测 + 预测轨迹可视化 |
| **`v1.3-Smoother`** | **v1.3** | **LQR** | **时间对齐查表 + Frenet(s,l) 双轴反馈 + 两级安全预警** |

## 场景与车辆

- 约 100 m 路线：50 m 长直道、半径 15 m 的 90°左转和末段直道；
- 航路点按真实参考线弧长每 0.5 m 采样；
- 两个最大坡度小于 10°的平滑土坡；
- 两个低矮圆弧截面的减速带；
- 可复现的随机障碍物边界框；
- 292 kg 后驱、前轮 Ackermann 转向四轮刚体模型；
- 质量、转动惯量、轮胎接触摩擦、转向限位、速度/加速度/jerk 限制；
- Gazebo 世界真实位姿 `/ground_truth/odom`；
- 1.5 cm 标准差、3σ有界的位置噪声。

当前模型属于基础刚体接触动力学，不包含悬架连杆、轮胎形变、动力电机转矩曲线、制动液压
和可变土壤沉陷，因此不能视为经过实车标定的完整整车动力学模型。

## 安装与构建

```bash
cd /mnt/data/baja_cloud_sim
sudo apt-get install -y ffmpeg
./build.sh
```

如果 ROS 2 Humble、Gazebo Harmonic 等环境尚未安装：

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

调试时可用 `--no-video` 关闭录像。

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

frenet_planner (10 Hz)
  └─ /planned_path (绿色, nav_msgs/Path)
       └─ path_follower (50 Hz)
            ├─ [LQR 模式] TrajectorySmoother
            │    ├─ Bézier 分段拟合 → /smoothed_path (蓝色)
            │    └─ 双通速度剖面 + 时间戳 → TrajectoryTable
            ├─ [LQR 模式] _lqr_control_step()
            │    ├─ 时间对齐查表 → (v_ref, δ_ff)
            │    ├─ Frenet(s,l) 双轴反馈 → (v_fb ±0.5, δ_fb ±3°)
            │    └─ 合成 + rate-limit + lowpass
            ├─ _compute_safety_speed()
            │    └─ /predicted_trajectory (黄色) vs planned_path → L1/L2 预警
            └─ /cmd_control (AckermannDriveStamped)
                 └─ actuator_adapter
                      └─ /model/baja_vehicle/cmd_vel
                           └─ Gazebo AckermannSteering
```

### RViz 可视化

| 颜色 | Topic | 含义 |
|------|-------|------|
| 绿色 | `/planned_path` | Planner 原始路径 |
| 蓝色 | `/smoothed_path` | Bézier 平滑后参考路径 |
| **黄色** | `/predicted_trajectory` | 当前 (v,δ) 运动学 2s 前推预测 |

## 代码结构

算法包位于 `src/baja_cloud_sim/baja_cloud_sim/`：

```text
core.py                   纯 Python 几何/规划/控制函数库（规划部分仅标准库；
                          LQR 控制路径需 numpy/scipy，由 path_follower_node 组装）
trajectory_smoother.py    Bézier 分段拟合 + 解析曲率 + 弧长重采样 + 双通速度剖面
                          （纯标准库）
lqr_controller.py         Bicycle model LQR + 离线增益表（DARE 迭代）+ 在线插值查表
                          （需 numpy、scipy）
scenario_generator.py     generate_scenario：按 seed 生成中心线、边界、障碍与 Gazebo 世界
truth_perception_node.py  truth_perception：由真值里程计派生定位/GPS/IMU/中心线/边界/障碍话题
frenet_planner_node.py    frenet_planner：Frenet 栅格 DP 规划核心，输出 /planned_path
path_follower_node.py     path_follower：支持三种模式 (legacy / stanley / lqr)
actuator_adapter_node.py  actuator_adapter：/cmd_control → Gazebo AckermannSteering 指令
evaluator_node.py         evaluator：跟踪误差等指标评价与 CSV 记录
video_recorder_node.py    video_recorder：录制 Gazebo 追踪相机视频
```

## v1.3-Smoother 控制架构

### 前馈层 — Trajectory Smoother

```
/planned_path (31点, ~1m间距)
    │
    ▼
TrajectorySmoother.generate()     ← _path_callback 触发 (≤10 Hz)
    │
    ├─ 分段三次 Bézier（3段 × 4控制点，G¹ 连续）
    ├─ 弧长重采样 (Δs=0.15m) + 解析曲率 κ(s)
    ├─ 双通速度剖面: 前向加速度约束 + 后向弯道预减速
    ├─ 时间戳: t[i] = t[i-1] + Δs / v_avg
    │
    ▼
TrajectoryTable
    generated_at: ROS 绝对时间
    points: [{t, x, y, yaw, v_ref, δ_ff, κ}, ...]
```

### 反馈层 — Frenet(s,l) 双轴反馈

每 50ms 控制周期：

1. **时间对齐查表**: `elapsed = now - table.generated_at` → 插值取参考行
2. **Frenet(s,l)**:
   - `l = signed_lateral()` — 横向偏差 (CTE)
   - `s = (pos - ref) · tangent` — 沿路径超前/落后量
3. **s 轴反馈**: `v_fb ∈ [-0.5, +0.5] m/s`，s>0（超前/下坡超速）→ 自动减速
4. **l 轴反馈**: LQR gain-scheduled δ_fb ∈ [-3°, +3°]
5. **合成**: `v = v_ref + v_fb`, `δ = δ_ff + δ_fb`
6. **后处理**: rate-limit + steering lowpass (α=0.6) + speed lowpass (α=0.6)

### 安全层 — 两级预警

对当前 (v,δ) 做运动学前推 40 步 (2s)，逐点计算到 `/planned_path` 的最短距离：

| 级别 | 最大偏差 | 动作 |
|------|---------|------|
| 正常 | < 0.8 m | 无干预 |
| L1 | 0.8 ~ 1.5 m | 强制 speed = 0.8 m/s |
| L2 | ≥ 1.5 m | 强制 speed = 0.4 m/s |

安全层独立于反馈层，只降速不改转向。

### 控制器模式

由参数 `controller_mode` 控制：

| 模式 | 描述 |
|------|------|
| `legacy` | 纯航向 P + 预瞄 (v1.0 回退) |
| `stanley` | Stanley + PD + 双阻尼 + 自适应速度/转向 (v1.1) |
| `lqr` | Bézier 平滑 + LQR + Frenet(s,l) + 两级预警 (v1.2 / v1.3) |

### v1.3 新增/调整参数

```yaml
path_follower_node:
  ros__parameters:
    controller_mode: lqr          # legacy | stanley | lqr
    lqr_q_cte: 10.0               # LQR 横向偏差权重
    lqr_q_cte_dot: 1.0            # LQR 横向速度权重
    lqr_q_heading: 5.0            # LQR 航向偏差权重
    lqr_q_yaw_rate: 0.5           # LQR 横摆率权重
    lqr_r_steer: 5.0              # LQR 控制量惩罚
    lqr_fb_limit_deg: 3.0         # l 轴反馈 ±3°
```

## 已知问题

### Frenet 规划器的时间戳与时间同步

`frenet_planner_node.py` 采用"最新值快照"策略，存在多源数据未对齐、
坐标变换用回调时刻位姿而非采样时刻位姿、路径时间戳为发布时刻等问题。
详见历史版本的 README。

### 下坡超速

Gazebo 地形含小土坡，车辆下坡时重力加速可能超过指令速度。
v1.3 通过 Frenet(s,l) 的 s 轴反馈（检测超前）主动降速 ±0.5 m/s，
配合两级安全预警做双重保护，但仍为间接手段。未来可考虑直接订阅
GPS 速度反馈做超速检测。

## 输出

```text
runtime/scenario_<seed>/baja_100m.sdf
runtime/scenario_<seed>/dirt_road.obj
runtime/scenario_<seed>/scenario.json
results/seed_<seed>/tracking_YYYYMMDD_HHMMSS.csv
results/seed_<seed>/gazebo_YYYYMMDD_HHMMSS.mp4
```

## 测试

```bash
cd /mnt/data/baja_cloud_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover -s src/baja_cloud_sim/test -v
```
