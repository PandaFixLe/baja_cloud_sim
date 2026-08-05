# Baja 云端规划控制仿真

面向 **Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic** 的自动驾驶赛车规划-控制闭环仿真工程。
以 BAJA SAE 方程式越野车为对象，覆盖「场景生成 → 感知模拟 → Frenet 局部规划 → 横纵向控制 → 执行器 → 指标评价」全链路。

当前分支 **`v2.2`**：横向为 Apollo 式 **LQR + 前馈**主控，纵向为**级联 PID**。
在 294 m 闭环赛道上可稳定跑完整圈，平均中心线偏差 **0.08 m**，转向饱和率 **0%**。

> 本分支面向**实车（Orin）移植对接**：在 v2.1 终点减速停车的基础上，新增
> `mock_perception_node` 模拟感知组信息流，并加固了感知-控制接口（障碍分类、
> 安全状态机 clearance 守卫、特殊地面兜底），为接入真实感知包做准备。

---

## 目录

- [性能指标](#性能指标)
- [快速开始](#快速开始)
- [场景与车辆](#场景与车辆)
- [系统架构](#系统架构)
- [控制器设计](#控制器设计)
- [参数说明](#参数说明)
- [代码结构](#代码结构)
- [工具链](#工具链)
- [测试](#测试)
- [输出产物](#输出产物)
- [已知问题](#已知问题)
- [版本历史](#版本历史)

---

## 性能指标

Gazebo 实测，单次运行约 100 s。「改造前」指 v1.5 的「纯追踪基底 + LQR 残差」方案。

| 指标 | seed 0（无障碍）改造前 → 后 | seed 42（5 障碍）改造前 → 后 |
|------|------|------|
| 完成进度 | 186.5% → **294.0%** | 70% → **280.5%** |
| 中心线偏差（均值） | 0.75 → **0.08 m** | 1.13 → **0.25 m** |
| 中心线偏差（最大） | 4.74 → **0.56 m** | — |
| 转向标准差 | 28.0° → **3.3°** | 23° → **6.1°** |
| 转向饱和率 | 52.9% → **0.0%** | 57% → **0.1%** |
| 碰撞次数 | 2 → **0** | 4 → **1** |

> **关于 progress 的读法**：赛道实际总长 **294.25 m**，但 `scenario.json` 里的 `length`
> 字段仍是历史遗留的 `100.0`，评价器按 `progress = 100 × s / length` 计算。
> 因此 **`progress ≈ 294%` 等价于跑完整整一圈**，并非三圈。

seed 42 残留的 1 次碰撞为单采样点 8 cm 擦碰（clearance −0.083 m @ t=56.6 s）后立即恢复，
属规划器 clearance 余量问题，非控制稳定性问题。

---

## 快速开始

### 环境安装

若尚无 ROS 2 Humble / Gazebo Harmonic：

```bash
./install_ubuntu2204.sh
```

### 构建

```bash
sudo apt-get install -y ffmpeg
./build.sh
```

> `build.sh` 使用 `colcon build --symlink-install`，`install/` 下多为指向 `src/` 的符号链接。
> 因此**修改 `params.yaml`、`model.sdf` 等配置无需重新构建**，但**修改 Python 源码需要**
> （`build/` 下存在实际副本）。

### 运行

```bash
./run.sh --seed 42 --obstacles 5 --finish-mode circle   # 闭环赛道 + 终点减速停车
./run_line.sh --seed 42 --obstacles 5 --finish-mode line # 直线开放赛道 + 终点减速停车
./run.sh --seed 0  --obstacles 0                         # 空赛道，纯跟踪性能测试（无终点逻辑）
```

| 参数 | 说明 |
|------|------|
| `--seed N` | 场景随机种子（决定障碍布局），默认 42 |
| `--obstacles N` | 障碍物数量，默认 5 |
| `--finish-mode MODE` | 终点模式：`none`（默认，不停车）/ `line` / `circle` / `time` |
| `--headless-gazebo` | 关闭 Gazebo 图形窗口（仍录像） |
| `--no-rviz` | 关闭 RViz |
| `--no-video` | 关闭录像（调试时加速） |

> **两种场景脚本**：
> - `run.sh` 始终加载**闭环赛道**（`baja_loop.sdf`），无论 `--finish-mode` 传什么场景都是圈；
> - `run_line.sh` 加载**直线开放赛道**（`baja_line.sdf`），需配合 `--finish-mode line`。
> 二者不能混用——`run.sh --finish-mode line` 跑的仍是圈道，只是按直线终点逻辑处理。

**终点逻辑（`finish_mode`）**：
- `line`：在赛道末端前 `finish_runout_m`（默认 20 m）处画红色终点线，过线后按匀减速 profile 在停车区停住；
- `circle`：跑满 `finish_target_lap`（默认 1 圈）后，过红线（弧长 0 处）减速停车；
- `time`：运行 `finish_time_limit_s`（默认 1200 s）后停止，**不画终点线**；
- `none`：一直跑，不停车（用于纯控制性能测试）。

RViz 中会以红色半透明 `LINE_STRIP` 显示终点线（`/finish_line_marker` 话题，`time` 模式除外）。

无桌面环境下的典型用法：

```bash
./run.sh --seed 42 --obstacles 5 --finish-mode circle --headless-gazebo --no-rviz
```

运行结束后自动调用 `tools/plot_tracking.py` 生成轨迹图。

### 无障碍基准测试

`run_test.sh` 是专用的空赛道脚本，用于测纯跟踪性能：

```bash
./run_test.sh --headless-gazebo --no-rviz
```

与 `run.sh` 的区别：固定 `seed=0` / `obstacles=0`（无 `--obstacles` 参数）、
使用独立的 `GZ_PARTITION`、结果写入 `results/flat_seed_0/`。
因此**可与 `run.sh` 并发运行**而互不干扰。

---

## 场景与车辆

### 赛道

矩形闭环，由原始 L-track 扩展闭合而成，**总长 294.25 m**：

```
        S3 (西, 50 m)
   ┌─────────────────┐
T3 │                 │ T2      四个转弯半径均为 15 m
   │                 │         (T1~T4 均为 90° 左转)
S4 │                 │ S2      起点 (0,0) 朝东
   │                 │
   └─────────────────┘
        S1 (东, 50 m)
      ▲ 起点
```

- 中心线按弧长每 **0.5 m** 采样，共 **589** 点；
- 地形起伏**仅存在于原始 L-track 段**（S1/T1/S2，即 s ≈ 0~124 m）：
  - 平滑土坡：s = 16~30 m（高 0.55 m）、s = 76~89 m（高 0.45 m）
  - 圆弧减速带：s = 39 m（宽 1.20 m / 高 0.08 m）、s = 92 m（宽 1.00 m / 高 0.07 m）
- 障碍物包围盒按 seed 可复现生成。
- **道路边缘轮胎（edge_tires）**：沿中心线在道路两侧边界（`half_width + 0.4 m` 外侧）
  每隔 **5 m** 弧长放置一个竖直黑色圆柱（半径 0.34 m、高 0.7 m），模拟真实越野赛道
  用立放轮胎标定的赛道边界。轮胎写入 `scenario.json` 的 `edge_tires` 字段，并在
  `/obstacle_markers` 下以 **`ns=tire`** 发布——**仅供感知组识别与对齐真实赛道
  landmark，不参与路径规划/避障，也不绘制膨胀框**。

### 车辆

| 项目 | 数值 |
|------|------|
| 质量 | 292 kg |
| 驱动 | 后驱 |
| 转向 | 前轮 Ackermann |
| 轴距 | 1.43 m |
| 最大转向角 | 35°（0.6109 rad） |
| 最大速度 | 5.0 m/s |

真值位姿来自 Gazebo `/ground_truth/odom`；感知话题由 `truth_perception` 叠加
**1.5 cm 标准差、3σ 有界**的位置噪声派生。

> 该模型为基础刚体接触动力学，**不含**悬架连杆、轮胎形变、电机转矩曲线、制动液压
> 与可变土壤沉陷，不能视为经实车标定的完整整车动力学模型。

---

## 系统架构

```text
Gazebo OdometryPublisher（世界真值位姿）
  └─ /ground_truth/odom
       ├─ truth_perception ──┬─ /localization/odom
       │                     ├─ /gps/fix
       │                     ├─ /imu/yaw
       │                     ├─ /reference_centerline
       │                     ├─ /road_boundary_markers
       │                     └─ /obstacle_markers  (ns=tall | flat_ground | tire)
       └─ evaluator ─────────── results/seed_N/tracking_*.csv

frenet_planner (10 Hz)
  ├─ /planned_path      (nav_msgs/Path, 绿色)
  └─ /planner/status    (FEASIBLE | INFEASIBLE)
       │
       ▼
path_follower (20 Hz, dt=0.05 s)
  ├─ 速度剖面   曲率上限 → 前向加速约束 → 后向减速约束
  │             → clearance/terrain 降速 → 后向可行性再传播
  ├─ 横向       LQR + 前馈（主）/ 纯追踪（fallback）
  ├─ 纵向       级联 PID + 坡度补偿 + 速率限制
  ├─ 安全       3 级状态机 NORMAL / SLOWDOWN / EMERGENCY
  └─ /cmd_control (AckermannDriveStamped)
       │
       ▼
actuator_adapter
  └─ /model/baja_vehicle/cmd_vel (Twist, angular.z = v·tan δ / L)
       │
       ▼
Gazebo AckermannSteering 插件

──── 实车对接准备 ────

mock_perception（调试/对接用，非运行必需）
  ├─ /road_boundary_markers  (base_link, LINE_STRIP, ns=road_left/road_right)
  └─ /obstacle_markers       (base_link, CUBE, ns=tall | flat_ground)
       │   ↑ 与 truth_perception 同话题契约，可无缝替换
       ▼
  算法核心层（frenet_planner / path_follower）  ← 平台无关，实车不改
```

> **平台边界**：`truth_perception`（仿真，读 Gazebo 真值）与未来的真实
> 定位/感知节点，以及 `actuator_adapter`（仿真，发 Gazebo cmd_vel）与未来的
> 底盘驱动节点，是**唯一**需要随平台替换的两端；中间规划-控制核心不变。
> `mock_perception` 用于在没有 Gazebo / 真实感知时验证核心节点能正确接收
> 约定格式的消息（详见[感知对接与模拟节点](#感知对接与模拟节点)）。

### RViz 可视化

`simulation.rviz` 中已配置的显示项：

| Topic | 类型 | 含义 |
|-------|------|------|
| `/reference_centerline` | Path | 赛道中心线（真值参考） |
| `/planned_path` | Path | 规划器当前输出路径 |
| `/actual_path` | Path | 车辆实际行驶轨迹 |
| `/lookahead_point` | PointStamped | 当前控制参考点 |
| `/road_boundary_markers` | MarkerArray | 赛道左右边界 |
| `/obstacle_markers` | MarkerArray | 障碍物包围盒 |
| `/planning_debug` | MarkerArray | 规划器候选轨迹调试 |
| `/simulation/metrics` | MarkerArray | 实时指标文字叠加 |
| `/finish_line_marker` | Marker (LINE_STRIP) | 红色半透明终点线（`time` 模式不发布） |

---

## 控制器设计

### 横向 — Apollo 式 LQR + 前馈（主控）

控制律为 **`δ = δ_ff + δ_fb`**，实现于
[`core/controller.py`](src/baja_cloud_sim/baja_cloud_sim/core/controller.py) 的
`compute_lqr_steering()`：

```
δ_ff  = 曲率前馈，始终生效
δ_fb  = −K·x，K 由 DARE 在线求解
x     = [∫e_y, e_y, ė_y, e_ψ, ė_ψ]        （5 状态增广 LQI）
```

- **5 状态增广 LQI**：积分项消除稳态横向偏移；转向饱和时冻结积分累加（anti-windup），
  `∫e_y` 限幅 ±0.5。
- **增益求解**：`scipy` Van-Loan / ZOH 离散化 + `solve_discrete_are` 在线解 DARE，
  按速度调度。
- **无残差限幅**：LQR 输出即最终指令，仅在节点层施加执行器物理限位。
- **参考点选取**：按弧长投影 + 固定 `s_proj_lookahead`（0.8 m），
  而非纯追踪的预瞄点。
- **曲率计算**：三点中心差分（node-centred），避免前向差分带来的半个网格偏置。
- **fallback**：仅当 LQR 不可用时（`enable_lqr=false`、车速低于 `lqr_min_velocity`、
  或参考曲率缺失）退回纯追踪。
- **速率限制**：4°/周期（≈80°/s）；航向误差 >5° 时放宽至 8°（≈160°/s）快速回正。

### 纵向 — 级联 PID

```
v_ref  ← 曲率速度剖面
e_v    = v_ref − v_actual
a_cmd  = Kp·e_v + Ki·∫e_v + Kd·ė_v  +  9.81·sin(θ_road)·slope_comp_gain
v_tgt  = v_actual + a_cmd·dt  →  加速度/jerk 速率限制  →  输出
```

- 坡度补偿由参考中心线的 z 梯度前馈，抵消上下坡重力分量；
- 保留 `idle_speed` 怠速下限，避免低速卡死；
- 加减速率分别限制为 ≈4.0 / 5.0 m/s²。

**这解决了直线速度振荡（P1）**：旧方案的速度上限直接由 `|cross_track|` 与 `|heading|`
硬阈值触发且无迟滞，厘米级的横向误差就会激起「减速—回正—加速—超调」的极限环。
现在横向误差不再直接进入速度环，仅通过状态机间接影响。

### 安全 — 3 级状态机

| 状态 | 触发 | 动作 |
|------|------|------|
| `NORMAL` | 默认 | 无干预 |
| `SLOWDOWN` | 连续不可行帧 / 跟踪误差偏大 | 速度上限 1.0 m/s |
| `EMERGENCY` | 持续不可行 / 误差严重超限 | 速度归零，转向归零 |

带迟滞切换。另有 **freewheel** 机制：规划器报 `INFEASIBLE` 时，沿用最后有效路径
最多 6 帧（≈300 ms）再停车，避免瞬时抖动导致急停。

`|track_err| > 0.5 m` 时强制保底 2.0 m/s，以保留转向权限（低速时 Ackermann
转向对轨迹的修正能力急剧下降）。

### 关键修复：执行器滞后

Gazebo `AckermannSteering` 插件将 `<min/max_acceleration>` 与 `<min/max_jerk>`
**同时**作用于线速度限幅器**和**转向角限幅器。为纵向真实感设定的数值
（2.4 m/s² / 6.0 m/s³）因此把转向速率压到 ≈1.1 rad/s：

| | 修复前 | 修复后 |
|---|---|---|
| 转向机构时间常数 t63 | 0.567 s | **0.075 s** |
| 闭环互相关滞后 | ~1020 ms | **110 ms** |

约 1 秒的横向回路延迟足以让任何高增益控制器失稳——**这正是此前「LQR 反而不如纯追踪」
的根因**（纯追踪增益低，能容忍该延迟）。

修复方式：`model.sdf` 中放宽至 ±50（accel）/ ±500（jerk）。纵向的加减速与 jerk 限制
已在上游的 `actuator_adapter_node` 与 `path_follower_node` 中实现，因此放开插件限幅
不影响纵向真实感（实测纵向加速度 p99 前后均为 ≈3.8~4.0 m/s²）。

> 已验证**无效**并已回退的尝试：`steer_p_gain` 18→200、转向关节 `damping` 7→0.5、
> `velocity` 2.5→10、`friction` 0.6→0.05。

---

## 参数说明

配置文件：[`src/baja_cloud_sim/config/params.yaml`](src/baja_cloud_sim/config/params.yaml)

### 横向控制

```yaml
path_follower_node:
  ros__parameters:
    enable_lqr: true
    lqr_Q: [0.05, 8.0, 2.0, 4.0, 0.5]  # [∫e_y, e_y, ė_y, e_ψ, ė_ψ]
    lqr_R: 3.0                          # 控制量惩罚（越大转向越柔和）
    lqr_v_norm: 1.5                     # 增益调度归一化速度
    s_proj_lookahead: 0.8               # 参考点弧长前视 (m)
    max_steering_angle: 35.0
```

### 纵向控制

```yaml
    lon_kp: 2.0
    lon_ki: 0.5
    lon_kd: 0.2
    lon_i_limit: 2.0        # 积分限幅
    idle_speed: 1.0         # 怠速下限 (m/s)
    slope_comp_gain: 1.0    # 坡度重力补偿增益
    lon_accel_step: 0.20    # m/s per 50 ms ≈ 4.0 m/s²
    lon_decel_step: 0.25    # ≈ 5.0 m/s²
```

### 速度剖面与安全

```yaml
    use_speed_profile: true
    speed_profile_max: 3.5
    max_lateral_accel: 1.8        # 曲率速度上限依据
    desired_clearance: 3.5        # 障碍降速起始距离
    min_speed_obstacle: 2.0
    terrain_slope_threshold: 0.06
    terrain_min_speed: 1.0
```

### 感知接口与安全

```yaml
path_follower_node:
  ros__parameters:
    use_terrain_profile: false     # 实车对接感知后统一由 flat_ground 处理地形降速
    obstacle_classes:
      tall:
        lateral_avoid: true        # 高障碍：横向避让 + 按 clearance 减速
        desired_clearance: 1.5
        min_speed: 1.5
      flat_ground:
        lateral_avoid: false       # 特殊地面：直线通过 + 平滑降速，不横向避让
        approach_distance: 5.0     # 提前/过后对称平滑过渡距离 (m)
        slow_speed: 2.0            # 通过特殊地面时的目标速度 (m/s)
        default_half_width: 0.9    # length 缺失时特殊地面半宽兜底
```

- 障碍类型由 `MarkerArray` 的 `ns` 字段区分：**`tall`** → 高障碍（参与 Frenet 横向
  走廊避让）、**`flat_ground`** → 特殊地面（仅纵向降速，直线通过）、
  **`tire`** → 道路边缘立放轮胎（仿真侧 `truth_perception` 按 5 m 间隔发布，
  仅供感知组识别真实赛道 landmark，**不参与避障、不画膨胀框**）。
  **`ns` 缺失或非上述值 → 忽略**（不静默默认，避免误分类）。
- 安全状态机现已订阅 `/metrics/planned_clearance`：corridor 余量 < 5 cm 触发
  `EMERGENCY`、< 30 cm 触发 `SLOWDOWN`，此前该守卫因 clearance 未接入而恒不触发。

---

## 感知对接与模拟节点

实车移植时，规划-控制核心**不改动**，只需把两端平台节点替换为真实传感器/底盘驱动，
并使它们的话题与下方约定对齐。为在真实感知包到达前验证算法核心能正确消费消息，
仓库内置 `mock_perception_node`：

### 话题契约（感知组 → 算法核心）

| 话题 | 帧 | 类型 / `ns` | 字段约定（★=必需，其余可选） |
|------|------|------------|---------|
| `/road_boundary_markers` | `base_link` | `LINE_STRIP`，`ns=road_left` / `road_right` | `points[]` 为相对车身坐标，前向 30–50 m、20–40 点 |
| `/obstacle_markers` | `base_link` | `CUBE`，`ns=tall` | ★`pose.position`=相对车身坐标；★`ns=tall`；★`scale.x`=沿车身前向长度（走廊膨胀用）；`scale.y`=宽度（**不传则用 `obstacle_classes.flat_ground.default_half_width` 兜底**）；`scale.z`=高度（**算法不用，仅 RViz 显示，可不传**）；`pose.orientation`=相对偏航 |
| `/obstacle_markers` | `base_link` | `CUBE`，`ns=flat_ground` | ★`pose.position`=相对车身坐标；★`ns=flat_ground`；★`scale.x`=特殊地面沿车身前向长度（降速过渡区用）；`scale.y/z` 同上为可选/不用 |
| `/obstacle_markers` | `base_link` | `CUBE`，`ns=tire` | 道路边缘立放轮胎（仿真 `truth_perception` 按 5 m 间隔发布）。★`pose.position`=相对车身坐标；★`ns=tire`；`scale.x/y/z`≈轮胎尺寸。算法核心（planner / path_follower）**忽略**此类 marker，仅作感知可视化与对齐，不触发避障、不绘制膨胀框 |

> **最小契约**：中心点坐标（`pose.position.x/y`）+ 类型（`ns`）+ 前向长度（`scale.x`）。
> 宽度与高度对算法核心非必需——宽度由兜底参数处理，高度仅用于 RViz 盒子显示。
> 仿真侧 `truth_perception` 已显式给随机障碍标 `ns="tall"`、给道路边缘轮胎标
> `ns="tire"`，故现有仿真测试不受影响，且 planner 的红色膨胀框只对 `tall` 障碍绘制、
> 不会覆盖轮胎。

### 使用模拟节点

```bash
# 单独运行（任意 ROS 2 环境，无需 Gazebo）
ros2 run baja_cloud_sim mock_perception

# 一键联调：算法核心 + mock 感知 + RViz（无 Gazebo，仅验证话题接线）
ros2 launch baja_cloud_sim mock_perception.launch.py
ros2 topic echo /obstacle_markers | grep ns      # 应见 tall / flat_ground
ros2 topic echo /road_boundary_markers | grep ns # 应见 road_left / road_right
```

> 该 launch 没有真实里程计，核心节点拿不到车辆位姿，**仅用于检查话题接线与
> RViz 显示**，不能跑实际行驶。参数 `road_half_width` / `road_forward` /
> `road_backward` / `publish_rate_hz` / `boundary_topic` / `obstacle_topic`
> 可在 `ros2 run` 时覆盖以适配对接场景。

### 终点逻辑（finish）

```yaml
    finish_mode: "none"            # none | line | circle | time
    finish_runout_m: 20.0          # 冲过终点线后再 20 m 作为停车区
    finish_decel: 1.5              # 终点减速停车减速度 (m/s²)
    finish_s: -1.0                 # 红线弧长位置；-1 = 自动
                                   #   line  → 总长 - finish_runout_m
                                   #   circle → 0（闭环 seam 处）
    finish_target_lap: 1           # circle 专用：跑满几圈后才允许停车
    finish_time_limit_s: 1200.0    # time 专用：超时停止（秒）
    finish_max_duration_s: 600.0   # 全局安全看门狗：防止终点触发失效而无限运行
```

过红线后 `path_follower` 进入"braking_to_stop"状态，豁免安全状态机与离道限速，
改由 `finish_decel` 驱动的匀减速 profile 平滑降速，在 `finish_s + finish_runout_m` 处停住。

### 规划器

```yaml
frenet_planner_node:
  ros__parameters:
    horizon_m: 30.0
    center_weight: 1.0
    clearance_weight: 12.0
    desired_clearance: 1.2
    vehicle_length: 3.0
    vehicle_width: 1.5
```

### 调参建议

| 现象 | 调整方向 |
|------|---------|
| 转向抖动 / 过于敏感 | 增大 `lqr_R`，或减小 `lqr_Q[1]`（e_y 权重） |
| 贴线不够紧 | 增大 `lqr_Q[1]`，或增大 `lqr_Q[0]`（积分项） |
| 弯道切内侧 | 增大 `s_proj_lookahead` |
| 弯道外抛 | 减小 `s_proj_lookahead`，或减小 `max_lateral_accel` |
| 直线速度波动 | 检查 `lon_kd` 是否过大；`lon_ki` 过大会积分超调 |
| 上坡掉速 | 增大 `slope_comp_gain` |

---

## 代码结构

算法包位于 `src/baja_cloud_sim/baja_cloud_sim/`。

### 活跃代码

```text
core/                      ← 当前生效的算法包
├── __init__.py              统一 re-export，保证 from .core import X 兼容
├── geometry.py              坐标变换、GPS↔局部、signed_lateral 等几何工具
├── track.py                 矩形闭环中心线生成（294.25 m）
├── terrain.py               地形高程：土坡 + 减速带
├── planner.py               Frenet 栅格 DP 规划核心
├── controller.py            ★ LQR/LQI 控制器 + compute_lqr_steering（主控入口）
├── speed_profile.py         曲率感知速度剖面（前向加速 + 后向减速双通）
├── collision.py             碰撞与 clearance 检测
└── config.py                控制器配置数据类（LQRConfig 等）

scenario_generator.py      按 seed 生成 SDF 世界 + OBJ 路面 + scenario.json
truth_perception_node.py   由真值里程计派生带噪声的定位/GPS/IMU/中心线/边界/障碍（仿真用）
mock_perception_node.py    ★ 模拟感知组信息流（实车对接调试用，非运行必需）
frenet_planner_node.py     10 Hz 规划，输出 /planned_path 与 /planner/status
path_follower_node.py      ★ 核心控制器（横向 LQR + 纵向 PID + 状态机）
actuator_adapter_node.py   /cmd_control → /model/baja_vehicle/cmd_vel（仿真用）
evaluator_node.py          指标计算与 CSV 记录
video_recorder_node.py     ffmpeg 录制 Gazebo 相机
```

> **实车移植角色**：`truth_perception_node` 与 `actuator_adapter_node` 是
> 平台相关两端（仿真读 Gazebo 真值 / 发 Gazebo cmd_vel），实车需替换为真实
> 定位感知节点与底盘驱动节点；其话题均已参数化（`ground_truth_odom_topic`、
> `cmd_vel_topic`、`odom_topic`）。`mock_perception_node` 用于在**无 Gazebo**
> 环境下按约定契约发感知消息，验证算法核心接收链路。

### 遗留代码（不影响运行）

以下文件为历史版本残留，**当前链路不会加载**，修改它们不会产生任何效果：

| 文件 | 说明 |
|------|------|
| `core.py`（顶层，1077 行） | v1.0 单体核心。与 `core/` 目录同名，Python 的包优先级使 `from .core import ...` **始终解析到 `core/` 目录**，此文件从未被导入 |
| `lqr_controller.py` | v1.2/v1.3 的 LQR 实现（含离线增益表）。当前使用的是 `core/controller.py` |
| `trajectory_smoother.py` | Bézier 平滑 + TrajectoryTable，仅被上面的 `lqr_controller.py` 依赖 |

> 修改控制算法请定位到 **`core/controller.py`** 与 **`path_follower_node.py`**。

---

## 工具链

```text
tools/plot_tracking.py        tracking CSV → 2D 轨迹时间热力图 + 速度/转向时域图
                              （run.sh / run_test.sh 结束后自动调用）
tools/offline_closed_loop.py  离线自行车模型闭环烟测，支持 --exit-code 作 CI 门禁
tools/steering_probe.py       记录 /cmd_control 指令转向角 vs /joint_states 实际前轮角
                              + 速度 + 横摆率，用于执行器辨识
```

### 离线烟测

```bash
python3 tools/offline_closed_loop.py --seed 7 --exit-code
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `--seed N` | 42 | 场景种子 |
| `--obstacles N` | 5 | 障碍数量 |
| `--scenario PATH` | 程序生成 | 指定 scenario JSON |
| `--controller` | `pure_pursuit` | 可选 `pure_pursuit` / `lqr_apollo` |
| `--speed V` | 2.5 | 目标速度 (m/s) |
| `--speed-profile` | 关 | 启用曲率速度剖面 |
| `--plot-dir DIR` | 无 | 输出 matplotlib 图 |
| `--exit-code` | 关 | 指标不达标时返回非零退出码（CI 门禁） |

输出含转向平滑度指标：

```
seed=7 progress=100/100 m collisions=0 center_error=0.11 m (mean 0.04)
  final_speed=2.9 m/s steer[std=6.4° sat=0.0% rate=12.3°/s] PASS
```

> ⚠️ **该 harness 使用理想自行车模型，不含执行器动态。** 在排查上述执行器滞后问题时，
> 它显示饱和率 0~1%、转向标准差 6.4°，而实车同期为 53% 和 28°。
> **它是控制器烟测，不能作为稳定性判据**——稳定性必须在 Gazebo 中验证。

### 执行器辨识

先启动仿真，再另开终端运行（输出路径为位置参数，缺省 `/tmp/steering_probe.csv`）：

```bash
python3 tools/steering_probe.py /tmp/probe.csv
```

生成的 CSV 字段：`t, cmd_steer_rad, left_joint_rad, right_joint_rad,
avg_joint_rad, speed_mps, yaw_rate_rps`。对比 `cmd_steer_rad` 与 `avg_joint_rad`
即可做阶跃响应或互相关滞后分析。

---

## 测试

```bash
cd /home/pandafixle/Desktop/baja_cloud_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/baja_cloud_sim python3 -m pytest src/baja_cloud_sim/test -q
```

当前状态：**30 通过 / 2 失败**。

两个失败项为**历史遗留断言**，与控制器改动无关：

| 失败用例 | 原因 |
|---------|------|
| `test_centerline_spacing_and_length` | 断言 201 点，当前闭环赛道为 589 点 |
| `test_centerline_is_straight_then_turns_ninety_degrees` | 断言旧 L-track 形状，当前为矩形闭环 |

---

## 输出产物

```text
runtime/scenario_<seed>/baja_100m.sdf      生成的 Gazebo 世界
runtime/scenario_<seed>/dirt_road.obj      路面网格
runtime/scenario_<seed>/scenario.json      中心线 / 边界 / 障碍 / 边缘轮胎定义
results/seed_<seed>/tracking_*.csv         逐帧指标
results/seed_<seed>/gazebo_*.mp4           Gazebo 相机录像
results/seed_<seed>/photos/*.png           轨迹图与时域图
```

`tracking_*.csv` 字段：

```
time_s, x, y, yaw_rad, speed_mps, command_speed_mps, steering_rad,
tracking_error_m, center_error_m, minimum_clearance_m,
planning_ms, planner_status, collision_count, progress_percent
```

---

## 已知问题

### 1. 规划器 clearance 余量偏小

seed 42 场景下仍会出现单采样点的轻微擦碰（约 8 cm）。控制器能立即恢复，
但根因在于 Frenet DP 的 `clearance_weight` / `desired_clearance` 组合在
狭窄通过段留的余量不足。可尝试提高 `frenet_planner_node.desired_clearance`。

### 2. 两个单元测试断言旧地图

见[测试](#测试)章节。需将期望值更新为当前的 589 点矩形闭环。

### 3. 规划器的时间戳与时间同步

`frenet_planner_node.py` 采用「最新值快照」策略：多源数据未做时间对齐、
坐标变换使用回调时刻位姿而非采样时刻位姿、路径时间戳为发布时刻而非采样时刻。
在当前 10 Hz / 低速场景下影响有限，高速场景需重构。

### 4. 文档与历史分支的漂移

`docs/` 下的部分文档描述的是 v1.0~v1.3 架构（TrajectorySmoother、
Frenet(s,l) 双轴反馈、两级安全预警、`controller_mode` 三模式切换等），
**这些机制在当前分支已不存在**。请以源码为准。

### 5. `progress` 指标的分母是历史遗留值

`scenario.json` 的 `length` 字段固定为 100.0，而赛道实际长 294.25 m。
详见[性能指标](#性能指标)的说明。

---

## 版本历史

| 分支 | 版本 | 横向控制 | 说明 |
|------|------|---------|------|
| `main` | v1.0 | Legacy | 纯航向 P + 预瞄 |
| `Baja-resource-origin-7.23-v1.1` | v1.1 | Stanley | Stanley + PD + 双阻尼 + 自适应速度/转向 |
| `v1.2-LQR` | v1.2 | LQR | Bézier 平滑 + LQR 切片预测 |
| `v1.3-Smoother` | v1.3 | LQR | 时间对齐查表 + Frenet(s,l) 双轴反馈 + 两级安全预警 |
| `v1.4` / `v1.4.1` | v1.4 | LQR | 过渡版本 |
| `v1.5` | v1.5 | PP + LQR 残差 | 纯追踪基底 + LQR 残差（±3° 反馈上限）+ 曲率速度剖面 + 3 级状态机 |
| **`v2`** | **v2** | **LQR + 前馈** | **Apollo 式主控 + 级联 PID 纵向 + 执行器滞后修复** |
| **`v2.1`** | **v2.1** | **LQR + 前馈** | **新增终点减速停车逻辑 + 红色终点线 RViz Marker** |
| **`v2.2`** | **v2.2** | **LQR + 前馈** | **实车对接准备：`mock_perception` 模拟感知 + 感知接口加固** |

### v2.1 相对 v2 的变更

1. **终点识别与减速停车**——新增 `finish_mode`（`line` / `circle` / `time` / `none`）：
   - 终点位置按赛道固定（可查表），过红线后进入匀减速 profile，在 `finish_s + finish_runout_m` 处平滑停住；
   - 停车阶段豁免安全状态机与离道限速，避免急刹或绕圈不止；
   - `line` 用满 20 m 缓冲；`circle` 跑满 `finish_target_lap` 圈后过红线停车；
   - `time` 模式按 `finish_time_limit_s` 超时停止，不画终点线。
2. **红色终点线 Marker**——`path_follower` 在 `finish_s` 处发布 `/finish_line_marker`
   （红色半透明 `LINE_STRIP`，`TRANSIENT_LOCAL` 持久化），RViz 中可视化终点位置。
3. **`.gitignore` 清理**——`build/`、`install/`、`log/`、`results/`、`runtime/` 不再入库，
   zip 体积大幅减小，且不再携带绝对路径符号链接；使用者解压后执行
   `./install_ubuntu2204.sh` 即可重新构建。

### v2.2 相对 v2.1 的变更

面向**实车（Orin）移植对接**的接口准备（算法核心层不变）：

1. **模拟感知组信息流节点**——新增 `mock_perception_node`（入口 `mock_perception`）
   与 `mock_perception.launch.py`。无 Gazebo 依赖，按约定契约发
   `/road_boundary_markers`（`base_link` `LINE_STRIP`，`ns=road_left/road_right`）
   与 `/obstacle_markers`（`base_link` `CUBE`，`ns=tall` / `flat_ground`），
   用于在真实感知包到达前验证算法核心的接收链路。
2. **感知-控制接口加固**：
   - `path_follower` 删除 `_obstacles` 重复赋值；
   - 安全状态机现订阅 `/metrics/planned_clearance`，clearance 守卫
     （`EMERGENCY` / `SLOWDOWN`）真正生效，此前因 clearance 未接入而恒不触发；
   - `flat_ground` 的 `default_half_width` 参数在感知未给 `length` 时兜底，
     避免特殊地面退化成点导致降速逻辑失效。
3. **障碍分类契约落地**（延续 v2.1 设计）：`ns=tall`→横向避让、`ns=flat_ground`→
   纵向降速直线通过、`ns` 缺失/其他→忽略；仿真侧 `truth_perception` 已显式标
   `ns="tall"`，故现有仿真测试零回归。
4. **仿真地图：道路边缘轮胎（edge_tires）**——`scenario_generator` 沿中心线两侧边界
   每隔 **5 m** 弧长放置竖直黑色圆柱（半径 0.34 m、高 0.7 m），写入 `scenario.json`
   的 `edge_tires`，并在 `/obstacle_markers` 下以 **`ns=tire`** 发布。轮胎对齐真实
   越野赛道用立放轮胎标定的赛道边界，专供**感知组识别 / 与真实赛道 landmark 对齐**，
   **不参与路径规划与避障**（planner 的红色膨胀框只对 `tall` 障碍绘制，不覆盖轮胎）。
5. **膨胀框范围修正**——`frenet_planner` 的 RViz 红色半透明 `inflated_obstacles` 膨胀框
   仅对 `tall` 正常障碍绘制；道路边缘轮胎因发在独立 `ns=tire` 下，既不进 planner 也不
   画膨胀框，避免干扰避障逻辑的观感判断。

### v2 相对 v1.5 的变更

1. **执行器滞后修复**（`model.sdf`）——放宽 `AckermannSteering` 的 accel/jerk 限幅，
   消除约 1 s 的转向回路延迟。
2. **横向改为 LQR 主控**——移除 ±3° 残差限幅与 smoothstep 增益带；
   修正 B 矩阵与增广 A 的不一致；`estimate_lqr_state` 全程改用世界系
   （里程计 twist 为车体系，先按 yaw 旋转）；参考点改为弧长投影；
   曲率改为三点中心差分。
3. **纵向改为级联 PID**——消除由横向误差硬阈值引发的直线速度极限环。
4. **工具链增强**——新增 `steering_probe.py`；离线 harness 增加转向平滑度指标。

---

## 文档索引

| 文档 | 用途 |
|------|------|
| [`docs/frenet_planner.md`](docs/frenet_planner.md) | Frenet 局部规划器的坐标基础、Apollo EM Planner 关联与运行时结构 |
| [`docs/releases/v1.0.md`](docs/releases/v1.0.md) | v1.0 基线发布说明 |
| [`docs/releases/v1.1.md`](docs/releases/v1.1.md) | v1.1 发布说明 |
| [`docs/planning/background-pan-code.md`](docs/planning/background-pan-code.md) | 项目背景与架构演进 |
| [`docs/planning/v1.1-backlog.md`](docs/planning/v1.1-backlog.md) | v1.1 待办清单 |
| [`docs/planning/post-v1.1-backlog.md`](docs/planning/post-v1.1-backlog.md) | v1.1 后的结构性改进项 |
| [`docs/planning/baja-simulation-realism-enhancement.md`](docs/planning/baja-simulation-realism-enhancement.md) | 仿真真实度增强方案 |

## 许可证

见 [LICENSE](LICENSE)。
