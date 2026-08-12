# Baja 云端规划控制仿真

面向 **Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic** 的自动驾驶赛车规划-控制闭环仿真工程。
以 BAJA SAE 方程式越野车为对象，覆盖「场景生成 → 感知模拟 → Frenet 局部规划 → 横纵向控制 → 执行器 → 指标评价」全链路。

当前分支 **`v2.7-test`**：横向为 Apollo 式 **LQR + 前馈**主控（含速度自适应反馈软化
与**渐进恢复门**），纵向为**实车开环**（仅发期望速度设定值，速度闭环交由电机控制器执行；
仿真期保留 idle 怠速保底）。在 294 m 闭环赛道上可稳定跑完整圈，平均中心线偏差 **0.08 m**，
转向饱和率 **0%**，弯道无蛇形振荡、直道稳态高频抖动被有效抑制；起步 1.5 m 横向 + 15° 偏航
大偏离可在约 2 s 内收敛贴线（center 1.5 → 0.005），且恢复"聪明"不靠运气。

> **v2.7-test 关键改进（相对 v2.7）**：
> 1. **起步大偏离恢复能力测试与调参（实验分支）**——在 v2.7 主线基础上通过
>   `scenario_generator` 方案 A 注入通用起始偏移（y +1.5 m 横向 + 初始偏航 +15° 朝外），
>   专门验证"大偏离恢复"能力，并据此调参（见下方参数与[版本历史](#版本历史)）。
> 2. **航向门（P0）双因子限速**——起步/突发大偏离时若直接按 4 m/s 巡航会"加速冲过轨迹"
>   导致蛇形。新增双因子门：`|e_ψ| > 8°` **且** `|e_y| > 0.5 m` 才钳到 `idle_speed(0.6)`
>   蠕行；弯道里车已在线上（|e_y| 通常 < 0.5 m）不会被误钳。
> 3. **渐进恢复门（P0.5）**——仅在"恢复模式"下触发：当 `|e_y|` 曾超过 0.6 m（触发过 P0
>   航向门）进入恢复模式，按 `|e_y|` 渐进抬高限速上限（`recovery_ceiling` 随贴线程度从
>   idle 线性升到 v_ref），并设 1.0 s 贴线保持计时器 + 7.5 s 超时强制退出。弯道正常跟踪
>   （|e_y| 一直 < 0.6 m，从未触发 P0）不进入恢复模式、不受限速——彻底根治此前"运气型
>   蛇形"与"弯道误触恢复模式卡 0.6 m/s"。
> 4. **弯道提前减速增强**——`horizon_m` 30→**40**（让速度剖面看到更远的弯道，使
>   `pre_decel` 真正生效）、`curvature_lookahead_m` 6→**8**、`pre_decel_lookahead_m`
>   18→**20**（配合 horizon 不超过其 50%）、`pre_decel_max` 1.0→**1.5**。实测进弯前
>   3.5→2.2 m/s 平滑降速，弯道 v_mean≈1.86 m/s、不冲出赛道。
> 5. **横向阻尼中提 + 反馈软化**——`lqr_Q` 由 v2.7 的 `[3.0,4.0,3.0,4.0]` 改为
>   `[0.05,3.0,6.0,4.0,5.0]`（增 `ė_y`/`ė_ψ` 阻尼，减饱和、让航向先稳）；`fb_speed_soften_alpha`
>   0.3→**0.4**、`fb_speed_ref` 2.5→**1.5**（1.5 m/s 以上才软化）、`fb_speed_beta_min` 0.6→**0.5**。
> 6. **其他调参**——`s_proj_lookahead` 0.8→**1.5**（参考点弧长前视，缓解跳变）、
>   `max_steering_angle` 35→**26**（物理钳位建模）、`steer_lowpass_alpha` 0.9→**1.0**、
>   `state_lowpass_alpha` 0.45→**0.75**、`e_psi_ma_window` 5→**0**（关移动平均去滞后）、
>   `idle_speed` 1.0→**0.6**、`tier_normal_speed` 4.0→**3.5**（降贴线后过冲动能）、
>   `max_lateral_accel` 0.7→**0.8**（弯道不再过慢，且触发恢复模式阈值放宽后不再误触）。
>   EPS 物理约束（15°/s 齿条转速上限）未改动。

> 本分支完成**仿真层与实车层的融合**（同 v2.6/v2.7）：规划-控制-感知核心
> （`baja_cloud_sim`）不动，供应商硬件驱动包整体搬入 `src/hardware/`，实车与仿真通过
> **launch 话题 remap** 对齐（如 `/chcnav/devpvt → /gps/fix`），核心节点代码零改动。
> 实车运行入口为 `run_real.sh`（自主）/ `run_real_remote.sh`（视驾）/ `run_record.sh`（录制路径）。
> 供应商原 `ros2_ws/` 已删除。

> **v2.7 关键改进（相对 v2.6）**：
> 1. **`use_boundary` 开关真正生效**——此前该 yaml 开关在 `frenet_planner_node` 侧未被读取、
>    `simulation.launch.py` 又硬编码默认 `true`，导致"默认开车道线检测"。现节点侧补上参数
>    声明/读取/回调判断，`simulation.launch.py` 默认值改从 `params.yaml` 读取，与
>    `real_car.launch.py` 对齐。当前 `params.yaml` 默认 `use_boundary=false` + `use_obstacle=false`
>    （纯跟踪、Frenet 走中心线 ±half_width 兜底走廊）。
> 2. **弯道稳定性再调优**——进弯提前减速更充分、弯道中道中稳定低速、出弯后平稳慢加速：
>    `max_lateral_accel` 0.8→**0.7**、`pre_decel_lookahead_m` 8→**18**、`pre_decel_max` 1.5→**1.0**、
>    `curvature_lookahead_m` 3→**6**、`curvature_smooth_window` 4→**8**、`v_ref_lowpass_alpha`
>    0.75→**0.5**、`steer_lowpass_alpha` 0.5→**0.9**、`lqr_Q[4]`(e_psi_dot) 0.5→**4.0**。
>    实跑验证：弯道提前减速、道中稳定低速、出弯平稳缓加速，无极限环/出界。
> 3. **`path_follower` 闭环赛道投影越界崩溃修复**——loop 模式下 `_projected_index` 的扫描窗口
>    当 `start>0` 时会出现 `i=n-1`，导致 `path[i+1]`（`path[n]`）`IndexError` 进程崩溃；改为
>    `i_next=(i+1)%n` wrap 并重写闭环弧长/lookahead 逻辑。

> 本分支完成**仿真层与实车层的融合**：规划-控制-感知核心（你的 `baja_cloud_sim`）不动，
> 供应商硬件驱动包（华测组合导航 `chcnav`、镭神激光雷达 `lslidar`、毫米波雷达
> `radar_can_parser`、超声波雷达 `ultrasonic_radar_driver`、自定义消息 `msg_interfaces`、
> 以及仅保留 CAN 桥接的 `car_autonomous_pkg`）整体搬入 `src/hardware/`。实车与仿真通过
> **launch 话题 remap** 对齐（如 `/chcnav/devpvt → /gps/fix`），核心节点代码零改动。
> 实车运行入口为 `run_real.sh`（自主）/ `run_real_remote.sh`（视驾）/ `run_record.sh`（录制路径）。
> 供应商原 `ros2_ws/` 已删除。

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
- [实车层运行（融合）](#实车层运行融合)
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
./run.sh --seed 42 --obstacles 5 --finish-mode line     # 直线开放赛道 + 终点减速停车
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
> - `run.sh` 默认加载**闭环赛道**（`baja_loop.sdf`）；
> - 传 `--finish-mode line` 时自动切换为**直线开放赛道**（`baja_100m.sdf` + 20 m 停止区），
>   行为与旧 `run_line.sh` 一致（该脚本已并入 `run.sh`，不再单独存在）。

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

### 无障碍基准测试（默认关感知，专测规划控制）

`run_test.sh` 是专用的空赛道脚本，用于测纯跟踪/规划控制核心性能：

```bash
./run_test.sh --headless-gazebo --no-rviz
```

与 `run.sh` 的区别：固定 `seed=0` / `obstacles=0`（无 `--obstacles` 参数）、
使用独立的 `GZ_PARTITION`、结果写入 `results/flat_seed_0/`。
因此**可与 `run.sh` 并发运行**而互不干扰。

**默认 `use_boundary=false` + `use_obstacle=false`（关闭感知）**：不拉 `gz_pcl_bridge`
与 `lidar_sim`，Frenet 用**中心线 ±half_width 兜底走廊**（`/road_boundary_markers` 不再由任何
节点发布，Frenet 内部兜底），且 `frenet_planner` 不消费障碍消息，让你只跑规划-控制核心验证算法。
若需恢复完整感知，加 `--with-perception`（等价于 `use_boundary=true use_obstacle=true`）。

感知开关已拆分为两个**独立**参数，可分别便捷开关：

| 开关 | 默认值（params.yaml） | 作用 |
|------|--------|------|
| `use_boundary` | `false` | `true` 用 `road_analyzer` 真实车道线；`false` Frenet 走 ±half_width 兜底走廊 |
| `use_obstacle` | `false` | `true` 启动障碍检测发 `/obstacle_markers`；`false` 关闭障碍（纯跟踪） |

> **v2.7 起开关真正生效**：此前 `use_boundary` 在 `frenet_planner_node` 侧未被读取、且
> `simulation.launch.py` 硬编码默认 `true`，导致"默认开车道线检测"、yaml 改了无效。现已
> 修复——节点补上 `use_boundary` 声明/读取/回调判断，`simulation.launch.py` 默认值改从
> `params.yaml` 的 `frenet_planner_node.use_boundary/use_obstacle` 读取（与 `real_car.launch.py`
> 一致）。当前默认 `false` + `false`，即默认纯跟踪、不拉感知链路。

命令行：`--no-boundary` / `--no-obstacle`（脚本）；或 launch 直接传 `use_boundary:=false use_obstacle:=false`。
yaml 里 `frenet_planner_node.use_boundary` / `use_obstacle` 均可单独便捷开关。

| 参数 | 说明 |
|------|------|
| `--with-perception` | 恢复完整 LiDAR 感知链路（默认关） |
| `--no-boundary` / `--no-obstacle` | 单独关闭车道线 / 障碍检测 |
| `--seed` / `--finish-mode` / `--no-rviz` / `--headless-gazebo` / `--no-video` | 同 `run.sh` |

> **也可对任意 launch 直接传参**关闭某一感知：
> `ros2 launch baja_cloud_sim simulation.launch.py world_file:=... scenario_file:=... use_boundary:=false use_obstacle:=false`
> 此时 Frenet 不再依赖任何外部车道线消息，自动用中心线 ±half_width 兜底；`truth_perception`
> 不再发布 `/road_boundary_markers`（已移除真值边界兜底，避免与 `road_analyzer` 双发布冲突）。

---

## 实车层运行（融合）

v2.6 起仿真层与实车层已融合到同一工作空间。规划-控制-感知核心（`baja_cloud_sim` / `perception`）
在仿真与实车间**代码零改动**，仅通过 launch 的**话题 remap** 把实车硬件话题映射到仿真标准
话题名（见[系统架构](#系统架构)的实车边界）。实车软件包位于 `src/hardware/`，由 `run_real*.sh`
脚本启动。

### 运行入口

| 脚本 | 模式 | 说明 |
|------|------|------|
| `run.sh` | 仿真自主 | Gazebo 闭环/直线 + 全链路 |
| `run_remote.sh` | 仿真视驾 | 仿真下用 `remote_control_node` 接管 `/cmd_control` |
| `run_real.sh` | **实车自主** | 硬件定位+感知 → Frenet → 路径跟踪 → CAN 桥接 → VCU |
| `run_real_remote.sh` | **实车视驾** | 硬件定位+感知 → `remote_control_node` → CAN 桥接 → VCU |
| `run_record.sh` | **实车路径录制** | 仅定位 + 录制节点，遥控走一圈生成 CSV 航点 |

```bash
# 实车自主（需先录制好最佳路径 CSV，再喂给 Frenet 在线规划）
./run_real.sh --csv path/recorded_path.csv

# 实车视驾（远程 UDP 手柄/键盘接管）
./run_real_remote.sh

# 实车路径录制（遥控开车，后台按 0.5 m 间距采 GPS 航点）
./run_record.sh --output path/recorded_path.csv
```

> **比赛工作流**：比赛前一夜用 `run_record.sh` 遥控跑出最佳路线 → 生成 `recorded_path.csv`；
> 比赛当日用 `run_real.sh --csv recorded_path.csv` 启动：`csv_to_centerline_node` 将 CSV 转为
> `/reference_centerline` 喂给 `frenet_planner`，实时 GPS/IMU + 障碍驱动在线规划。
> `run_record.sh` 仅启动定位与录制节点，车辆由你的手柄/遥控器独立驱动，二者互不冲突。

### 接口对齐（remap 对照）

| 核心节点订阅 | 仿真来源 | 实车来源（硬件包） | 对齐方式 |
|------|------|------|------|
| `/gps/fix`（NavSatFix） | `truth_perception` | `chcnav` → `/chcnav/devpvt` | launch remap |
| `/imu/yaw`（Float32, 度） | `truth_perception` | `chcnav` → `/imu_yaw` | launch remap |
| `/ground_truth/odom`（Odometry） | `truth_perception` | `chcnav` → `/chcnav/odom` | launch remap |
| `/cmd_control`（AckermannDriveStamped） | `actuator_adapter` → Gazebo | `can_bridge_node` → VCU | **天然对齐**，不 remap |
| `/obstacle_markers` / `/road_boundary_markers` | 感知组 | 感知组（同包，输入点云 remap 到 `/cx/lslidar_point_cloud`） | 话题一致 |

实车参数文件：`src/planning_control/baja_cloud_sim/config/real_car_params.yaml`
（无 `use_sim_time`、无 Gazebo 专用节点，含 `csv_to_centerline` / `frenet_planner` /
`path_follower` / `evaluator(use_scenario=false)` / `remote_control` / `can_bridge` 参数）。

### 实车 launch 清单

`src/planning_control/baja_cloud_sim/launch/` 下新增：

- `real_car.launch.py`：实车自主（chcnav + lslidar + 感知 + csv_to_centerline + frenet + path_follower + can_bridge + evaluator + rviz）
- `real_car_remote.launch.py`：实车视驾（chcnav + lslidar + remote_control + can_bridge + evaluator + rviz）
- `path_record.launch.py`：路径录制（chcnav + path_recorder）
- `remote_simulation.launch.py`：仿真视驾（`remote_control` 替代 `path_follower`）

### 硬件包（`src/hardware/`）

| 包 | 角色 | 关键节点/文件 |
|----|------|---------------|
| `chcnav` | 华测组合导航 | `chcnav_full_node`（vcan2，发 `/chcnav/devpvt`+`/imu_yaw`+`/chcnav/odom`） |
| `lslidar_ros2-master` | 镭神激光雷达 | `lslidar_driver`（发 `/cx/lslidar_point_cloud`，**编译需 `libpcap-dev`**） |
| `radar_can_parser` | 毫米波雷达 | can0 解析 |
| `ultrasonic_radar_driver` | 超声波雷达 | vcan3 |
| `msg_interfaces` | 自定义消息 | — |
| `car_autonomous_pkg` | **仅保留 CAN 桥接** | `can_bridge_node` + `can_manager` + `message_handler` + `vehicle_params` + `*.dbc`（规划控制节点已删除，由 `baja_cloud_sim` 替代） |

> **注意**：供应商原 `ros2_ws/` 目录已删除，硬件包全部迁入 `src/hardware/`。
> `lslidar_driver` 编译依赖 `libpcap-dev`，实车环境需 `sudo apt-get install -y libpcap-dev`。

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
  用立放轮胎标定的赛道边界。轮胎为 **纯 Gazebo 物理几何体**，写入 `scenario.json` 的
  `edge_tires` 字段，**不通过 `/obstacle_markers` 发布**——感知组直接在仿真里用其
  自有 Gazebo 传感器检测，**不参与路径规划/避障，也不绘制膨胀框**。

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
       │                     └─ /reference_centerline
       └─ evaluator ─────────── results/seed_N/tracking_*.csv

──── 实车边界（v2.6 融合，同一算法核心，仅 remap 换源）────

chcnav (vcan2)
  ├─ /chcnav/devpvt   ──remap→  /gps/fix         (NavSatFix)
  ├─ /imu_yaw         ──remap→  /imu/yaw         (Float32, 度)
  └─ /chcnav/odom     ──remap→  /ground_truth/odom (Odometry)
       │
lslidar_driver (cx) ── /cx/lslidar_point_cloud ──remap→ 感知组输入点云
       │
   [ 算法核心层 frenet_planner / path_follower / evaluator 不变 ]
       │
       ▼ /cmd_control (AckermannDriveStamped)
can_bridge_node (vcan1) ── VCU CAN  (★ 天然对齐，无需 remap)
       │
   [ 实车不再启动 truth_perception / actuator_adapter / video_recorder / gz_pcl_bridge ]

感知组 LiDAR 管道（lidar3d_bringup + patchwork++ + lidar3d_perception_cpp，实车传感器在仿真中的部署）
  ├─ Gazebo gpu_lidar ── /lidar/points
  │    └─ pointcloud_filter → patchworkpp → surface_detector (C++) → obstacle_adapter
  │         └─ /obstacle_markers  (base_link, CUBE, ns=tall | flat_ground)
  └─ road_analyzer ───── /road_boundary_markers (base_link, LINE_STRIP, ns=road_left/road_right)
       ↑ 雷达/点云直接检测 Gazebo 中的物理障碍盒 / 轮胎 / 边界，不依赖真值注入

frenet_planner (10 Hz)
  ├─ /planned_path      (nav_msgs/Path, 绿色)
  └─ /planner/status    (FEASIBLE | INFEASIBLE)
       │
       ▼
path_follower (20 Hz, dt=0.05 s)
  ├─ 速度剖面   曲率上限 → 前向加速约束 → 后向减速约束
  │             → clearance 降速 → 后向可行性再传播
  ├─ 横向       LQR + 前馈（主）/ 纯追踪（fallback）
  ├─ 纵向       实车开环：决策层(速度剖面/finish/障碍) + idle 保底 → 期望速度设定值
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
- **速度自适应反馈软化（β(v)）**：高速时反馈项乘以
  `β = 1 / (1 + α·(v − v_ref))`，饱和到 `[β_min, 1.0]`。小误差在高速下不再被过激
  放大成蛇形修正；低速（v ≤ v_ref）`β=1.0` 保持完整修正力。补偿的是真实齿条（EPS）
  约束，**实车仍需保留**。
- **航向误差移动平均（e_psi MA，K1）**：参考点跳变 / DARE 重解会让 `e_ψ` 出现阶跃，
  直接进反馈放大成转向突变。对 `e_ψ` 取 `e_psi_ma_window`（默认 5 步 ≈ 0.25 s）窗口
  移动平均后再参与 LQR 状态向量，滤除瞬时阶跃、显著减少变频率而不破坏弯道稳态。
- **指令端绝对死区（cmd_steer_deadband，K3）**：平滑后的转向指令绝对值小于
  `cmd_steer_deadband_deg`（默认 1.5°）时强制归零；与 EPS 的 1° 死区 + 0.5° 输出死区
  共同构成约 3° 的小角度不响应区间，吃掉直道稳态残余抖动。弯道正常转角（≥2°）不受影响。

### 纵向 — 实车开环（期望速度设定值）

```
v_ref  ← 曲率速度剖面 → finish 限速 → 障碍/地形降速 → 安全状态机
v_out  = max(v_ref, idle_speed)        # idle 怠速/蠕行保底
        → 作为 /cmd_control.speed 发出（期望速度设定值）
```

- **v2.4 起改为开环**：上游只发**期望速度设定值**，由电机控制器自带速度闭环自行
  计算所需加速度/扭矩去追随。不再在 ROS 节点内做 PID 速度环、坡度前馈积分与
  accel/jerk 速率限制——那些是仿真弱执行器（Gazebo `AckermannSteering` 直接积分
  `cmd_vel`）的兜底；在带速度闭环的实车电机控制器上，两层限制会叠加使加减速变肉、
  且增益互相"吃掉"。
- 保留 `idle_speed` 怠速下限，避免起点无动力趴窝、或静止触发 finish 后被取消蠕行
  导致永远起不了步（仅当 `_finished` 或 finish 刹车进站时才允许降到 0）。
- 上游软件侧的 cascaded-PID 速度环参数（`lon_kp/lon_ki/lon_kd/lon_i_limit/
  lon_accel_step/lon_decel_step/slope_comp_gain`）已在 v2.7 清理中**彻底移除**（yaml
  声明、节点 `declare_parameter`、积分状态量、以及 `_compute_longitudinal_target` 里的
  旁路分支全部删除），不再保留历史残留；如需仿真期 PID 跟随，需重新实现该速度环。
- 坡度补偿（上坡前馈）在实车上交由电机控制器处理。
- **特殊地形降速**已统一由 `flat_ground` 感知障碍物类（`_ground_derate` 纵向平滑降速）
  处理；旧版基于中心线坡度的 `use_terrain_profile` / `terrain_slope_threshold` /
  `terrain_min_speed` 及 `_derate_speed_profile` 内整段 terrain derating、`_terrain_slope_at`
  方法均在 v2.7 清理中删除。

**历史说明（仿真期级联 PID，v2.7 已删除）**：v2.3 及以前纵向为
`a_cmd = Kp·e_v + Ki·∫e_v + Kd·ė_v + 9.81·sin(θ_road)·slope_comp_gain`，
积分得 `v_tgt` 再经 accel/jerk 速率限制。该方案解决了横向误差硬阈值引发的直线速度
极限环（P1）；v2.4 为对接实车电机闭环改为开环发设定值；**对应的 `lon_kp/ki/kd`、
`slope_comp_gain`、`lon_accel_step/decel_step` 等参数与实现已在 v2.7 清理中彻底移除**，
此处仅作历史参考。

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

### EPS 执行器模型（仿真层）

为使 LQR 在**真实齿条约束**下得到验证，仿真层 `actuator_adapter_node` 内置了 EPS
（Electric Power Steering）模型，模拟真实转向机构：

| 约束 | 值 | 说明 |
|------|------|------|
| 齿条最大转速 | **15°/s** | `eps_max_rate_deg`，满打 35° 约 2.3 s，拟合真实电动助力齿条 |
| 死区 | **1°** | `eps_deadband_deg`，消除小幅抖动/传感器噪声 |
| 一阶滞后 | `eps_tau` = **0.0** | 设为 0（关闭显式滞后）：Gazebo `AckermannSteering` 插件已自带
  ~80 ms 物理转向响应，显式滞后会与之一阶双重建模、引入额外相位滞后 → 蛇形；
  依赖 Gazebo 物理响应即可 |

模型实现（`_apply_eps`）：`|Δ| < deadband` → 保持原角（去抖）；否则先按 `eps_tau`
（=0 时退化为瞬时）做一阶滞后，再按 `eps_max_rate × dt` 限幅（齿条转速上限）。
最终 `angular.z = v · tan(δ_eps) / L` 发给 Gazebo。

**诊断话题**：`/metrics/cmd_steer`（LQR 理想指令角）与 `/metrics/eps_actual_steer`
（经 EPS 模型后的实际齿条角，Float32），用于叠加对比、量化滞后。
`evaluator_node` 记录 `eps_steer_rad` 列，`tools/plot_tracking.py` 的 `time_*.png`
叠加绿色实际齿条角与滞后带。

> **关键认知**：EPS 的 15°/s 转速上限会破坏「LQR 指令立即到达」的假设，是引入该模型后
> 控制恶化的根源——**不是 LQR 算法被改坏**，而是执行器约束首次被如实建模。修复方式是
> 让 LQR 侧保留完整修正权限（见下方速度自适应反馈软化 + 控制器侧不重复限速），由 EPS
> 层施加真实约束。此后反馈软化（`beta(v)`）**在实车上仍需保留**——它补偿的是真实齿条，
> 非仿真专属。

---

## 参数说明

配置文件：[`src/baja_cloud_sim/config/params.yaml`](src/baja_cloud_sim/config/params.yaml)

### 横向控制

```yaml
path_follower_node:
  ros__parameters:
    enable_lqr: true
    lqr_Q: [0.05, 3.0, 6.0, 4.0, 5.0]  # [∫e_y, e_y, ė_y, e_ψ, ė_ψ]
                                        #   v2.7-test 最终值: 测试中阻尼中提(Q2=6,Q4=5)减饱和,
                                        #   配合渐进恢复门让大偏离恢复一步到位; 积分0.05保持稳态贴线
    lqr_R: 3.0                          # 控制量惩罚（越大转向越柔和）
    lqr_v_norm: 1.5                     # 增益调度归一化速度
    lqr_velocity_recompute_threshold: 1.0  # DARE 重解速度触发阈值 (m/s)
    dare_solve_interval: 50             # DARE 强制重解步数间隔 (50×0.05s=2.5s)
    s_proj_lookahead: 1.5               # 参考点弧长前视 (m) — v2.7-test: 0.8→1.5, 加大预瞄缓解参考点跳变
    max_steering_angle: 26.0            # 物理钳位 26°（EPS 齿条满打上限, 仿真真实建模）
    max_steer_rate: 1.2                 # 控制器侧转向速率上限 (rad/s) ≈ 69°/s — 宽松,
                                        # 让 LQR 修正指令完整到达 EPS; 真实齿条 15°/s
                                        # 约束由 EPS(actuator_adapter)施加, 此处不重复限速
    steer_lowpass_alpha: 1.0            # 输出一阶低通系数 — v2.7-test: 0.9→1.0, 彻底去相位滞后让指令即时到位
    state_lowpass_alpha: 0.75           # 反馈状态(yaw_rate/vel)低通系数 — v2.7-test: 0.45→0.75, 原0.25过度滞后致弯道饱和
    fb_speed_soften_alpha: 0.4          # 速度自适应反馈软化系数 α (v2.7-test: 0.3→0.4, 中等软化)
    fb_speed_ref: 1.5                   # 参考速度 (m/s), 低于此速度 β=1.0(不软化) — v2.7-test: 2.5→1.5, 1.5以上才软化
    fb_speed_beta_min: 0.5              # β 下限, 防止极端高速反馈完全失效 (v2.7-test: 0.6→0.5, 高速反馈弱化至 50%)
    e_psi_ma_window: 0                  # K1: v2.7-test 关闭 e_psi 移动平均(原5≈0.25s滞后延长饱和段),
                                        # s_proj_lookahead 加大 + last_idx 跟踪已缓解参考点跳变, 关闭后起步收敛更快
    cmd_steer_deadband_deg: 1.5         # K3: 指令端绝对死区(deg), 平滑后|<1.5° 强制归零, 配合 EPS 共约 3° 不响应区间
```

> **速率限制与突变保护**：节点层对 LQR 输出依次施加
> （1）一阶低通 → （2）按 `max_steer_rate × dt` 的速率限幅 → （3）**突变保护**：
> 当阶跃幅度超过正常步长的 2 倍（即 DARE 重解 / 投影跳变造成的脉冲）时，
> 改用 0.5× 步长收紧截断。三层叠加确保前轮转角平滑、无极端阶跃，
> 实车轮胎不会承受频繁反向急打。

### 纵向控制

```yaml
    idle_speed: 0.6         # 怠速下限 (m/s) — v2.7-test: 1.0→0.6, 起步大偏离时由航向门钳到该蠕行速度防冲过线
    # 注：级联 PID 速度环参数已在 v2.7 清理中删除；实车纵向为开环发设定值，
    #     速度闭环由电机控制器完成，此处不再列示 lon_kp/ki/kd 等参数。
```

**航向门（v2.7-test 新增，`_compute_longitudinal_target` 内）**——起步/突发大偏离时车身尚未对齐规划线，
若直接按 4 m/s 巡航会"加速冲过轨迹"导致蛇形。新增双因子门：

```python
e_psi = float(command.get("heading_error", 0.0))   # 车头相对参考航向角误差
e_y_val = abs(float(command.get("e_y", 0.0)))       # 横向偏移
# 双条件同时满足才钳到 idle_speed(0.6): 车头偏且横向也明显偏
if abs(e_psi) > math.radians(8.0) and e_y_val > 0.5 and not braking_to_stop:
    v_ref = min(v_ref, self._idle_speed)
```

设计要点：
- **双因子（e_ψ > 8° 且 |e_y| > 0.5 m）**而非单看 e_ψ：弯道里 |e_y| 通常 < 0.5 m（车已在线上只是车头偏），
  不会被误钳；只有起步/被推离线的"车头偏 + 横移 > 0.5 m"组合才触发防冲过线。
- 触发后降到 `idle_speed(0.6)` 蠕行，待车身回正、|e_y| 收敛后自动解除，速度恢复速度剖面。
- 与 `idle_speed` 的区别：`idle_speed` 是无条件怠速保底（防趴窝），航向门是条件性"纠偏期限速"。

### 速度剖面与安全

```yaml
    use_speed_profile: true
    speed_profile_max: 4.0        # fallback/历史兼容: tier 非法时退回此值
    max_lateral_accel: 0.8        # 弯道侧向加速度上限 (m/s²) — v2.7-test: 0.7→0.8, 弯道不过慢; 恢复模式阈值放宽后不再误触
    curvature_lookahead_m: 8.0    # 曲率前瞻+后视距离 (m): 双向约束, 出弯加速延后防弯切直蛇形
                                #   v2.7-test: 6.0→8.0, 更大窗口→进弯更早减速
    curvature_smooth_window: 8    # 曲率平滑窗口 (v2.7: 4→8), 抑制局部尖峰导致的 v_ref 抖动
    pre_decel_lookahead_m: 20.0   # 前向预减速前瞻 (m): v2.7-test: 18.0→20.0, 配合 horizon=40 不超过其50%使 pre_decel 真正生效
    pre_decel_max: 1.0            # 预减速段最大减速度 (m/s²): v2.7: 1.5→1.0, 越温和过渡带越长
    v_ref_lowpass_alpha: 0.5      # v_ref 帧间低通系数: v2.7: 0.75→0.5, 更平滑抹平 10Hz 刷新锯齿
    desired_clearance: 3.5        # 障碍降速起始距离
    min_speed_obstacle: 2.0
    tier_slow_speed: 2.5          # slow 档: 直线段最大速度
    tier_normal_speed: 3.5        # normal 档: 直线段最大速度 — v2.7-test: 4.0→3.5, 降贴线后过冲动能
    tier_fast_speed: 4.5          # fast 档: 直线段最大速度
```

### 感知接口与安全

```yaml
path_follower_node:
  ros__parameters:
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
  走廊避让）、**`flat_ground`** → 特殊地面（仅纵向降速，直线通过）。
  **`ns` 缺失或非上述值 → 忽略**（不静默默认，避免误分类）。
  > 道路边缘立放轮胎（`edge_tires`）为**纯 Gazebo 物理圆柱体**，**不发布**在
  > `/obstacle_markers` 上——感知组在仿真里用自己的 Gazebo 传感器直接检测，
  > 算法核心无需处理轮胎，也不画膨胀框。
- 安全状态机现已订阅 `/metrics/planned_clearance`：corridor 余量 < 5 cm 触发
  `EMERGENCY`、< 30 cm 触发 `SLOWDOWN`，此前该守卫因 clearance 未接入而恒不触发。

### EPS 执行器模型（仿真层参数）

EPS 模型仅在仿真层 `actuator_adapter_node` 生效，实车层用不同 launch 文件替换该节点
即去除：

```yaml
actuator_adapter_node:
  ros__parameters:
    eps_tau: 0.0                  # 一阶滞后时间常数 (s) → 0: 依赖 Gazebo 物理响应(~80ms)
    eps_max_rate_deg: 15.0        # 齿条最大转速 (deg/s), 真实约束, 15 为拟合值
    eps_deadband_deg: 1.0         # 死区 (deg), 消除小幅抖动/传感器噪声
    eps_output_deadband_deg: 0.5  # 输出端绝对死区(deg), 最终齿条角绝对值<0.5° 强制归零, 治理直道稳态高频抖动
```

诊断话题：

```yaml
/metrics/cmd_steer          # Float32, LQR 理想指令角
/metrics/eps_actual_steer   # Float32, 经 EPS 模型后的实际齿条角
```

`evaluator_node` 记录 `eps_steer_rad` 列，`tools/plot_tracking.py` 的 `time_*.png`
叠加绿色实际齿条角与滞后带，用于量化 EPS 引入的相位滞后。

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

> **最小契约**：中心点坐标（`pose.position.x/y`）+ 类型（`ns`）+ 前向长度（`scale.x`）。
> 宽度与高度对算法核心非必需——宽度由兜底参数处理，高度仅用于 RViz 盒子显示。
> 仿真联调阶段，`/obstacle_markers` 由**感知组的 Gazebo 雷达**直接检测物理障碍盒
> 后发布（标 `ns=tall` / `ns=flat_ground`），`truth_perception` **不再注入障碍真值**。
> 道路边缘轮胎为纯 Gazebo 物理圆柱体，同样由雷达检测、不经真值发布，
> 故 planner 的红色膨胀框只对 `tall` 障碍绘制、不会被轮胎干扰。

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
    horizon_m: 40.0                # v2.7-test: 30.0→40.0, 让速度剖面看到更远弯道使 pre_decel 生效
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
| 直线速度波动 | 检查 `v_ref_lowpass_alpha` 是否过小；`speed_profile_max` 与 tier 速度是否匹配 |
| 上坡掉速 | 实车纵向开环，速度闭环在电机控制器；若感知有 `flat_ground` 标记，检查 `_ground_derate` 的 `slow_speed` |

---

## 感知组子系统（Perception，对接融合）

感知组在仿真中部署了实车 LiDAR 传感器链路，替代 `truth_perception` 发布
`/road_boundary_markers`，并提供 `/obstacle_markers`。本仓库已将其源码并入
`src/perception/`，并在 `simulation.launch.py` 中通过 `lidar_sim.launch.py`
一键拉起（见下文启动方式）。以下为其节点关系与契约（提炼自感知组文档）。

### 节点关系图

```text
Gazebo gpu_lidar (baja_vehicle/base_link/lidar)
   └─ /lidar/points (PointCloud2)
        │
        ▼ pointcloud_filter (lidar3d_bringup, Python)
   /lidar/points_filtered (降采样, 下游按需再降)
        │
        ├─► /cx/lslidar_point_cloud_filtered ─► patchworkpp (patchwork-plusplus, C++)
        │                                           └─► /ground_seg/points/ground (地面点)
        │                                                └─► ground_filter (lidar3d_bringup, Python)
        │                                                     └─► /lidar/obstacle_points (非地面点)
        │
        └─► surface_detector (lidar3d_perception_cpp, C++, 订阅 /lidar/points_filtered)
             ├─► /lidar/road_boundary_markers (base_link, LINE_STRIP, ns=road_left/road_right)
             └─► /lidar/obstacle_markers      (base_link, CUBE, ns=tall | flat_ground)

road_analyzer        (lidar3d_bringup, Python, 订阅 /lidar/road_boundary_markers)
   └─► /road_boundary_markers  (重映射, base_link, LINE_STRIP, ns=road_left/road_right)
obstacle_adapter     (lidar3d_bringup, Python, 订阅 /lidar/obstacle_markers)
   └─► /obstacle_markers       (重映射, base_link, CUBE, ns=tall | flat_ground)
```

> 注：`patchworkpp` 等 C++ 节点当前以 `use_sim_time=false` 构建消息，但 lidar_sim
> 启动时会按 `use_sim_time` 条件重映射 `/clock` 并修正其时间戳，使其在仿真下可用。
> 感知管道发布的 Marker 帧统一为 `base_link`（相对车体），与算法核心契约一致。

### 包清单（`src/perception/`）

| 包 | 语言 | 角色 | 关键节点 |
|----|------|------|----------|
| `lidar3d_bringup` | Python | 启动编排 + Python 辅助节点 | `pointcloud_filter`、`ground_filter`、`road_analyzer`、`obstacle_adapter`、`tf_bridge`、`start_gazebo` |
| `lidar3d_perception_cpp` | C++ | 点云→边界/障碍主检测 | `surface_detector` |
| `lidar_cluster_ros2` | C++（第三方 fork） | 欧氏聚类 | `lslidar_cluster` |
| `patchwork-plusplus` | C++（第三方 fork） | 地面分割 | `patchworkpp` |

### 话题契约（与算法核心对齐，已校验一致）

| 话题 | 帧 | 类型 / `ns` | 算法核心如何使用 |
|------|------|------------|--------------|
| `/road_boundary_markers` | `base_link` | `LINE_STRIP`，`ns=road_left` / `road_right` | planner 转世界系做走廊；follower 做限速 |
| `/obstacle_markers` | `base_link` | `CUBE`，`ns=tall` | planner 取 tall 做横向走廊膨胀；follower 做动态减速 |
| `/obstacle_markers` | `base_link` | `CUBE`，`ns=flat_ground` | follower 做纵向降速直线通过；planner 跳过 |

其余 `/gps/fix`、`/imu/yaw`、`/localization/odom`、`/reference_centerline` 仍由本仓库
`truth_perception_node` 从 Gazebo 真值发布（**边界已退役**，仅保留定位/参考线）。

### 启动方式（一键）

感知管道已并入 `simulation.launch.py`，因此你原有的 `./run.sh`、`run_test.sh`
（circle/time/line 等模式）**一个命令即同时拉起规划-控制与感知**。
如需单独调试感知，也可在仿真运行后另开终端：

```bash
# 仅单独拉起感知管道（仿真已在运行）
ros2 launch lidar3d_bringup lidar_sim.launch.py
```

### 依赖

感知组 C++ 包需 PCL，已在 `install_ubuntu2204.sh` 补装
`ros-humble-pcl-ros`、`libpcl-dev`、`python3-transforms3d`、`ros-humble-tf-transformations`。
干净环境请重新执行安装脚本；你本地历史环境若已具备可跳过。

> **LiDAR 传感器**：已在车辆模型 `models/baja_vehicle/model.sdf` 的 `base_link` 上
> 挂载 `gpu_lidar`（高 1.5 m，frame 解析为 `baja_vehicle/base_link/lidar`，发布
> `/lidar/points` 经 `config/bridge.yaml` 转发），参数（1800×16 线、±π 水平、±15°
> 垂直、200 m 量程）沿用感知组仿真设定，使端到端链路可在同一 Gazebo 世界内跑通。

---

## 代码结构

算法包现按职责分为 `src/planning_control/` 与 `src/perception/` 两类。

- **规划-控制核心**：`src/planning_control/baja_cloud_sim/baja_cloud_sim/`
- **感知组子系统**：`src/perception/`（见上节）

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
                          ★ v2.7-test 方案A：组装 start 时对所有 scenario 注入通用起始偏移
                            （y +1.5 m 横向 + 初始偏航 +15° 朝外），用于测试"起步大偏离恢复能力"
truth_perception_node.py   由真值里程计派生带噪声的定位/GPS/IMU/中心线/边界/障碍（仿真用）
mock_perception_node.py    ★ 模拟感知组信息流（实车对接调试用，非运行必需）
frenet_planner_node.py     10 Hz 规划，输出 /planned_path 与 /planner/status
path_follower_node.py      ★ 核心控制器（横向 LQR + 纵向开环 + 状态机）
actuator_adapter_node.py   /cmd_control → /model/baja_vehicle/cmd_vel（仿真用）
                          ★ 内置 EPS 执行器模型（15°/s 转速上限 + 1° 死区，仿真层）
evaluator_node.py          指标计算与 CSV 记录（实车模式 use_scenario=false，从 /reference_centerline + /obstacle_markers 实时获取）
video_recorder_node.py     ffmpeg 录制 Gazebo 相机
remote_control_node.py     UDP 远程视驾控制（仿真/实车共用，发布 /cmd_control）
csv_to_centerline_node.py  CSV 航点 → /reference_centerline（喂给 Frenet，实车赛前录制路径复用）
path_recorder_node.py      实车路径录制（订阅 /gps/fix + /imu/yaw，按 0.5 m 采点写 CSV）
```

硬件层（`src/hardware/`，供应商驱动，实车用）：

```text
chcnav/                   华测组合导航（vcan2 → /chcnav/devpvt, /imu_yaw, /chcnav/odom）
lslidar_ros2-master/      镭神激光雷达（→ /cx/lslidar_point_cloud，编译需 libpcap-dev）
radar_can_parser/         毫米波雷达（can0）
ultrasonic_radar_driver/  超声波雷达（vcan3）
msg_interfaces/           自定义消息包
car_autonomous_pkg/       ★ 仅保留 CAN 桥接：can_bridge_node + can_manager + message_handler + vehicle_params + *.dbc
                          （原 path_follower/obstacle_avoider/path_recorder 等规划控制节点已删除，由 baja_cloud_sim 替代）
```

> **实车移植角色**：`truth_perception_node` 与 `actuator_adapter_node` 是
> 平台相关两端（仿真读 Gazebo 真值 / 发 Gazebo cmd_vel），实车需替换为真实
> 定位感知节点与底盘驱动节点；其话题均已参数化（`ground_truth_odom_topic`、
> `cmd_vel_topic`、`odom_topic`）。`mock_perception_node` 用于在**无 Gazebo**
> 环境下按约定契约发感知消息，验证算法核心接收链路。
> **EPS 执行器模型仅存在于仿真层 `actuator_adapter_node`**，实车用真实底盘驱动节点
> 替换该节点即自动去除 EPS 建模（EPS 的 15°/s 转速上限是真实齿条约束，实车由硬件
> 物理实现，软件无需重复；但 `beta(v)` 反馈软化需保留在算法核心侧以补偿真实齿条）。

### 遗留代码（不影响运行）

以下文件为历史版本残留，**当前链路不会加载**，修改它们不会产生任何效果：

| 文件 | 说明 |
|------|------|
| `core.py`（顶层，1077 行） | v1.0 单体核心。与 `core/` 目录同名，Python 的包优先级使 `from .core import ...` **始终解析到 `core/` 目录**，此文件从未被导入 |

> 修改控制算法请定位到 **`core/controller.py`** 与 **`path_follower_node.py`**。
> 旧版 `lqr_controller.py`（v1.2/v1.3 含离线增益表的 LQR 实现）及其单测
> `test/test_lqr_controller.py` 已在 v2.7 清理中删除，当前 LQR 仅由
> `core/controller.py` 提供；`trajectory_smoother.py` 若仅被旧 LQR 依赖亦随之移除。

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

### 6. 多次运行残留 Gazebo 实例导致 `/clock` 冲突（RViz 全屏闪烁 + 蛇形）

**现象**：RViz 中所有节点频繁闪烁，日志反复报
`[tf2_buffer] Detected jump back in time. Clearing TF buffer.`；
小车蛇形走位、跟踪线效果差。

**根因**：`run.sh` / `run_test.sh` 未确保旧实例退出时，后台会残留多个
`gz sim` + `ros_gz_bridge` + `robot_state_publisher`。多个 Gazebo 各自发布
`/clock`，形成 **`/clock` 双发布者**，时钟时间戳互相冲突 → tf2 反复清空缓冲
→ 闪烁；`path_follower` 位姿时间轴混乱 → 蛇形。

**验证**：`ros2 topic info /clock` 应显示 `Publisher count: 1`；若 >1 即冲突。

**修复**：所有算法节点（含 `truth_perception` / `path_follower` / `frenet_planner` /
`actuator_adapter` / `evaluator`）现已统一设 `use_sim_time: True`，与 Gazebo
时间源对齐；运行前务必清理残留实例：

```bash
pkill -f "gz sim"; pkill -f "ros_gz_bridge"; pkill -f "robot_state_publisher"
```

> 各 launch 使用固定的 `GZ_PARTITION="baja_$USER"`，多次运行若无清理会复用
> 同一分区并叠加实例。后续可考虑每次随机 partition 或启动前自动清理。

### 7. 转向平滑与 DARE 重解脉冲

早期版本在弯道段出现过两类转向异常，均已修复：

1. **DARE 重解阶跃脉冲**：`dare_solve_interval`（原 10 步 = 0.5 s）每个周期强制
   重解 Riccati 方程，增益矩阵 `K` 不连续跳变，在弯道处被放大成 ±20° 转向脉冲。
   已将间隔放宽到 **50 步（2.5 s）** 并新增**输出突变保护**（阶跃超过正常步长 2 倍时
   用 0.5× 步长截断），详见[参数说明](#参数说明)的转向平滑项。
2. **弯道高频锯齿抖动 / 直道稳态抖动**：早期通过降低输出低通系数
   （`steer_lowpass_alpha` 0.35 → 0.22）与收紧速率上限平滑；v2.3 进一步通过
   **e_psi 移动平均（K1）** 滤除参考点/DARE 跳变阶跃、`**指令端绝对死区 1.5° + EPS 输出
   死区 0.5°** 吃掉直道残余高频抖动，突变频率已明显收敛。

个别曲率更大的弯道（如闭环第三弯）仍需要较大的必要转角，这是轨迹几何决定的
**合理需求**，非控制不稳定——压低它会增大跟踪误差、切弯。

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
| **`v2.3`** | **v2.3** | **LQR + 前馈 + β(v)** | **EPS 执行器模型 + 速度自适应反馈软化 + 双向曲率前瞻 + 三档速度分级** |
| **`v2.4`** | **v2.4** | **LQR + 前馈 + 开环纵向** | **纵向改为实车开环(发期望速度设定值, 速度闭环交电机) + 感知开关(`use_perception` / `run_test.sh` 默认关感知) + 车道线真值自动切换** |
| **`v2.5`** | **v2.5** | **LQR + 前馈 + 开环纵向** | **环境快照/恢复脚本 + 测试缓存清理(`.pytest_cache`/`.claude` 入 ignore) + 安装/运行脚本与文档同步** |
| **`v2.6`** | **v2.6** | **LQR + 前馈 + 开环纵向** | **仿真层与实车层融合：`src/hardware/` 迁入供应商硬件驱动（chcnav/lslidar/radar/ultrasonic/msg_interfaces/car_autonomous_pkg 仅留 CAN 桥接）；核心节点零改动 + launch remap 对齐；新增实车 launch 与 `run_real*.sh`/`run_record.sh`；`csv_to_centerline`/`path_recorder` 节点；`evaluator` 支持 `use_scenario=false` 实车模式；删除 `ros2_ws/` 与 `run_line.sh`** |
| **`v2.7`** | **v2.7** | **LQR + 前馈 + 开环纵向** | **弯道稳定性再调优（实跑验证：弯道提前减速、道中稳定低速、出弯平稳缓加速）：`max_lateral_accel` 0.8→0.7、`pre_decel_lookahead_m` 8→18、`pre_decel_max` 1.5→1.0、`curvature_lookahead_m` 3→6、`curvature_smooth_window` 4→8、`v_ref_lowpass_alpha` 0.75→0.5、`steer_lowpass_alpha` 0.5→0.9、`lqr_Q[4]`(ė_ψ) 0.5→4.0；修复 `use_boundary` 开关失效（`frenet_planner_node` 补参数声明/读取/回调判断 + `simulation.launch.py` 默认值改从 `params.yaml` 读取，默认 `false` 纯跟踪）；修复 `path_follower` 闭环赛道投影 `IndexError` 崩溃（loop 模式 `i_next=(i+1)%n` wrap）** |
| **`v2.7-test`** | **v2.7-test** | **LQR + 前馈 + 开环纵向** | **起步大偏离恢复能力测试与调参（脱离 v2.7 主线的实验分支，最终收敛版）：方案A 起始位姿偏移（`scenario_generator` 对所有 scenario 注入 y+1.5m + 初始偏航 +15°，测试"大偏离恢复"）；新增**航向门(P0)**双因子限速（`_compute_longitudinal_target`：e_ψ>8° 且 \|e_y\|>0.5m 才钳到 idle_speed 防冲过线，弯道不误伤）+ **渐进恢复门(P0.5)**（`path_follower_node._recovery_active` 状态机：\|e_y\|>0.6m 进入恢复模式并随贴线程度从 idle 线性抬高限速上限，设 1.0s 贴线保持计时器 + 7.5s 超时强制退出，弯道 \|e_y\|<0.6m 永不误触）；调参 `lqr_Q`→[0.05,3.0,6.0,4.0,5.0]（增 ė_y/ė_ψ 阻尼减饱和）、`s_proj_lookahead` 0.8→1.5、`max_steering_angle` 35→26（物理钳位）、`steer_lowpass_alpha` 0.9→1.0、`state_lowpass_alpha` 0.45→0.75、`fb_speed_soften_alpha` 0.3→0.4、`fb_speed_ref` 2.5→1.5、`fb_speed_beta_min` 0.6→0.5、`e_psi_ma_window` 5→0、`idle_speed` 1.0→0.6、`tier_normal_speed` 4.0→3.5、`max_lateral_accel` 0.7→0.8、`curvature_lookahead_m` 6→8、`pre_decel_lookahead_m` 18→20、`pre_decel_max` 1.0→1.5、`horizon_m` 30→40（使 pre_decel 真正生效）。未动 EPS 物理约束（15°/s 转速上限）。验证：起步 1.5m+15° 偏离约 2s 收敛贴线（center 1.5→0.005）、直道 3.5m/s 巡航、弯道 v_mean≈1.86m/s 不冲出赛道、转向饱和率 0%** |

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
   的 `edge_tires`。轮胎为**纯 Gazebo 物理几何体**，**不通过 `/obstacle_markers` 发布**；
   感知组在仿真里用其自有 Gazebo 传感器直接检测，专供**感知组识别 / 与真实赛道
   landmark 对齐**，**不参与路径规划与避障**。
5. **膨胀框范围修正**——`frenet_planner` 的 RViz 红色半透明 `inflated_obstacles` 膨胀框
   仅对 `tall` 正常障碍绘制；道路边缘轮胎为物理几何体、不在 ROS 话题中，既不进 planner
   也不画膨胀框，避免干扰避障逻辑的观感判断。
6. **障碍真值不再由 `truth_perception` 注入**——感知组在 Gazebo 小车上加装了雷达，
   障碍盒与轮胎均由雷达直接检测并以 `/obstacle_markers`（`ns=tall`/`flat_ground`）发布；
   `truth_perception` 移除 `/obstacle_markers` 发布者，仅保留定位/GPS/IMU/中心线/边界真值。
   仿真联调阶段算法核心消费的是雷达输出，与真实车一致，不再有"真值→算法"捷径。
7. **起点附近幽灵红框（已知小问题，影响极小）**——现象：起点附近 RViz 出现一个
   `tall` 红色膨胀框，但该处实际没有障碍物，且车辆仅绕一个极小弧度。根因：
   `mock_perception_node` 是**真感知到达前的占位调试工具**，内置一个测试用 `tall`
   障碍（base_link 下 `x=12.0 m, y=1.2 m`，见 `mock_perception_node.py`）。若它残留
   在后台与仿真/真雷达**同时运行**，就会持续发这个假障碍，既画红框又触发极小横向
   避让，且会因卡住避障走廊导致车辆不动。**该节点不应与真实仿真/雷达同跑。**
   `run.sh` 已加入启动前自动 `pkill -9 -f mock_perception`，避免旧进程残留。若手动
   启动，跑新仿真前务必先 `pkill -9 -f mock_perception` 清理。影响评估：红框仅来自
   假障碍，清理后消失，实际轨迹修正极小，可忽略。
8. **障碍原点杂波过滤**——`frenet_planner` 与 `path_follower` 新增 `min_obstacle_range`
   参数（默认 0.5 m），过滤传感器原点附近的虚假 `tall`/`flat_ground` 检测（车体自身/
   地面杂波），作为一般性鲁棒性守卫，与第 7 条相互独立。

### v2.2 稳定性与转向平滑补丁（本版累计）

针对实车移植前的对接联调，额外补齐以下修复（均在 v2.2 分支内）：

1. **`use_sim_time` 统一**——`truth_perception` / `path_follower` / `frenet_planner` /
   `actuator_adapter` / `evaluator` 全部在 launch 中显式设 `use_sim_time: True`，
   与 Gazebo（`ros_gz_bridge` / `robot_state_publisher`）时间源对齐。此前未设的节点
   用墙钟时间发布位姿，与仿真时间错位导致 tf2 反复报 `jump back in time`、RViz 全屏
   闪烁、控制位姿混乱引发蛇形。详见[已知问题 #6](#6-多次运行残留-gazebo-实例导致-clock-冲突riviz-全屏闪烁--蛇形)。
2. **`_ground_derate` 缩进 bug 修复**——原 `_ground_derate`（特殊地面降速）误嵌在
   `_derate_speed_profile` 的 `return` 之后，成为不可达死代码且缩进错乱，导致
   `self._ground_derate(...)` 在收到 `flat_ground` 障碍时抛 `AttributeError` 使控制崩溃；
   已正确挂回类方法，并修正 `self.idle_speed` → `self._idle_speed` 笔误。
3. **DARE 重解脉冲消除 + 转向平滑**——`dare_solve_interval` 10 → **50**（重解周期
   0.5 s → 2.5 s）；新增**输出突变保护**层（阶跃超正常步长 2 倍时用 0.5× 步长截断）；
   收紧 `steer_lowpass_alpha` 0.35 → 0.22、`max_steer_rate` 1.5 → 1.2 rad/s、
   `state_lowpass_alpha` 0.5 → 0.45。弯道段转向变化率峰值由 ~105°/s 降至 ~50°/s 以内，
   无极端阶跃。详见[已知问题 #7](#7-转向平滑与-dare-重解脉冲) 与[参数说明](#参数说明)。

### v2.2 代码清理（屎山治理）

1. **删除遗留的 `core.py`（1077 行）**——算法实现早已迁移到 `core/` 子包
   （`controller.py` / `planner.py` / `track.py` / `geometry.py` / `terrain.py`），
   原 `core.py` 成为与子包**重复实现**的桥接残留，且内含 `stanley_path_control` /
   `lqr_path_control` 等**死代码**（仅被旧 import 引用，node 层从未调用）。
   删除后 `baja_cloud_sim.core` 自动解析到 `core/` 包，`core/__init__.py` 的
   re-export 已覆盖全部外部符号（`legacy_path_control`、`signed_lateral` 等），删除安全。
2. **拆分 `path_follower_node.py` 的 `_control` 超级方法（278 行）**——原方法揉合了
   finish watchdog、infeasible 守卫、LQR/纯追踪分发、纵向开环、安全状态机、转向输出
   等多职责，且 finish pre-check 时序耦合脆弱（大量"绕坑"注释）。
   拆分为四个职责清晰的方法：
   - `_guard_finish_and_infeasible()`：finish watchdog / pre-check / infeasible 计数 / 硬停车守卫
   - `_compute_lateral_command()`：LQR 主 + 纯追踪 fallback 横向控制
   - `_compute_longitudinal_target()`：速度剖面 → finish 限速 → 地形降速 → PID → 安全状态机 → 加加速度限幅
   - `_publish_actuation()`：转向平滑（低通 + 速率限幅 + 突变保护）+ 发布 Ackermann + lookahead marker
   - `_control()` 退化为编排层（约 35 行）。拆分后逻辑等价，已实跑验证无 AttributeError。

### v2.3 相对 v2.2 的变更

面向**实车移植前的控制鲁棒性验证**，在保持算法核心架构不变的前提下：

1. **EPS（电动助力转向）执行器模型**——`actuator_adapter_node` 新增 `_apply_eps`：
   - 齿条转速上限 `eps_max_rate_deg` = **15°/s**（拟合真实电动助力齿条，满打 35° 约 2.3 s）；
   - 死区 `eps_deadband_deg` = **1°**（消除小幅抖动/传感器噪声）；
   - 一阶滞后 `eps_tau` = **0.0**（关闭显式滞后，依赖 Gazebo `AckermannSteering` 插件
     自带的 ~80 ms 物理转向响应，避免与之一阶双重建模引入额外相位滞后导致蛇形）。
   - 最终 `angular.z = v · tan(δ_eps) / L` 发给 Gazebo。
   - 新增诊断话题 `/metrics/cmd_steer`（LQR 理想指令角）与 `/metrics/eps_actual_steer`
     （实际齿条角，Float32）；`evaluator_node` 记录 `eps_steer_rad` 列，
     `tools/plot_tracking.py` 的 `time_*.png` 叠加绿色实际齿条角与滞后带。
   - **仅仿真层**：实车用不同 launch 文件替换 `actuator_adapter_node` 即去除 EPS 建模。
2. **速度自适应反馈软化（β(v)）**——`core/controller.py` 的 `compute_lqr_steering`
   新增 β(v) 项：`β = 1 / (1 + α·(v − v_ref))`，饱和到 `[β_min, 1.0]`。高速时反馈项
   乘 β<1，小误差不再被过激放大成蛇形；低速（v ≤ v_ref）β=1.0 保持完整修正力。
   参数 `fb_speed_soften_alpha=0.3` / `fb_speed_ref=2.5` / `fb_speed_beta_min=0.5`。
   **该软化补偿真实齿条约束，实车仍需保留**（非仿真专属）。
3. **双向曲率前瞻速度剖面**——`core/speed_profile.py` 的曲率限幅由「当前点曲率」改为
   受 `[i−look, i+look]` 窗口内**最大曲率**约束（`curvature_lookahead_m`=3.0）：
   - 前瞻（i+look）：提前减速进弯（车还没到弯、前方有弯就压速）；
   - 后视（i−look）：延后加速出弯（车还在弯里/刚出弯、后方弯道曲率仍约束速度，
     离开弯道 3 m 后才允许提速），根除「弯切直时过早加速 → LQR 跟不上 → 蛇形」。
4. **三档速度分级**——`tier_slow_speed`=2.5 / `tier_normal_speed`=3.5 /
   `tier_fast_speed`=4.5 m/s（直线段分级），`speed_profile_max`=3.5 作为兜底；
   配合 β(v) 与双向前瞻，弯道与直道过渡更平稳。
5. **转向平滑参数回稳**——`max_steer_rate` 恢复为 1.2 rad/s（此前误设为 0.25 与 EPS
   形成双重限速、吃掉修正能力）；`steer_lowpass_alpha` 由 0.22 提至 0.5（减相位滞后，
   避免平滑吃掉纠偏高频分量）；`max_lateral_accel` 实测定为 1.2（1.6 自跑出界更重）。
6. **直道稳态高频抖动治理（K1 + K3，面向实车移植补齐）**——EPS 建模后直道段出现
   转向角高频往复突变（峰值已被 β(v) 压到 ±12.5° 以内，但突变频率未改善）：
   - **K1 — `e_psi` 移动平均**：`core/controller.py` 的 `compute_lqr_steering` 新增
     `e_psi_ma_window`（默认 5 步 ≈ 0.25 s）窗口移动平均；参考点跳变 / DARE 重解的
     `e_ψ` 阶跃在进入反馈前被平滑，从根上减少突变频率，不破坏弯道稳态。
   - **K3 — 指令端绝对死区**：`path_follower_node` 发布前对平滑后转向指令施加
     `cmd_steer_deadband_deg`（默认 1.5°）绝对死区（`< 阈值强制归零`），与 EPS 的
     1° 死区 + 0.5° 输出死区共同构成约 3° 小角度不响应区间，吃掉直道残余抖动；
     弯道正常转角（≥2°）正常突破。
   - 参数：`fb_speed_soften_alpha` 0.3→**0.6**、`fb_speed_beta_min` 0.5→**0.4**
     （进一步弱化高速直道反馈）；`eps_output_deadband_deg`=**0.5**（EPS 输出端绝对死区）。

> **EPS 引入后的调试历程（根因澄清）**：控制恶化不是 LQR 算法被改坏，而是 EPS 的
> 15°/s 转速上限首次被如实建模，使「LQR 指令立即到达」的假设失效。修复路径为：
> （a）控制器侧不重复限速（`max_steer_rate` 恢复 1.2）让修正指令完整到达 EPS；
> （b）β(v) 高速弱化反馈；（c）双向曲率前瞻防过早出弯加速；（d）低通系数提至 0.5；
> （e）K1（e_psi 移动平均）滤除参考点/DARE 跳变阶跃减突变频率 + K3（指令端绝对死区
> + EPS 输出死区）吃掉直道残余抖动。最终实跑验证弯道无蛇形、直道修正有力、抖动频率收敛。

### v2.4 相对 v2.3 的变更

面向**实车（Orin）移植**的关键对接调整——电机控制器自带速度闭环，上游不再重复做：

1. **纵向控制改为实车开环**——`path_follower_node._compute_longitudinal_target`
   删除仿真期的 PID 速度环（`a_cmd` 比例/积分/微分 + 坡度前馈积分）与 accel/jerk
   速率限制（`prev_target_speed` 每帧爬坡）。现逻辑为：
   `v_out = max(v_ref, idle_speed)`，直接作为 `/cmd_control.speed` 发出的**期望速度
   设定值**，由电机控制器自带速度闭环自行计算所需加速度/扭矩去追随。
   - 保留 `idle_speed` 怠速/蠕行保底（仅 `_finished` 或 finish 刹车进站时允许降到 0）；
   - 速度剖面、finish 限速、障碍/地形降速、安全状态机这些**决策层**全部保留（产出
     `v_ref`）——它们决定"该跑多快"，不属于执行器闭环，应留在算法核心；
   - 上游软件侧的 cascaded-PID 速度环参数（`lon_kp/lon_ki/lon_kd/lon_i_limit/
     lon_accel_step/lon_decel_step/slope_comp_gain`）已在 v2.7 清理中**彻底删除**（yaml
     声明、节点 `declare_parameter`、积分状态量、以及 `_compute_longitudinal_target`
     里的旁路分支全部移除），不再保留历史残留；如需仿真期 PID 跟随需重新实现。
   - **设计依据**：电机控制器速度闭环与 ROS 节点 PID 是串级关系，上游再做一层速度环
     会与底层叠加限幅，使加减速变肉、增益互相吸收；实车只需发设定值。
2. **感知开关 `use_perception`**——`simulation.launch.py` 新增启动参数：
   - `use_perception=true`（默认）：启动完整 LiDAR 感知组（`lidar_sim` + `gz_pcl_bridge`），
     `truth_perception.publish_ground_truth_boundary` 自动 `false`（车道线由 `road_analyzer` 发布）；
   - `use_perception=false`：不启动感知组，`truth_perception.publish_ground_truth_boundary`
     自动 `true`，由 `truth_perception` 直接发**车道线真值**（`/road_boundary_markers`），
     路径跟随规划-控制核心在无感知下也能跑（单独验证算法用）。
   - 用 `PythonExpression` 在节点 `parameters` 内按 `use_perception` 求值该开关（执行阶段
     才解析，规避 `SetLaunchConfiguration` 的时序/作用域坑）；`ros_gz_bridge` 始终保留
     （它还桥定位 `odom` 等，规划控制需要）。
3. **`run_test.sh` 默认关感知**——固定 `seed=0`/`obstacles=0` 的无障碍脚本现默认
   `use_perception=false`（专测规划控制），新增 `--with-perception` 可恢复完整感知。
   与 `run.sh` 并发互不干扰（独立 `GZ_PARTITION`）。
4. **`params.yaml` 显式化车道线真值开关**——`truth_perception_node.publish_ground_truth_boundary`
   显式写出（默认 `false`），实际由 launch 的 `use_perception` 动态覆盖。


### v2.5 相对 v2.4 的变更

在保持算法核心（横向 LQR + 前馈、纵向实车开环、EPS 仿真层模型）不变的前提下，
聚焦**工程化与可复现性**的增量改进：

1. **环境快照与恢复脚本**——新增 `env_snapshot.md`（当前系统依赖/环境快照记录）
   与 `env_restore.sh`（按快照一键恢复依赖环境），换系统/重装后无需从零排查依赖。
   配合 `install_ubuntu2204.sh` 使用，可在干净 Ubuntu 22.04 上快速重建开发环境。
2. **测试缓存清理**——`.gitignore` 增补 `.pytest_cache/` 与 `.claude/`，避免 pytest
   缓存与编辑器/agent 配置混入版本库；已跟踪的 `.pytest_cache` 目录已从索引中移除
   （本地文件保留）。
3. **安装/运行脚本与文档同步**——`install.sh` / `install_ubuntu2204.sh` /
   `start_remote_rviz.sh` 等与 `docs/`、`README.md` 的改动保持对齐，修正文档中
   过时的路径与说明（如测试章节的工作目录）。
4. **工具链同步**——`tools/plot_tracking.py`、`tools/offline_closed_loop.py`、
   `tools/steering_probe.py`、`tools/verify_eps.py` 等随核心参数/接口变更同步更新。

5. **感知开关**——`simulation.launch.py` 提供 `use_perception` 启动参数（源码默认 `true`：
   跑完整 LiDAR 感知链路；置 `false` 时自动将 `publish_ground_truth_boundary` 覆盖为 `true`
   发车道线真值，便于单独验证规划-控制核心）。本地可直接把该 `DeclareLaunchArgument` 的
   `default_value` 改为 `false`，让所有仿真默认关闭感知。

6. **弯道稳定性改进（VM→物理机退化的根因修复）**——物理机 DDS 调度抖动放大，原 4 m/s 进弯 +
   弯道半径 10 m 致 `a_lat≈1.6` 接近极限 → 转向饱和 ±26° 且 EPS 15°/s 跟不上 → 第三弯脱轨。
   在保持横向 LQR / 纵向开环 / EPS 模型结构不变的前提下，新增/调整以下增量：
   - **前向预减速** `pre_decel_lookahead_m=8.0` + `pre_decel_max=1.5`
     （`speed_profile.py`）：在 `plan_speed_profile` 的 backward pass 之后新增前向 pass，
     进弯前 8 m 起平滑降速，避免仅在弯前 3 m 急刹触发转向饱和。
   - **v_ref 稳定** `v_ref_lowpass_alpha=0.75`（`path_follower_node.py`）：用 s-弧长投影
     `_projected_index`（带 lookahead）替代全局欧氏 `_closest_index` 取参考速度，并对 `v_ref`
     做一阶低通，抹平 `planned_path` 10 Hz 刷新导致的 `v_ref` 锯齿/多峰。
   - **饱和降速（P1）**：转向指令达 `max_steering*0.95` 即标记 `saturated`，纵向目标 ×0.6，
     主动降低过弯速度需求。
   - **走廊自适应扩张（P2）**（`frenet_planner_node.py`）：车辆横向偏移 `>0.5 m` 时在窗口内对
     `left/right_limits` 扩张 0.5 m，提升扰动后恢复能力。
   - **参数联动**：`max_lateral_accel` 实际调为 `0.8`（回退自 1.6，1.6 自跑出界更重）；
     `curvature_lookahead_m=3.0`。**注意**：EPS 参数（`eps_max_rate_deg=15°/s` 等）为真实车
     硬件约束，**不在仿真调参范围内**。真车上车建议把 `max_lateral_accel` 进一步降到 0.5~0.6，
     或 `tier_normal_speed` 降到 2.5~3.0 以留安全裕度。

> 说明：v2.5 相对 v2.4 **改动了一部分控制/规划行为**（上述第 6 条的弯道稳定性增量），
> 但**未改动**算法核心结构（横向 LQR + 前馈、纵向实车开环、EPS 仿真层模型）与感知开关逻辑本身。

### v2.6 相对 v2.5 的变更

面向**仿真层与实车层融合到同一工作空间**，规划-控制-感知核心（`baja_cloud_sim` /
`perception`）零改动，仅通过 launch 话题 remap 对接实车硬件：

1. **供应商硬件包迁入 `src/hardware/`**——从原 `ros2_ws/src/` 整体搬入：
   - `chcnav`（华测组合导航，vcan2 发 `/chcnav/devpvt` + `/imu_yaw` + `/chcnav/odom`）；
   - `lslidar_ros2-master`（镭神激光雷达，发 `/cx/lslidar_point_cloud`，**编译需 `libpcap-dev`**）；
   - `radar_can_parser`（毫米波雷达，can0）、`ultrasonic_radar_driver`（超声波雷达，vcan3）；
   - `msg_interfaces`（自定义消息）；
   - `car_autonomous_pkg`：**拆解仅保留 CAN 桥接**——`can_bridge_node` + `can_manager` +
     `message_handler` + `vehicle_params` + `*.dbc`；原 `path_follower_node` / `obstacle_avoider` /
     `path_recorder` / `trajectory_visualizer` / `control_logger` 等规划控制节点全部删除，由
     `baja_cloud_sim` 替代；`setup.py` 的 entry_points 仅留 `can_bridge_node`，`package.xml` 精简依赖。
2. **核心节点零改动 + launch remap 对齐**——`path_follower` / `frenet_planner` / `evaluator`
   的订阅话题保持仿真标准名（`/gps/fix` / `/imu/yaw` / `/ground_truth/odom`），实车 launch 通过
   remap 把硬件话题（`/chcnav/devpvt` / `/imu_yaw` / `/chcnav/odom`）映射过来；`/cmd_control`
   （AckermannDriveStamped）与 `can_bridge_node` **天然对齐**，无需改动。仿真专用节点
   （`truth_perception` / `actuator_adapter` / `video_recorder` / `gz_pcl_bridge`）在实车 launch
   中不启动。
3. **新增实车运行入口与 launch**——
   - `run_real.sh`（实车自主）、`run_real_remote.sh`（实车视驾）、`run_record.sh`（实车路径录制）；
   - `real_car.launch.py` / `real_car_remote.launch.py` / `path_record.launch.py`；
   - `remote_simulation.launch.py`（仿真视驾，`remote_control_node` 仿真/实车共用）。
4. **新增 `csv_to_centerline_node` + `path_recorder_node`**——
   - `csv_to_centerline`：将赛前录制的最佳路径 CSV 航点转为 `/reference_centerline`
     （latched Path）喂给 Frenet 在线规划，兼容"先跑最佳线、再实时规划"的比赛流程；
   - `path_recorder`：订阅 `/gps/fix` + `/imu/yaw`，按 0.5 m 间距自动采点写 CSV（录制时与
     手柄/遥控器独立驱动车辆互不冲突）。
5. **`evaluator_node` 支持实车模式**——新增 `use_scenario` 参数（默认 `true`）：仿真保留原
   scenario_file 逻辑；实车置 `false` 时从 `/reference_centerline` + `/obstacle_markers` 实时
   获取中心线与障碍，转向反馈改订阅 `can_bridge` 发布的 `/vehicle_status`。
6. **参数分裂**——新增 `config/real_car_params.yaml`（无 `use_sim_time`、无 Gazebo 专用节点，
   含 csv_to_centerline/frenet/path_follower/evaluator/remote_control/can_bridge 参数）与
   `config/real_car.rviz`。
7. **删除冗余**——原 `ros2_ws/` 目录（硬件包已迁入 `src/hardware/`，用户已有备份）与
   `run_line.sh`（直线赛道逻辑已并入 `run.sh --finish-mode line`）均删除。
8. **感知开关重构（`use_perception` 拆为 `use_boundary` + `use_obstacle`）**——
   - 旧 `use_perception` 一个总开关拆分为两个**正交独立**开关：
     - `use_boundary`（默认 `true`）：`true` 用 `road_analyzer` 真实车道线（`/road_boundary_markers`）；
       `false` 时 Frenet 改用**中心线 ±half_width 兜底走廊**，不再依赖任何外部车道线消息源；
     - `use_obstacle`（默认 `true`）：`true` 启动障碍检测发 `/obstacle_markers`；`false` 时
       `frenet_planner` 不消费障碍消息（纯跟踪）。两者均可命令行（`--no-boundary`/`--no-obstacle`）
       或 yaml（`frenet_planner_node.use_obstacle`）便捷调整。
   - **移除 `truth_perception` 的 `publish_ground_truth_boundary` 真值边界发布**（含 `MarkerArray`
     发布者、`_boundary_marker` 方法及相关死 import）——车道线不再由真值节点中转，彻底消除
     "仿真传递冗余真值"与"和 road_analyzer 双发布冲突"问题；`truth_perception` 仅发定位/GPS/IMU/
     中心线/TF。
   - **Frenet 内部加无条件 ±half_width 兜底**：`_plan` 早返回不再因 `left/right_world` 缺失而 return；
     边界构建时若真实边界缺失或退化（半宽 `< min_half_width`，默认 1.5m），用中心线点自带
     `half_width`（闭环中段 3.75m / 直线 4.0m，fallback `default_half_width` 4.0m）沿法向 ±half_width
     展开。仿真实车统一，实车无 scenario 真值时也能稳跑。
   - `lidar_sim.launch.py` 拆出 `use_boundary`/`use_obstacle`，分别控制 `road_analyzer` 的 remap 与
     `obstacle_adapter`+`surface_detector` 的启动；`gz_pcl_bridge` 与 `lidar_sim` include 条件改为
     `use_boundary OR use_obstacle`。`run.sh`/`run_test.sh`/`run_real.sh`/`run_real_remote.sh` 同步
     替换为新开关（`run_test.sh` 默认 `use_boundary=false use_obstacle=false` 纯跟踪）。

> **融合原则**：核心算法与控制器（横向 LQR + 前馈、纵向实车开环、EPS 仿真层模型）保持不动；
> 平台差异（仿真 Gazebo 真值 vs 实车硬件传感器、Gazebo cmd_vel vs VCU CAN）**仅由 launch
> 与 `src/hardware/` 包裹**，不侵入规划-控制-感知核心。

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
