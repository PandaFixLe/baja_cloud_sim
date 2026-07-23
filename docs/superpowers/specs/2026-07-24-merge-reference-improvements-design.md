---
title: Merge baja_cloud_sim (2) reference improvements into current baja_cloud_sim
date: 2026-07-24
status: approved
---

# 把参考版（baja_cloud_sim (2)）的有用部分融合进当前版

## 背景

`/home/pandafixle/Desktop/baja_cloud_sim`（当前版，分支 `v2`）与
`/home/pandafixle/Desktop/baja_cloud_sim（2）`（参考版）是同一项目的两个
并行迭代版本。用户希望把参考版中"更好"的代码挑选性地整合进当前版，
同时保留当前版在 PID 控制器、车体尺寸、云端运行脚本上的强项。

比较工作已通过四个并行子代理完成，对比报告涉及所有源代码、配置、
启动文件、URDF/SDF、README 与安装脚本。

## 目标

1. 采纳参考版的**场景几何**（直行 + 90° 转弯 + terrain_height 起伏）。
2. 采纳参考版 `truth_perception_node.py` 的**定位噪声模拟**（严格超集）。
3. 修复跟踪线**残影**问题（多个 LINE_STRIP 来源的 marker lifetime / QoS）。
4. 修复参考版 **regression**：把 `core.segment_is_safe` 的四点角部扫描
   保留下来，避免回退到已知的"角落擦碰"bug。
5. 修复当前版 **bug**：`scenario_generator._obstacle_sdf` 的重复 `return`
   死代码。

## 非目标（明确不做）

- 不替换 `pid_path_follower_node.py` 为参考版的简单 P 控制器。
- 不引入视频录制节点 / ffmpeg 依赖。
- 不改变车体物理尺寸（保留 1.70 m × 1.50 m × 0.40 m，质量 252 kg）。
- 不重命名 `odometry` 话题为 `ground_truth_odometry`（保留契约）。
- 不增加 `recording_camera` 传感器。

## 决策摘要

| 主题 | 决策 |
|---|---|
| 场景几何 | 采纳参考版（直行 + 90° 转弯 + 起伏） |
| 车体尺寸 | 保留当前版（1.70×1.50×0.40 / 252 kg） |
| 控制器 | 保留当前 `pid_path_follower_node.py` |
| 跟踪线残影 | 修复 `/actual_path` / `/reference_centerline` / `/planned_path` |
| 定位噪声 | 采纳参考版 `truth_perception_node.py` 严格超集 |
| 视频录制 | 不引入 |
| 合并方式 | 逐文件 review，逐个确认后实施 |

## 文件级改动清单

| 文件 | 动作 | 说明 |
|---|---|---|
| `src/.../baja_cloud_sim/core.py` | 整合 | 取 RPY/quat 工具、`terrain_height`、`smooth_hill/circular_speed_bump`、参考版 `generate_centerline`（直行+90°转弯）、参考版 `generate_obstacles`（按地形 z 贴附）。**保留**当前版 `segment_is_safe` 四点扫描。`PlannerConfig.vehicle_length=1.70` 保持。 |
| `src/.../baja_cloud_sim/scenario_generator.py` | 整合 | 取真实法线、动态 `base_ground`、ogre2/阴影/天空、500Hz 物理频率。**保留** `_terrain_features`/段式碰撞、`--obstacles-config`/`--save-obstacles-config` CLI。**修复** `_obstacle_sdf` 重复 return。车身 M=252，起步 z 用 `start['z']+0.52`。 |
| `src/.../baja_cloud_sim/truth_perception_node.py` | 替换 | 整文件替换为参考版（严格超集）。 |
| `src/.../baja_cloud_sim/frenet_planner_node.py` | 保留+微调 | 保留解耦的 `visual_margin` 和暴露的可调参数。采纳 marker lifetime 缩短到 180 ms。强制 `vehicle_length=1.70`。 |
| `src/.../baja_cloud_sim/pid_path_follower_node.py` | 保留+定点修复 | 不替换。修复：`dt` 改用真实耗时（不再硬编码 0.05）；`kd_heading` 调到 0.10–0.15；`steering_alpha` 上调到 ~0.85；Stanley 分母改用实际下发的速度。 |
| `src/.../baja_cloud_sim/path_follower_node.py` | 保留 | 仅作为 fallback，不主动使用。 |
| `src/.../baja_cloud_sim/evaluator_node.py` | 修复残影 | 审查 `/actual_path` 发布：`header.stamp`、`qos durability`、`frame_id`、pose buffer 清理；必要时改为 RELIABLE + VOLATILE（或 latched + clear-on-reset）。 |
| `src/.../baja_cloud_sim/actuator_adapter_node.py` | 不动 | 两版本字节一致。 |
| `src/.../models/baja_vehicle/model.sdf` | 保留+小补 | 保留车体尺寸与质量。仅采纳 `<steer_p_gain>18.0</steer_p_gain>`。不增加 recording_camera。 |
| `src/.../urdf/baja_vehicle.urdf` | 不动 | 已与 SDF 一致。 |
| `src/.../launch/simulation.launch.py` | 保留+小补 | 保留 `pid_path_follower` 节点启动。增加 `truth_perception` 节点的 `localization_*_stddev_*` 参数透传。不引入 video_recorder。 |
| `config/params.yaml` | 更新 | `truth_perception` 加 σ 字段；`frenet_planner` 用参考版 12.0/1.2 默认 + `vehicle_length=1.70`；`pid_path_follower` 保留且同步控制参数；不引入 video。 |
| `config/bridge.yaml` | 保留+加新桥 | 保留当前 `odometry` 桥。增加 `/wheel_odom` 桥。不增加 recording_camera 桥。 |
| `config/simulation.rviz` | 保留 | 当前版更完整（9.5KB）。 |
| `run.sh` | 保留 | pkill + 软件渲染环境变量对云端运行有用；不引入 ffmpeg preflight。 |
| `install_ubuntu2204.sh` | 小补 | 采纳参考版的幂等 apt 源判断；不增加 ffmpeg。 |
| `setup.py` | 小补 | `data_files` 过滤 `__pycache__/.pyc/.pyo`。保留 pid_path_follower 入口点。 |
| `package.xml` | 小补 | 版本号升到 1.1.0；不增加 ffmpeg 依赖。 |
| `worlds/` | 删除空目录 | 实际不使用，脚手架遗留。 |
| `README.md` | 重写 | 反映合并后的能力与参数。 |

## 关键 Bug 修复

1. **`core.segment_is_safe` regression**（参考版引入）：参考版把
   `segment_is_safe` 简化为对每个采样点做 `point_to_oriented_box_clearance`，
   丢失了当前版四点角部扫描逻辑。这会导致已知 bug——车辆后侧角擦碰
   障碍物。当前版 4 角测试必须保留。详见当前版 `core.py` 119-161 行的
   注释块。

2. **`scenario_generator._obstacle_sdf` 重复 return**：当前版在函数末尾
   有两个连续的 `return f"""..."""` 语句，第二个不可达。在合并时
   删除第二个。

## PID 控制器定点修复

参考版没有 PID 版本，但当前版存在两类潜在问题：

- `dt = 0.05` 硬编码（第 327 行附近）会随定时器抖动注入 D 项噪声。
- `kd_heading = 0.3` 偏高，配合硬编码 dt 容易导致蛇形。

修复策略：

1. 把 `dt` 改为 `(now - self.prev_time).nanoseconds / 1e9`，
   保存 `self.prev_time`。
2. 把 `kd_heading` 默认值调到 0.10–0.15 之间。
3. 把 `steering_alpha` 上调到 ~0.85 增强平滑。
4. Stanley 分母改用 `self.commanded_speed` 而不是 `self.target_speed`。

## 残影修复目标

三个 LINE_STRIP 来源需逐个审查：

- `evaluator_node.py` 的 `/actual_path`：累积 3000 pose 缓冲。
- `truth_perception_node.py` 的 `/reference_centerline`、边界 markers。
- `frenet_planner_node.py` 的 `/planned_path`。

修复方向：

- 明确 `header.stamp` 为发布时刻（避免一直使用 latched 旧 stamp）。
- 设置 `marker.lifetime` 显式过期时间（参考版用 180 ms）。
- QoS：`/actual_path` 建议 `RELIABLE + VOLATILE + KEEP_LAST(1)`，
  配合 RViz 显示周期，避免残影。

## 验证与回归保护

**修改前**：

- 备份关键文件到 `runtime/pre_merge_backup/`。
- 确认 `git status` 干净；建议先 commit 当前 working tree 作为基线。

**修改中**：

- 每个文件改完立即 `colcon build --packages-select baja_cloud_sim`。
- 单元测试（`test/` 目录）按文件跑通过。

**修改后**：

- `bash run.sh --seed 42`：完整端到端运行。
- RViz 验证三条线**干净无残影**：
  - 一条 `evaluator /actual_path` 跟随轨迹线；
  - `truth /reference_centerline`、`frenet /planned_path` 不应有重影。
- 场景视觉验证：直行段 + 90° 转弯 + 起伏路面（不再正弦弯曲）。
- 残影修复验证：运行中 `Ctrl+C` 重启，确认不会留下幽灵线。
- PID 调参验证：~2.5 m/s 通过弯道，轨迹平滑无蛇形。
- 定位噪声验证：`ros2 topic echo /localization/odom --once`
  应看到非零位姿协方差。

**回滚策略**：单文件回滚，源为 `runtime/pre_merge_backup/`。
不整体回滚。

## 风险

| 风险 | 缓解 |
|---|---|
| 参考版 `core.generate_centerline` 输出结构变化影响 `scenario_generator.py` 解析 | 同步检查 `scenario_generator.py` 中对 `centerline` 字段的依赖；如有不一致，作为同一组改动一起应用 |
| 定位噪声 σ 默认值与 PID 控制器相互作用导致控制不稳 | 修改后先做 PID 闭环仿真；若不稳，临时把 σ 调到极小验证不是噪声引入的问题 |
| `/actual_path` 残影修复引入 frame / QoS 不兼容 | 严格遵循参考版的 RELIABLE+VOLATILE 模式；测试两个版本同时订阅同一话题 |
| 500 Hz 物理频率对部分云端 GPU 不够强 | 当前版用 200 Hz 也能跑；如有问题，按 `max_step_size` 调回 0.005 |

## 不在本次范围

- 多车协同、自动驾驶完整闭环测试、性能基准对比。
- 新增控制器（例如 MPC、纯追踪以外的方案）。
- 重写 `truth_perception_node.py` 的仿真物理模型。
- 任何对 URDF 拓扑结构的修改。