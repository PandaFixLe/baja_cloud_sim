# Frenet 局部规划器说明

本文件说明本项目规划核心——Frenet 局部路径规划器的坐标基础、在自动驾驶中的作用、与百度
Apollo EM Planner 的关联，以及在本项目中的简化形式与运行时逻辑结构。实现分布在
`src/baja_cloud_sim/baja_cloud_sim/core.py`（纯几何与规划算法）和
`src/baja_cloud_sim/baja_cloud_sim/frenet_planner_node.py`（ROS 2 节点）。

## Frenet 坐标

Frenet 坐标（道路坐标系、SL 坐标系）是一种沿参考线定义的曲线坐标系，把车辆位置从直角坐标
`(x, y)` 换成两个沿路的量：

- **s（纵向）**：沿参考线（车道中心线）走过的弧长，表示"沿路走了多远"；
- **l（横向）**：车辆到参考线的带符号垂直距离，表示"偏离中心线多少、偏左还是偏右"。

坐标基底是参考线上每一点的切向量与法向量，即随曲率变化的移动标架。本项目用以下纯几何函数
完成两坐标系之间的换算：

- `signed_lateral(point, reference)`：世界坐标点投影为横向偏移 `l`，即
  `-sin(yaw)·dx + cos(yaw)·dy`；
- `frenet_to_world(reference, lateral)`：由参考点与横向 `l` 还原世界坐标 `(x, y)`；
- 参考线上每个点的 `s` 为累积弧长，`generate_centerline` 按 0.5 m 采样累加得到。

## 在自动驾驶中的作用

直角坐标下"贴着弯道行驶"是一条复杂曲线，难以描述与优化；换到 Frenet 坐标后：

1. **降维解耦**：弯曲车道被"拉直"为 s 轴，行为分解为纵向决策（沿 s：跟车、加减速、停车）与
   横向决策（沿 l：居中、避障、换道）两个近乎独立的子问题，分别求解再合成。
2. **语义直观**：`l = 0` 即车道正中，`|l|` 即偏移量；道路边界、障碍物、车道宽度都能用
   `(s, l)` 简洁表达，代价函数（居中、贴边惩罚、平顺）写法自然。
3. **与曲率无关**：无论直道还是急弯，横向采样与边界约束写法一致，算法不必区分路形。
4. **便于约束与安全**：车宽与安全间距可直接换算为 l 方向的可行区间（本项目
   `bounds = [right + 半车宽, left - 半车宽]`），可行走廊一目了然。

## 与 Apollo EM Planner 的关联

本项目规划器可视为百度 Apollo EM Planner 思想的极简教学版：保留"参考线 + Frenet + 分层
动态规划选路 + 多项代价"的骨架，砍掉二次规划平滑、ST 速度规划与时空障碍投影，因此可以只用
Python 标准库运行。对照关系如下：

| Apollo EM Planner | 本项目 |
| --- | --- |
| 基于参考线构建 Frenet 坐标 | `centerline` + `signed_lateral` / `frenet_to_world` |
| E 步 + M 步：在 SL / ST 两投影上迭代优化 | 仅做 SL（横向）一层，无 ST 速度规划 |
| DP path（撒点选粗解）→ QP path（二次规划平滑） | 仅保留 DP path：分层撒点 + 动态规划选路，无 QP 精修 |
| DP speed → QP speed（速度规划） | 无速度规划，纵向速度交由下游 `path_follower` 按转角降速 |
| 平滑、向心加速度、避障、参考线偏移等多项代价 | 保留居中、贴障惩罚、平顺/坡度三类代价 |
| 障碍投影到 ST 图做时空避障 | 仅做静态几何避障（`point_to_oriented_box_clearance` + 膨胀框），无时间维 |

## 本项目的简化形式

规划核心 `plan_frenet_path` 是一个分层 Frenet 格（layered lattice）加动态规划的横向路径
搜索，流程如下：

1. **纵向分层**：沿参考线从 `start_index` 起，每 `layer_spacing_m = 1.0 m` 取一层，直到
   `horizon_m = 30 m`，形成一列"关卡"。
2. **每层横向撒点**：由 `left_limits` / `right_limits` 减去半车宽得到可行区间
   `[lower, upper]`，再按 `lateral_spacing_m = 0.25 m` 离散出候选横向偏移 `l`。
3. **节点代价**：居中项 `center_weight·l²`，加贴障/贴边惩罚
   `clearance_weight·max(0, desired_clearance − 间距)²`；间距取"到障碍膨胀框"与"到边界"的
   最小值。
4. **转移代价与约束**：层间连边时限制单步横向变化 `≤ max_lateral_step (0.9 m)`，用
   `segment_is_safe` 对连线采样做碰撞检查；转移代价含平顺项 `smooth_weight·Δl²` 与坡度项
   `slope_weight·|Δl|`。
5. **动态规划求全局最优**：逐层递推累积代价与父指针，末层取最小代价并回溯，得到最优横向
   序列。
6. **回投与评估**：`frenet_to_world` 将每层选中的 `l` 还原为世界坐标路径点，统计
   `min_clearance`，返回 `PlanResult`（含 `feasible`、`planning_ms`、`reason`）。

若任一层走廊比车身还窄，或找不到无碰撞连接，则返回 `feasible = False` 并给出 `reason`。

## 运行时逻辑结构

`frenet_planner_node.py` 采用"多路异步订阅存快照 + 定时器周期规划"的结构：

```text
订阅回调（各自异步，仅更新最新值，不使用时间戳）
  ├─ /reference_centerline  → _centerline_callback → self.centerline（含 s、yaw、half_width）
  ├─ /gps/fix               → _gps_callback        → self.position（经纬度 → 局部 xy）
  ├─ /imu/yaw               → _yaw_callback         → self.yaw_world（导航角 → 世界角）
  ├─ /road_boundary_markers → _boundary_callback    → self.left_world / right_world（base → world）
  └─ /obstacle_markers      → _obstacle_callback    → self.obstacles（带朝向的框）

定时器 10 Hz（0.10 s）→ _plan()
  1. 前置检查：position / centerline / 左右边界是否齐备
  2. nearest_index：从上次最近点附近增量搜索当前所在的参考线索引
  3. 由 left_world / right_world 投影出每层的 left_limits / right_limits（可行走廊）
  4. 调用 plan_frenet_path(...) → PlanResult
  5. 发布：
       /planner/status             FEASIBLE / INFEASIBLE:reason
       /metrics/planning_ms         规划耗时
       /metrics/planned_clearance   最小间距
       /planned_path                nav_msgs/Path（每点由 yaw 转四元数）
  6. _publish_debug：障碍膨胀框 CUBE 与状态文字 Marker（RViz 可视化）
```

关键设计点：

- **规划与感知解耦**：回调只负责存最新值，规划固定 10 Hz 触发并使用当前快照计算；结构简单，
  但也是时间戳/时间同步隐患的根源，详见 `README.md`"已知问题：Frenet 规划器的时间戳与时间
  同步"一节。
- **增量最近点**：以 `self.last_nearest` 缓存，`nearest_index` 从 `last_nearest − 4` 起搜，
  避免每帧全局扫描。
- **只出路径、不出速度**：节点仅发布 `/planned_path`，纵向速度由 `path_follower` 决定，与
  "砍掉 ST 速度规划"一致。
- **可行性显式反馈**：不可行时发布 `INFEASIBLE:<reason>`，并在 RViz 中以红色 `STOP` 文字
  提示，便于闭环调试。
