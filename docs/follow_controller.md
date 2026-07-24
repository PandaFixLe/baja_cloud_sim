# 下层控制器：follow 函数与潘代码的原理

本文件讲解下层控制器（`path_follower_node`）的算法原理，以及潘
`path_follower_node_path3.py` 的贡献与移植到本工程 `core.py` 的过程。分层架构背景见
[`background.md`](background.md)。

## 一、几何核心：追一个目标点

下层控制器的几何核心始终是"追一个目标点"（pure pursuit 风格），自 v1.0 起到 v1.1 再到
v1.1-yaml 不变。在本工程里：

- 在路径上找距离当前位置 ≥ `lookahead_distance`（自适应版本会随速度放宽到 `5 m`）
  的第一个航点，作为目标点。
- 期望航向 `desired = atan2(东, 北)`（导航角约定，本工程 `path_follower_node.py`
  同样使用），误差 wrap 到 `[-π, π]`。

`legacy_path_control`（v1.0）只做这件事：纯航向 P 控制 + 按转角分档降速。它的优点是
简单稳健；缺点是弯道切内道、横向偏差大、对帧间路径摆动敏感。

## 二、Stanley 主项：横向偏差

```text
cte    = 当前位置到当前跟踪段的带符号距离（米）
v_safe = max(target_speed, min_spd)   # 防止低速除零
stanley = atan2(k_stanley · cte, v_safe)
```

- `atan2(k·cte/v)` 是 Stanley 控制器横向项的标准形式：高速时温和、低速时激进；
- `min_spd` 防止低速除零；本工程仿真默认 `target_speed = 2.5 m/s`，可直接复用。

在本工程里 CTE 由 `core.signed_lateral(point, reference)` 提供（左正右负），无需自己
实现几何投影。

## 三、PD + 阻尼（防振荡、防冲过头）

为了在 Stanley 主项之外再让跟踪稳定，潘的实车代码叠加了 PD 与两组阻尼项：

- **PD 主项**：`kp · error + kd · error_rate`，其中 `error_rate = (error − prev_error) / dt`。
- **cte_dot 阻尼**：`cte_dot = (cte − prev_cte) / dt`，对横向偏差变化率施加反向抑制。
- **yaw_rate 阻尼**：`(yaw − prev_yaw) / dt` 经低通滤波后作为角速度阻尼，避免出弯时
  车头惯性冲过头。

## 四、弯道自适应增强

- `_get_preview_curvature` 扫描前方约 `max(3 m, target_speed × 1.2 s)` 路径，按每米
  航向变化率的最大值估"前瞻曲率"；
- 阻尼项按前瞻曲率自增强：`scale = 1 + min(preview_curv × k, max_boost)`，弯越大
  阻尼越强。

移植到本工程后，前瞻曲率同时驱动**自适应最大转角**（弯越大允许转角越小）和**自适应
速度**（弯越大预判降速）。

## 五、转向输出平滑

- **动态最大转角**：按当前曲率限制允许的最大转向角，避免低速过激；
- **低通滤波**：`steering_filtered = α · raw + (1−α) · prev`；
- **速率限制**：单帧转向增量 `≤ max_steer_rate`（度/周期），防止突变；
- **按转角分档降速**：`>25° → 0.35x`、`>18° → 0.5x`、`>12° → 0.7x`、`>6° → 0.85x`。

## 六、自适应速度（前瞻减速）

- 反应式：按当前转角分档降速（同上）；
- 主动式：按前瞻曲率预判减速（`preview_curv > 0.15` 急弯降到 `0.3x` 等）；
- 终点减速：到终点 `< 3 m` 时按距离线性减速。

## 七、四轮转向与避障（本架构下应裁剪）

- 4WS：在 `four_wheel_steering_enabled = True` 且满足速度/转角阈值时，后轮同向按
  `rear_steering_ratio` 跟随前轮转角，并发布 `/rear_steering_cmd`。
- 避障：`/virtual_target` 触发，进入避障模式使用相对坐标虚拟目标点，含 5 s 与 30 s
  两级超时；恢复时禁用 4WS 并清空虚拟目标点。
- 本工程仿真为前轮 Ackermann，**4WS 保持参数门控但默认关**；**避障完全下移到
  replanner，follow 不再做**。

## 八、为什么这套原理与"分层架构"严丝合缝

1. **避障归 replanner**：replanner 给的路径已无碰撞 → follow 砍掉 `virtual_target`
   状态机，跟踪问题彻底从"避障模式/正常模式"切回"无条件跟随单一目标"。
2. **阻尼与滤波同时治 replan 抖动**：replanner 10 Hz 重规划，局部路径每帧轻微摆动，
   纯追点会产生转向 chatter。低通 `α` 与转向速率限制 `max_steer_rate` 正好吸收帧间
   抖动——这是分层结构里近端控制器的本职。
3. **前瞻曲率预判速度**：让下层控制器"看见"弯道并提前减速，符合"远端规划已知路况、
   近端控制负责执行"的分工；但 v1.1 仍由 follow 自算（接口不变），留待 post-v1.1
   由 planner 下发速度剖面。
4. **Stanley 主项 + 航向 P**：横向精度与稳定性双提升，特别适合急弯不切内道。

## 九、潘代码与本工程 `path_follower_node.py` 的对照

| 维度 | 本工程 `path_follower_node.py`（v1.0） | 潘 `path_follower_node_path3.py` |
| --- | --- | --- |
| 行数 | 约 112 行（含节点壳）| 约 788 行（全部内联）|
| 路径来源 | 在线 `/planned_path` | 离线 CSV（潘的输入）|
| 控制律 | 纯航向 P + 预瞄 + 分档降速（`legacy_path_control`） | Stanley 主项 + PD + 双阻尼 + 滤波 + 速率限制 |
| 避障 | 无（避障在 replanner）| `/virtual_target` 状态机（本架构下应砍）|
| 4WS | 无 | 有（仿真车不适用，默认关）|
| 表现契约 | 在线 `/cmd_control`、`/lookahead_point`、`/planned_path` 订阅 | 同名 `/cmd_control`（共享），但有 4WS 额外话题 |

**v1.1 的方向**：把潘控制律的"精华"（Stanley 主项 + 双阻尼 + 滤波 + 速率限制）移植进
`core.py` 的 `stanley_path_control` 纯函数，节点薄壳保留。表现契约不动；只在现有话题
内增加信息（如调试日志）。

## 十、v1.1 实际移植的内容

- `core.stanley_path_control`：Stanley 主项 + PD + CTE 变化率阻尼 + 航向角速度阻尼 +
  转向低通滤波 + 转向速率限制。可选自适应最大转角与自适应速度。
- `core.StanleyControllerConfig`：继承 `ControllerConfig`，新增 9 个 PD/Stanley/滤波/
  自适应字段。
- `core.StanleyState`：控制器有状态量（`prev_*`、`yaw_rate_filtered`、`last_nearest`、
  `initialized`），显式传入传出，便于单测。
- `core.augment_path_for_cte` / `core.preview_curvature`：纯函数辅助，公开给测试。
- `path_follower_node` 升级：声明 9 个新参数；在 `_control` 中按 `controller_mode`
  分派 legacy / stanley 调用；节点持有 `self.stanley_state`。

详细参数表、测试与未变契约见 [`releases/v1.1-mvp0.md`](releases/v1.1-mvp0.md)。

## 十一、对多人协作与 real/sim 复用的影响

- **接口冻结 = 独立测试的支点**：planner 与 follow 各以话题为界面，互不依赖内部实现，
  可各自单测与回归。录一段 `/planned_path` 回放喂 follow，可脱离整链联调。
- **planner CSV 日志 + 真实 CSV 输入同源**：v1.1 让 planner 写入 `planned_path_*.csv`
  作为旁路日志；post-v1.1 让 follow 的输入也支持"参考路径来自 CSV"以承接潘的原世界——
  real/sim 共用同一套控制律与跟踪逻辑。
- **Stanley 与 4WS 解耦**：Stanley 控制律不依赖 4WS；4WS 默认关时退化为纯前轮控制，
  不影响仿真车的执行链路与表现契约。