# 工程规范：参数真源与运行时调参

本文件是本工程的"参数规则"——回答两个问题：

1. **算法参数放在哪？** → yaml 顶层 `algorithm_defaults` 段，`core.py` 三个 dataclass
   的 30+ 字段是它的语义真源。
2. **怎么运行时切换节点行为？** → 节点功能开关统一命名为 `enable_<feature>`（默认
   `true`），关闭时退化为最简行为，通过 `ros2 param set` 在仿真运行中切换。

本规范分两步落地。第一步是 `algorithm_defaults` + `from_yaml`（v1.1-yaml 引入），
第二步是 `enable_*` 运行时开关（v1.1-yaml 引入 `enable_obstacles` 作为范式）。

## 一、算法参数真源：`algorithm_defaults`

### 1.1 文件位置

`src/baja_cloud_sim/config/params.yaml` 末尾的顶层块：

```yaml
algorithm_defaults:
  planner:           # core.PlannerConfig 字段
    horizon_m: 30.0
    layer_spacing_m: 1.0
    ...
  controller:        # core.ControllerConfig 字段
    target_speed: 2.5
    ...
  stanley_controller: # core.StanleyControllerConfig 字段
    kd_heading: 0.3
    k_stanley: 0.8
    ...
```

### 1.2 命名与结构规范

- **顶层分逻辑组**：每个 dataclass 一组。`planner` / `controller` / `stanley_controller`
  顺序按"基类→子类"排列。
- **字段顺序与 dataclass 严格一致**：便于对照阅读；新增字段时同步插入到 yaml 与
  dataclass 的同一位置。
- **不要求 snake_case / 单位后缀**（v1.1-yaml 决定不引入额外规范以保持改动最小）；
  字段名直接沿用 `core.py` 中 dataclass 的字段名。
- **注释只放在顶层块头部**（YAML 注释风格），不重复到每个字段，避免 yaml 维护负担。

### 1.3 加载工具：`load_algorithm_defaults`

`core.load_algorithm_defaults(source, key="algorithm_defaults")`：

- `source` 接受**文件路径**或**已解析 dict**；
- 优先调用 PyYAML（若已装），回退到**手写最小 YAML 解析器**（覆盖本项目用的
  块式语法与 `#` 注释，**不引入第三方依赖**到 `core.py` 的 stdlib-only 性质）；
- 任何失败（文件不存在、解析错误）→ 返回 `{}`，**不抛异常**。

### 1.4 构造入口：`from_yaml`

`core.py` 三个 dataclass 各加 `from_yaml(source, key, group, **overrides)`：

- 从 yaml 取对应子组，与 `overrides` 合并后调 `__init__`；
- 字段缺失 → 静默使用 dataclass 默认值；
- **保留** `PlannerConfig()` / `ControllerConfig()` / `StanleyControllerConfig()` 无参
  构造的现有用法——向后兼容；
- **当前状态**：基础设施已准备好（`core.py` 与 4 个 yaml 加载测试已合入），但
  `frenet_planner_node` / `path_follower_node` 仍按 v1.1 风格用 `PlannerConfig(...)`
  显式构造；**切换调用方留待后续子版本**。

### 1.5 调用示例

```python
# 当前风格（仍可用）
from baja_cloud_sim.core import PlannerConfig
cfg = PlannerConfig(horizon_m=30.0, safety_margin=0.25)

# yaml 覆盖风格（v1.1-yaml 之后推荐，调用方未切换）
from baja_cloud_sim.core import PlannerConfig
cfg = PlannerConfig.from_yaml("src/baja_cloud_sim/config/params.yaml")
# 字段缺失或文件不存在 → 静默回退到 dataclass 默认值

# overrides 优先于 yaml
cfg = PlannerConfig.from_yaml(
    "src/baja_cloud_sim/config/params.yaml",
    horizon_m=80.0,
    clearance_weight=15.0,
)
```

## 二、运行时调参：`enable_*` 范式

### 2.1 命名与默认值

- 节点功能类开关统一用 `enable_<feature>` 命名；
- 默认 `true`（功能开启）；
- 关闭时（`false`）退化为**最简行为**——通常以"发空消息 / 不构造 / 不输出"形式出现，
  订阅端自然降级到无该功能的状态。

### 2.2 第一个范例：`enable_obstacles`

`truth_perception_node` 加 `enable_obstacles`（默认 `true`）：

- `True`：照常发 `/obstacle_markers` 中的 obstacle MarkerArray。
- `False`：**不构造** obstacle marker，但**仍发空 MarkerArray**（`MarkerArray()`），
  让 `frenet_planner._obstacle_callback` 收到空列表，自然把 `self.obstacles = []`——
  彻底"无障碍物"状态。

切换命令（**无需重启**）：

```bash
ros2 param set /truth_perception_node enable_obstacles false
ros2 param set /truth_perception_node enable_obstacles true
```

### 2.3 yaml 中的写法

```yaml
truth_perception_node:
  ros__parameters:
    ...
    # v1.1-yaml-spec-test: runtime-tunable switch.
    # ros2 param set /truth_perception_node enable_obstacles false
    # makes the /obstacle_markers topic publish an empty MarkerArray,
    # simulating obstacle-free world without restarting the simulation.
    enable_obstacles: true
```

### 2.4 后续可加的开关（按需引入）

- `frenet_planner_node.enable_path_log`（已存在，等价范式）
- `frenet_planner_node.enable_road_boundaries`（post-v1.1 引入）
- `path_follower_node.enable_<diagnostic>`（post-v1.1 引入）
- 等等

## 三、与 ROS 节点参数的关系

`params.yaml` 现在分两类：

- **ROS 节点参数**（顶层各节点块下的 `ros__parameters`）：由节点 `declare_parameter`
  声明，启动时由 launch 注入，运行时可通过 `ros2 param set` 切换。代表"节点行为"。
- **`algorithm_defaults`**（顶层独立块）：由 `core.py` 在需要时按 dataclass 加载，
  改变 `core.py` 三个 dataclass 构造时的字段值。代表"算法参数"。

二者**不重叠**。ROS 节点参数里有几个是"算法参数"被 ROS 注入（如 `path_follower_node`
的 `target_speed` 既是 ROS 参数也对应 `ControllerConfig.target_speed`）——v1.1-yaml
之后切换调用方时，节点应该**优先用 `from_yaml`** 注入算法参数，让 ROS 参数段只承担
"运行时可调"职责。

## 四、未在本规范范围

- **节点里散落的硬编码常量**（如 `path_follower_node` 的 400 ms staleness、
  `truth_perception_node` 的 `perception_forward=34.0`、`frenet_planner_node` 的
  `half_width=4.0` 等）：未整理到 yaml；这是工程规范的**第三步**，分批引入。
- **场景几何硬编码**（如 `core.py` 里的 100 m 路线、15 m 半径 90°左转、4 个 hill
  位置等）：未整理到 yaml；重 build 场景才生效，暂缓。
- **速度/曲率剖面下放**（post-v1.1 BL-1）：本规范不涉及。
- **时间戳治理**（post-v1.1 BL-4）：本规范不涉及。

## 五、参考

- v1.1-mvp0（Stanley 控制律移植）：[`releases/v1.1-mvp0.md`](releases/v1.1-mvp0.md)
- v1.1-yaml（yaml 规范首个落地版本）：[`releases/v1.1-yaml.md`](releases/v1.1-yaml.md)
- v1.1 总体完成状态与待办：[`releases/v1.1-status.md`](releases/v1.1-status.md)
- v1.1 之后内容：[`releases/post-v1.1.md`](releases/post-v1.1.md)