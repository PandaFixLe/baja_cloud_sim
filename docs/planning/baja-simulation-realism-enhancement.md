# 提升 Baja 仿真环境赛道真实感 — 四层渐进方案

> **状态**: 规划中 (Planning)  
> **日期**: 2026-07-27  
> **版本**: v1.1

---

## Context

当前仿真环境是一个 100m 单弯道（50m 直道 + 90° 左转 + 直道）的简化赛道，车辆无悬挂、纯运动学速度控制、均匀路面摩擦。需要逐步提升仿真环境的真实感以更贴合 Baja SAE 赛车场地。

经过对 `model.sdf`、`scenario_generator.py`、`core.py`、`actuator_adapter_node.py` 等关键文件的深入分析，按缺失严重程度梳理如下：

| 优先级 | 缺失项 | 当前状态 |
|--------|--------|----------|
| **致命** | 无悬挂系统 | 车轮刚性连接，仅靠 ODE 接触刚度 |
| **高** | 单弯道 100m 赛道 | 硬编码直道+单左转 |
| **高** | 无动力总成/轮胎模型 | 纯运动学速度控制 |
| **高** | 均匀路面摩擦 | 整条路 mu=1.25 |
| **中** | 地形特征有限 | 仅 2 坡+2 减速带 |
| **中** | 无赛道外环境 | 8m 宽路面外是平坦绿面 |
| **中** | 无赛道标识 | 无锥桶、发车线、计时点 |

---

## 方案总览

| 层级 | 名称 | 工作量 | 核心改动文件 |
|------|------|--------|-------------|
| **Tier 1** | 路面变化 + 地形增强 + 赛道外环境 | 3-5 天 | `core.py`, `scenario_generator.py`, `params.yaml` |
| **Tier 2** | 悬挂系统 + 动力总成 + 轮胎模型 | 7-10 天 | `model.sdf`, `actuator_adapter_node.py`, `core.py` |
| **Tier 3** | 多弯赛道生成 + 计时门 + 多圈 | 7-10 天 | `core.py`, `scenario_generator.py`, `evaluator_node.py` |
| **Tier 4** | 天气 + 传感器噪声 + 多车 | 10-15 天 | 多个文件 + 新增 2 个节点 |

每层独立可落地，后层叠加前层。建议从 Tier 1 开始快速见效，根据需求逐步推进。

---

## Tier 1: 路面变化 + 地形增强 + 赛道外环境

**目标**：在现有 100m 单弯赛道框架内，大幅提升视觉和行为真实感。

**工作量**：3-5 天

### 改动内容

#### 1. `src/baja_cloud_sim/baja_cloud_sim/core.py` — 扩展地形功能

- `terrain_height()` 新增 `lateral` 参数，支持横向变化的地形特征
- 新增 3 种地形特征函数：
  - `_whoops_section()` — 正弦搓板路（washboard），振幅指数衰减
  - `_rut_feature()` — 沿赛道方向的浅车辙（高斯凹陷，深 3-6cm）
  - `_rock_garden()` — 稀疏随机低矮石块（高 2-5cm）
- 新增 `surface_friction(s, lateral)` 函数，返回 `(mu, mu2)`，定义摩擦区：
  - 0-20m: 硬土 (`mu=1.35`)
  - 20-35m: 松碎石 (`mu=1.05`)
  - 35-55m(弯道): 压实土 (`mu=1.30`)
  - 55-70m: 泥泞段 (`mu=0.90`)
  - 70-100m: 硬土 (`mu=1.25`)

#### 2. `src/baja_cloud_sim/baja_cloud_sim/scenario_generator.py` — 生成多样化场景

- `_write_obj()` 按摩擦区分段生成子网格，每个区作为独立 `<model>` 写入 SDF，各自配置不同 `<surface><friction>` 值
- 新增 `generate_track_markers()` 函数，沿左右边界每 5m 放置橙色锥桶（圆柱体 collision + visual）
- 新增 `off_track_grass` 地面模型：`mu=0.85, mu2=0.75`，草绿色，造成冲出赛道时的摩擦力惩罚
- 在 `scenario.json` 中增加 `surface_zones` 和 `track_markers` 元数据

#### 3. 新增 `src/baja_cloud_sim/baja_cloud_sim/surface_monitor_node.py`

- 轻量级 ROS2 节点（~60 行），接收 ground truth odometry
- 查找最近 centerline 索引，判断横向偏移是否超出 half_width
- 发布 `/surface/type`（String 消息）：`"dirt"`, `"gravel"`, `"mud"`, `"grass"`, `"off_track"`

#### 4. `src/baja_cloud_sim/baja_cloud_sim/path_follower_node.py`

- 订阅 `/surface/type`，在 `_control()` 速度计算中集成 `off_track_speed_factor: 0.55`

#### 5. `src/baja_cloud_sim/config/params.yaml`

- 新增 `off_track_speed_factor: 0.55`
- 新增 `off_track_friction_threshold: 1.0`

### 风险

- 摩擦区分段网格增加 SDF 模型数量（4-5 个），ODE 碰撞对略有增加但在可控范围
- `terrain_height()` 签名变更需同步更新 `test_core.py` 测试

---

## Tier 2: 悬挂系统 + 动力总成 + 轮胎模型

**目标**：修复最致命的物理缺失 — 增加四轮独立悬挂、扭矩驱动替代运动学速度控制、滑移相关摩擦。

**工作量**：7-10 天

### 改动内容

#### 1. `src/baja_cloud_sim/models/baja_vehicle/model.sdf` — 重构车辆物理模型

**新增 4 个悬挂连杆**（`suspension_fl/fr/rl/rr`），每个 3-5kg

**新增 4 个棱柱关节**（prismatic joint），提供弹簧-阻尼悬挂：

```xml
<joint name="suspension_fl_joint" type="prismatic">
  <axis><xyz>0 0 1</xyz>
    <limit><lower>-0.075</lower><upper>0.075</upper></limit>
    <dynamics>
      <spring_stiffness>40000</spring_stiffness>    <!-- N/m -->
      <damping>3000</damping>                       <!-- N·s/m -->
    </dynamics>
  </axis>
</joint>
```

- 前悬参数：35-45 N/mm 弹簧，行程 ±75mm（共 150mm）
- 后悬参数：50-60 N/mm 弹簧（后驱需更高刚度）

**移除 AckermannSteering 插件**（lines 401-424），替换为：
- 4 个 `JointVelocityController` 插件（或 `JointForceController`，每个后轮独立控制）
- 2 个 `JointPositionController` 插件（前轮转向关节）
- 保留 `OdometryPublisher`（lines 425-434）提供地面真值里程计

**运动链重构**：
- 当前：`base_link → wheel`（刚性 revolute joint）
- 新版：`base_link → 棱柱关节(suspension) → 旋转关节(wheel)`

#### 2. `src/baja_cloud_sim/baja_cloud_sim/core.py` — 新增动力总成模型

新增 `PowertrainConfig` 数据类：
```python
@dataclass
class PowertrainConfig:
    max_engine_torque: float = 65.0      # Nm at crank
    gear_ratio: float = 8.5              # CVT + final drive combined
    drivetrain_efficiency: float = 0.88
    rolling_resistance_coeff: float = 0.03  # Crr for dirt tires
    drag_coeff: float = 0.7
    frontal_area: float = 0.55           # m^2
    wheel_radius: float = 0.292
```

新增 `compute_wheel_torque(target_speed, current_speed, config)` 函数：
- PI 扭矩控制器 + 滚动阻力补偿（`Crr * m * g * r`）+ 空气阻力补偿（`0.5 * rho * Cd * A * v^2 * r`）
- 扭矩限制 ±`max_engine_torque * gear_ratio`

新增 `slip_dependent_friction(slip_ratio, peak_mu, slide_mu)` 函数：
- 简化滑移曲线：0-12% 线性上升到峰值摩擦 → 12-30% 线性下降到滑动摩擦 → >30% 全滑动

#### 3. `src/baja_cloud_sim/baja_cloud_sim/actuator_adapter_node.py` — 重大重写

- **不再发布单一 `Twist`**，改为发布到 4 个独立话题：
  - `/model/baja_vehicle/rear_left_velocity`（或 torque）
  - `/model/baja_vehicle/rear_right_velocity`（或 torque）
  - `/model/baja_vehicle/front_left_steering_position`
  - `/model/baja_vehicle/front_right_steering_position`
- 后轮差速器公式：
  ```
  omega_left  = speed * (1 - track/2 * tan(steering)/wheelbase) / wheel_radius
  omega_right = speed * (1 + track/2 * tan(steering)/wheelbase) / wheel_radius
  ```
- 前轮 Ackermann 几何：
  ```
  delta_left  = atan(wheelbase / (wheelbase/tan(delta) - track/2))
  delta_right = atan(wheelbase / (wheelbase/tan(delta) + track/2))
  ```
- 新增参数：`use_torque_control` (bool), `track_width` (1.32), `wheel_radius` (0.292)
- 从里程计回调获取实际轮速，用于滑移计算和闭环扭矩控制

#### 4. `src/baja_cloud_sim/config/params.yaml` + `bridge.yaml`

- 扩展 `actuator_adapter_node` 参数段
- 新增 bridge 通道覆盖 4 个独立 joint 话题

### 风险

- **棱柱关节悬挂稳定性**：ODE 0.002s 步长下弹簧可能振荡。缓解：提高接触 kp 至 1e6，kd 至 300，必要时降低实时因子
- **移除 AckermannSteering 插件**：失去 `wheel_odometry` topic，但当前代码无节点消费该 topic（全部使用 `ground_truth_odometry`）
- **扭矩控制 PI 增益需调优**：初始 P=30.0 为估计值，需从 P-only 开始递增到轻微超调再加积分项
- **Gazebo JointController 插件**：为 Gazebo Sim (Ignition) 内置插件，确认版本兼容

---

## Tier 3: 多弯赛道生成 + 计时门 + 多圈

**目标**：用程序化多弯赛道生成器替代硬编码单弯，支持 S 弯、减速弯、发卡弯；增加计时点和多圈比赛。

**工作量**：7-10 天

### 改动内容

#### 1. `src/baja_cloud_sim/baja_cloud_sim/core.py` — 新增 `generate_track()` 函数

新增 `TrackSpec` 数据类：
```python
@dataclass
class TrackSpec:
    total_length: float = 500.0       # meters
    min_turn_radius: float = 8.0      # tightest turn
    max_turn_radius: float = 25.0     # widest turn
    max_turn_angle_deg: float = 150.0 # up to hairpin
    straight_min: float = 15.0        # min straight between turns
    straight_max: float = 60.0        # max straight
    chicane_probability: float = 0.3  # chance of chicane vs single turn
    variable_width: bool = True
```

`generate_track(spec, spacing, seed)` — 基于种子的可复现赛道生成：
- 随机排列 `StraightSegment` 和 `TurnSegment`（含左右方向）
- 减速弯（chicane）实现：`TurnSegment(radius, -angle, dir)` → 短直道 → `TurnSegment(radius, angle, -dir)`
- 变量宽度（弯道处高斯收窄）从现有代码保留复用
- 保持旧版 `generate_centerline()` 作为向后兼容封装（调用 `generate_track` 传入 100m 单弯参数）

#### 2. `src/baja_cloud_sim/baja_cloud_sim/scenario_generator.py` — 赛道参数化

- 新增 CLI 参数：
  ```bash
  --track-length 500.0     # 赛道总长
  --track-seed 42          # 可复现随机种子
  --laps 1                 # 圈数
  --track-complexity medium  # easy/medium/hard
  ```
- `_write_obj()` 扩展处理任意长度赛道（法线计算已与方向无关，天然支持左右弯）
- 新增 `generate_gates()` 函数：每 50m 布置计时门（s, x, y 元数据存入 scenario.json）
- 更新 `scenario.json` 包含：`track_spec`, `lap_count`, `gates`

#### 3. `src/baja_cloud_sim/baja_cloud_sim/evaluator_node.py` — 多圈计时

新增 `__init__` 状态：
```python
self.current_lap = 1
self.lap_times = []        # [lap1_time, lap2_time, ...]
self.lap_start_time = self.started
self.gate_times = {}       # {gate_id: pass_time}
self.last_gate_passed = -1
self.total_laps = self.scenario.get("lap_count", 1)
```

圈数检测逻辑：当 vehicle progress 从 ~100% 回绕到 ~0% 且 `center_index < 10` 时判定一圈完成

CSV 输出新增列：`lap_num`, `lap_time_s`, `best_lap_s`, `gate_last_passed`

RViz 文字叠加显示圈数、圈时、最佳圈

#### 4. `src/baja_cloud_sim/baja_cloud_sim/frenet_planner_node.py` — 多圈规划

- 当 `last_nearest` 接近赛道末尾（horizon 覆盖不足）且 `total_laps > 1` 时，将规划 horizon 延伸到下一圈起点
- `horizon_m` 从 30m 增大到 40m（适应更长赛道）

#### 5. `src/baja_cloud_sim/config/params.yaml`

```yaml
evaluator_node:
  ros__parameters:
    total_laps: 3
    gate_spacing: 50.0

frenet_planner_node:
  ros__parameters:
    horizon_m: 40.0
```

### 风险

- **赛道自交**：程序化生成可能产生交叉。缓解：生成时 O(n²) 碰撞检测拒绝自交段（200-500 点量级 negligible）
- **多圈规划回绕**：Frenet planner 假设非回绕 centerline。缓解：unroll 为 N*lap_length 的线性数组
- **高速下 Stanley 增益可能需要重调**：`steering_alpha` 和 `max_steer_rate_deg` 可能需调整

---

## Tier 4: 天气 + 传感器噪声 + 多车竞速

**目标**：增加环境动态性、传感器真实性、多车对抗场景。

**工作量**：10-15 天

### 改动内容

#### 1. `src/baja_cloud_sim/baja_cloud_sim/scenario_generator.py`

天气配置写入 `scenario.json`：
```json
{
  "weather": {
    "type": "clear|overcast|light_rain|heavy_rain",
    "fog_visibility_m": 150.0,
    "time_of_day_h": 14.0,
    "dynamic_weather": false,
    "rain_friction_scale": 0.75
  }
}
```

基于太阳角度的光照参数：
- 日出日落亮度正弦模型
- 环境光/漫反射/方向根据太阳高度角计算

多车支持：`<include>` 多辆车辆，错开发车（5m 间距），各自独立命名空间

#### 2. `src/baja_cloud_sim/baja_cloud_sim/truth_perception_node.py`

- **IMU 偏置漂移**：随机游走模型（`bias_drift = 0.0001` per tick）
- **GPS 树冠遮挡**：虚拟"树区"内位置标准差增大 3-5 倍
  ```python
  for tree in self.scenario.get("trees", []):
      if math.hypot(x - tree["x"], y - tree["y"]) < tree["radius"]:
          tree_factor = 3.5
  ```
- 发布 `/weather/state` 话题

#### 3. `src/baja_cloud_sim/baja_cloud_sim/video_recorder_node.py`

后处理特效（OpenCV 实现）：
- **雨痕**：随机斜线叠加（透明度 ∝ 雨强）
- **雾化**：像素向灰白色混合（距离依赖或均匀）
- **相机抖动**：基于车辆垂直加速度的小随机偏移

#### 4. 新增 `weather_controller_node.py`（~80 行）

- 读取 scenario 天气配置
- 发布 `/weather/state`（rain_intensity, fog_level, time_of_day）
- 动态天气：定时器驱动状态机切换

#### 5. 新增 `multi_vehicle_manager_node.py`

- 发布对手车辆位置到 `/opponent_positions`（`MarkerArray`）
- 跟踪车辆间差距时间，发布到 `/race/gaps`
- 管理发车流程（倒计时、错开或同时发车）

#### 6. 其他文件改动

- `actuator_adapter_node.py`：订阅 `/weather/state`，雨天缩放有效摩擦（`effective_mu = base_mu * rain_scale`）
- `frenet_planner_node.py`：订阅对手位置，作为动态障碍物避开（复用已有 `point_to_oriented_box_clearance`）
- `evaluator_node.py`：多车指标（差距时间、排行榜）
- `simulation.launch.py`：条件启动多车节点栈

### 风险

- **相机后处理性能**：1280×720@30fps 的 OpenCV 滤波增加 CPU 负载。缓解：使用 cv2 优化函数，必要时降分辨率
- **多车计算负载**：4 辆车 × 9 节点 = 36 节点 + 基础设施。建议 ≤3 辆车，planner 降至 5Hz
- **Gazebo 运行时改光照困难**：`<light>` 和 `<scene>` 在世界加载时设定，动态修改需 Transport API。建议场景启动时设定天气，运行中不动态切换
- **GPS 树区模型**：需要在 scenario 中定义树区位置，可在 `scenario_generator.py` 中程序化散布

---

## 推荐实施顺序

1. **Tier 1 先行** — 最快见效，改动最小，无物理稳定性风险
2. **Tier 2** — 驾驶手感质的飞跃，但需仔细调参与稳定性测试
3. **Tier 3** — 赛道多样性，适合算法鲁棒性测试
4. **Tier 4** — 锦上添花，按需选择子功能

---

## 验证方式

| 层级 | 验证方法 |
|------|----------|
| Tier 1 | `./build.sh && ./run.sh`，RViz 中观察锥桶和摩擦区可视化；确认冲出赛道时速度下降 |
| Tier 2 | 运行仿真，观察车辆是否有车身侧倾和俯仰；检查扭矩控制是否达到目标速度 |
| Tier 3 | 生成新赛道：`python3 -m baja_cloud_sim.scenario_generator --output runtime/test_track --track-length 500 --track-seed 123`，运行验证 |
| Tier 4 | 检查天气参数是否正确影响光照和摩擦；多车场景是否正常启动 |
| 全部 | `cd src/baja_cloud_sim && python3 -m pytest test/test_core.py -v` 确保向后兼容 |
