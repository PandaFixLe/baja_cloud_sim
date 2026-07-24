# baja_cloud_sim v2.1

Cloud-ready 100 m dirt-road planning and control simulation for ROS 2 Humble + Gazebo Harmonic.

面向 Ubuntu 22.04 云服务器的完整闭环仿真。道路几何：50 m 直行 → 15 m 半径 90° 左转 → 终末直行，带两座起伏丘（最大坡度 < 8°）和两个低速限速丘。物理频率 500 Hz，ogre2 渲染。

## 主要特性

- **场景**：`core.terrain_height` 定义道路高程轮廓，场景生成器生成带真实法线和动态底座的土路网格。车辆起步高度按地形自适应（`start['z'] + 0.52`）。
- **定位噪声**：`truth_perception_node` 模拟厘米级高斯噪声（可配置 σ），发布 `/localization/odom`（6×6 位姿协方差）。噪声以 ±3σ 限幅。
- **PID 路径跟踪器**：Stanley 横向修正 + PD 航向控制 + 自适应转角/速度 + 曲率预瞄 + 虚拟目标覆盖。`kd_heading=0.12`，`steering_alpha=0.85`，真实耗时 dt（非硬编码 0.05）。保留了避障虚拟目标支持。
- **Frenet 规划器**：纯 Python 分层格子规划，四点角部扫描 `segment_is_safe` 防止墙角擦碰。
- **残影修复**：`/actual_path`（RELIABLE QoS，发布时刻时间戳，1500 位姿缓冲）、frenet/truth 感知节点的 marker 寿命 180 ms。
- **车体参数**：长 1.70 m、宽 1.50 m、轴距 1.43 m、质量 210 kg，Ackermann 转向 P 增益 18.0。

## 运行

```bash
# 本地（Gazebo GUI + RViz）
./run.sh --seed 42 --obstacles 5

# 云端无头模式
./run.sh --seed 42 --obstacles 5 --headless-gazebo --no-rviz
```

## 数据流

```
Gazebo odometry
  └─ truth_perception
       ├─ /gps/fix                    sensor_msgs/NavSatFix
       ├─ /imu/yaw                    std_msgs/Float32
       ├─ /localization/odom          nav_msgs/Odometry (含噪声)
       ├─ /obstacle_markers           visualization_msgs/MarkerArray
       ├─ /road_boundary_markers      visualization_msgs/MarkerArray
       └─ /reference_centerline       nav_msgs/Path
              ↓
         frenet_planner
              └─ /planned_path        nav_msgs/Path
                        ↓
         pid_path_follower
              └─ /cmd_control  ackermann_msgs/AckermannDriveStamped
                                ↓
                         actuator_adapter
                                ↓
                   Gazebo AckermannSteering
```

## 关键参数

`src/baja_cloud_sim/config/params.yaml`：

| 参数 | 值 |
|---|---|
| `truth_perception_node.localization_position_stddev_m` | 0.015 |
| `truth_perception_node.localization_altitude_stddev_m` | 0.020 |
| `truth_perception_node.localization_yaw_stddev_deg` | 0.12 |
| `frenet_planner_node.vehicle_length` | 1.70 |
| `frenet_planner_node.clearance_weight` | 12.0 |
| `frenet_planner_node.desired_clearance` | 1.2 |
| `pid_path_follower_node.kd_heading` | 0.12 |
| `pid_path_follower_node.steering_alpha` | 0.85 |
| `pid_path_follower_node.target_speed` | 6.0 |

## 输出

```
runtime/scenario_<seed>/baja_100m.sdf
runtime/scenario_<seed>/dirt_road.obj
runtime/scenario_<seed>/scenario.json
results/seed_<seed>/tracking_YYYYMMDD_HHMMSS.csv
```

## 回滚

合并前快照位于 `runtime/pre_merge_backup/`，提交 `aacbbed`。单文件回滚：

```bash
cp runtime/pre_merge_backup/<file> src/baja_cloud_sim/baja_cloud_sim/<file>
colcon build --packages-select baja_cloud_sim
```
