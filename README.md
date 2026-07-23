# Baja 云端规划控制仿真

面向 Ubuntu 22.04 云服务器的 ROS 2 Humble + Gazebo Harmonic 完整闭环工程。默认生成约 100 m 的弯曲土路、0.5 m 间距的道路中心航路点和 5 个可复现的随机有向障碍框，车辆使用基础后驱 Ackermann 四轮底盘。

算法侧只使用 Python 标准库和 ROS 2 标准消息，不依赖 Torch、NumPy、SciPy、Shapely、OpenCV 或其他端侧较重的算法库。

## 包含内容

- Gazebo 土路网格：低幅随机起伏、土路摩擦和纵横向滑移；
- 基础动力学底盘：292 kg、轴距 1.43 m、轮距 1.49 m、35°转向限位；
- 后轮驱动、前轮 Ackermann 转向、轮胎接触刚度/阻尼和转动惯量；
- 理想真值输入：`NavSatFix`、航向 `Float32`、障碍物 `MarkerArray/CUBE`、道路边界 `MarkerArray/LINE_STRIP`；
- 纯 Python Frenet 分层采样和动态规划；
- 保留原有3 m预瞄、航向比例控制、转角限幅和弯道降速逻辑；
- RViz 显示中心线、左右边界、障碍框、膨胀区、规划轨迹、实际轨迹、车辆和实时指标；
- 每次运行自动输出闭环 CSV。

## 1. 云服务器安装

```bash
cd baja_cloud_sim
chmod +x *.sh tools/*.py
./install_ubuntu2204.sh
```

安装脚本会安装：ROS 2 Humble Desktop、Gazebo Harmonic、`ros_gz`、RViz、构建工具及可选 noVNC。不会安装 CUDA、Torch 或深度学习框架；RTX 3080 不是算法运行的必要条件。

如果服务器已经安装相应环境，只需：

```bash
./build.sh
```

## 2. 启动

带 Gazebo GUI 和 RViz：

```bash
./run.sh --seed 42 --obstacles 5
```

仅启动 Gazebo 服务端和 RViz，适合云服务器：

```bash
./run.sh --seed 42 --obstacles 5 --headless-gazebo
```

完全无图形运行：

```bash
./run.sh --seed 42 --obstacles 5 --headless-gazebo --no-rviz
```

相同种子会生成完全一致的道路微起伏和障碍物；改变 `--seed` 可获得新的可复现实验。

## 3. 浏览器远程查看 RViz

```bash
./start_remote_rviz.sh --seed 42 --obstacles 5
```

默认只监听服务器本机地址。先在客户端建立 SSH 隧道：

```bash
ssh -L 6080:localhost:6080 user@server
```

然后浏览器打开：

```text
http://localhost:6080/vnc.html
```

如果确实要在受保护的内网安全组中直接监听，可设置 `NOVNC_LISTEN=0.0.0.0`。不要将无密码的6080端口暴露到公网。

## 数据流

```text
Gazebo odometry
  └─ truth_perception
       ├─ /gps/fix                    sensor_msgs/NavSatFix
       ├─ /imu/yaw                    std_msgs/Float32
       ├─ /obstacle_markers           visualization_msgs/MarkerArray
       ├─ /road_boundary_markers      visualization_msgs/MarkerArray
       └─ /reference_centerline       nav_msgs/Path
              ↓
         frenet_planner
              └─ /planned_path        nav_msgs/Path
                        ↓
                  path_follower
                        └─ /cmd_control ackermann_msgs/AckermannDriveStamped
                                      ↓
                               actuator_adapter
                                      ↓
                         Gazebo AckermannSteering
```

规划内部统一使用 `map` 平面坐标，感知数据使用 `base_link`：x向前、y向左。兼容现有控制器的航向仍采用“北向为零、顺时针为正”。

## 规划目标

规划代价包含：

```text
中心航路点距离²
+ 安全净空不足惩罚²
+ 横向变化平滑项
+ 连续层斜率项
```

同时施加道路内、膨胀障碍物外和最大横向步长等硬约束。默认规划范围30 m、纵向层间距1 m、横向采样0.25 m、重规划频率10 Hz。

主要参数位于 `src/baja_cloud_sim/config/params.yaml`：

- `center_weight`：增大后更贴近中心航路点；
- `clearance_weight`：增大后更倾向高净空；
- `desired_clearance`：达到该净空后不再产生额外绕行奖励；
- `target_speed`、`lookahead_distance`、`kp_heading`：现有控制器参数。

## 输出

运行时生成：

```text
runtime/scenario_<seed>/baja_100m.sdf
runtime/scenario_<seed>/dirt_road.obj
runtime/scenario_<seed>/scenario.json
results/seed_<seed>/tracking_YYYYMMDD_HHMMSS.csv
```

CSV 包括位置、速度、转角、规划/跟踪误差、中心偏差、最小净空、规划耗时、碰撞次数和完成进度。

## 快速算法自检

不安装 ROS 2 也能执行纯标准库闭环冒烟测试：

```bash
python3 tools/offline_closed_loop.py --seed 42 --obstacles 5
```

预期结果为进度超过97 m、碰撞0次、中心偏差小于1 m。该测试使用运动学车辆；最终动力学结果以 Gazebo 为准。

## 动力学边界

当前底盘包含质量、惯量、轮胎摩擦/滑移、接触刚度、转向速率、驱动加速度和 jerk，足以评价低速规划控制闭环。它不是经过轮胎试验标定的高保真整车模型，也没有土壤形变、悬架连杆和电机电流模型。实车参数确认后，应优先标定总质量、质心位置、横摆惯量、轮胎摩擦/滑移、转向时延和驱动加速度。
