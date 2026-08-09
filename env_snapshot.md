# 环境对照快照 — 虚拟机 → 双系统 Ubuntu 22.04

> 最后更新：2026-08-08（发布者虚拟机环境快照）
> 配套脚本：`env_restore.sh`（真机上一键对照/补齐：`bash env_restore.sh --apply`）
> 前置条件：真机已先跑 `install_ubuntu2204.sh` + `install.sh`

---

## 0. 迁移总览（迁移到双系统必做）

1. **代码**：`src/` 整包 + `.git` 拷到 Windows 备份，真机用 `git clone` 或解压。
2. **系统**：真机装 Ubuntu 22.04（双系统），装好 **NVIDIA 驱动**（闭源或开源均可）。
3. **依赖**：真机执行
   ```bash
   sudo bash install_ubuntu2204.sh
   sudo bash install.sh
   ```
4. **补齐**：`bash env_restore.sh --apply` 补齐下方缺失项。
5. **GPU 注意**：真机走真实 GPU 驱动，**虚拟机那套 `--render-engine ogre` workaround 通常不需要了**，gpu_lidar 应能直接正常出点。

---

## 1. 系统 / 内核指纹

| 项目 | 基准（虚拟机） | 真机需一致 |
|------|---------------|-----------|
| OS | Ubuntu 22.04.5 LTS | 22.04 |
| 内核 | 6.8.0-136-generic | 通常不影响算法，影响 Gazebo GPU 行为 |
| Python | 3.10.12 | 建议一致 |
| gcc | 11.4.0 | 建议一致 |
| cmake | 3.22.1 | 建议一致 |
| git | 2.34.1 | 建议一致 |

---

## 2. Python 关键包（项目真实用到，与 `install.sh` 指纹一致）

> 完整列表请用 `pip3 list > pip_freeze.txt` 在虚拟机导出自行备份。

| 包 | 基准版本 |
|----|---------|
| numpy | 1.21.5 |
| scipy | 1.8.0 |
| matplotlib | 3.5.1 |
| PyYAML | 5.4.1 |
| pytest | 6.2.5 |
| pytest-cov | 3.0.0 |
| transforms3d | 0.3.1 |
| setuptools | 59.6.0 |

---

## 3. apt 源指纹

| 源 | 内容 |
|----|------|
| ROS apt | `deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main` |
| Gazebo apt | `deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main` |
| rosdep 镜像 | `https://mirrors.tuna.tsinghua.edu.cn/rosdistro` |

> 真机若网络可直连 GitHub，可改用官方源；国内建议保留清华镜像。

---

## 4. ⚠️ Shell 环境加载（最容易漏的一项）

**发布者虚拟机的 `~/.bashrc` 里没有 `source /opt/ros/humble/setup.bash`！**
这说明原环境可能靠 IDE/桌面快捷方式手动加载。真机必须手动补，否则新终端找不到 `ros2` 命令。

请在真机 `~/.bashrc` 末尾追加：

```bash
# >>> baja_cloud_sim ROS env >>>
source /opt/ros/humble/setup.bash
# <<< baja_cloud_sim ROS env <<<
```

然后 `source ~/.bashrc`。

---

## 5. 一键命令清单（真机执行）

```bash
# 1. 安装系统级依赖 + 首次 build
sudo bash install_ubuntu2204.sh

# 2. 精细对齐 Python 包 / ROS 源指纹
sudo bash install.sh

# 3. 补齐本快照中的缺失项（含 .bashrc source 行、rosdep 镜像等）
bash env_restore.sh --apply
source ~/.bashrc

# 4. 构建运行（普通用户，勿用 sudo）
./build.sh
./run.sh --seed 42
```

---

## 6. 备注 / 已知差异

- 真机 GPU 驱动下，Gazebo 的 `--render-engine ogre` 和 gpu_lidar `-inf` 问题一般消失，无需保留 workaround。
- `use_sim_time` 仍是 `true`（仿真），不受影响。
- 若真机是干净 Ubuntu 22.04，`install_ubuntu2204.sh` 已用 apt 装齐全部依赖，`rosdep` 报错可忽略（脚本已容错）。
