# ros2-leida —— 巡检机器人 SLAM 建图 + Nav2 自主导航

基于 **ROS 2 Humble** 的巡检机器人系统：Mid-360S 激光雷达 + FAST-LIO 紧耦合建图 + Nav2 导航栈 + 浏览器可视化控制台。

已在真机（Ubuntu 22.04 虚拟机 + 实物雷达）上跑通「建图 → 存图 → 加载 → 自主导航」完整闭环。

---

## 一、系统架构

```
                        ┌─────────────────────────────────────────┐
                        │              硬件层                      │
                        │  Livox Mid-360S   RTK GPS   四轮四转底盘  │
                        └────┬────────────────┬──────────┬────────┘
                             │                │          │
              /livox/lidar   │  /livox/imu    │          │ /cmd_vel
              (CustomMsg)    │                │          │
                             ▼                ▼          │
                  ┌──────────────────┐  ┌───────────┐    │
                  │ imu_unit_fix.py  │  │ 底盘驱动   │────┘
                  │  g → m/s²        │  └───────────┘
                  └────────┬─────────┘        │
                           │ /livox/imu_si    │ /odom
                           ▼                  │
                  ┌──────────────────────────┴──────────────┐
                  │            FAST-LIO2 (紧耦合 LIO)        │
                  └───┬──────────────┬─────────────┬─────────┘
                      │              │             │
        /cloud_registered   /cloud_registered_body │ TF: odom→base_footprint
                      │              │             │
                      │              ▼             │
                      │      ┌────────────────┐    │
                      │      │ cloud_filter.py│    │
                      │      │ 体素去重 5cm    │    │
                      │      └───────┬────────┘    │
                      │              │ /cloud_body_filtered
                      ▼              │             │
        ┌────────────────────────┐   │             │
        │      map_manager       │   │             │
        │  建图/存图/加载 三合一   │   │             │
        └──┬──────┬───────┬──────┘   │             │
           │      │       │          │             │
        /map  /map_cloud │          │             │
                  /nav_map│          │             │
                          ▼          ▼             ▼
        ┌──────────────────────────────────────────────────────┐
        │                    Nav2 导航栈                        │
        │  global_costmap: StaticLayer(/nav_map) + ObstacleLayer│
        │                  frame=map   44×44m 固定             │
        │  local_costmap : ObstacleLayer                           │
        │                  frame=odom  6×6m 滚动窗口            │
        │  planner=NavFn   controller=DWB(全向)  行为树恢复      │
        └───────────────────────┬──────────────────────────────┘
                                │ /cmd_vel
                                ▼
                        底盘 (与建图同一接口)

        ┌──────────────────────────────────────────────────────┐
        │  Web 控制台: rosbridge(9090) + web_server(8090)        │
        │  demo_web: roslib + three.js 浏览器 3D 可视化(8091)    │
        └──────────────────────────────────────────────────────┘
```

**关键设计**：`robot_sim` 包同时提供「无实物仿真」与「真机配套工具」。真机阶段用的是其中的 `map_manager` / `web_server` / `patrol_manager`；`mock_*` 系列节点在无硬件时按**与真机完全一致的接口**模拟底盘/GPS/雷达，上层软件零改动。

---

## 二、目录结构

```
ros2_ws/
├── src/
│   └── robot_sim/                ★ 自研 ROS2 包
│       ├── robot_sim/
│       │   ├── map_manager.py          建图/存图/加载 三合一(核心)
│       │   ├── web_server.py           Web 控制台后端(8090)
│       │   ├── patrol_manager.py       巡检任务(到点拍照/低电回充)
│       │   ├── map_loader.py           PCD → 栅格地图
│       │   ├── scenario.py             虚拟场景
│       │   ├── mock_base.py            模拟底盘(真机替代 base_control_ros2)
│       │   ├── mock_gps.py             模拟 RTK GPS
│       │   ├── mock_lidar.py           模拟 Mid-360S 点云
│       │   ├── mock_gimbal_camera.py   模拟二轴云台相机
│       │   └── mock_mapper.py          仿真用建图节点
│       ├── launch/
│       │   ├── robot_sim.launch.py     仿真总入口
│       │   ├── nav2_sim.launch.py      Nav2 导航栈
│       │   └── web_interface.launch.py Web + rosbridge
│       ├── config/
│       │   ├── nav2_sim.yaml           ★ Nav2 全部参数
│       │   └── sim.rviz                RViz 预置配置
│       └── web/                        Web 控制台前端(three.js)
│
├── scripts/                      ★ 一键启停 + 工具节点
│   ├── start_slam.sh                   建图栈一键启动(按依赖顺序)
│   ├── start_nav.sh                    导航栈一键启动
│   ├── stop_slam.sh                    全部停止
│   ├── slam_ctl.py                     交互式菜单控制台
│   ├── imu_unit_fix.py                 IMU 单位换算 g → m/s²
│   ├── cloud_filter.py                 点云体素去重
│   └── README.md                       启动手册(含三大坑)
│
├── demo_web/                     浏览器端 3D 可视化 demo
├── maps/                         建图成果存档(*.pcd)
├── livox_view.rviz
│
└── third_party_patches/          ★ 第三方源码的本地改动补丁
    ├── fast_lio_local_changes.patch
    └── livox_ros_driver2_local_changes.patch
```

---

## 三、硬件

| 部件 | 型号 | 说明 |
|---|---|---|
| 激光雷达 | Livox **Mid-360S** | 装高 0.30m，上电位置 = 地图原点 = 导航起点 |
| 定位 | RTK GPS（ZED-F9P + NMEA） | 固定解，2cm 噪声 |
| 底盘 | 冰达 `base_control_ros2` | 四轮四转，可全向运动（DWB 用 omni 模型） |
| 相机 | 二轴云台 | 巡检拍照 |

---

## 四、环境与依赖

- Ubuntu 22.04 / ROS 2 Humble
- Python 3.10 + numpy、opencv

```bash
# ROS 2 依赖
sudo apt install ros-humble-nav2-bringup ros-humble-navigation2 \
                 ros-humble-rosbridge-suite \
                 ros-humble-teleop-twist-keyboard

# 编译
cd ~/ros2_ws
colcon build --packages-select robot_sim
source install/setup.bash
```

---

## 五、第三方源码部署（重要）

`FAST_LIO_ROS2` 与 `livox_ros_driver2` 体积大（FAST_LIO 含 128M 文档）且属于上游仓库，**未纳入本仓库**。

本仓库只保存了在这两个包上做过的**本地改动补丁**，因为它们是系统能跑起来的关键（雷达外参、网络配置、FAST-LIO 参数、launch 文件）。

### 5.1 克隆上游并应用补丁

```bash
cd ~/ros2_ws/src

# ① FAST-LIO
git clone https://github.com/hku-mars/FAST_LIO.git FAST_LIO_ROS2
git -C FAST_LIO_ROS2 checkout a4743b095409588842a5b30ddfa27e29d2f99164
git -C FAST_LIO_ROS2 apply ../../third_party_patches/fast_lio_local_changes.patch

# ② Livox 驱动
git clone https://github.com/Livox-SDK/livox_ros_driver2.git livox_ros_driver2
git -C livox_ros_driver2 checkout 4a1def929e5b59c7a8122d19fce6efba581ce9f7
git -C livox_ros_driver2 apply ../../third_party_patches/livox_ros_driver2_local_changes.patch
```

### 5.2 补丁内容

| 补丁 | 基线上游 commit | 涉及文件 |
|---|---|---|
| `fast_lio_local_changes.patch` | `a4743b0` (hku-mars/FAST_LIO) | `config/mid360.yaml`（雷达外参、参数）、`rviz/fastlio.rviz`、`src/laserMapping.cpp` |
| `livox_ros_driver2_local_changes.patch` | `4a1def9` (Livox-SDK/livox_ros_driver2) | `config/MID360_config.json`、`config/MID360s_config.json`、`launch_ROS2/msg_MID360s_launch.py`，以及新增的 `launch/` 目录（9 个 launch 文件） |

> `MID360s_config.json` 里是雷达的 IP 与主机网卡配置，换机器时需按实际网段修改。

---

## 六、快速开始

### 6.1 一键方式

```bash
# 建图:雷达驱动 → IMU换算 → FAST-LIO → 静态TF → 建图管理
bash ~/ros2_ws/scripts/start_slam.sh

# 导航:Web控制台 + Nav2 + RViz
bash ~/ros2_ws/scripts/start_nav.sh

# 停止全部
bash ~/ros2_ws/scripts/stop_slam.sh
```

日志全部在 `~/ros2_ws/logs/` 下，排查问题先看这里。

### 6.2 建图流程

```bash
bash start_slam.sh

# 低速推车绕场一圈（也可键盘遥控）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 存图 → ~/ros2_ws/maps/map_<时间戳>.pcd
ros2 service call /map_mode/save example_interfaces/srv/Trigger

# 加载为导航图 /nav_map（Nav2 全局层的输入）
ros2 service call /map_mode/load_latest example_interfaces/srv/Trigger

# 清空重新建图
ros2 service call /map_mode/start_mapping example_interfaces/srv/Trigger
```

> **顺序不能颠倒**：`start_slam.sh` 里的 `map_manager` 默认以 `mapping` 模式启动，**必须显式调用 `load_latest` 才会发布 `/nav_map`**。跳过这一步 Nav2 的 `static_layer` 拿不到任何数据，全局代价地图会是一片空白。

### 6.3 导航流程

```bash
bash start_nav.sh
```

| 服务 | 地址 |
|---|---|
| Web 控制台 | http://localhost:8090 |
| rosbridge WebSocket | ws://localhost:9090 |
| demo 页（需另起） | `cd ~/ros2_ws/demo_web && python3 -m http.server 8091` → http://localhost:8091/demo.html |

在 RViz 里用 **2D Goal Pose** 点目标点，小车自动导航。

### 6.4 启动后 30 秒健康检查

```bash
ros2 topic hz /livox/lidar                      # 必须 ≈10Hz（掉到 2-3Hz 说明组播丢包）
ros2 topic hz /livox/imu                        # ≈200Hz
ros2 topic echo /Odometry --field pose.pose.position   # 米级小数 = 正常
grep -c "No Effective" ~/ros2_ws/logs/fastlio.log      # 持续增长 = SLAM 失明
ros2 lifecycle get /controller_server           # 应为 active [3]
```

---

## 七、话题与服务

### 话题

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 雷达驱动 | FAST-LIO 的输入 |
| `/livox/imu` → `/livox/imu_si` | `sensor_msgs/Imu` | 驱动 → `imu_unit_fix` | 单位换算 g → m/s² |
| `/cloud_registered` | `PointCloud2` | FAST-LIO | 世界系配准点云 |
| `/cloud_registered_body` | `PointCloud2` | FAST-LIO | 机体系配准点云 |
| `/cloud_body_filtered` | `PointCloud2` | `cloud_filter` | 体素去重后，**Nav2 两个 costmap 的观测源** |
| `/map` | `OccupancyGrid` | `map_manager` | 显示用（latched） |
| `/nav_map` | `OccupancyGrid` | `map_manager` | **导航专用存档图**（latched） |
| `/map_mode` | `String` | `map_manager` | 当前模式 `mapping` / `loaded` |
| `/local_costmap/costmap` | `OccupancyGrid` | Nav2 | 局部代价地图，frame=odom |
| `/global_costmap/costmap` | `OccupancyGrid` | Nav2 | 全局代价地图，frame=map |
| `/cmd_vel` | `Twist` | Nav2 / 遥控 | 底盘速度指令 |

### 服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `/map_mode/start_mapping` | `example_interfaces/srv/Trigger` | 清空并进入建图模式 |
| `/map_mode/save` | `example_interfaces/srv/Trigger` | 保存当前地图到 `~/ros2_ws/maps/` |
| `/map_mode/load_latest` | `example_interfaces/srv/Trigger` | 加载最新存档，进入导航模式 |

### TF 树

```
map ──(静态恒等, 过渡方案)──> odom ──(FAST-LIO)──> base_footprint ──> livox_frame (z=0.30)
                                                                ├──> gps_link   (z=0.60)
                                                                └──> gimbal_base
```

---

## 八、Nav2 参数要点（`src/robot_sim/config/nav2_sim.yaml`）

| | `local_costmap` | `global_costmap` |
|---|---|---|
| 坐标系 | `odom` | `map` |
| 范围 | 6×6m **滚动窗口** | 44×44m 固定 |
| 图层 | `ObstacleLayer` (2D) + `InflationLayer` | `StaticLayer` + `ObstacleLayer` (2D) + `InflationLayer` |
| 更新频率 | 5 Hz | 1 Hz |
| 观测源 | `/cloud_body_filtered` | `/cloud_body_filtered` |
| 用途 | DWB 局部避障 | NavFn 全局规划 |

- 规划器 `NavFnPlanner`，`allow_unknown: true`
- 控制器 `DWBLocalPlanner`，`motion_model: omni`（适配四轮四转全向底盘）
- 机器人半径 0.35m，膨胀半径 0.7m

> 注意：本配置**不含 AMCL**。仿真阶段 `map == odom`，由静态 TF 补齐；真机阶段 `map→odom` 目前也是静态恒等，属于重定位接入前的过渡方案。

---

## 九、已知问题与注意事项

### 9.1 运行前必读

1. **雷达上电位置 = 地图原点 = 导航起点。**
   `map→odom` 是静态恒等变换，重启 FAST-LIO 会把 `odom` 原点重置到**当前物理位置**。所以每次重启后开始导航前，**必须把机器人放回建图时的出发点附近**，否则地图坐标系下位姿会整体错位。
   验证方法：对比实时雷达扫出的障碍与存档地图的重合度（见 9.2）。

2. **建图期间不要让其他电脑跑 LivoxViewer。** 组播冲突会导致点云掉到 2-3 Hz。

3. **`imu_unit_fix.py` 必须在 FAST-LIO 之前启动**，否则一动位姿就飞。检查 `/livox/imu_si` 存在且 z ≈ 9.7。

### 9.2 诊断命令备忘

```bash
# 检查 Nav2 是否真的激活
for n in /controller_server /planner_server /local_costmap/local_costmap; do
    ros2 lifecycle get $n
done

# 只有实时雷达层时 global costmap 长什么样（临时关掉静态层）
ros2 param set /global_costmap/global_costmap static_layer.enabled false
ros2 param set /global_costmap/global_costmap static_layer.enabled true
```

### 9.3 待修问题（按影响排序）

| # | 问题 | 现象 | 建议 |
|---|---|---|---|
| 1 | `bond_timeout: 4.0` 高负载误判 | 日志 `CRITICAL FAILURE: SERVER xxx IS DOWN after not receiving a heartbeat for 4000 ms`，触发 reset；若此时按 Ctrl-C，`lifecycle_manager` 会**卡死在 preshutdown**，85% CPU 空转且 SIGTERM 杀不掉（只能 `kill -9`）。多个僵尸 manager 同名抢节点后会连锁导致整个 Nav2 起不来 | 调到 `10.0`，或设 `0.0` 关闭 bond（单机不需要它做故障恢复） |
| 2 | `stop_slam.sh` 只杀外层进程 | `pkill -f "ros2 run robot_sim map_manager"` 和 `pkill -f "nav2_sim.launch"` 杀不到实际子进程，会留下孤儿 | 改成按完整路径匹配，如 `pkill -f "lib/robot_sim/map_manager"` |
| 3 | ~~`local_costmap` 用 3D `VoxelLayer`~~ **已修复** | 人员/移动物体走过之后障碍**永久残留**：3D 射线只清除被它穿过的体素，人员上半身标记的体素在人走后没有射线能穿到，永远清不掉 | ✅ 已改为 2D `ObstacleLayer`，清除在 XY 平面做直线，与高度无关 |
| 4 | ~~`raytrace_min_range: 0.3`~~ **已修复** | 车体 0.3m 内完全不执行清除，贴车走过的痕迹清不掉 | ✅ 已改为 `0.05` |
| 5 | ~~`unknown_threshold: 15` 配 `z_voxels: 8`~~ **已弃用** | 一列最多只有 8 个未知体素，阈值 15 **永远不生效**，属参数错配 | ✅ VoxelLayer 已弃用，相关参数一并删除 |
| 6 | `track_unknown_space: false` | 未观测区域被当作**自由空间**，车刚转向、雷达还没扫到的格子会先被判为可通行 | 视需要改为 `true` |
| 7 | `cloud_filter.py` 队列深度 10 | 系统卡顿时积压旧点云，触发 tf2 `Message Filter dropping message ... timestamp earlier than all the data in the transform cache` | 订阅/发布的 depth 改为 `1` |
| 8 | global costmap 实测 0.33~0.5 Hz | 配置是 1.0 Hz，高负载（RViz 软件渲染占 120%+ CPU）时被拖慢，对动态障碍响应迟钝 | 跑导航时关掉 RViz；或 `update_frequency`/`publish_frequency` 提到 2.0 |
| 9 | `patrol_manager.py` 与 `web/` 前端耦合 | Web 控制台功能依赖 rosbridge 与前端话题约定 | 待补充接口文档 |

---

## 十、许可

自研部分（`src/robot_sim/`、`scripts/`、`demo_web/`）Apache-2.0。
引用的第三方（FAST-LIO、livox_ros_driver2、Nav2、three.js、roslib.js）遵循各自原始许可。
