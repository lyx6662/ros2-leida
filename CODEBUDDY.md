# CODEBUDDY.md
This file provides guidance to CodeBuddy when working with code in this repository.

巡检机器人 ROS2 项目：Ubuntu 22.04 + ROS 2 Humble。
Livox Mid-360S 激光雷达 + FAST-LIO 建图 + Nav2 导航（AMCL 定位）+ Web 控制台。

配套文档（先读这两个，不要重复它们的内容）：
- `README.md` —— 系统架构图、部署步骤、话题/服务清单、Nav2 参数表
- `TODO.md` —— 未完成/未验证清单，每项含验证步骤与通过标准

**代码只放 `~/ros2_ws/`**（ASCII 路径；中文路径会导致 C++ 包编译失败）。
`~/巡检机器人完整方案/` 是文档与克隆的实车包，**不要在那里编译**。

---

## 常用命令

### 环境（每个新终端都必须先执行）

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 编译

```bash
cd ~/ros2_ws
colcon build --packages-select robot_sim      # 只编自研包
colcon build                                   # 编全部（含 FAST-LIO、雷达驱动）
```

`robot_sim` 是 Python 包，改 `.py` 后必须重新 build 才会同步到 `install/`；
`config/` 与 `launch/` 下的文件通过 `setup.py` 的 `data_files` 安装，同样需要重新 build。

### 单节点调试（改代码后快速验证）

```bash
ros2 run robot_sim map_manager       # 地图模式管理器
ros2 run robot_sim web_server        # Web 后端（8090）
ros2 run robot_sim mock_base         # 仿真：底盘（真机用冰达 base_control_ros2）
ros2 run robot_sim mock_lidar        # 仿真：雷达
```

### 仿真模式（无实物硬件时）

```bash
ros2 launch robot_sim robot_sim.launch.py       # 底盘/GPS/雷达/建图/巡检任务 + 静态 TF
ros2 launch robot_sim nav2_sim.launch.py        # Nav2 导航栈
ros2 launch robot_sim web_interface.launch.py   # Web 控制台（页面 8090 / rosbridge 9090）
rviz2 -d ~/ros2_ws/install/robot_sim/share/robot_sim/config/sim.rviz
```

### 真机模式（一键脚本，日志落在 `~/ros2_ws/logs/`）

```bash
bash scripts/start_slam.sh        # 雷达驱动 → IMU换算 → FAST-LIO → 静态TF → map_manager
bash scripts/start_nav.sh         # Web + Nav2（静态定位：车必须停在建图起点附近）
bash scripts/start_nav_amcl.sh    # Web + Nav2 + AMCL（车可停在任意位置）
bash scripts/stop_slam.sh         # 全部停止
python3 scripts/slam_ctl.py       # 交互式菜单（不用记命令）
```

`start_slam.sh` 顺序不可调换（IMU 换算必须早于 FAST-LIO）；
`start_nav_amcl.sh` 会先杀掉静态 `map→odom` 发布者，否则两个发布者抢同一个 TF。

### 地图操作

```bash
ros2 service call /map_mode/start_mapping example_interfaces/srv/Trigger   # 清空重新建图
ros2 service call /map_mode/save          example_interfaces/srv/Trigger   # 存到 ~/ros2_ws/maps/
ros2 service call /map_mode/load_latest   example_interfaces/srv/Trigger   # 加载为 /nav_map
```

**`load_latest` 不可跳过**：`map_manager` 默认以 `mapping` 模式启动，
不调用它就不会发布 `/nav_map` → Nav2 的 `static_layer` 拿不到数据 → 全局代价地图空白。

### 健康检查

```bash
ros2 topic hz /livox/lidar          # ≈10Hz（掉到 2-3Hz = 组播丢包）
ros2 topic hz /livox/imu_si         # ≈200Hz
ros2 topic hz /scan                 # ≈10Hz（AMCL 输入）
ros2 topic hz /nav_map              # ≈1Hz（有抖动，窗口给 20 秒）
ros2 run tf2_ros tf2_echo map odom  # 定位是否正常
for n in /amcl /controller_server /planner_server /bt_navigator; do ros2 lifecycle get $n; done
grep -c "No Effective" ~/ros2_ws/logs/fastlio.log   # 持续增长 = SLAM 失明
```

### 测试

**本项目没有单元测试，全部验证靠端到端实跑。** 标准流程：
改完 → `colcon build` → 重启相关节点 → 发指令/服务 → 看话题、RViz、网页表现。

用户依赖**实测结果**而非"应该能跑"。声称某功能可用之前，必须有 `ros2 topic hz` /
`ros2 lifecycle get` / `ros2 param get` 之类的实测输出作证据。
具体的验证项目与通过标准见 `TODO.md`。

---

## 架构

### 1. 一套代码，两种运行模式

`robot_sim` 包同时服务仿真与真机，靠 launch 文件区分：

| | 仿真 | 真机 |
|---|---|---|
| 底盘/雷达/GPS | `mock_*` 节点（`robot_sim.launch.py`） | livox_ros_driver2 + 冰达驱动 |
| SLAM | mock_mapper 或 FAST-LIO | FAST-LIO（`start_slam.sh`） |
| 定位 | 静态 TF `map→odom` 恒等 | AMCL 或静态 TF |

`mock_*` 节点**刻意对齐真驱动的接口**（话题名/类型/频率/TF 结构一致），
上层 Nav2/Web 感知不到真假，因此仿真里调好的参数可以直接搬到真机。
`mock_base` 用 best_effort QoS 与真驱动保持一致，这是刻意的、别改成 reliable。

### 2. 数据流与 TF 树

```
/livox/lidar (CustomMsg)
   → FAST-LIO ─┬→ /cloud_registered ──→ map_manager ──→ /map, /map_cloud, /nav_map
               ├→ /cloud_registered_body → cloud_filter.py → /cloud_body_filtered
               └→ TF: odom → base_footprint            ↓
                                          cloud_to_scan → /scan → AMCL
                                                                    ↓
                                            Nav2 两个 costmap 的观测源    TF: map → odom
```

```
map ──(AMCL 或静态恒等)──> odom ──(FAST-LIO)──> base_footprint ──> livox_frame (z=0.30)
```

`imu_unit_fix.py` 把 `/livox/imu`（单位 g）换算成 `/livox/imu_si`（m/s²）供 FAST-LIO 使用，
**漏掉它 FAST-LIO 一动就飞**。验证：`ros2 topic echo /livox/imu_si` 的 z 应 ≈9.7。

### 3. map_manager 的双图设计

`map_manager` 是地图唯一管理者，两状态（`mapping` / `loaded`）由服务切换。
**它发布两张不同的图，这个分离是关键，不要合并**：

- `/map` —— **显示用**。建图模式=实时累积图（未知区 = -1）；导航模式=存档图
- `/nav_map` —— **导航专用**。永远是已加载的存档 PCD 投影；从没加载过就不发布

Nav2 的 `static_layer.map_topic` 指向 `/nav_map`，
而 AMCL 的 `map_topic` 也必须指向 `/nav_map`（不能是 nav2 默认的 `/map`）——
否则 AMCL 用 A 图定位、规划器用 B 图，位姿与障碍判断脱节。
两张图内容确实不同（约 2268 格 vs 5057 格），不是同一个东西。

`/nav_map` 是 latched 且每秒重发（每次带新时间戳），
所以 AMCL 必须设 `first_map_only: true`，否则会把重发误判成"地图更新"而反复重置粒子滤波。

地图生成逻辑在 `map_manager.py` 的 `publish_maps()` / `_build_nav_map()`，
含 3D 射线清除（`dynamic_clear_en`）与噪点过滤（`_denoise`）两套机制。

### 4. 定位：两条路，会互相打架

| 方式 | 启动脚本 | `map→odom` 来源 | 对起点的要求 |
|---|---|---|---|
| 静态恒等 | `start_nav.sh` | `static_transform_publisher` | 车必须停在建图起点附近 |
| AMCL | `start_nav_amcl.sh` | `nav2_amcl`（`tf_broadcast: true`） | 可停在任意位置，但需粗略初值 |

**两套的 `map→odom` 都来自外部，不在 `nav2_sim.launch.py` 里。**
切换时必须停掉当前那个，否则 tf2 会报 `TF_REPEATED_DATA`，位姿来回跳。

AMCL 的固有行为（实测确认，不是 bug）：
- **完全静止时不跑滤波更新**，`/amcl_pose` 无输出（`update_min_d/a` 门限未满足）
- **静止时无法重定位**：粒子只能被里程计"搬运"，锁错位置后观测模型只能降权、
  无法指示方向，粒子会困在原地且**协方差照样很小**（不报警）。
  所以导航前要目视确认 RViz 里 `/scan` 与 `/nav_map` 的墙对齐。

### 5. 时间戳约定（防点云拖影）

雷达节点把点云 header 时间戳盖为**它所使用 TF 的时刻**；
建图节点按该时间戳查 TF 做反变换，正逆变换严格抵消。

TF 查询一律用 `Duration(seconds=0.0)` 非阻塞版本，
**回调里绝不能阻塞等 TF**（会堵死执行器形成死循环）。
违反这两条的症状：建图拖影/散点，或 TF "extrapolation into the future" 死循环。

### 6. Nav2 配置结构

`config/nav2_sim.yaml` 一份文件承载全部参数：控制器、规划器、行为树、
两个 costmap、两个 lifecycle manager。

| | `local_costmap` | `global_costmap` |
|---|---|---|
| 坐标系 / 范围 | `odom` / 6×6m 滚动窗口 | `map` / 44×44m 固定 |
| 图层 | `ObstacleLayer`(2D) + `InflationLayer` | `StaticLayer` + `ObstacleLayer`(2D) + `InflationLayer` |
| 用途 | DWB 局部避障 | NavFn 全局规划 |

**两个 costmap 都必须是 2D `ObstacleLayer`，不要改回 3D `VoxelLayer`。**
3D 体素层的清除只作用于被 3D 射线真正穿过的体素，
人员上半身标记的体素在人员离开后没有任何射线能穿过 → 永久残留 →
DWB 无意义绕路，严重时堵死通道触发 spin/backup。

避障的三层机制：感知（代价地图标障碍）→ 控制（DWB 的 `BaseObstacle` critic，
踩到致命障碍**直接抛异常丢弃该轨迹**，这是硬约束、与权重 `scale` 无关）
→ 恢复（默认行为树的 `ClearEntireCostmap` + spin/wait/backup）。
DWB 不启用 `ObstacleFootprintCritic`，车体轮廓用 `robot_radius: 0.35` 圆形近似。

### 7. 第三方源码与补丁

`src/FAST_LIO_ROS2/`（含子模块 `include/ikd-Tree`）与 `src/livox_ros_driver2/`
的**源码已入库**，目的是让工作区自包含（迁到新机器 clone 后直接 `colcon build`）。

被排除的（见 `.gitignore`）：`doc/`（128M 文档图片）、`Log/`（79M 运行日志）、
`PCD/`、以及上游 git 历史（三个内层 `.git` 已改名为 `.git_upstream`）。

本地改动同时以补丁形式保存在 `third_party_patches/`，
含基线 commit 号，供将来对照上游升级。改这两个库时**两处都要更新**。

关键配置：`src/FAST_LIO_ROS2/config/mid360.yaml`（雷达外参、`publish_frame_world: odom`）、
`src/livox_ros_driver2/config/MID360s_config.json`（雷达 IP + 主机网卡，换机器要改）。

---

## 已踩过的坑

### Git / 仓库

- **`.gitignore` 不支持行尾注释**。`src/xxx/doc/   # 注释` 会被当成一个整体模式，
  永远匹配不上 → 曾导致 130M 内容被误暂存。模式必须独占一行，注释单独成行。
- **嵌套 `.git` 目录**：父仓库 `git add` 时只会存一个 commit 指针、不存内容，
  迁到新机器上是空的。必须先改名为 `.git_upstream` 或删除。

### ROS 2 / Nav2

- `bond_timeout: 4.0` 在高负载 VM 上会误判节点掉线 → 触发 reset；
  若此时按 Ctrl-C，`lifecycle_manager` 会**卡死在 preshutdown**
  （85% CPU 空转，SIGTERM 杀不掉，只能 `kill -9`），
  多个同名僵尸 manager 会让整个 Nav2 起不来。
- `stop_slam.sh` 里 `pkill -f "ros2 run ... map_manager"` 只杀外壳，
  实际子进程会变孤儿。要按完整路径匹配（如 `lib/robot_sim/map_manager`）。
- **costmap 的 `width`/`height` 必须是整数**（Humble），写 `6.0` 报
  `InvalidParameterTypeException`。
- **`NavSatFix.position_covariance` 所有元素必须是 float**，写整数 `0` 会断言崩溃。
- **`OccupancyGrid` 第 0 行对应 `y = origin`（最南）**，渲染直接正向映射，不要翻转。
- `motion_model: "omni"` 对应 DWB 全向模型，配四轮四转底盘。
- `pkill` 会匹配到自己所在的命令行：用方括号正则技巧，如 `pkill -f "xxx[moc]k"`。
- 后台节点日志是块缓冲，排查时用 `PYTHONUNBUFFERED=1`，
  或者直接 `ros2 topic echo` 问真实状态。
- `ros2 daemon` 状态陈旧会导致 `node list` / `topic echo` 假性失败：
  `ros2 daemon stop` 重启。
- `ros2 node list` 偶尔列不全节点（DDS 发现问题），只影响调试体验，不影响功能。

### 其他

- `setup.cfg` 里 `$base` 必须保持字面量。
- Web 页面 `web/index.html` 由 web_server 从 `install/.../web/` 实时读取，
  改完编译后浏览器 Ctrl+F5 即生效，**不需要重启节点**。
- rosbridge 网页自动连接 `ws://<页面主机名>:9090`，跨机器访问 localhost 会连错。

---

## 约定

- 地图存档在 `~/ros2_ws/maps/*.pcd`（ASCII PCD）。
  散点数异常通常意味着 TF 变换或时间戳有 bug。
- 重要架构决策同步更新到 `~/巡检机器人完整方案/方案.md`。
- 运行时日志统一放 `~/ros2_ws/logs/`（一键脚本自动创建），排查问题先看这里。
- 每个新终端都要重新 `source`，否则 `ros2` 找不到包。
