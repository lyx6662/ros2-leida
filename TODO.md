# 待完成 / 待验证清单

> **快照时间**：2026-09-15
> **当时状态**：雷达在位运行，**底盘送修中**（`/cmd_vel` 无接收者）
>
> **一句话结论**：系统搭好了，每一段都单独验证过，但**从未有一次端到端跑通**。

---

## 一、现状速览

| 模块 | 完成度 | 已实测的证据 | 主要缺口 |
|---|---|---|---|
| 硬件接口层 | **95%** | 雷达 9.96Hz、`/livox/imu_si` 200Hz（z=9.48 正确）、`/cloud_body_filtered` 10Hz | 未做长时间压测 |
| SLAM 建图（FAST-LIO） | **90%** | 建图→存图（24 张存档）→加载全流程跑通；33% CPU；0 失明告警；TF 链完整 | 未做建图精度评估 |
| 建图管理（map_manager） | **85%** | mapping/loaded 双模式、3 个服务、`/map`+`/nav_map` 双发布 | **地图障碍密度仅 0.65%** |
| 定位（AMCL） | **60%** | `map→odom` 9.9Hz 实时发布；与规划器共用 `/nav_map` | 重定位能力受限；运动场景未验证 |
| Nav2 配置层 | **90%** | 8 个 lifecycle 节点全 `active`；`/navigate_to_pose` 等 4 个 action server 就绪 | — |
| Nav2 **实测**层 | **10%** | ❗**从未发送过任何导航目标点** | 端到端链路未闭合 |
| 避障能力 | **40%** | 三层机制齐全；局部层残留问题已修复并目视验证 | 绕行 / 恢复行为全未实测 |
| 巡检任务系统 | **10%** | 代码齐全（`patrol_manager.py`） | ❗**从未在真机流程中启动过** |
| Web 控制台 | **50%** | 页面 HTTP 200 正常服务（29KB）；rosbridge 9090 在监听 | 交互功能未验证 |
| 工程化 | **95%** | 4 个一键脚本、README 完整、GitHub 已上传 | — |

---

## 二、未验证项

> 每一项都给出**前置条件、操作步骤、通过标准、失败排查**，可直接照着做。

### V1 · 端到端导航链路 ❗最高优先级

**现状**
`/navigate_to_pose` action server 已就绪、8 个 lifecycle 节点全 `active`、代价地图有数据、
`/cmd_vel` 话题存在——但**一次都没有真的发过目标点**。整条链路是：

```
设目标 → 全局规划 → 局部避障 → 输出 /cmd_vel → 车实际移动 → 到达
  ✅       ✅         ✅          ✅              ❓            ❓
```

最后一环从未闭合。配置检查再仔细也替代不了实跑。

**为什么优先做这个**
这是唯一能暴露下列问题的手段，而这些问题**都无法通过读配置发现**：

- DWB 的速度上限（`max_vel_x: 1.5`）与底盘实际能力不匹配
- 规划出的路径底盘走不了（转弯半径、原地转向能力）
- `/cmd_vel` 的坐标系约定与底盘驱动不一致（车会跑反或乱转）
- 到达判据（`xy_goal_tolerance: 0.15`）在实际底盘精度下达不到，导致永远不结束

**前置条件**
1. 底盘可用，`/cmd_vel` 有接收者
2. `/nav_map` 已加载（`ros2 topic echo /map_mode` → `loaded`）
3. 定位正常（见 V4 的判据）
4. 场地清空，目标方向上留出 5m 以上无障碍

**操作步骤**

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# 0. 健康检查（见第五节），确认全部 active
# 1. 记录起始位姿
ros2 topic echo /amcl_pose --once

# 2. 开几个终端同步观察
ros2 topic echo /plan --once         # 全局路径
ros2 topic echo /local_plan --once   # 局部路径
ros2 topic echo /cmd_vel             # 速度指令
ros2 topic echo /amcl_pose           # 位姿是否跟着走

# 3. 发目标点（车前方 5m 处）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map},
           pose: {pose: {position: {x: 5.0, y: 0.0, z: 0.0},
                         orientation: {w: 1.0}}}}}" --feedback

# 也可以用 RViz 工具栏的 "2D Goal Pose" 点一个点（更直观）
```

**通过标准**

| 检查项 | 期望 |
|---|---|
| 全局路径 | 不穿墙、不穿障碍，几何上像一条真能走的路 |
| `/cmd_vel` | 有非零输出；**线速度不超过底盘实际能力**（先按 0.3 m/s 量级判断） |
| 车实际运动 | 朝目标方向移动，轨迹平滑，无明显抖动或画龙 |
| 最终停止 | 距目标点 xy 误差 < 0.15m |
| action 结果 | `status` 为 `SUCCEEDED`，不是 `ABORTED` |

**失败排查表**

| 现象 | 可能原因 | 检查命令 |
|---|---|---|
| 发目标后毫无反应 | `bt_navigator` 没激活 / BT 文件路径问题 | `ros2 lifecycle get /bt_navigator` |
| 规划直接失败 | 目标点落在障碍或膨胀层内；全局代价地图没收到 `/nav_map` | `ros2 topic hz /nav_map` |
| 有路径但 `/cmd_vel` 恒为 0 | DWB 找不到合法轨迹（被障碍包围）；或代价地图里全是残留障碍 | 手动 clear 验证，见 V2 |
| 车动了但方向反了 / 乱转 | `/cmd_vel` 坐标系约定与底盘不符 | 单独测 V3 |
| 直接撞上去不避 | 障碍层没数据 | `ros2 topic hz /cloud_body_filtered` |
| 一直原地 spin / backup | 通道被残留障碍堵死 | `ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Empty` |

---

### V2 · 避障能力

**现状**
三层机制在配置上齐全，但**一次都没实测过**：

```
第1层 感知   : local/global 的 ObstacleLayer 订阅 /cloud_body_filtered      ✅ 有数据
第2层 控制   : DWB 的 BaseObstacle critic（撞致命障碍直接丢弃轨迹，硬约束） ✅ 已加载
第3层 恢复   : 行为树 RecoveryNode(6次) → ClearEntireCostmap → spin/wait/backup ✅ 服务在线
```

已修复的问题：`local_costmap` 原来用 3D `VoxelLayer`，人员走过会留下**永久残留**；
已改成 2D `ObstacleLayer`，目视验证"人走过时有残留，走后消失"。

**操作步骤（三个测试逐级做）**

```bash
# 测试 2-1：静态障碍绕行
#   车前方 3m 放一个纸箱 → 发纸箱后方的目标点 → 车应绕过去
# 测试 2-2：动态障碍
#   车巡航中，人从侧面 3~4m 处斜插到车前方 → 观察 local costmap 是否及时出现
#   障碍块、车速是否下降或转向
#   注意两人都要在 obstacle_max_range: 4.0m 以内才会被标记
# 测试 2-3：堵死后的恢复行为
#   用箱子把通道完全封死 → 发目标点
#   预期：日志出现 ClearEntireCostmap，车执行 spin → wait → backup，最后报目标失败
```

**通过标准**

| 测试 | 期望 |
|---|---|
| 2-1 | 规划出绕行路径，平稳绕过，不蹭不撞 |
| 2-2 | 4m 内出现障碍块；车速下降或有横向避让；不撞 |
| 2-3 | 按预期触发恢复序列，不是直接卡死或报错退出 |

**失败排查**

| 现象 | 原因 |
|---|---|
| 残留清不掉 | 该位置被遮挡（雷达看不到）→ 属正常；若开阔处也不清，把 `max_obstacle_height` 从 1.8 压到 1.0 |
| 一直绕开空地 | 是 `global_costmap` 的残留（它不滚动）→ `ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty` |
| 贴墙时突然急打方向 | `inflation_radius: 0.7` + `cost_scaling_factor: 6.0` 梯度太陡 → 把后者降到 3.0 |

---

### V3 · 底盘运动控制标定

**现状**
`/cmd_vel` → 底盘 → 实际位移这条链**从未验证过**（车送修）。
而 DWB 的速度参数目前是**猜的**：`max_vel_x/y: 1.5`、`max_vel_theta: 2.0`。

**操作步骤**

```bash
# 1. 单独发一个已知速度，测量实际位移
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}}" --times 30
#    发 3 秒（0.2 m/s）→ 理论应该前进 0.6m，用尺子量

# 2. 测转向
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{angular: {z: 0.5}}" --times 20
#    发 2 秒（0.5 rad/s）→ 理论应该转 1.0 rad ≈ 57°

# 3. 测全向平移（四轮四转底盘的关键能力）
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {y: 0.2}}" --times 30
#    应该横向平移；如果不动，说明底盘不支持横移，DWB 的 motion_model 要改成 diff
```

**通过标准**
- 实测位移与理论值偏差 < 20%
- 方向正确（`x` 前进、`y` 左移、`z` 逆时针）
- 横移可用（否则 `nav2_sim.yaml` 里 `motion_model: omni` 必须改成 `diff`）

**产出**：据此修正 `nav2_sim.yaml` 的 `max_vel_x/y/theta` 与 `acc_lim_*`，并在本文件里记录实测值。

---

### V4 · 定位（AMCL）的运动场景验证

**现状**
`map→odom` 能以 9.9Hz 实时发布，静止时定位精度到毫米级。但有两个**已实测确认的局限**：

**局限 A：完全静止时无法重定位（已用实验证明）**

实验方法：用 `/set_initial_pose` 强行把位姿错位 8m（等效于"雷达在 8m 外重启"），
然后反复调用 `/request_nomotion_update` 强制更新 45 轮。

```
结果: 偏差 7.899 m → 7.733 m      (只缩小 17 厘米)
      σx 0.210 → 0.130            (粒子云只缩不散)
结论: 完全没有随机粒子注入发生, 粒子云只是"在错误的位置上越缩越紧"
```

**局限 B：地图太稀疏，特征不足**

```
/nav_map   : 5057 个占据格 = 12.6 m²
覆盖面积   : 44 × 44 m     = 1936 m²
障碍物占比 : 0.65%
```

这么稀疏的地图，似然场在错误位置和正确位置的得分差异很小，AMCL 难以区分对错。

**根本原因（需要理解）**
AMCL 的粒子只能被**里程计"搬运"**，它自己不会搜索。
一旦粒子云锁在错误位置，观测模型只能给粒子**降权**，无法告诉它们"该往哪个方向挪"。
所以：**必须有运动**（哪怕只动 10cm，`update_min_d: 0.10` 就会被满足）。

**操作步骤（车修好后做）**

```bash
# 前置：已经给过粗略初值（RViz 的 2D Pose Estimate，或 /set_initial_pose 服务）

# 1. 记录当前 map→odom
ros2 run tf2_ros tf2_echo map odom

# 2. 遥控车前进/后退 2~3 米（低速）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 3. 观察 map→odom 是否在持续小幅修正（补偿 FAST-LIO 累积漂移）
#    并观察 /amcl_pose 是否有输出（有运动就应该有）
ros2 topic echo /amcl_pose --field pose

# 4. 进阶：测"任意位置启动"
#    把车开到离建图起点 5~8m 处 → 重启 FAST-LIO（odom 原点会重置到当前位置）
#    → 观察 AMCL 能否在走动几米后自己收敛回正确位置
pkill -x fastlio_mapping
sleep 3
nohup ros2 launch fast_lio mapping.launch.py rviz:=false > ~/ros2_ws/logs/fastlio.log 2>&1 &
```

**通过标准**

| 项目 | 期望 |
|---|---|
| 走动时位姿跟随 | `map→odom` 连续小幅修正，位姿无跳变、无发散 |
| RViz 目视 | `/scan` 与 `/nav_map` 的墙始终贴合 |
| 任意位置启动 | 走 3~5 米后能自己收敛（若不能，见 T5） |

**每次导航前的 10 秒保险**：在 RViz 里扫一眼 `/scan` 和 `/nav_map` 是否对齐。
AMCL 锁错位置时**不会告警**（协方差照样很小），只能靠目视发现。

---

### V5 · Web 控制台功能

**现状**
页面 `http://localhost:8090` 返回 HTTP 200（29012 字节），rosbridge 9090 在监听。
但**实际交互功能一次都没点过**。

**操作步骤**

```bash
# 服务已在跑（start_nav.sh 会启动）。若没跑：
ros2 launch robot_sim web_interface.launch.py
```

浏览器打开 `http://localhost:8090`，逐项验证：

| 检查项 | 期望 |
|---|---|
| 页面加载 | 3D 场景正常渲染，无 JS 报错（F12 Console） |
| 话题订阅 | 能显示机器人位姿、点云、地图 |
| 建图模式切换 | 能调 `/map_mode/start_mapping`、`/save`、`/load_latest` |
| 发导航目标 | 能通过页面发 2D Nav Goal |
| 手机/平板访问 | 同网段设备用 `http://<虚拟机IP>:8090` 能打开 |

**已知问题**：`/livox/lidar` 是 `livox_ros_driver2/CustomMsg` 类型，roslib.js **不支持**这种自定义消息，
Web 端如果要显示点云，需要用 `/cloud_body_filtered`（标准 `PointCloud2`）。

---

### V6 · 长时间稳定性

**现状**
所有验证都是"跑几分钟"级别的。**没有跑过小时级的连续运行。**

**操作步骤**

```bash
# 起全套后挂机 1~2 小时，期间每隔 10 分钟记录一次：
watch -n 600 'echo "== $(date +%H:%M) =="; \
  ros2 topic hz /livox/lidar 2>&1 | grep -m1 average; \
  ros2 topic hz /amcl_pose 2>&1 | grep -m1 average; \
  ros2 lifecycle get /controller_server; \
  uptime'
```

**通过标准**

| 指标 | 期望 |
|---|---|
| 点云频率 | 1 小时内始终 ≈10Hz，不衰减（衰减 = 组播丢包或内存泄漏） |
| 内存 | 各节点 RSS 不持续增长（重点看 `map_manager`、`fastlio_mapping`） |
| lifecycle | 无节点意外 deactivate（bond 掉线） |
| CPU | 不出现"越跑越占"的趋势 |

**重点怀疑对象**：`map_manager` 有 `max_voxels: 600000` 上限，长时间建图会触顶。

---

## 三、未完成项

### T1 · 巡检任务系统接入真机流程

**现状**：`patrol_manager.py`（到点拍照、任务调度、低电回充）代码齐全，
但**从未在真机流程中启动过**——它只存在于 `robot_sim.launch.py`（仿真总入口），
而真机走的是 `start_slam.sh` / `start_nav.sh`，里面都没有它。

**要做的**
1. 把 `patrol_manager` 加进 `start_nav.sh`（或新建 `start_patrol.sh`）
2. 确认它的依赖：
   - 位姿来源：现在应该有 `/amcl_pose`，需确认它订阅的是什么
   - 云台相机：真机的 `mock_gimbal_camera` 不存在，需要真云台驱动
   - 电池：`/battery` 谁发布？（真机应来自底盘驱动）
3. 跑一次完整巡检任务：到点 → 拍照 → 下一站 → 低电回充

### T2 · 提升地图密度（定位鲁棒性的根本）

**现状**：`/nav_map` 障碍密度 **0.65%**，这是所有基于匹配的定位方法（AMCL、GICP）的先天短板。

**要做的**
1. 建图时放宽 `map_manager.py` 的 Z 范围：
   ```python
   Z_BAND = (0.05, 1.5)   →   改成 (0.05, 2.5)
   ```
   保留更多结构（横梁、门框上沿、高处设备）
2. 走图更慢更全，把角落和柱子扫完整
3. 重新 `save` + `load_latest`，然后重跑 V4 的验证

### T3 · 停机位初始位姿

**现状**：`initial_pose` 写死为 `(0,0,0) = 地图原点`。若车总是从充电桩出发，
其实可以直接写死成充电桩坐标，**上电即用、零操作**。

**要做的**
1. 在 RViz 里把鼠标移到充电桩位置，读出它的地图坐标（左下角会显示）
2. 写进 `src/robot_sim/config/amcl_nav2.yaml` 的 `initial_pose`
3. 给 `start_nav_amcl.sh` 加传参口：
   ```bash
   bash start_nav_amcl.sh x:=3.2 y:=-1.5 yaw:=1.57
   ```

### T4 · 网络与 ROS 域配置固化

**现状**：`ROS_DOMAIN_ID` 未设置（默认 0），`ROS_LOCALHOST_ONLY=0`。
迁到工控机后如果同一网段还有别的 ROS 机器，话题会串。

**要做的**
- 在 `~/.bashrc` 或启动脚本里固定 `ROS_DOMAIN_ID`
- 决定是否需要 `ROS_LOCALHOST_ONLY=1`（单机场景能减少组播开销）

### T5 · 定位方案 B：3D GICP 全局重定位（进阶）

**现状**：方案 A（2D AMCL）的边界是"需要粗略初值或运动"。
**真正的"任意位置零初值冷启动"需要 3D 全局配准**（描述子 / GICP），这是 AMCL 做不到的。

**已有的基础**
- `ros-humble-fast-gicp` 可从 apt 安装
- `ros-humble-pcl-ros` 2.4.5 已装
- 存档 PCD 在 `~/ros2_ws/maps/`

**要做的**
1. 写一个节点：订阅 `/cloud_registered` + 加载存档 PCD → GICP 配准 → 发布 `map→odom`
2. 难点是**初值**：GICP 需要不错的初始猜测，否则会收敛到局部最优。
   可行做法：先用 AMCL 做粗定位 → 再 GICP 精配准（两者互补）
3. 与方案 A 做精度/鲁棒性对比

---

## 四、已知限制与风险

| # | 问题 | 影响 | 现状 |
|---|---|---|---|
| 1 | `bond_timeout: 4.0` 在高负载时误判节点掉线 | 触发 reset；若此时 Ctrl-C，`lifecycle_manager` 会**卡死在 preshutdown**（85% CPU 空转，SIGTERM 杀不掉，只能 `kill -9`），并连带导致后续 Nav2 起不来 | ⚠️ 未修。建议调到 10.0 或设 0.0 |
| 2 | `stop_slam.sh` 只杀外层进程 | `pkill -f "ros2 run robot_sim map_manager"` 和 `pkill -f "nav2_sim.launch"` 杀不到实际子进程，会留下孤儿 | ⚠️ 未修。需改成按完整路径匹配 |
| 3 | 地图障碍密度 0.65% | 定位鲁棒性差；AMCL 特征匹配区分度低 | 见 T2 |
| 4 | AMCL 静止时无法重定位，且**不告警** | 锁错位置时协方差仍显示很小，看起来很"自信" | 已实测确认。靠目视 `/scan` 与地图的对齐度发现 |
| 5 | `cloud_filter.py` 队列深度 10 | 系统卡顿时积压旧点云，触发 tf2 `Message Filter dropping` 告警 | ⚠️ 未修。建议改成 1 |
| 6 | `global_costmap` 实测只有 0.33~0.5Hz（配置 1.0） | 对动态障碍响应迟钝 | 负载所致，跑导航时建议关掉 RViz |
| 7 | `track_unknown_space: false` | 未观测区域被当作自由空间，车转向时新露出的格子会先被判为可通行 | 视需要改为 `true` |
| 8 | `robot_radius: 0.35` 圆形近似 | 四轮四转底盘实际是矩形，车头/车尾/侧向安全裕度不一致 | 建议实测后改 `footprint` 多边形 |
| 9 | `ros2 node list` 偶尔列不全节点 | DDS 发现问题，只影响调试体验，不影响功能 | 已知现象 |

---

## 五、验证前置：健康检查清单

**每次开始验证前，先跑一遍这个。任何一项不通过就不要往下做。**

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# ── 1. 进程是否齐全 ──
ps -eo pid,args | grep -E "[l]ivox_ros_driver2_node|[f]astlio_mapping|[i]mu_unit_fix\
|[c]loud_filter|[m]ap_manager|[c]loud_to_scan|[a]mcl|[n]av2_"
# 应有: 雷达驱动 / fastlio / imu_unit_fix / cloud_filter / map_manager
#       / cloud_to_scan / amcl / nav2 六件套(lifecycle_manager + controller + planner
#       + behavior + bt_navigator + waypoint_follower)

# ── 2. 关键话题频率 ──
ros2 topic hz /livox/lidar              # ≈10Hz   (掉到 2-3Hz = 组播丢包)
ros2 topic hz /livox/imu_si             # ≈200Hz
ros2 topic hz /cloud_body_filtered      # ≈10Hz   (两个 costmap 的观测源)
ros2 topic hz /scan                     # ≈10Hz   (AMCL 的输入)
ros2 topic hz /nav_map                  # ≈1Hz    (有抖动, 给 20 秒窗口)

# ── 3. 定位是否正常 ──
ros2 run tf2_ros tf2_echo map odom       # 应稳定输出, 数值不跳变
# 并在 RViz 里目视确认 /scan 与 /nav_map 的墙对齐

# ── 4. Nav2 是否全部激活 ──
for n in /amcl /controller_server /planner_server /behavior_server \
         /bt_navigator /waypoint_follower /local_costmap/local_costmap \
         /global_costmap/global_costmap; do
    echo -n "$n -> "; ros2 lifecycle get $n
done
# 全部应为 active [3]

# ── 5. 导航 Action 是否就绪 ──
ros2 action list | grep navigate
# 应有: /navigate_to_pose  /navigate_through_poses

# ── 6. SLAM 是否失明 ──
grep -c "No Effective" ~/ros2_ws/logs/fastlio.log    # 持续增长 = 失明, 需排查
```

---

## 六、建议执行顺序

```
【车修好后立刻做】
  第 0 步  V3  底盘运动标定      ← 必须先做！否则后面所有参数都是猜的
  第 1 步  健康检查（第五节）     ← 任何一项不过就停
  第 2 步  V1  端到端导航实测     ← 一次能暴露一大堆问题
  第 3 步  V2  避障三个测试
  第 4 步  V4  定位运动场景验证

【稳定后】
  第 5 步  T3  停机位初始位姿     ← 小改动，立刻提升可用性
  第 6 步  T1  巡检任务系统接入
  第 7 步  V5  Web 控制台功能
  第 8 步  V6  长时间稳定性

【中期改进】
  第 9 步  T2  提升地图密度       ← 定位鲁棒性的根本
  第 10 步 T5  3D GICP 重定位（可选进阶）

【随时可做】
  T4  网络与 ROS 域配置固化
  第四节表格里 #1 / #2 / #5  三处已知缺陷的修复
```

---

## 附：本次（2026-09-15）会话中已完成的改动

| 提交 | 内容 |
|---|---|
| `6e11f0d` | 首次入库：自研源码 + 启动脚本 + 建图存档 + 第三方改动补丁 |
| `dfaea16` | 补充 README（架构 / 目录 / 部署 / 启动 / 已知问题） |
| `cbec7aa` | `local_costmap` 弃用 3D `VoxelLayer`，改用 2D `ObstacleLayer`（修复人员走过后的障碍永久残留） |
| `18a5697` | 接入 Nav2 AMCL，用实时位姿代替静态 `map→odom`（`map_topic` 修正为 `/nav_map`、开启 `first_map_only`） |
| `b40bf0a` | 打开 augmented MCL 自适应恢复，调小运动门限 |
| （本次） | 第三方源码入库（工作区自包含，便于迁移到工控机）+ 本文件 |
