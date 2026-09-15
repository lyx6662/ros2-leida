# 开机启动手册(真雷达 SLAM + Nav2)

## 一键方式(推荐)

```bash
bash ~/ros2_ws/scripts/start_slam.sh   # 雷达驱动+IMU换算+FAST_LIO+TF+建图
bash ~/ros2_ws/scripts/start_nav.sh    # Web+Nav2+RViz (建图完成后)
bash ~/ros2_ws/scripts/stop_slam.sh    # 全部停止
```

日志都在 `~/ros2_ws/logs/` 下,排查先看这些。

## 手动方式(逐条理解版)

```bash
# ── 0. 环境变量(每个新终端都要,或已写进 .bashrc) ──
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# ── 1. 雷达驱动(必须最先,顺序无强依赖但建议如此) ──
ros2 launch livox_ros_driver2 msg_MID360s_launch.py

# ── 2. IMU 单位换算 g→m/s²(必须在 FAST_LIO 之前!) ──
python3 ~/ros2_ws/scripts/imu_unit_fix.py

# ── 3. FAST_LIO(出 TF: odom→base_footprint 和 /cloud_registered) ──
ros2 launch fast_lio mapping.launch.py rviz:=false

# ── 4. 静态 map→odom(重定位未接入前的过渡) ──
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

# ── 5. 建图管理器(订阅重映射到 FAST_LIO 输出) ──
ros2 run robot_sim map_manager --ros-args -r livox/lidar:=/cloud_registered

# ── 6. 建图/存图/加载(低速推车绕场一圈后) ──
ros2 service call /map_mode/save example_interfaces/srv/Trigger         # 存图 -> ~/ros2_ws/maps/
ros2 service call /map_mode/load_latest example_interfaces/srv/Trigger  # 加载为导航图 /nav_map
ros2 service call /map_mode/start_mapping example_interfaces/srv/Trigger # 清空重新建图

# ── 7. 导航验证三件套 ──
ros2 launch robot_sim web_interface.launch.py    # Web(8090) + rosbridge(9090)
ros2 launch robot_sim nav2_sim.launch.py         # Nav2
rviz2 -d ~/ros2_ws/src/FAST_LIO_ROS2/rviz/fastlio.rviz
# demo 页: cd ~/ros2_ws/demo_web && python3 -m http.server 8091
#          http://localhost:8091/demo.html
```

## 健康检查(启动后 30 秒内确认)

```bash
ros2 topic hz /livox/lidar     # 必须 ≈10Hz 稳定(丢了包会掉到 2-3Hz)
ros2 topic hz /livox/imu       # ≈200Hz
ros2 topic echo /Odometry --field pose.pose.position   # 米级小数=正常
grep -c "No Effective" ~/ros2_ws/logs/fastlio.log       # 持续增长=失明,排查
```

## 三大坑(出问题先查这三个)

1. **IMU 换算节点没启** → 一动位姿就飞。看 `/livox/imu_si` 存在且 z≈9.7
2. **组播丢包**(另一台电脑开 LivoxViewer) → 点云掉到 2-3Hz。让对方退出,或换单播
3. **SLAM 中毒不自愈**(位姿公里级) → 停 fast_lio → 重启 → `start_mapping` 清图

## 注意事项

- 建图期间别让其他电脑连 LivoxViewer
- 雷达上电位置 = 地图原点 = 导航起点,建图从导航出发点位开始
- FAST_LIO 的 PCD(`src/FAST_LIO_ROS2/PCD/scans.pcd`,Ctrl+C 时才写出)是备份,
  项目正式存档走 `/map_mode/save`(`~/ros2_ws/maps/*.pcd`)
- map→odom 目前是静态恒等:测导航时雷达要放回建图起点附近
