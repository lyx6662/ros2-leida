# robot_sim —— 无实物阶段的硬件模拟包

模拟巡检机器人的三块硬件,接口与真机驱动**完全一致**,实物到手后直接替换,上层软件(Nav2 / robot_localization / FAST-LIO 等)零改动。

## 模拟内容

| 节点 | 替代的真机 | 接口 |
|---|---|---|
| `mock_base` | 冰达 base_control_ros2 | 订阅 `/cmd_vel`;发布 `/odom`(50Hz)、`/imu`(50Hz)、`/battery`(10Hz);TF `odom→base_footprint` |
| `mock_gps` | ZED-F9P + NMEA解析 | 发布 `/gps/fix`(5Hz,RTK固定解,2cm噪声) |
| `mock_lidar` | Mid-360S | 发布 `/livox/lidar`(10Hz PointCloud2,20×20m模拟场地) |
| `mock_mapper` | FAST-LIO2 + pcd2pgm | 累积点云发 `/map_cloud`;投影2D栅格发 `/map`;服务 `/save_map` 存 .pcd |

## 建图演示

```bash
# 仿真已启动时,遥控小车走动(模拟无遮挡,首帧即可见全场):
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# RViz 中绿色 MapCloud 为累积点云图,黑色 Map 为2D栅格图(Nav2 用)
# 保存点云地图(实车上由 FAST-LIO2 产出同款 .pcd):
ros2 service call /save_map example_interfaces/srv/Trigger
# 文件位置: ~/ros2_ws/maps/map_<时间戳>.pcd
```

时间戳约定:雷达节点把点云盖为所用 TF 的时刻,建图节点按该时刻查 TF
做反变换,正逆变换严格抵消——运动中无拖影(实测体素数稳定收敛,不随
行驶时间增长)。实车 FAST-LIO2 内部同理。

另有静态 TF:`base_footprint→livox_frame`(高0.30m)、`base_footprint→gps_link`(高0.60m),装车后按实测改 launch 文件。

## 使用

```bash
cd ~/ros2_ws
colcon build --packages-select robot_sim
source install/setup.bash

# 启动全部模拟
ros2 launch robot_sim robot_sim.launch.py

# 可选:指定基地经纬度(建议改成你场地实测值)
ros2 launch robot_sim robot_sim.launch.py base_lat:=39.9042 base_lon:=116.4074
```

另开终端测试:

```bash
# 键盘遥控(需要先 sudo apt install ros-humble-teleop-twist-keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 或直接发速度指令
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# 观察数据
ros2 topic echo /odom
ros2 topic hz /livox/lidar
```

RViz2 中查看:`rviz2`,添加 TF、PointCloud2(选 `/livox/lidar`),Fixed Frame 设为 `odom`。

## 与真机的差异(注意)

1. **雷达消息类型**:真机发 `livox_ros_driver2/CustomMsg`(FAST-LIO2 需要),本包发 `PointCloud2`(测避障/costmap 够用)。FAST-LIO2 建图请用实物或官方 pcap 回放。
2. **运动学为理想模型**:无打滑、无延迟,实车调参时预留差异空间。
3. mock_base 有 0.5s 指令看门狗和速度限幅(vx 1.5 / vy 1.0 / wz 1.5),参数可在 launch 里覆盖。

## 已验证(2026-09-02)

- ✅ 五个话题全部按预期频率发布
- ✅ `/cmd_vel` → `/odom` 运动积分正确(发 0.5m/s 前进,里程计同步、GPS 经度同步东移)
- ✅ GPS 固定解状态、噪声模型正常
- ✅ TF 动静态链路正常
