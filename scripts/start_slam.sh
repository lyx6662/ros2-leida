#!/bin/bash
# start_slam.sh -- 真雷达 SLAM 建图一键启动(按依赖顺序)
# 用法: bash ~/ros2_ws/scripts/start_slam.sh
set -x
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

LOGDIR=~/ros2_ws/logs
mkdir -p $LOGDIR

# 1. 雷达驱动 (CustomMsg 格式, FAST_LIO 依赖)
nohup ros2 launch livox_ros_driver2 msg_MID360s_launch.py \
    > $LOGDIR/livox.log 2>&1 &
sleep 5

# 2. IMU 单位换算节点 (g -> m/s^2, 必须在 fast_lio 之前!)
nohup python3 ~/ros2_ws/scripts/imu_unit_fix.py \
    > $LOGDIR/imu_fix.log 2>&1 &
sleep 2

# 3. FAST_LIO (TF: odom->base_footprint, 点云 /cloud_registered)
nohup ros2 launch fast_lio mapping.launch.py rviz:=false \
    > $LOGDIR/fastlio.log 2>&1 &
sleep 4

# 4. 静态 map->odom (重定位接入前的过渡方案)
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom \
    > $LOGDIR/static_tf.log 2>&1 &
sleep 1

# 5. map_manager (订阅重映射到 FAST_LIO 输出)
nohup ros2 run robot_sim map_manager --ros-args -r livox/lidar:=/cloud_registered \
    > $LOGDIR/map_manager.log 2>&1 &
sleep 5

echo "=== 全部启动完成, 健康检查: ==="
ros2 topic hz /livox/lidar --window 5 2>&1 | head -1
timeout 4 ros2 topic echo /Odometry --once --field pose.pose.position 2>/dev/null | head -3
grep -c "No Effective" $LOGDIR/fastlio.log || echo "0 (无失明告警)"
tail -2 $LOGDIR/map_manager.log
