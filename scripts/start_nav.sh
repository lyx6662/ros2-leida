#!/bin/bash
# start_nav.sh -- 建图完成后, 启动导航验证三件套 (Web + Nav2 + RViz)
# 前提: start_slam.sh 已跑, 且已 save+load 地图
set -x
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

LOGDIR=~/ros2_ws/logs
mkdir -p $LOGDIR

# Web 控制台 (rosbridge:9090 + 页面:8090)
nohup ros2 launch robot_sim web_interface.launch.py \
    > $LOGDIR/web.log 2>&1 &
sleep 3

# Nav2 导航栈 (静态层订阅 /nav_map)
nohup ros2 launch robot_sim nav2_sim.launch.py \
    > $LOGDIR/nav2.log 2>&1 &
sleep 3

# RViz (软件渲染, 规避 VM 显卡 bug)
nohup bash -c "source /opt/ros/humble/setup.bash && export LIBGL_ALWAYS_SOFTWARE=1 && \
    rviz2 -d ~/ros2_ws/src/FAST_LIO_ROS2/rviz/fastlio.rviz" \
    > $LOGDIR/rviz.log 2>&1 &

echo "启动完成: Web=http://localhost:8090  demo=http://localhost:8091/demo.html"
