#!/bin/bash
# stop_slam.sh -- 停掉全部相关节点 (进程名精确匹配, 不会误杀)
for name in fastlio_mapping rviz2 web_server; do
    pkill -x $name 2>/dev/null
done
pkill -f "livox_ros_driver2_node" 2>/dev/null
pkill -f "msg_MID360s_launch" 2>/dev/null
pkill -f "mapping.launch.py" 2>/dev/null
pkill -f "static_transform_publisher" 2>/dev/null
pkill -f "imu_unit_fix" 2>/dev/null
pkill -f "ros2 run robot_sim map_manager" 2>/dev/null
pkill -f "rosbridge_websocket" 2>/dev/null
pkill -f "nav2_sim.launch" 2>/dev/null
pkill -f "http.server 8091" 2>/dev/null
sleep 2
echo "剩余相关进程:"
pgrep -a -f "livox|fastlio|rviz|nav2|rosbridge|map_manager|imu_unit_fix|web_server" | grep -v stop_slam || echo "(无)"
