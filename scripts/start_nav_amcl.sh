#!/bin/bash
# start_nav_amcl.sh -- 用 AMCL 做定位的导航启动
#
# 与 start_nav.sh 的区别:
#   start_nav.sh      走 nav2_sim.launch.py,  map→odom 是静态恒等
#                     → 机器人必须停在建图起点附近
#   start_nav_amcl.sh 走 nav2_amcl.launch.py, map→odom 由 AMCL 实时计算
#                     → 机器人可以停在任意位置
#
# 前提:
#   1) start_slam.sh 已在跑(雷达 / imu_unit_fix / FAST-LIO / map_manager / cloud_filter)
#   2) 已调用过 /map_mode/load_latest, /nav_map 正在发布
#   3) 已安装 ros-humble-pointcloud-to-laserscan
#
# 用法:
#   bash ~/ros2_ws/scripts/start_nav_amcl.sh
#
# 日志: ~/ros2_ws/logs/nav2_amcl.log
set -x
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

LOGDIR=~/ros2_ws/logs
mkdir -p $LOGDIR

# ══════════════════════════════════════════════════════════════
# 步骤 0(关键): 停掉静态 map→odom 发布者
#
# AMCL 的 tf_broadcast: true 会发布同一个 map→odom 变换。
# 两个发布者同时存在时 tf2 会报 TF_REPEATED_DATA,
# 表现为机器人位姿在两个值之间来回跳变, 导航直接失效。
#
# 注意: 如果之后再跑一次 start_slam.sh, 它会把这个静态 TF 重新拉起来,
#       那就必须再执行本脚本(或手动停掉)才能用 AMCL。
# ══════════════════════════════════════════════════════════════
if pkill -f "static_transform_publisher 0 0 0 0 0 0 map odom"; then
    echo "[0/3] 已停掉静态 map→odom 发布者"
else
    echo "[0/3] 未发现静态 map→odom 发布者(可能之前已经停过了)"
fi
sleep 2
# ros2 run 的外壳被杀后会留下孤儿节点, 补一刀确保干净
pkill -9 -f "static_transform_publisher 0 0 0 0 0 0 map odom" 2>/dev/null
sleep 1

# ══════════════════════════════════════════════════════════════
# 步骤 1: 停掉旧的 Nav2 栈(如果还在跑)
# 用完整路径精确匹配, 避免误杀其他节点
# ══════════════════════════════════════════════════════════════
echo "[1/3] 清理旧的 Nav2 进程..."
for exe in \
    /opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager \
    /opt/ros/humble/lib/nav2_controller/controller_server \
    /opt/ros/humble/lib/nav2_planner/planner_server \
    /opt/ros/humble/lib/nav2_behaviors/behavior_server \
    /opt/ros/humble/lib/nav2_bt_navigator/bt_navigator \
    /opt/ros/humble/lib/nav2_waypoint_follower/waypoint_follower \
    /opt/ros/humble/lib/nav2_amcl/amcl
do
    pkill -f "$exe" 2>/dev/null
done
pkill -f "robot_sim nav2_sim.launch.py" 2>/dev/null
pkill -f "robot_sim nav2_amcl.launch.py" 2>/dev/null
sleep 3
for exe in \
    /opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager \
    /opt/ros/humble/lib/nav2_controller/controller_server \
    /opt/ros/humble/lib/nav2_planner/planner_server \
    /opt/ros/humble/lib/nav2_behaviors/behavior_server \
    /opt/ros/humble/lib/nav2_bt_navigator/bt_navigator \
    /opt/ros/humble/lib/nav2_waypoint_follower/waypoint_follower \
    /opt/ros/humble/lib/nav2_amcl/amcl
do
    pkill -9 -f "$exe" 2>/dev/null
done
sleep 2

# ══════════════════════════════════════════════════════════════
# 步骤 2: 启动 AMCL 版 Nav2
# ══════════════════════════════════════════════════════════════
echo "[2/3] 启动 AMCL + Nav2..."
nohup ros2 launch robot_sim nav2_amcl.launch.py \
    > $LOGDIR/nav2_amcl.log 2>&1 &
sleep 12

# ══════════════════════════════════════════════════════════════
# 步骤 3: 健康检查
# ══════════════════════════════════════════════════════════════
echo "[3/3] === 健康检查 ==="

echo -n "  /scan 频率              : "
timeout 10 ros2 topic hz /scan 2>&1 | grep -m1 "average rate" || echo "!!! 无数据, 检查 cloud_to_scan"

for n in /amcl /controller_server /planner_server /global_costmap/global_costmap; do
    echo -n "  $(printf '%-24s' $n): "
    timeout 5 ros2 lifecycle get $n 2>&1 | head -1
done

echo -n "  /amcl_pose 话题          : "
timeout 10 ros2 topic hz /amcl_pose 2>&1 | grep -m1 "average rate" || echo "(暂无, 需机器人移动后才会更新)"

echo
echo "地图: 确认 /nav_map 已在发布"
echo -n "  /nav_map                : "
# 注意: publish_maps 要遍历数万体素, 发布间隔抖动在 1~5 秒, 窗口给短了会误报无数据
timeout 20 ros2 topic hz /nav_map 2>&1 | grep -m1 "average rate" || echo "!!! 无数据, 先调 /map_mode/load_latest"

echo
echo "日志: $LOGDIR/nav2_amcl.log"
echo
echo "下一步(如果机器人不在建图起点):"
echo "  方式一: RViz 里用 '2D Pose Estimate' 点出机器人实际位置和朝向"
echo "  方式二: ros2 service call /reinitialize_global_localization std_srvs/srv/Empty"
echo "          然后遥控走 2~3 米让粒子收敛"
