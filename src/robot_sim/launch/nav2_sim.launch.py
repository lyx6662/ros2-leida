#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nav2 导航启动(接 robot_sim 仿真)

前置:先启动 robot_sim.launch.py(提供 /map、/livox/lidar、TF、/cmd_vel)

用法:
  ros2 launch robot_sim nav2_sim.launch.py
然后 RViz 里点 "2D Goal Pose"(绿色箭头)设目标点,小车自动导航。

无 AMCL:仿真里 map==odom,由 robot_sim.launch.py 的静态 TF 补齐;
实车阶段换回标准 nav2_bringup(含 AMCL/定位)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('robot_sim'),
                          'config', 'nav2_sim.yaml')

    lifecycle_nodes = ['controller_server', 'planner_server',
                       'behavior_server', 'bt_navigator',
                       'waypoint_follower']

    return LaunchDescription([
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[params]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen',
             parameters=[params]),

        # 生命周期管理:一键把上面所有节点激活
        Node(package='nav2_lifecycle_manager',
             executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             output='screen',
             parameters=[params,
                         {'node_names': lifecycle_nodes}]),
    ])
