#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nav2 导航 + AMCL 定位启动

与 nav2_sim.launch.py 的区别:
  nav2_sim.launch.py   map→odom 由外部静态发布者提供 —— 车必须停回建图起点
  nav2_amcl.launch.py  map→odom 由 AMCL 实时计算   —— 车可以停在任意位置

数据链:
  /cloud_body_filtered → cloud_to_scan → /scan → nav2_amcl → TF map→odom

用法:
  bash ~/ros2_ws/scripts/start_nav_amcl.sh    # 推荐: 会先停掉静态 map→odom
  或
  ros2 launch robot_sim nav2_amcl.launch.py

⚠️ 启动前必须停掉静态 map→odom 的发布者, 否则两个节点抢同一个 TF。
⚠️ 需要 ros-humble-pointcloud-to-laserscan。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_sim')
    amcl_params = os.path.join(pkg, 'config', 'amcl_nav2.yaml')

    # ─────────── 1. 点云 → 2D 激光扫描 ───────────
    # /cloud_body_filtered 在 base_footprint 系, 雷达装高 0.30m。
    # 取 z ∈ [0.20, 1.20] 这一高度带投到水平面:
    #   · 地面点被切掉 —— Mid-360 垂直 FOV 下沿 -7°, 装高 0.3m 时约 2.4m 外
    #     就开始打到地面, 这些点在 base_footprint 系 z≈0
    #   · 高于 1.2m 的结构(横梁/天花板/门框上沿)不参与定位, 避免引入假特征
    cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='cloud_to_scan',
        output='screen',
        remappings=[('cloud_in', '/cloud_body_filtered'),
                    ('scan', 'scan')],
        parameters=[{
            'target_frame': 'base_footprint',   # 在该系下做投影
            'transform_tolerance': 0.1,
            'min_height': 0.20,
            'max_height': 1.20,
            'angle_min': -3.14159265,
            'angle_max': 3.14159265,
            'angle_increment': 0.00872665,      # 0.5° → 720 束
            'scan_time': 0.1,                   # 点云 10Hz
            'range_min': 0.30,
            'range_max': 10.0,
            'use_inf': True,                    # 无回波方向置 inf
            'concurrency_level': 1,
        }],
    )

    # ─────────── 2. AMCL 本体 ───────────
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params],
    )

    # ─────────── 3. AMCL 的生命周期管理 ───────────
    # 单独一个 manager —— 这是 nav2_bringup 的标准做法:
    #   lifecycle_manager_localization 管定位(map_server/amcl)
    #   lifecycle_manager_navigation   管导航(controller/planner/...)
    # bond_timeout 放宽到 10s: 这台 VM 负载偏高, 4s 容易误判节点掉线
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': ['amcl'],
        }],
    )

    # ─────────── 4. 导航五件套(复用现有配置) ───────────
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'nav2_sim.launch.py')))

    return LaunchDescription([
        cloud_to_scan,
        amcl,
        lifecycle_localization,
        navigation,
    ])
