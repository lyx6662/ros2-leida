#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无实物仿真总入口:模拟底盘 + GPS + 雷达 + 静态TF + (可选)RViz

用法:
  ros2 launch robot_sim robot_sim.launch.py            # 不带RViz
  ros2 launch robot_sim robot_sim.launch.py rviz:=true # 带RViz(推荐)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    base_lat = LaunchConfiguration('base_lat')
    base_lon = LaunchConfiguration('base_lon')
    rviz_on = LaunchConfiguration('rviz')
    gimbal_on = LaunchConfiguration('gimbal')

    rviz_cfg = os.path.join(get_package_share_directory('robot_sim'),
                            'config', 'sim.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('base_lat', default_value='39.9042'),
        DeclareLaunchArgument('base_lon', default_value='116.4074'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否同时启动 RViz2'),
        DeclareLaunchArgument('gimbal', default_value='true',
                              description='是否启动模拟云台相机'),

        # 1. 模拟底盘(接口与冰达 base_control 完全一致)
        Node(
            package='robot_sim',
            executable='mock_base',
            name='base_control',
            output='screen',
        ),

        # 2. 模拟 GPS(RTK固定解)
        Node(
            package='robot_sim',
            executable='mock_gps',
            name='gps_mock',
            output='screen',
            parameters=[{
                'base_lat': base_lat,
                'base_lon': base_lon,
            }],
        ),

        # 3. 模拟雷达点云
        Node(
            package='robot_sim',
            name='livox_mock',
            executable='mock_lidar',
            output='screen',
        ),

        # 4. 地图模式管理器:建图/保存/加载 三合一(网页可切换模式)
        Node(
            package='robot_sim',
            executable='map_manager',
            name='map_manager',
            output='screen',
        ),

        # 5. 模拟云台相机(yaw/pitch 两轴,渲染虚拟场景视频流)
        Node(
            package='robot_sim',
            executable='mock_gimbal_camera',
            name='mock_gimbal_camera',
            output='screen',
            condition=IfCondition(gimbal_on),
        ),

        # 6. 巡检任务系统(到点对设备拍照、任务/低电触发回充)
        Node(
            package='robot_sim',
            executable='patrol_manager',
            name='patrol_manager',
            output='screen',
        ),

        # 5. 静态 TF:map -> odom 恒等变换
        #    仿真里里程计无误差,地图系与里程计系重合;实车此 TF 由定位节点发布
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        ),

        # 6. 静态 TF:雷达/天线安装位置(装车后按实测改)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_livox',
            arguments=['--x', '0', '--y', '0', '--z', '0.30',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'livox_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gps',
            arguments=['--x', '0', '--y', '0', '--z', '0.60',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'gps_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gimbal',
            arguments=['--x', '0.15', '--y', '0', '--z', '0.35',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'gimbal_base'],
        ),

        # 7. RViz2(预置配置,开箱即用)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(rviz_on),
            output='screen',
        ),
    ])
