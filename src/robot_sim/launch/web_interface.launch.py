#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Web 控制台启动(rosbridge WebSocket + 静态页面服务)

前置:robot_sim.launch.py 仿真已在跑
依赖:sudo apt install ros-humble-rosbridge-suite

用法:
  ros2 launch robot_sim web_interface.launch.py
然后浏览器(本机或同网段平板/手机)打开启动日志里打印的地址。
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # rosbridge:把 ROS2 话题/服务桥接为 WebSocket (端口9090)
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge_websocket', output='screen',
             parameters=[{'port': 9090}]),

        # Web 静态页面服务(端口8090)
        Node(package='robot_sim', executable='web_server',
             name='web_server', output='screen'),
    ])
