#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mock_gps —— 模拟 ZED-F9P RTK 定位输出(无实物阶段)

模拟内容:
  1. 订阅 /odom 获取机器人局部位置
  2. 把局部坐标(x,y)换算成经纬度(以基地坐标为原点)
  3. 叠加 RTK 固定解级别的噪声(水平 2cm / 1σ)

发布: /gps/fix (sensor_msgs/NavSatFix) 5Hz,status=GBAS_FIX 表示 RTK 固定解

注意:真机 F9P 从串口输出 NMEA,由解析节点转成 NavSatFix;
本节点跳过串口环节直接发话题,供 robot_localization 调试用。
"""
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

# 纬度每度约 111320 米(足够模拟用)
M_PER_DEG_LAT = 111320.0


class MockGps(Node):
    def __init__(self):
        super().__init__('gps_mock')

        # 基地坐标(默认北京附近,可用参数改成你场地实测值)
        self.declare_parameter('base_lat', 39.9042)
        self.declare_parameter('base_lon', 116.4074)
        self.declare_parameter('rate', 5.0)          # Hz
        self.declare_parameter('noise_std', 0.02)    # 米,RTK固定解噪声
        self.lat0 = self.get_parameter('base_lat').value
        self.lon0 = self.get_parameter('base_lon').value
        noise = self.get_parameter('noise_std').value

        self.fix_pub = self.create_publisher(
            NavSatFix, 'gps/fix', qos_profile_sensor_data)
        self.create_subscription(
            Odometry, 'odom', self.odom_cb, qos_profile_sensor_data)

        self.px = 0.0   # 跟踪的局部位置
        self.py = 0.0
        self.noise_std = noise

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self.publish_fix)
        self.get_logger().info(
            'mock_gps 启动:基地(%.6f, %.6f), %dHz, 噪声%.0fcm'
            % (self.lat0, self.lon0, rate, noise * 100))

    def odom_cb(self, msg: Odometry):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

    def publish_fix(self):
        # 局部坐标 -> 经纬度 + 高斯噪声
        lat = self.lat0 + (self.py + random.gauss(0, self.noise_std)) / M_PER_DEG_LAT
        lon_scale = M_PER_DEG_LAT * math.cos(math.radians(self.lat0))
        lon = self.lon0 + (self.px + random.gauss(0, self.noise_std)) / lon_scale

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX  # 模拟RTK固定解
        fix.status.service = (NavSatStatus.SERVICE_GPS
                              | NavSatStatus.SERVICE_GLONASS)
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = 45.0 + random.gauss(0, 0.05)
        # 位置协方差(对角,平方米)
        fix.position_covariance = [4e-4, 0.0, 0.0,
                                   0.0, 4e-4, 0.0,
                                   0.0, 0.0, 9e-4]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = MockGps()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
