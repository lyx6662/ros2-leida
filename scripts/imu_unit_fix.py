#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_unit_fix.py -- IMU 加速度单位换算节点 (g -> m/s^2)

背景:
    Livox Mid-360S 内置 IMU 输出的 linear_acceleration 单位是 g
    (重力加速度倍数)。静止时读数约 1.0,而 FAST-LIO 内部按 m/s^2 处理,
    重力常量取 9.81,于是产生约 8.8 m/s^2 的常值残差,
    导致雷达一运动 /Odometry 位姿就被二次积分带飞。

本节点订阅原始 IMU 话题,把 linear_acceleration 乘以 9.80665 后转发,
角速度和姿态原样透传。之后把 FAST-LIO 配置里的 IMU 话题改成输出话题即可。

用法:
    python3 imu_unit_fix.py
    python3 imu_unit_fix.py --ros-args -p in_topic:=/livox/imu \
                                       -p out_topic:=/livox/imu_si \
                                       -p acc_scale:=9.80665

验证:
    修正后静止时 linear_acceleration.z 应约为 9.7 左右。
    ros2 topic echo /livox/imu_si --once
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

G_M_S2 = 9.80665


class ImuUnitFix(Node):
    def __init__(self):
        super().__init__('imu_unit_fix')

        self.declare_parameter('in_topic', '/livox/imu')
        self.declare_parameter('out_topic', '/livox/imu_si')
        self.declare_parameter('acc_scale', G_M_S2)
        self.declare_parameter('print_every', 500)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.scale = float(self.get_parameter('acc_scale').value)
        self.print_every = int(self.get_parameter('print_every').value)
        self.count = 0

        # 订阅用 BEST_EFFORT: 既能接 best_effort 发布者, 也能接 reliable 发布者
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        # 发布用 RELIABLE: 既能被 reliable 订阅者接收, 也能被 best_effort 订阅者接收
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )

        self.pub = self.create_publisher(Imu, out_topic, pub_qos)
        self.sub = self.create_subscription(Imu, in_topic, self.cb, sub_qos)

        self.get_logger().info(
            'imu_unit_fix started: %s -> %s, acc_scale=%.5f'
            % (in_topic, out_topic, self.scale)
        )

    def cb(self, msg):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration.x = msg.linear_acceleration.x * self.scale
        out.linear_acceleration.y = msg.linear_acceleration.y * self.scale
        out.linear_acceleration.z = msg.linear_acceleration.z * self.scale
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out)

        self.count += 1
        if self.print_every > 0 and self.count % self.print_every == 0:
            a = out.linear_acceleration
            norm = (a.x * a.x + a.y * a.y + a.z * a.z) ** 0.5
            self.get_logger().info(
                'acc = (%.4f, %.4f, %.4f)  |a| = %.4f  [m/s^2]'
                % (a.x, a.y, a.z, norm)
            )


def main():
    rclpy.init()
    node = ImuUnitFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
