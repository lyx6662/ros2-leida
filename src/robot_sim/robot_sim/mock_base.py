#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mock_base —— 模拟冰达 base_control_ros2 底盘驱动(无实物阶段)

接口与真机驱动完全对齐,可直接替换:
  订阅: /cmd_vel  (geometry_msgs/Twist)   全向速度指令 x/y/yaw
  发布: /odom    (nav_msgs/Odometry)      50Hz 轮式里程计+IMU融合航向
  发布: /imu     (sensor_msgs/Imu)        50Hz 底盘板载IMU
  发布: /battery (sensor_msgs/BatteryState) 10Hz 电压电流
  TF:   odom -> base_footprint            50Hz

运动学:四轮四转全向模型,cmd_vel 三通道直接积分。
安全:cmd_vel 超过 0.5s 未更新自动归零(模拟看门狗)。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from tf2_ros import TransformBroadcaster

from robot_sim.scenario import CHARGER, CHARGER_RADIUS

CMD_TIMEOUT = 0.5  # 秒,超时后速度归零


def yaw_to_quat(yaw):
    """欧拉角 yaw -> 四元数 (x, y, z, w)"""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class MockBase(Node):
    def __init__(self):
        super().__init__('base_control')  # 节点名与真驱动一致,上层无感切换

        # ---- 参数(与真机一致,可用 launch 覆盖) ----
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('max_vx', 3.0)   # m/s 限幅(手动遥控最快 3,导航由 DWB 自限 1.5)
        self.declare_parameter('max_vy', 3.0)
        self.declare_parameter('max_wz', 1.5)    # rad/s
        # 电池模型
        self.declare_parameter('battery_start', 100.0)   # 初始电量 %
        self.declare_parameter('discharge_rate', 0.05)   # %/s,默认 100→20 约 27min
        self.declare_parameter('charge_rate', 0.5)       # %/s,回充演示节奏
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value
        self.battery = self.get_parameter('battery_start').value / 100.0
        self.discharge_rate = self.get_parameter('discharge_rate').value
        self.charge_rate = self.get_parameter('charge_rate').value

        # ---- 状态 ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.prev_time = self.get_clock().now()
        self.bat_prev = self.get_clock().now()

        # ---- 接口 ----
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        # 与真驱动同款 QoS(best_effort + 浅队列),保证与 Nav2 对齐
        self.odom_pub = self.create_publisher(
            Odometry, 'odom', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(
            Imu, 'imu', qos_profile_sensor_data)
        self.bat_pub = self.create_publisher(
            BatteryState, 'battery', qos_profile_sensor_data)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id = self.base_frame
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu'

        self.create_timer(1.0 / 50, self.odom_tick)     # 50Hz 里程计+TF+IMU
        self.create_timer(1.0 / 10, self.battery_tick)  # 10Hz 电池

        self.get_logger().info('mock_base 启动:等待 /cmd_vel ...')

    # ---------------- cmd_vel ----------------
    def cmd_vel_cb(self, msg: Twist):
        clamp = lambda v, m: max(-m, min(m, v))
        self.vx = clamp(msg.linear.x, self.max_vx)
        self.vy = clamp(msg.linear.y, self.max_vy)
        self.wz = clamp(msg.angular.z, self.max_wz)
        self.last_cmd_time = self.get_clock().now()

    # ---------------- 50Hz 主循环 ----------------
    def odom_tick(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        # 看门狗:超时归零(真机行为一致)
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > CMD_TIMEOUT:
            self.vx = self.vy = self.wz = 0.0

        # 全向运动学积分(车体速度 -> 世界系)
        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.yaw += self.wz * dt

        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        stamp = now.to_msg()

        # /odom
        self.odom_msg.header.stamp = stamp
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.z = qz
        self.odom_msg.pose.pose.orientation.w = qw
        self.odom_msg.twist.twist.linear.x = self.vx
        self.odom_msg.twist.twist.linear.y = self.vy
        self.odom_msg.twist.twist.angular.z = self.wz
        self.odom_pub.publish(self.odom_msg)

        # TF odom -> base_footprint
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        # /imu(航向来自同一状态,模拟底盘IMU与轮速融合后的输出)
        self.imu_msg.header.stamp = stamp
        self.imu_msg.orientation.x = qx
        self.imu_msg.orientation.y = qy
        self.imu_msg.orientation.z = qz
        self.imu_msg.orientation.w = qw
        self.imu_msg.angular_velocity.z = self.wz
        self.imu_pub.publish(self.imu_msg)

    # ---------------- 10Hz 电池(充放电模型) ----------------
    def battery_tick(self):
        now = self.get_clock().now()
        dt = (now - self.bat_prev).nanoseconds * 1e-9
        self.bat_prev = now
        dt = min(dt, 0.5)

        in_dock = math.hypot(self.x - CHARGER['x'], self.y - CHARGER['y']) < CHARGER_RADIUS
        if in_dock and self.battery < 1.0:
            self.battery = min(1.0, self.battery + self.charge_rate * dt / 100.0)
            cur = -6.0 if self.battery < 0.999 else 0.0      # 充电电流为负
            status = BatteryState.POWER_SUPPLY_STATUS_FULL if self.battery >= 0.999 \
                else BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            self.battery = max(0.0, self.battery - self.discharge_rate * dt / 100.0)
            moving = abs(self.vx) + abs(self.vy) + abs(self.wz) > 0.05
            cur = 2.5 if moving else 0.9
            status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.base_frame
        # 6S 锂电简化:满电 25.2V → 空电 21V
        msg.voltage = 21.0 + 4.2 * self.battery
        msg.current = cur
        msg.percentage = self.battery
        msg.power_supply_status = status
        self.bat_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
