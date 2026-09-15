#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cloud_filter.py -- 点云体素去重节点

订阅 /cloud_registered_body (FAST_LIO 每帧输出, xyzinormal 32字节),
把落进同一体素(默认5cm)的多个点合并为一个(保留该体素内最近的点),
发布到 /cloud_body_filtered, 供 RViz/避障使用, 避免近距点堆叠臃肿。

用法:
    python3 ~/ros2_ws/scripts/cloud_filter.py
    python3 ~/ros2_ws/scripts/cloud_filter.py --ros-args -p voxel:=0.05
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct


class CloudFilter(Node):
    def __init__(self):
        super().__init__('cloud_filter')
        self.declare_parameter('in_topic', '/cloud_registered_body')
        self.declare_parameter('out_topic', '/cloud_body_filtered')
        self.declare_parameter('voxel', 0.05)     # 体素边长(米), 越小越稠密
        self.declare_parameter('print_every', 200)

        self.in_topic = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.voxel = float(self.get_parameter('voxel').value)
        self.print_every = int(self.get_parameter('print_every').value)
        self.count = 0

        self.sub = self.create_subscription(
            PointCloud2, self.in_topic, self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, self.out_topic, 10)
        self.get_logger().info(
            '%s -> %s, 体素=%.3fm' % (self.in_topic, self.out_topic, self.voxel))

    def cb(self, msg):
        n = msg.width * msg.height
        if n == 0:
            return
        # x y z intensity 恒在前4个float(offset 0/4/8/12)
        step = msg.point_step // 4
        arr = np.frombuffer(bytes(msg.data), dtype=np.float32) \
                .reshape(n, step)[:, :4].astype(np.float64)
        xyz = arr[:, :3]

        # 体素索引 -> 去重(同一体素只留1个点, np.unique 取首现)
        keys = np.floor(xyz / self.voxel).astype(np.int64)
        # 三维键打包成一维, 加偏移避免负数撞键
        keys += 1 << 20
        packed = (keys[:, 0] << 42) | (keys[:, 1] << 21) | keys[:, 2]
        _, idx = np.unique(packed, return_index=True)
        idx = np.sort(idx)
        out_pts = arr[idx]

        # 重组 PointCloud2 (xyz intensity, 16字节/点)
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = len(out_pts)
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * len(out_pts)
        out.is_dense = True
        out.data = struct.pack('<%df' % (4 * len(out_pts)),
                               *out_pts.astype(np.float32).ravel())
        self.pub.publish(out)

        self.count += 1
        if self.print_every > 0 and self.count % self.print_every == 0:
            self.get_logger().info('帧%d: %d 点 -> %d 点 (体素%.2fm)'
                                   % (self.count, n, len(out_pts), self.voxel))


def main():
    rclpy.init()
    node = CloudFilter()
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
