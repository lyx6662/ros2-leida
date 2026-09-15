#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_loader —— 加载已保存的 PCD 地图,发布 /map(模拟实车 map_server 流程)

实车标准流程:
  建图(FAST-LIO2)→ 保存 .pcd → pcd2pgm 转栅格 → map_server 加载
本节点把中间几步合并:直接读 PCD → 投影 2D 栅格 → 发布 latched /map。

用法:
  # 先停掉实时建图节点(避免两个 /map 发布者冲突):
  pkill -f mock_mapper

  # 加载指定地图:
  ros2 run robot_sim map_loader --ros-args -p pcd_file:=~/ros2_ws/maps/map_xxx.pcd
  # 或不加参数,自动加载 ~/ros2_ws/maps/ 里最新的一份:
  ros2 run robot_sim map_loader

发布: /map (OccupancyGrid, latched) + /map_cloud (PointCloud2, latched)
"""
import glob
import os
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid

# 与 mock_mapper 保持一致的投影参数
GRID_RES = 0.1
GRID_HALF = 15.0
GRID_N = int(GRID_HALF * 2 / GRID_RES)
Z_BAND = (0.05, 1.5)
MAPS_DIR = os.path.expanduser('~/ros2_ws/maps')


def find_latest_pcd():
    files = sorted(glob.glob(os.path.join(MAPS_DIR, '*.pcd')))
    return files[-1] if files else None


def read_pcd(path):
    """读取 ASCII PCD,返回 (N,3) float64(跳过头部)"""
    pts = []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) == 3:
                try:
                    pts.append([float(v) for v in parts])
                except ValueError:
                    pass  # 头部行
    return np.array(pts, dtype=np.float64)


class MapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')

        self.declare_parameter('pcd_file', '')
        pcd = self.get_parameter('pcd_file').value
        if not pcd:
            pcd = find_latest_pcd()
        if not pcd or not os.path.exists(pcd):
            self.get_logger().error(
                '找不到地图文件!请先建图并 /save_map,或指定 pcd_file 参数')
            raise SystemExit(1)

        pts = read_pcd(pcd)
        self.get_logger().info('加载地图: %s (%d 点)' % (pcd, len(pts)))

        # 体素去重(和建图时同规格)
        keys = np.floor(pts / 0.1 + 0.5).astype(np.int64)
        voxels = {}
        for k, p in zip(keys, pts):
            voxels[(k[0], k[1], k[2])] = p

        # ---- 点云消息 ----
        pl = list(voxels.values())
        cloud = PointCloud2()
        cloud.header.frame_id = 'map'
        cloud.height = 1
        cloud.width = len(pl)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(pl)
        cloud.is_dense = True
        cloud.data = struct.pack('<%df' % (3 * len(pl)),
                                 *[v for p in pl for v in p])

        # ---- 栅格消息:与实车 pcd2pgm 一致——PCD无射线信息,
        # 障碍带内=占据,其余=自由(默认已建完全场图) ----
        grid = np.zeros((GRID_N, GRID_N), dtype=np.int8)
        n_occ = 0
        for k, (x, y, z) in voxels.items():
            if not (Z_BAND[0] <= z <= Z_BAND[1]):
                continue
            gx = int((x + GRID_HALF) / GRID_RES)
            gy = int((y + GRID_HALF) / GRID_RES)
            if 0 <= gx < GRID_N and 0 <= gy < GRID_N:
                grid[gy, gx] = 100
                n_occ += 1
        # 占据格膨胀1格(保守处理,防规划贴墙)
        occ = grid == 100
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                grid[np.roll(np.roll(occ, dy, 0), dx, 1)] = 100

        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.info.resolution = GRID_RES
        g.info.width = GRID_N
        g.info.height = GRID_N
        g.info.origin.position.x = -GRID_HALF
        g.info.origin.position.y = -GRID_HALF
        g.info.origin.orientation.w = 1.0
        g.data = grid.ravel().tolist()

        # ---- 发布(latched) ----
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(PointCloud2, 'map_cloud', latched)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'map', latched)

        # 周期性重发(便宜且保证晚加入的订阅者拿到)
        self.timer = self.create_timer(5.0, self.republish)
        self.cloud_msg = cloud
        self.grid_msg = g

        self.get_logger().info(
            '地图已就绪: %d 体素, %d 占据格 | 发布 /map 与 /map_cloud'
            % (len(pl), n_occ))

    def republish(self):
        stamp = self.get_clock().now().to_msg()
        self.cloud_msg.header.stamp = stamp
        self.grid_msg.header.stamp = stamp
        self.cloud_pub.publish(self.cloud_msg)
        self.grid_pub.publish(self.grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapLoader()
    node.republish()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
