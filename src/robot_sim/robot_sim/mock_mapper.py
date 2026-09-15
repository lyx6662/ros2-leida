#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mock_mapper —— 模拟"建图"过程(无实物阶段)

订阅 /livox/lidar,把每帧点云经 TF 变换到 map 系并做体素累积:
  发布 /map_cloud (PointCloud2)  累积点云地图(latched)
  发布 /map       (OccupancyGrid) 2D 栅格地图(Nav2 用):
                    100=占据  0=射线扫过的自由空间  -1=未探索区域
  服务 /save_map  存盘为 .pcd 文件
  服务 /reset_map 清空地图,重新建图(测试"自己开车建图"用)

自由空间雕刻:沿每条雷达射线从机器人位置到回拨点逐格标 0(占用格除外),
numpy 矢量化实现。实车阶段由 FAST-LIO2(建点云图)+ pcd2pgm(转栅格)替代,
话题语义一致。
"""
import math
import os
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from example_interfaces.srv import Trigger

# 栅格地图参数
GRID_RES = 0.1          # 米/格
GRID_HALF = 15.0        # 地图半边长(米),30x30m 场地
GRID_N = int(GRID_HALF * 2 / GRID_RES)
Z_BAND = (0.05, 1.5)    # 投影到2D的高度区间(滤地面/过高点)


def quat_to_rot(x, y, z, w):
    """四元数 -> 3x3 旋转矩阵"""
    return [
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ]


class MockMapper(Node):
    def __init__(self):
        super().__init__('map_mapper')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('voxel_size', 0.1)
        self.map_frame = self.get_parameter('map_frame').value
        self.voxel = self.get_parameter('voxel_size').value

        # 体素字典:(ix,iy,iz) -> (x,y,z),天然去重
        self.voxels = {}
        # 自由空间栅格(2D bool):射线扫过的格子
        self.free_grid = np.zeros((GRID_N, GRID_N), dtype=bool)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(PointCloud2, 'map_cloud', latched)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'map', latched)
        self.create_subscription(
            PointCloud2, 'livox/lidar', self.cloud_cb, qos_profile_sensor_data)

        self.create_service(Trigger, 'save_map', self.save_map_cb)
        self.create_service(Trigger, 'reset_map', self.reset_map_cb)
        self.create_timer(1.0, self.publish_maps)   # 1Hz 发布累积结果
        self.get_logger().info(
            'mock_mapper 启动:体素%.2fm, 栅格%dx%d, 未探索区=-1,'
            ' 服务 /save_map /reset_map 待命' % (self.voxel, GRID_N, GRID_N))

    # ---------------- 点云累积 + 自由空间雕刻 ----------------
    def cloud_cb(self, msg: PointCloud2):
        # 按点云自身时间戳查 TF(与雷达所用 TF 严格同刻,无拖影)
        try:
            t_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            # 超时0:非阻塞。TF未就绪立即抛异常丢帧,不堵执行器
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, t_stamp,
                Duration(seconds=0.0))
        except Exception as e:
            self.get_logger().warn('TF 查询失败: %s' % e, throttle_duration_sec=5)
            return

        tr = tf.transform.translation
        R = quat_to_rot(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w)

        n = msg.width * msg.height
        if n == 0:
            return
        data = np.frombuffer(bytes(msg.data), dtype=np.float32) \
                 .reshape(n, 3).astype(np.float64)
        # 旋转 + 平移 -> map 系
        pts = data @ np.array(R).T + np.array([tr.x, tr.y, tr.z])

        # 占据体素
        keys = np.floor(pts / self.voxel + 0.5).astype(np.int64)
        for k, p in zip(keys, pts):
            tk = (k[0], k[1], k[2])
            if tk not in self.voxels:
                self.voxels[tk] = (p[0], p[1], p[2])

        # 自由空间雕刻:从机器人位置到每个回拨点沿直线标 free
        # (2D投影;z 限制在高度带内,避免地面点把沿途全标掉)
        band = (pts[:, 2] >= Z_BAND[0]) & (pts[:, 2] <= Z_BAND[1])
        if band.any():
            ex, ey = pts[band, 0], pts[band, 1]
            self.carve_free(tr.x, tr.y, ex, ey)

    def carve_free(self, px, py, ex, ey):
        """沿射线 (px,py)->(ex,ey) 标记自由格(numpy 矢量化)"""
        gx0 = int((px + GRID_HALF) / GRID_RES)
        gy0 = int((py + GRID_HALF) / GRID_RES)
        gxe = ((ex + GRID_HALF) / GRID_RES).astype(np.int64)
        gye = ((ey + GRID_HALF) / GRID_RES).astype(np.int64)

        steps = np.maximum(np.abs(gxe - gx0), np.abs(gye - gy0))
        max_s = int(steps.max()) if len(steps) else 0
        if max_s == 0:
            return
        # 统一步数插值:(N, max_s)
        frac = np.linspace(0.0, 1.0, max_s + 1)
        xs = gx0 + np.round((gxe - gx0)[:, None] * frac[None, :]).astype(np.int64)
        ys = gy0 + np.round((gye - gy0)[:, None] * frac[None, :]).astype(np.int64)
        valid = (xs >= 0) & (xs < GRID_N) & (ys >= 0) & (ys < GRID_N)
        self.free_grid[ys[valid], xs[valid]] = True
        # 注意:free 会在发布时让位于占据(occ 优先)

    # ---------------- 发布点云图 + 栅格图 ----------------
    def publish_maps(self):
        if not self.voxels:
            return

        # 1) 累积点云
        pts = list(self.voxels.values())
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = self.map_frame
        cloud.height = 1
        cloud.width = len(pts)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(pts)
        cloud.is_dense = True
        cloud.data = struct.pack('<%df' % (3 * len(pts)),
                                *[v for p in pts for v in p])
        self.cloud_pub.publish(cloud)

        # 2) 2D 栅格:占据 > 自由 > 未知
        occ = np.zeros((GRID_N, GRID_N), dtype=bool)
        for (_, _, kz), (x, y, _) in self.voxels.items():
            if not (Z_BAND[0] <= kz * self.voxel <= Z_BAND[1]):
                continue
            gx = int((x + GRID_HALF) / GRID_RES)
            gy = int((y + GRID_HALF) / GRID_RES)
            if 0 <= gx < GRID_N and 0 <= gy < GRID_N:
                occ[gy, gx] = True

        grid = np.full((GRID_N, GRID_N), -1, dtype=np.int8)   # 未探索
        grid[self.free_grid] = 0                              # 已扫过自由区
        grid[occ] = 100                                       # 障碍(优先级最高)

        g = OccupancyGrid()
        g.header.stamp = cloud.header.stamp
        g.header.frame_id = self.map_frame
        g.info.resolution = GRID_RES
        g.info.width = GRID_N
        g.info.height = GRID_N
        g.info.origin.position.x = -GRID_HALF
        g.info.origin.position.y = -GRID_HALF
        g.info.origin.orientation.w = 1.0
        g.data = grid.ravel().tolist()
        self.grid_pub.publish(g)

        self.get_logger().info(
            '地图: %d 体素 | 占据 %d 格 | 自由 %d 格'
            % (len(pts), int(occ.sum()), int(self.free_grid.sum())),
            throttle_duration_sec=5)

    # ---------------- 服务 ----------------
    def save_map_cb(self, request, response):
        out_dir = os.path.expanduser('~/ros2_ws/maps')
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, 'map_%s.pcd'
                            % time.strftime('%Y%m%d_%H%M%S'))
        pts = list(self.voxels.values())
        with open(path, 'w') as f:
            f.write('# .PCD v0.7 - mock_mapper\n')
            f.write('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
                    'TYPE F F F\nCOUNT 1 1 1\n')
            f.write('WIDTH %d\nHEIGHT 1\n' % len(pts))
            f.write('VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n'
                    % len(pts))
            for x, y, z in pts:
                f.write('%.3f %.3f %.3f\n' % (x, y, z))
        response.success = True
        response.message = '已保存 %d 点 -> %s' % (len(pts), path)
        self.get_logger().info(response.message)
        return response

    def reset_map_cb(self, request, response):
        n = len(self.voxels)
        self.voxels.clear()
        self.free_grid[:] = False
        response.success = True
        response.message = '地图已清空(原 %d 体素),开车重新建图吧' % n
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
