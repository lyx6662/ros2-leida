#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mock_lidar —— 模拟 Mid-360S 点云(无实物阶段,带视线遮挡)

模拟一个 40m x 40m 的巡检场地(从 scenario.py 加载):
  - 四周围墙
  - 场内 8 根立柱(模拟设备/灯杆)
  - 4 台设备(长方体,巡检对象)
  - 1 个充电桩
  - 3 个动态行走人员(往返折线)

真实感设计(为了"手动建图"体验):
  1. 视线遮挡:目标点与雷达之间的射线若被墙/立柱/设备/桩挡住,则不可见
     (numpy 矢量化射线求交,10Hz 无压力)
  2. 量程限制:默认 18m,超出量程的点不返回(真实 Mid-360 是 40m,
     这里调小是为了让"开车探索"有意义)
  3. 建图模式下人员点剔除,避免鬼影障碍
  => 必须像真车一样开车绕场,地图才会一点点长出来

时间戳约定:点云头时间戳 = 所用 TF 的时刻,建图节点按该时刻反查 TF,
正逆变换严格抵消,运动中无拖影。

发布: /livox/lidar (sensor_msgs/PointCloud2) 10Hz
      /lidar_blind (nav_msgs/OccupancyGrid,1Hz,锁存) —— 当前视角盲区
真机驱动发的是 livox CustomMsg(FAST-LIO2 用);本节点发 PointCloud2。
"""
import math
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from robot_sim import scenario
from robot_sim.scenario import WALL, SEGMENTS, POLES, quat_to_rot, person_points

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


def build_world():
    """生成世界系(odom系)静态障碍点(米)——遮挡物表面采样点(全由 WALL/DEVICES/POLES 推导)"""
    pts = []
    # 墙:步长 0.2m,沿 4 条边
    ts = np.arange(-WALL, WALL + 0.1, 0.2)
    for side in (-WALL, WALL):
        for t in ts:
            for z in (0.2, 1.2, 2.0):
                pts.append((side, t, z))
                pts.append((t, side, z))
    # 立柱
    for cx, cy, r in POLES:
        for a in [i * 0.5 for i in range(13)]:
            for z in (0.2, 0.8, 1.4, 2.0):
                pts.append((cx + r * math.cos(a),
                            cy + r * math.sin(a), z))
    # 房屋墙体(带门洞):每段线段沿 0.2m 步长采样,墙高内取 3 层
    from robot_sim.scenario import HOUSE_WALLS, HOUSE_H
    for (x1, y1, x2, y2) in HOUSE_WALLS:
        L = math.hypot(x2 - x1, y2 - y1)
        n = max(int(L / 0.2), 1)
        for i in range(n + 1):
            u = i / n
            px, py = x1 + (x2 - x1) * u, y1 + (y2 - y1) * u
            for z in (0.2, 0.8, 1.4):
                pts.append((px, py, z))
    # 房内家具:四壁采样
    for cx, cy, w, d, h in scenario.HOUSE_FURNITURE:
        for t in np.arange(-w/2, w/2 + 0.1, 0.25):
            for z in (0.2, 0.5, min(0.9, h)):
                pts.append((cx + t, cy - d/2, z))
                pts.append((cx + t, cy + d/2, z))
        for t in np.arange(-d/2, d/2 + 0.1, 0.25):
            for z in (0.2, 0.5, min(0.9, h)):
                pts.append((cx - w/2, cy + t, z))
                pts.append((cx + w/2, cy + t, z))
    # 设备:每台长方体 4 条竖边采样
    for _, cx, cy, w, d, _, _ in scenario.DEVICES:
        for t in np.arange(-w/2, w/2 + 0.1, 0.25):
            for z in (0.2, 0.5, 0.9, 1.4, 1.7):
                pts.append((cx + t, cy - d/2, z))
                pts.append((cx + t, cy + d/2, z))
        for t in np.arange(-d/2, d/2 + 0.1, 0.25):
            for z in (0.2, 0.5, 0.9, 1.4, 1.7):
                pts.append((cx - w/2, cy + t, z))
                pts.append((cx + w/2, cy + t, z))
    # 充电桩
    c = scenario.CHARGER
    for t in np.arange(-c['w']/2, c['w']/2 + 0.05, 0.1):
        for z in (0.2, 0.4):
            pts.append((c['x'] + t, c['y'] - c['d']/2, z))
            pts.append((c['x'] + t, c['y'] + c['d']/2, z))
        for t in np.arange(-c['d']/2, c['d']/2 + 0.05, 0.1):
            for z in (0.2, 0.4):
                pts.append((c['x'] - c['w']/2, c['y'] + t, z))
                pts.append((c['x'] + c['w']/2, c['y'] + t, z))
    return np.array(pts, dtype=np.float64)


def visible_mask(px, py, targets, max_range):
    """计算每个目标点是否可见(2D 射线 vs 线段/圆遮挡)

    px, py: 雷达位置
    targets: (N,2) 目标点 xy
    max_range: 量程(米)
    返回 (N,) bool
    """
    d = targets - np.array([px, py])                 # (N,2) 射线方向
    dist = np.hypot(d[:, 0], d[:, 1])
    ok = dist < max_range
    ok &= dist > 1e-6
    if not ok.any():
        return ok

    # ---- 线段遮挡:t = cross(S1-P, e)/cross(d,e), u 同理 ----
    seg = np.array(SEGMENTS)                          # (M,4)
    s1 = seg[:, :2]                                   # (M,2)
    e = seg[:, 2:] - s1                               # (M,2)
    # (N,M) 广播
    w = s1[None, :, :] - np.array([px, py])[None, None, :]
    cross_de = d[:, None, 0]*e[None, :, 1] - d[:, None, 1]*e[None, :, 0]
    cross_we = w[..., 0]*e[None, :, 1] - w[..., 1]*e[None, :, 0]
    cross_wd = w[..., 0]*d[:, None, 1] - w[..., 1]*d[:, None, 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.where(np.abs(cross_de) > 1e-12, cross_we / cross_de, -1.0)
        u = np.where(np.abs(cross_de) > 1e-12, cross_wd / cross_de, -1.0)
    hit_seg = (t > 0.02) & (t < 0.98) & (u >= 0.0) & (u <= 1.0)
    ok &= ~hit_seg.any(axis=1)

    # ---- 圆形遮挡:射线最近点参数 tc,距离 < r 则挡住 ----
    cen = np.array([(c[0], c[1]) for c in POLES])     # (K,2)
    r = np.array([c[2] for c in POLES])               # (K,)
    dd = (d*d).sum(axis=1)                            # (N,)
    with np.errstate(divide='ignore', invalid='ignore'):
        tc = ((cen[None, :, 0]-px)*d[:, None, 0] +
              (cen[None, :, 1]-py)*d[:, None, 1]) / dd[:, None]
    closest = np.array([px, py])[None, None, :] + \
        np.clip(tc, 0.02, 0.98)[..., None] * d[:, None, :]   # (N,K,2)
    dc = np.hypot(closest[..., 0]-cen[None, :, 0],
                  closest[..., 1]-cen[None, :, 1])
    hit_cir = (dc < r[None, :]) & (tc > 0.02) & (tc < 0.98)
    ok &= ~hit_cir.any(axis=1)

    return ok


class MockLidar(Node):
    def __init__(self):
        super().__init__('livox_mock')

        self.declare_parameter('frame_id', 'livox_frame')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('max_range', 18.0)  # 米;实车 Mid-360 是 40m
        self.frame = self.get_parameter('frame_id').value
        self.max_range = self.get_parameter('max_range').value
        rate = self.get_parameter('rate').value

        self.world = build_world()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_mode = 'mapping'           # 默认建图,见 /map_mode 订阅
        self.people_in_mapping = False      # 建图模式下若 True 则保留人员点

        self.pub = self.create_publisher(
            PointCloud2, 'livox/lidar', qos_profile_sensor_data)
        self.create_subscription(String, 'map_mode', self._map_mode_cb, 10)
        # 雷达盲区(锁存,前端打开页面立刻有)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.blind_pub = self.create_publisher(OccupancyGrid, 'lidar_blind', latched)
        self.create_timer(1.0 / rate, self.publish_cloud)
        self.create_timer(1.0, self.publish_blind)
        self.get_logger().info(
            'mock_lidar 启动:%d 个静态障碍点, %dHz, 量程%.0fm, 带视线遮挡'
            % (len(self.world), rate, self.max_range))

    def _map_mode_cb(self, msg):
        self.map_mode = msg.data

    def publish_cloud(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', self.frame, rclpy.time.Time(),
                Duration(seconds=0.0))
        except Exception as e:
            self.get_logger().warn('等待 TF odom->%s: %s' % (self.frame, e),
                                   throttle_duration_sec=3)
            return

        t = tf.transform.translation
        R = quat_to_rot(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w)

        # 静态障碍 + 动态人员(建图模式默认剔除,避免体素存档出现鬼影)
        world = self.world
        if self.map_mode != 'mapping' or self.people_in_mapping:
            world = np.vstack([self.world, person_points(time.time())])

        # 视线遮挡 + 量程过滤(2D,世界系)
        vis = visible_mask(t.x, t.y, world[:, :2], self.max_range)
        pts_w = world[vis]

        # 世界(odom系) -> 雷达系:  P_lidar = R^T (P_world - t)
        # 行向量批量运算: (R^T @ p) 等价于 (p @ R)
        rel = pts_w - np.array([t.x, t.y, t.z])
        pts = rel @ R

        cloud = PointCloud2()
        cloud.header.stamp = tf.header.stamp
        cloud.header.frame_id = self.frame
        cloud.height = 1
        cloud.width = len(pts)
        cloud.fields = FIELDS
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(pts)
        cloud.is_dense = True
        cloud.data = struct.pack('<%df' % (3 * len(pts)),
                                *[v for p in pts for v in p])
        self.pub.publish(cloud)

    def publish_blind(self):
        """1Hz 发布当前雷达位姿的视野盲区(粗 0.5m 网格)
        复 visible_mask:被墙/设备/桩遮挡 或 超量程 = 盲区(100)
        墙外=未知(-1);可见=0。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', self.frame, rclpy.time.Time(),
                Duration(seconds=0.0))
        except Exception as e:
            self.get_logger().warn('盲区计算等 TF: %s' % e, throttle_duration_sec=3)
            return

        t = tf.transform.translation
        res = 0.5
        n = int(2 * WALL / res)
        axis = (np.arange(n) + 0.5) * res - WALL
        X, Y = np.meshgrid(axis, axis)                 # gy=0 → y=-WALL(最南)
        targets = np.stack([X.ravel(), Y.ravel()], axis=1)
        vis = visible_mask(t.x, t.y, targets, self.max_range)
        grid = np.where(vis, 0, 100).astype(np.int8).reshape(n, n)
        grid[(np.abs(X) > WALL - 0.25) | (np.abs(Y) > WALL - 0.25)] = -1
        g = OccupancyGrid()
        g.header.stamp = tf.header.stamp
        g.header.frame_id = 'map'
        g.info.resolution = res; g.info.width = n; g.info.height = n
        g.info.origin.position.x = -WALL; g.info.origin.position.y = -WALL
        g.info.origin.orientation.w = 1.0
        g.data = grid.ravel().tolist()
        self.blind_pub.publish(g)


def main(args=None):
    rclpy.init(args=args)
    node = MockLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
