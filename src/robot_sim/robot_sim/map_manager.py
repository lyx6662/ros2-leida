#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_manager —— 地图模式统一管理器(建图 / 保存 / 加载,一个节点全包)

取代原来 mock_mapper(实时建图)+ map_loader(加载存档)两个节点,
用"模式"组织,网页/命令行都能切,不再需要杀进程:

  模式:
    mapping —— 实时建图:订阅 /livox/lidar 累积成图,未扫区=未知
    loaded  —— 导航模式:已加载存档 pcd,/map 来自文件,忽略雷达

  服务(全部 example_interfaces/srv/Trigger):
    /map_mode/start_mapping  清空并进入建图模式
    /map_mode/save           保存当前地图到 ~/ros2_ws/maps/xxx.pcd
    /map_mode/load_latest    加载最新存档,进入导航模式

  话题:
    发布 /map      (latched)  显示用:建图模式=实时图,导航模式=存档图
    发布 /nav_map  (latched)  导航专用:永远是"已加载的存档地图"
                              (Nav2 订阅它;没加载过存档时不发布,导航不可用)
    发布 /map_cloud (latched) 累积点云(RViz)
    发布 /map_mode (std_msgs/String) 当前模式:'mapping' / 'loaded'

用法: ros2 run robot_sim map_manager
      (参数 start_mode: mapping[默认] / loaded; pcd_file: 指定存档)
"""
import glob
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
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from example_interfaces.srv import Trigger

GRID_RES = 0.05        # 栅格分辨率(米)。1cm 不可行: 栅格数x100, 消息/内存爆炸且超雷达精度
GRID_HALF = 22.0         # 40m 场地(WALL=20)+ 2m 边距;避开 np.roll 绕圈需 > WALL+0.1
GRID_N = int(GRID_HALF * 2 / GRID_RES)
Z_BAND = (0.05, 1.5)
CARVE_STEP = 0.05   # 动态清除的射线采样步长(米), 与体素一致
MAPS_DIR = os.path.expanduser('~/ros2_ws/maps')


def quat_to_rot(x, y, z, w):
    return [
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ]


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('voxel_size', 0.05)   # 3D体素与栅格分辨率保持一致
        self.declare_parameter('dynamic_clear_en', True)  # 射线穿透清除动态残留
        self.declare_parameter('carve_every', 5)     # 每5帧执行一次3D清除(控CPU)
        self.declare_parameter('skip_end', 0.25)     # 命中点前保护距离(米)
        self.declare_parameter('max_voxels', 600000) # 体素上限,防数据无限膨胀
        self.declare_parameter('insert_votes', 3)    # 入库投票: 同一体素需命中N次才入图(3票可挡快走行人)
        self.declare_parameter('vote_window', 30)    # 投票时间窗(帧): 超窗票数清零,防固定反光慢慢凑票
        self.declare_parameter('carve_confirm', 5)   # 删除确认: 需被穿透N轮(每轮carve_every帧)才删
        self.declare_parameter('start_mode', 'mapping')   # mapping / loaded
        self.declare_parameter('pcd_file', '')            # loaded模式用,空=最新
        self.map_frame = self.get_parameter('map_frame').value
        self.voxel = self.get_parameter('voxel_size').value

        self.voxels = {}                                   # (ix,iy,iz)->(x,y,z)
        self.dynamic_clear_en = self.get_parameter('dynamic_clear_en').value
        self.carve_every = max(1, int(self.get_parameter('carve_every').value))
        self.skip_end = float(self.get_parameter('skip_end').value)
        self.max_voxels = int(self.get_parameter('max_voxels').value)
        self.insert_votes = max(1, int(self.get_parameter('insert_votes').value))
        self.vote_window = max(1, int(self.get_parameter('vote_window').value))
        self.pending = {}      # 待定体素: key -> [票数, x, y, z, 最后投票帧]
        self.last_seen = {}    # 体素最后被真实命中的帧号(证据保护)
        self.free_cnt = {}     # 体素被射线穿透的累计次数(删除确认用)
        self.carve_confirm = max(1, int(self.get_parameter('carve_confirm').value))
        self.frame_cnt = 0
        self.cap_warned = False
        self.free_grid = np.zeros((GRID_N, GRID_N), dtype=bool)
        self.current_file = ''

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(PointCloud2, 'map_cloud', latched)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'map', latched)
        self.rej_pub = self.create_publisher(PointCloud2, 'map_rejected_grid',
                                             latched)  # 调试: 被噪点过滤拒绝的2D格
        self.nav_pub = self.create_publisher(OccupancyGrid, 'nav_map', latched)
        self.mode_pub = self.create_publisher(String, 'map_mode', latched)
        self.nav_grid_msg = None   # 导航专用存档图(加载后才有)

        # 雷达订阅常开,回调里按模式决定是否使用
        self.create_subscription(
            PointCloud2, 'livox/lidar', self.cloud_cb, qos_profile_sensor_data)

        self.create_service(Trigger, 'map_mode/start_mapping',
                            self.start_mapping_cb)
        self.create_service(Trigger, 'map_mode/save', self.save_cb)
        self.create_service(Trigger, 'map_mode/load_latest', self.load_latest_cb)

        self.create_timer(1.0, self.publish_maps)
        self.get_logger().info(
            'map_manager 启动 | 服务: /map_mode/start_mapping  /save  /load_latest')

        # 启动模式
        start = self.get_parameter('start_mode').value
        if start == 'loaded':
            ok, msg = self._load(self.get_parameter('pcd_file').value)
            self.get_logger().info(msg)
        else:
            self._enter_mapping()

    # ---------------- 模式切换 ----------------
    def _set_mode(self, mode):
        self.mode = mode
        m = String()
        m.data = mode
        self.mode_pub.publish(m)

    def _enter_mapping(self):
        self.voxels.clear()
        self.free_grid[:] = False
        self._set_mode('mapping')
        self.get_logger().info('▶ 建图模式:开车扫描,未扫区域显示为未知')

    def start_mapping_cb(self, request, response):
        self._enter_mapping()
        response.success = True
        response.message = '已清空,进入建图模式'
        return response

    def _load(self, path):
        if not path:
            files = sorted(glob.glob(os.path.join(MAPS_DIR, '*.pcd')))
            if not files:
                return False, '没有任何存档!先进建图模式扫一张并保存'
            path = files[-1]
        if not os.path.exists(path):
            return False, '文件不存在: %s' % path

        pts = []
        for line in open(path):
            parts = line.split()
            if len(parts) == 3:
                try:
                    pts.append([float(v) for v in parts])
                except ValueError:
                    pass
        if not pts:
            return False, '文件为空或格式不对: %s' % path

        arr = np.array(pts)
        keys = np.floor(arr / self.voxel + 0.5).astype(np.int64)
        self.voxels.clear()
        for k, p in zip(keys, arr):
            self.voxels[(k[0], k[1], k[2])] = (p[0], p[1], p[2])
        # 存档无射线信息:除障碍外视为自由(与实车pcd2pgm一致)
        self.free_grid[:] = True
        self.current_file = path
        self._build_nav_map()          # 生成导航专用 /nav_map
        self._set_mode('loaded')
        return True, '已加载 %s(%d 点),进入导航模式' % (
            os.path.basename(path), len(pts))

    @staticmethod
    def _denoise(occ, min_neighbors=1):
        """孤立噪点过滤: 3x3 邻居数为0的完全孤立占据格视为 SLAM 抖动噪点剔除
        (阈值取1: 成线/成片的墙体保留, 建图冷启动的稀疏虚线也保留;
         若地图噪点仍多, 可在调用处改 min_neighbors=2)"""
        padded = np.zeros((occ.shape[0] + 2, occ.shape[1] + 2), dtype=bool)
        padded[1:-1, 1:-1] = occ
        nb = np.zeros_like(occ)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nb += padded[1+dy:occ.shape[0]+1+dy, 1+dx:occ.shape[1]+1+dx]
        return occ & (nb >= min_neighbors)

    def _build_nav_map(self):
        """把当前占据体素固化为导航专用静态图(自由+占据,无未知区)"""
        grid = np.zeros((GRID_N, GRID_N), dtype=np.int8)
        for _, (x, y, z) in self.voxels.items():
            if not (Z_BAND[0] <= z <= Z_BAND[1]):
                continue
            gx = int((x + GRID_HALF) / GRID_RES)
            gy = int((y + GRID_HALF) / GRID_RES)
            if 0 <= gx < GRID_N and 0 <= gy < GRID_N:
                grid[gy, gx] = 100
        occ = self._denoise(grid == 100)
        # 1 格膨胀(防规划贴墙)—— 零填充切片膨胀,避免 np.roll 绕圈
        padded = np.zeros((occ.shape[0] + 2, occ.shape[1] + 2), dtype=bool)
        padded[1:-1, 1:-1] = occ
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                grid[padded[1+dy:occ.shape[0]+1+dy, 1+dx:occ.shape[1]+1+dx]] = 100
        g = OccupancyGrid()
        g.header.frame_id = self.map_frame
        g.info.resolution = GRID_RES
        g.info.width = GRID_N
        g.info.height = GRID_N
        g.info.origin.position.x = -GRID_HALF
        g.info.origin.position.y = -GRID_HALF
        g.info.origin.orientation.w = 1.0
        g.data = grid.ravel().tolist()
        self.nav_grid_msg = g

    def load_latest_cb(self, request, response):
        response.success, response.message = self._load('')
        self.get_logger().info(response.message)
        return response

    def save_cb(self, request, response):
        if not self.voxels:
            response.success = False
            response.message = '当前没有地图可保存,先建图'
            return response
        os.makedirs(MAPS_DIR, exist_ok=True)
        path = os.path.join(MAPS_DIR, 'map_%s.pcd'
                            % time.strftime('%Y%m%d_%H%M%S'))
        with open(path, 'w') as f:
            f.write('# .PCD v0.7 - map_manager\n')
            f.write('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
                    'TYPE F F F\nCOUNT 1 1 1\n')
            f.write('WIDTH %d\nHEIGHT 1\n' % len(self.voxels))
            f.write('VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n'
                    % len(self.voxels))
            for x, y, z in self.voxels.values():
                f.write('%.3f %.3f %.3f\n' % (x, y, z))
        self.current_file = path
        response.success = True
        response.message = '已保存 %d 点 -> %s' % (len(self.voxels),
                                                  os.path.basename(path))
        self.get_logger().info(response.message)
        return response

    # ---------------- 建图数据流 ----------------
    def cloud_cb(self, msg: PointCloud2):
        if self.mode != 'mapping':
            return   # 导航模式下忽略雷达,防止覆盖存档地图
        try:
            t_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
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
        # 按实际 point_step 解析(兼容 xyz 12字节与 FAST_LIO xyzinormal 32字节),
        # x/y/z 恒在前三个字段(offset 0/4/8)
        floats = np.frombuffer(bytes(msg.data), dtype=np.float32) \
            .reshape(n, msg.point_step // 4)
        data = floats[:, :3].astype(np.float64)
        pts = data @ np.array(R).T + np.array([tr.x, tr.y, tr.z])

        band = (pts[:, 2] >= Z_BAND[0]) & (pts[:, 2] <= Z_BAND[1])

        # ---- 动态清除: 射线中途穿过的旧体素 = 动态残留(人/移动杂物), 删除 ----
        # 每条射线从传感器出发到命中点, 命中点前 skip_end 米内不删(保护真实表面)
        # 注意: 用全高度点云作射线目标——若只用带内点(≤1.5m),
        # 头肩高度的残留永远等不到穿过它的射线(几何盲区)
        self.frame_cnt += 1
        if self.dynamic_clear_en and self.frame_cnt % self.carve_every == 0:
            tr3 = np.array([tr.x, tr.y, tr.z])
            d = pts - tr3
            dist = np.hypot(d[:, 0], d[:, 1], d[:, 2]) if len(pts) else d
            ok = dist > self.skip_end + 0.1
            d, dist = d[ok], dist[ok]
            if len(d):
                max_steps = int((dist.max() - self.skip_end) / CARVE_STEP)
                if 0 < max_steps < 20000:
                    fr = np.arange(1, max_steps + 1) * CARVE_STEP
                    samp = tr3 + d[:, None, :] * (fr[None, :, None]
                                                  / dist[:, None, None])
                    m = fr[None, :] <= (dist - self.skip_end)[:, None]
                    k = np.floor(samp / self.voxel + 0.5).astype(np.int64)
                    k += 1 << 20
                    pk = (k[..., 0] << 42) | (k[..., 1] << 21) | k[..., 2]
                    for p in np.unique(pk[m]).tolist():
                        key = ((p >> 42) - (1 << 20),
                               ((p >> 21) & ((1 << 21) - 1)) - (1 << 20),
                               (p & ((1 << 21) - 1)) - (1 << 20))
                        # 证据保护: 近2帧内被真实命中过的体素=真实表面,不删
                        if self.frame_cnt - self.last_seen.get(key, -9999) <= 2:
                            continue
                        # 删除需双重确认: 累计被穿透 N 轮 且 长时间无真实命中
                        # (远处细薄物偶尔被波束擦中即重置, 不会误删)
                        fc = self.free_cnt.get(key, 0) + 1
                        if (fc < self.carve_confirm
                                or self.frame_cnt - self.last_seen.get(key, -9999)
                                <= self.vote_window):
                            self.free_cnt[key] = fc
                            continue
                        self.voxels.pop(key, None)
                        self.last_seen.pop(key, None)
                        self.free_cnt.pop(key, None)

        # ---- 入库投票: 同一体素命中>=N次才入图, 单帧毛刺(反光/拖影)进不来 ----
        keys = np.floor(pts / self.voxel + 0.5).astype(np.int64)
        votes = self.insert_votes
        for k, p in zip(keys, pts):
            tk = (int(k[0]), int(k[1]), int(k[2]))
            if tk in self.voxels:
                self.last_seen[tk] = self.frame_cnt   # 记录真实命中时间(证据保护用)
                self.free_cnt.pop(tk, None)           # 被命中即清空穿透计数
                continue
            if len(self.voxels) >= self.max_voxels:
                if not self.cap_warned:
                    self.get_logger().warn(
                        '体素已达上限 %d, 之后新区域不再入库!建议重新建图或调大 max_voxels'
                        % self.max_voxels)
                    self.cap_warned = True
                break
            ent = self.pending.get(tk)
            if ent is None:
                self.pending[tk] = [1, p[0], p[1], p[2], self.frame_cnt]
            else:
                # 时间窗: 距上次投票超过 vote_window 帧, 旧票作废重新计
                if self.frame_cnt - ent[4] > self.vote_window:
                    ent[0] = 1
                else:
                    ent[0] += 1
                ent[4] = self.frame_cnt
                if ent[0] >= votes:
                    self.voxels[tk] = (ent[1], ent[2], ent[3])
                    self.last_seen[tk] = self.frame_cnt
                    self.free_cnt.pop(tk, None)
                    del self.pending[tk]
        if len(self.pending) > 200000:   # 待定池防膨胀(毛刺票数不会累积到入库)
            self.pending.clear()

        # 自由空间雕刻(高度带内的射线) — band 已在上面算好
        if band.any():
            gx0 = int((tr.x + GRID_HALF) / GRID_RES)
            gy0 = int((tr.y + GRID_HALF) / GRID_RES)
            gxe = ((pts[band, 0] + GRID_HALF) / GRID_RES).astype(np.int64)
            gye = ((pts[band, 1] + GRID_HALF) / GRID_RES).astype(np.int64)
            steps = np.maximum(np.abs(gxe - gx0), np.abs(gye - gy0))
            max_s = int(steps.max())
            if max_s:
                frac = np.linspace(0.0, 1.0, max_s + 1)
                xs = gx0 + np.round((gxe - gx0)[:, None]
                                    * frac[None, :]).astype(np.int64)
                ys = gy0 + np.round((gye - gy0)[:, None]
                                    * frac[None, :]).astype(np.int64)
                valid = ((xs >= 0) & (xs < GRID_N)
                         & (ys >= 0) & (ys < GRID_N))
                self.free_grid[ys[valid], xs[valid]] = True

    # ---------------- 发布 ----------------
    def publish_maps(self):
        if not self.voxels:
            return

        pts = list(self.voxels.values())
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = self.map_frame
        cloud.height = 1
        cloud.width = len(pts)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(pts)
        cloud.is_dense = True
        cloud.data = struct.pack('<%df' % (3 * len(pts)),
                                *[v for p in pts for v in p])
        self.cloud_pub.publish(cloud)

        # 占据层
        occ = np.zeros((GRID_N, GRID_N), dtype=bool)
        for (_, _, kz), (x, y, z) in self.voxels.items():
            if not (Z_BAND[0] <= z <= Z_BAND[1]):
                continue
            gx = int((x + GRID_HALF) / GRID_RES)
            gy = int((y + GRID_HALF) / GRID_RES)
            if 0 <= gx < GRID_N and 0 <= gy < GRID_N:
                occ[gy, gx] = True
        raw_occ = int(occ.sum())          # 调试: 过滤前占据数
        occ_raw_mask = occ.copy()
        occ = self._denoise(occ)
        rej = occ_raw_mask & ~occ         # 被过滤拒绝的格(诊断用)

        # 发布被拒格: 若聚成环/线 = 真实几何被误杀(调参方向反了);
        # 若散布随机 = 正常噪声过滤
        if rej.any():
            rys, rxs = np.nonzero(rej)
            rpts = np.stack([rxs * GRID_RES - GRID_HALF,
                             rys * GRID_RES - GRID_HALF,
                             np.full(len(rxs), 0.5)], axis=1)
            rm = PointCloud2()
            rm.header.stamp = self.get_clock().now().to_msg()
            rm.header.frame_id = self.map_frame
            rm.height = 1
            rm.width = len(rpts)
            rm.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
            rm.is_bigendian = False
            rm.point_step = 12
            rm.row_step = 12 * len(rpts)
            rm.is_dense = True
            rm.data = struct.pack('<%df' % (3 * len(rpts)),
                                  *rpts.astype(np.float32).ravel())
            self.rej_pub.publish(rm)

        # 栅格:建图模式=未知/自由/占据;导航模式=自由/占据(存档无射线)
        grid = (np.full((GRID_N, GRID_N), -1, dtype=np.int8)
                if self.mode == 'mapping'
                else np.zeros((GRID_N, GRID_N), dtype=np.int8))
        if self.mode == 'mapping':
            grid[self.free_grid] = 0
        grid[occ] = 100

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

        # 导航专用图:有存档就一直发布(即使回到建图模式,导航仍可用)
        if self.nav_grid_msg is not None:
            self.nav_grid_msg.header.stamp = cloud.header.stamp
            self.nav_pub.publish(self.nav_grid_msg)

        self.get_logger().info(
            '[%s] 地图: %d 体素, 占据 %d 格(过滤前%d)'
            % (self.mode, len(pts), int(occ.sum()), raw_occ),
            throttle_duration_sec=5)


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
