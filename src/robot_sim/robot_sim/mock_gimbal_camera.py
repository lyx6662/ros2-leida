#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mock_gimbal_camera —— 模拟可控云台相机(yaw+pitch 两轴,针孔模型渲染视频流)

场景与 mock_lidar 完全同一个世界(import 其 WALL/SEGMENTS/POLES 常量):
地面网格 + 六根立柱 + 2x2 箱柜@(3,-3) + 四面墙,相机画面、雷达点云、
栅格地图三者看到的是同一个场地。

接口约定:
  发布 /gimbal/image_raw/compressed (sensor_msgs/CompressedImage, JPEG, best_effort)
        —— 命名对齐 image_transport 的 compressed 话题习惯,真相机可无缝替换
  发布 /gimbal/state (geometry_msgs/Vector3, 锁存 latched)
        —— x=yaw(°), y=pitch(°),当前实际角(斜坡过程中缓慢逼近目标)
  订阅 /gimbal/cmd_pos (geometry_msgs/Vector3)
        —— 绝对角目标:x=yaw(°,右转为正), y=pitch(°,抬头为正), z 忽略
  订阅 /gimbal/cmd_vel (geometry_msgs/Twist)
        —— 速度模式:angular.z=yaw 角速度, angular.y=pitch 角速度(rad/s)
        与底盘一致的安全约定:0.5s 未收到新指令自动归零(看门狗);
        收到 cmd_pos 时清掉 cmd_vel 的速度,两个控制源不叠加。

云台动态:目标角恒速斜坡逼近(默认 yaw 90°/s, pitch 60°/s,接近真云台限速),
不跳变。限位 yaw ±170°, pitch -30°~+90°。

TF:节点自发 base_footprint 以下动态链
    gimbal_base → gimbal_yaw(绕z=yaw) → camera_link(绕y=pitch)
    (gimbal_base 的安装静态 TF 由 launch 里的 static_transform_publisher 发布)
    RViz 里把 TF + Image 打开即可看到视锥随云台转动。
    红线:绝不发 odom→base_footprint(与 mock_base 冲突)。

时间戳约定(防拖影,与 mock_lidar 相同):图像头时间戳 = 所用 TF 的时刻。
"""
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import CompressedImage
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# 场景常量与几何:从 scenario 取,与雷达/地图共用同一个世界
from robot_sim import scenario
from robot_sim.scenario import WALL, SEGMENTS, POLES, person_positions, \
    HOUSE_WALLS, HOUSE_H, HOUSE_FURNITURE, \
    person_points, PERSON_R, PERSON_H, CHARGER
from robot_sim.mock_lidar import quat_to_rot

Z_NEAR = 0.1        # 近平面(米),投影前裁剪
GRID_STEP = 1.0     # 地面网格间距(米)
POLE_H = 2.0        # 立柱高(米),与 mock_lidar 采样高度一致
WALL_H_VIS = 1.2    # 墙可视化色带高度

# ---------------- 颜色(BGR) ----------------
SKY_TOP = (95, 65, 35)      # 天空顶部偏深蓝
SKY_HOR = (240, 200, 170)   # 天空地平线偏暖
GROUND = (48, 50, 44)       # 地面墨绿灰
GRID_C = (70, 74, 64)       # 网格线
WALL_C = (90, 96, 110)      # 墙
DEVICE_C = (60, 120, 180)   # 设备(暖棕)
CHARGER_C = (70, 180, 80)   # 充电桩(亮绿)
HOUSE_C = (70, 70, 160)     # 房屋墙体(砖红)
POLE_C = (150, 140, 60)     # 立柱(冷灰蓝)
PERSON_C = (80, 120, 220)   # 人员(暖橙蓝)
HUD_C = (0, 220, 140)       # HUD 十字/文字


def _add_box_faces(faces, cx, cy, w, d, h, color, yaw=0.0):
    """长方体(底面贴地)5 面:4 侧 + 顶;底面贴地跳过"""
    ca, sa = math.cos(yaw), math.sin(yaw)
    c = lambda dx, dy: (cx + dx*ca - dy*sa, cy + dx*sa + dy*ca)
    a, b, c2, d_ = c(-w/2, -d/2), c(w/2, -d/2), c(w/2, d/2), c(-w/2, d/2)
    A, B, C, D = (a[0], a[1], h), (b[0], b[1], h), (c2[0], c2[1], h), (d_[0], d_[1], h)
    for quad in ((a, b, B, A), (b, c2, C, B), (c2, d_, D, C), (d_, a, A, D), (A, B, C, D)):
        faces.append((np.array([(p[0], p[1], p[2] if len(p) > 2 else 0)
                                for p in quad], dtype=float), color))


def build_static_faces():
    """静态场景面片(立柱 + 设备 + 充电桩 + 围墙带)"""
    faces = []
    for cx, cy, r in POLES:
        rr = r * 1.15
        ring = [(cx + rr * math.cos(i * math.pi / 4),
                 cy + rr * math.sin(i * math.pi / 4)) for i in range(8)]
        for i in range(8):
            x1, y1 = ring[i]; x2, y2 = ring[(i+1) % 8]
            faces.append((np.array([[x1, y1, 0], [x2, y2, 0],
                                    [x2, y2, POLE_H], [x1, y1, POLE_H]]), POLE_C))
        top = np.array([[cx, cy, POLE_H]] +
                       [[x, y, POLE_H] for x, y in ring])
        faces.append((top, POLE_C))
    for _, cx, cy, w, d, h, _ in scenario.DEVICES:
        _add_box_faces(faces, cx, cy, w, d, h, DEVICE_C)
    c = CHARGER
    _add_box_faces(faces, c['x'], c['y'], c['w'], c['d'], c['h'], CHARGER_C, c['yaw'])
    for (sx1, sy1, sx2, sy2) in SEGMENTS[:4]:
        faces.append((np.array([[sx1, sy1, 0], [sx2, sy2, 0],
                                [sx2, sy2, WALL_H_VIS], [sx1, sy1, WALL_H_VIS]]), WALL_C))
    # 房屋墙体(带门洞,每段线段拉高到 HOUSE_H)
    for (x1, y1, x2, y2) in HOUSE_WALLS:
        faces.append((np.array([[x1, y1, 0], [x2, y2, 0],
                                [x2, y2, HOUSE_H], [x1, y1, HOUSE_H]]), HOUSE_C))
    # 房内家具
    for cx, cy, w, d, h in HOUSE_FURNITURE:
        _add_box_faces(faces, cx, cy, w, d, h, DEVICE_C)
    return faces


def person_faces(t):
    """每帧按 person_positions(t) 渲染人员 8 棱柱(简化:4 竖面 + 顶)"""
    faces = []
    for _, x, y, _ in person_positions(t):
        rr = PERSON_R * 1.1
        ring = [(x + rr * math.cos(i * math.pi / 4),
                 y + rr * math.sin(i * math.pi / 4)) for i in range(8)]
        for i in range(8):
            x1, y1 = ring[i]; x2, y2 = ring[(i+1) % 8]
            faces.append((np.array([[x1, y1, 0], [x2, y2, 0],
                                    [x2, y2, PERSON_H], [x1, y1, PERSON_H]]), PERSON_C))
    return faces


class MockGimbalCamera(Node):
    def __init__(self):
        super().__init__('mock_gimbal_camera')

        # ---- 参数 ----
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('hfov_deg', 90.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('yaw_min', -170.0)
        self.declare_parameter('yaw_max', 170.0)
        self.declare_parameter('pitch_min', -30.0)
        self.declare_parameter('pitch_max', 90.0)
        self.declare_parameter('max_yaw_rate_deg', 90.0)    # 斜坡限速(°/s)
        self.declare_parameter('max_pitch_rate_deg', 60.0)
        self.declare_parameter('cmd_timeout', 0.5)          # cmd_vel 看门狗
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('default_yaw', -90.0)         # 巡检巡航默认位
        self.declare_parameter('default_pitch', 90.0)        # 抬头朝天(用户指定)

        self.W = int(self.get_parameter('width').value)
        self.H = int(self.get_parameter('height').value)
        self.fps = self.get_parameter('fps').value
        hfov = math.radians(self.get_parameter('hfov_deg').value)
        self.jpeg_q = int(self.get_parameter('jpeg_quality').value)
        self.yaw_min = self.get_parameter('yaw_min').value
        self.yaw_max = self.get_parameter('yaw_max').value
        self.pitch_min = self.get_parameter('pitch_min').value
        self.pitch_max = self.get_parameter('pitch_max').value
        self.yaw_rate = self.get_parameter('max_yaw_rate_deg').value
        self.pitch_rate = self.get_parameter('max_pitch_rate_deg').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.world_frame = self.get_parameter('world_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.hfov_deg = self.get_parameter('hfov_deg').value

        # 内参(针孔模型,fx=fy)
        self.fx = self.W / (2.0 * math.tan(hfov / 2.0))
        self.fy = self.fx
        self.cx = self.W / 2.0
        self.cy = self.H / 2.0

        # ---- 云台状态 ----
        d_yaw = self.get_parameter('default_yaw').value
        d_pit = self.get_parameter('default_pitch').value
        self.yaw = d_yaw
        self.pitch = d_pit
        self.target_yaw = d_yaw
        self.target_pitch = d_pit
        self.vel_yaw = 0.0
        self.vel_pitch = 0.0
        self.last_cmd_vel_time = self.get_clock().now()
        self.last_tick = self.get_clock().now()

        # ---- 场景(静态面片 + 地面网格线端点,预生成) ----
        self.static_faces = build_static_faces()
        gs = np.arange(-WALL, WALL + GRID_STEP / 2, GRID_STEP)
        self.grid_lines = []
        for g in gs:
            self.grid_lines.append(((g, -WALL), (g, WALL)))    # 纵线
            self.grid_lines.append(((-WALL, g), (WALL, g)))     # 横线

        # 天空竖直渐变模板(逐行插值,渲染时只 copy)
        self.sky = np.zeros((self.H, self.W, 3), np.uint8)
        for row in range(self.H):
            a = row / max(self.H - 1, 1)
            self.sky[row, :] = [int(SKY_TOP[i] * a + SKY_HOR[i] * (1 - a))
                                for i in range(3)]

        # ---- ROS 接口 ----
        self.img_pub = self.create_publisher(
            CompressedImage, 'gimbal/image_raw/compressed', qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(Vector3, 'gimbal/state', latched)
        self.create_subscription(Vector3, 'gimbal/cmd_pos', self.cmd_pos_cb, 10)
        self.create_subscription(Twist, 'gimbal/cmd_vel', self.cmd_vel_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broad = TransformBroadcaster(self)
        self.tf_static_broad = StaticTransformBroadcaster(self)
        # camera_link -> camera_optical 固定旋转(ROS 光学系约定)
        msg = TransformStamped()
        msg.header.frame_id = 'camera_link'
        msg.child_frame_id = 'camera_optical'
        msg.transform.rotation.x = 0.5
        msg.transform.rotation.y = -0.5
        msg.transform.rotation.z = 0.5
        msg.transform.rotation.w = 0.5   # quaternion(0, -90°, 90°) → 光学系
        self.tf_static_broad.sendTransform(msg)

        self.create_timer(1.0 / self.fps, self.render_tick)
        self.create_timer(0.1, self.state_tick)
        self.get_logger().info(
            'mock_gimbal_camera 启动:%dx%d @%.0fHz, hfov=%.0f°, 限速 yaw %.0f°/s pitch %.0f°/s'
            % (self.W, self.H, self.fps, self.hfov_deg,
               self.yaw_rate, self.pitch_rate))

    # ================= 云台控制 =================
    def cmd_pos_cb(self, msg):
        """绝对角目标(°):x=yaw, y=pitch。收到即清 cmd_vel 速度,两源不叠加。"""
        self.target_yaw = msg.x
        self.target_pitch = msg.y
        self.vel_yaw = self.vel_pitch = 0.0
        self.clamp_targets()

    def cmd_vel_cb(self, msg):
        """速度模式(rad/s):angular.z=yaw, angular.y=pitch。0.5s 看门狗归零。"""
        self.vel_yaw = msg.angular.z
        self.vel_pitch = msg.angular.y
        self.last_cmd_vel_time = self.get_clock().now()

    def clamp_targets(self):
        ty = min(max(self.target_yaw, self.yaw_min), self.yaw_max)
        tp = min(max(self.target_pitch, self.pitch_min), self.pitch_max)
        if abs(ty - self.target_yaw) > 1e-6 or abs(tp - self.target_pitch) > 1e-6:
            self.get_logger().warn(
                '云台目标角被限位截断: (%.1f, %.1f) -> (%.1f, %.1f)'
                % (self.target_yaw, self.target_pitch, ty, tp),
                throttle_duration_sec=3)
        self.target_yaw, self.target_pitch = ty, tp

    def gimbal_step(self, dt):
        """云台动态:cmd_vel 看门狗 -> 速度积分;cmd_pos 恒速斜坡。"""
        if (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds * 1e-9 \
                > self.cmd_timeout and (self.vel_yaw or self.vel_pitch):
            self.vel_yaw = self.vel_pitch = 0.0

        if self.vel_yaw or self.vel_pitch:
            # 速度模式:直接积分(跟手),积分后进限位
            self.yaw += math.degrees(self.vel_yaw) * dt
            self.pitch += math.degrees(self.vel_pitch) * dt
            self.target_yaw = self.yaw = min(max(self.yaw, self.yaw_min), self.yaw_max)
            self.target_pitch = self.pitch = \
                min(max(self.pitch, self.pitch_min), self.pitch_max)
        else:
            # 位置模式:恒速斜坡逼近目标
            for cur, tgt, rate, attr in (
                    (self.yaw, self.target_yaw, self.yaw_rate, 'yaw'),
                    (self.pitch, self.target_pitch, self.pitch_rate, 'pitch')):
                err = tgt - cur
                step = rate * dt
                if abs(err) < 0.1:
                    new = tgt
                else:
                    new = cur + max(-step, min(step, err))
                setattr(self, attr, new)

    def publish_gimbal_tf(self):
        """gimbal_base→gimbal_yaw(绕z)→camera_link(绕y),随渲染 tick 发布

        本节点约定 yaw 右转为正、pitch 抬头为正,与 ROS 正旋转(左转/低头)
        方向相反,故四元数取负角。
        """
        if not self.publish_tf:
            return
        now = self.get_clock().now().to_msg()

        m1 = TransformStamped()
        m1.header.stamp = now
        m1.header.frame_id = 'gimbal_base'
        m1.child_frame_id = 'gimbal_yaw'
        m1.transform.rotation.z = -math.sin(math.radians(self.yaw) / 2)
        m1.transform.rotation.w = math.cos(math.radians(self.yaw) / 2)

        m2 = TransformStamped()
        m2.header.stamp = now
        m2.header.frame_id = 'gimbal_yaw'
        m2.child_frame_id = 'camera_link'
        m2.transform.rotation.y = -math.sin(math.radians(self.pitch) / 2)
        m2.transform.rotation.w = math.cos(math.radians(self.pitch) / 2)

        self.tf_broad.sendTransform([m1, m2])

    def state_tick(self):
        msg = Vector3()
        msg.x = float(self.yaw)
        msg.y = float(self.pitch)
        self.state_pub.publish(msg)

    # ================= 渲染 =================
    def render_tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        dt = min(dt, 0.2)

        self.gimbal_step(dt)
        self.publish_gimbal_tf()

        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, 'camera_optical', rclpy.time.Time(),
                Duration(seconds=0.0))
        except Exception as e:
            self.get_logger().warn('等待 TF %s->camera_optical: %s'
                                   % (self.world_frame, e),
                                   throttle_duration_sec=3)
            return

        stamp = tf.header.stamp
        t = np.array([tf.transform.translation.x, tf.transform.translation.y,
                      tf.transform.translation.z])
        R = quat_to_rot(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w)

        img = self.draw_scene(R, t)
        self.draw_hud(img)

        ok, buf = cv2.imencode('.jpg', img,
                               [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp            # 铁律:图像时间戳 = 所用 TF 时刻
        msg.header.frame_id = 'camera_optical'
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.img_pub.publish(msg)

    def draw_scene(self, R, t):
        """针孔模型画虚拟场景:地平线分天空/地面 -> 网格 -> 静态+人员障碍(painter)

        地面按"地平线"水平线整屏分色(pitch>0 抬头时地平线下移,直接下侧填 GROUND)。
        相机无横滚,墙高 1.2m > 相机高 0.35m,墙外地面本来就被完全遮挡,无需剔除。
        人员面片每帧按 wall clock 取位置,与雷达点云天然一致。
        """
        H, W = self.H, self.W
        img = self.sky.copy()

        # 地平线以下填 GROUND;>85°/-85° 截断避免 tan 爆炸
        ph = min(max(math.radians(self.pitch), -1.48), 1.48)
        v_h = int(self.cy + self.fy * math.tan(ph))
        if v_h <= 0:
            img[:, :] = GROUND            # 低头过头:整屏地面
        elif v_h < H:
            img[v_h:, :] = GROUND         # 正常:地平线在屏中
        # else: 抬头过头:img 已是 sky,整屏天空

        def to_cam(pts_w):
            return (np.asarray(pts_w, dtype=float) - t) @ R

        def to_px(pc):
            """相机系 (N,3) -> 像素 (N,2);调用方保证 z>Z_NEAR"""
            u = self.fx * pc[:, 0] / pc[:, 2] + self.cx
            v = self.fy * pc[:, 1] / pc[:, 2] + self.cy
            return np.stack([u, v], axis=1)

        # ---- 网格线 ----
        for (x1, y1), (x2, y2) in self.grid_lines:
            a = to_cam([[x1, y1, 0]])
            b = to_cam([[x2, y2, 0]])
            seg = self.clip_segment_near(a[0], b[0])
            if seg is None:
                continue
            p = to_px(np.array(seg)).astype(np.int32)
            cv2.line(img, (p[0][0], p[0][1]), (p[1][0], p[1][1]),
                     GRID_C, 1, cv2.LINE_AA)

        # ---- 障碍物面片:静态(立柱/设备/桩/墙)+ 动态(人员),按深度降序 painter ----
        drawn = []
        all_faces = self.static_faces + person_faces(time.time())
        for verts_w, color in all_faces:
            vc = to_cam(verts_w)
            if (vc[:, 2] < Z_NEAR).any():
                continue            # 整面跨近面直接跳过(简化,伪影可接受)
            depth = float(vc[:, 2].mean())
            pts = to_px(vc)
            if (pts[:, 0] < -50).all() or (pts[:, 0] > W + 50).all() or \
               (pts[:, 1] < -50).all() or (pts[:, 1] > H + 50).all():
                continue            # 整面在画幅外
            drawn.append((depth, pts.astype(np.int32), color))
        drawn.sort(key=lambda d: -d[0])     # 远的先画
        for depth, pts, color in drawn:
            c = self.fog(color, depth)
            cv2.fillPoly(img, [pts], c)

        return img

    @staticmethod
    def clip_segment_near(a, b):
        """线段两端点(相机系 3D)近面裁剪,返回 [p1, p2] 或 None(全在后方)"""
        za, zb = a[2], b[2]
        if za < Z_NEAR and zb < Z_NEAR:
            return None
        if za < Z_NEAR or zb < Z_NEAR:
            s = (Z_NEAR - za) / (zb - za)
            p = a + s * (b - a)
            return [p, b] if za < Z_NEAR else [a, p]
        return [a, b]

    @staticmethod
    def fog(color, depth):
        """远雾:距离越远颜色越接近天空色,掩盖远处锯齿"""
        f = min(depth / 25.0, 0.75)          # 25m 处褪去 75%
        return tuple(int(c * (1 - f) + SKY_HOR[i] * f) for i, c in enumerate(color))

    def draw_hud(self, img):
        H, W = self.H, self.W
        cx, cy = int(self.cx), int(self.cy)
        cv2.line(img, (cx - 18, cy), (cx - 6, cy), HUD_C, 1, cv2.LINE_AA)
        cv2.line(img, (cx + 6, cy), (cx + 18, cy), HUD_C, 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy - 18), (cx, cy - 6), HUD_C, 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy + 6), (cx, cy + 18), HUD_C, 1, cv2.LINE_AA)
        cv2.putText(img, 'yaw %+.1f  pitch %+.1f' % (self.yaw, self.pitch),
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, HUD_C, 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = MockGimbalCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
