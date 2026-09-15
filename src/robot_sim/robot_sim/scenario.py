#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scenario —— 巡检场景唯一事实源(纯数据 + 纯函数,无 ROS 依赖)

mock_lidar / mock_gimbal_camera / mock_base / patrol_manager 全部从这里导入,
保证"雷达点云、相机画面、栅格地图、巡检任务"四者看到同一个场地。
时间约定:人员位置函数只吃"秒数 t",节点自行决定用哪个时钟
(默认 wall clock: time.time(),人员与机器人无 TF 耦合,无需仿真时间对齐)。
"""
import math

import numpy as np

# ---------------- 场地 ----------------
WALL = 20.0          # 场地半宽 → 40x40m(原 10.0)
WALL_H = 1.2         # 围墙可视高度(相机画色带用)


# ---------------- 设备(巡检对象,长方体) ----------------
# 字段:(name, cx, cy, w, d, height, inspect_z)
#   inspect_z = 云台对准的目标高度(设备中部偏上)
DEVICES = [
    ('device_a', -12.0, -8.0, 2.0, 1.5, 1.8, 1.5),
    ('device_b',  10.0, -12.0, 1.8, 1.8, 1.6, 1.4),
    ('device_c',  14.0,  8.0, 2.5, 1.2, 1.5, 1.3),
    ('device_d', -14.0, 12.0, 1.5, 1.5, 2.0, 1.6),
]
DEVICE_MAP = {d[0]: d for d in DEVICES}


# ---------------- 场地中央"房屋"(四房间隔间结构,便于建图) ----------------
HOUSE_H = 2.2         # 房屋墙高(米),高于人员/设备,雷达相机都真实
HW = 8.0              # 房屋外框半宽:外墙在 x,y = ±HW(占中心 16x16m)
DOOR = 1.6            # 门洞宽(米),> 机器人直径+膨胀,可通行
# 墙体线段(带门洞)。四个门:南门(-4~-2.4, -8) 北门(2.4~4, 8)
#                     西门(-8, 2.4~4) 东门(8, -4~-2.4) —— 四房间各自直通室外
HOUSE_WALLS = [
    # 南墙 y=-HW
    (-HW, -HW, -4.0, -HW), (-2.4, -HW, HW, -HW),
    # 北墙 y=+HW
    (-HW, HW, 2.4, HW), (4.0, HW, HW, HW),
    # 西墙 x=-HW
    (-HW, -HW, -HW, 2.4), (-HW, 4.0, -HW, HW),
    # 东墙 x=+HW
    (HW, -HW, HW, -4.0), (HW, -2.4, HW, HW),
    # 横内墙 y=0(东西各一门)
    (-HW, 0, -4.0, 0), (-2.4, 0, 2.4, 0), (4.0, 0, HW, 0),
    # 纵内墙 x=0(南北各一门)
    (0, 0, 0, 2.4), (0, 4.0, 0, HW),
    (0, -HW, 0, -4.0), (0, -2.4, 0, 0),
]
# 房内家具:(cx, cy, w, d, h)
HOUSE_FURNITURE = [
    (-4.5, -4.5, 1.4, 1.2, 1.2),   # 西南房 工作台
    (4.5, 4.5, 1.2, 1.4, 1.5),     # 东北房 机柜
    (4.5, -4.5, 1.0, 1.0, 0.8),    # 东南房 矮柜
]


# ---------------- 立柱(圆形遮挡物) ----------------
POLES = [(-16, -16, 0.12), (-16, 16, 0.12), (16, -16, 0.12), (16, 16, 0.12),
         (-8, -14, 0.12), (8, 14, 0.12), (-14, 2, 0.12), (14, -2, 0.12)]


# ---------------- 充电桩 ----------------
# approach = 导航停靠点(桩前 1.2m,避开障碍膨胀区);在桩判定用 CHARGER_RADIUS 覆盖它
CHARGER = {'x': 0.0, 'y': -18.0, 'yaw': 0.0, 'w': 0.6, 'd': 0.4, 'h': 0.5,
           'approach': {'x': 0.0, 'y': -16.8}}
CHARGER_RADIUS = 1.5          # 判定"在桩"的半径(覆盖 approach 点)


# ---------------- 巡检点(停车点) ----------------
# device=None 的点只停车拍照(拍全景),不转云台
PATROL_POINTS = [
    {'name': 'P1_设备A', 'x': -12.0, 'y': -5.0, 'device': 'device_a'},
    {'name': 'P2_设备B', 'x':  10.0, 'y': -9.0, 'device': 'device_b'},
    {'name': 'P3_设备C', 'x':  14.0, 'y':  5.0, 'device': 'device_c'},
    {'name': 'P4_设备D', 'x': -14.0, 'y':  9.0, 'device': 'device_d'},
    {'name': 'P5_东区通道', 'x': 17.0, 'y':  0.0, 'device': None},
    {'name': 'P6_充电桩区', 'x':  3.0, 'y': -15.0, 'device': None},
]
POINT_MAP = {p['name']: p for p in PATROL_POINTS}


# ---------------- 人员动态路线(往返折线) ----------------
PERSON_R = 0.25
PERSON_H = 1.7
PEOPLE = [
    {'name': 'person_1', 'points': [(-16, -16), (16, -16)],
     'speed': 0.8, 'phase': 0.0},
    {'name': 'person_2', 'points': [(-16, 16), (16, 16), (16, 2)],
     'speed': 1.0, 'phase': 5.0},
    {'name': 'person_3', 'points': [(-8, 0), (8, 0)],
     'speed': 0.6, 'phase': 2.0},
]


# ---------------- 几何工具 ----------------
def box_corners(cx, cy, w, d, yaw=0.0):
    """长方体底面四角(支持带 yaw 旋转)→ [(x,y) x 4]"""
    ca, sa = math.cos(yaw), math.sin(yaw)
    out = []
    for dx, dy in ((-w/2, -d/2), (w/2, -d/2), (w/2, d/2), (-w/2, d/2)):
        out.append((cx + dx*ca - dy*sa, cy + dx*sa + dy*ca))
    return out


def box_segments(corners):
    """四角 → 4 条边线段 (x1,y1,x2,y2)"""
    return [(corners[i][0], corners[i][1],
             corners[(i+1) % 4][0], corners[(i+1) % 4][1]) for i in range(4)]


def build_segments():
    """所有线段类遮挡物 = 四墙 + 房屋墙体(带门洞) + 各设备四边 + 充电桩四边 + 家具四边"""
    segs = [(-WALL, -WALL,  WALL, -WALL),
            (-WALL,  WALL,  WALL,  WALL),
            (-WALL, -WALL, -WALL,  WALL),
            ( WALL, -WALL,  WALL,  WALL)]
    segs += HOUSE_WALLS                      # 房屋墙体(含门洞,真实遮挡)
    for _, cx, cy, w, d, _, _ in DEVICES:
        segs += box_segments(box_corners(cx, cy, w, d))
    for cx, cy, w, d, _ in HOUSE_FURNITURE:
        segs += box_segments(box_corners(cx, cy, w, d))
    c = CHARGER
    segs += box_segments(box_corners(c['x'], c['y'], c['w'], c['d'], c['yaw']))
    return segs


SEGMENTS = build_segments()


# ---------------- 人员位置:预计算路径表 + 三角波往返 ----------------
def _build_route_table(points):
    """预计算:[(p1, p2, s0, L)], total —— 供往返插值"""
    segs, total = [], 0.0
    for i in range(len(points) - 1):
        L = math.hypot(points[i+1][0]-points[i][0], points[i+1][1]-points[i][1])
        segs.append((points[i], points[i+1], total, L))
        total += L
    return segs, total


_ROUTES = [_build_route_table(p['points']) for p in PEOPLE]


def person_positions(t):
    """返回 [(name, x, y, yaw)] —— 三角波往返,phase 让多人错开"""
    out = []
    for r, (segs, total) in zip(PEOPLE, _ROUTES):
        s = (r['speed'] * (t + r['phase'])) % (2.0 * total)
        if s > total:                # 折返段
            s = 2.0 * total - s
        for p1, p2, s0, L in segs:
            if s0 <= s <= s0 + L:
                u = (s - s0) / max(L, 1e-9)
                x = p1[0] + (p2[0]-p1[0]) * u
                y = p1[1] + (p2[1]-p1[1]) * u
                yaw = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
                out.append((r['name'], x, y, yaw))
                break
    return out


def person_points(t, zs=(0.3, 0.9, 1.5), n=8):
    """人员点云采样:身体圆柱面按高度带取 n 点/环 → (M,3) float64
    M = 3 人 * 3 高度 * 8 环 = 72 点/帧,雷达开销可忽略"""
    pts = []
    for _, x, y, _ in person_positions(t):
        for z in zs:
            for i in range(n):
                a = 2 * math.pi * i / n
                pts.append((x + PERSON_R*math.cos(a),
                            y + PERSON_R*math.sin(a), z))
    return np.array(pts, dtype=np.float64)


def quat_to_rot(x, y, z, w):
    """四元数 → 旋转矩阵(与 mock_lidar 原版一致,这里 re-export 便于相机直接 import)"""
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])
