#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patrol_manager —— 巡检任务系统(核心)

状态机:
  IDLE ──start──▶ RUNNING ──完成──▶ DOCKING ──到桩──▶ DONE ──▶ IDLE
                     │
                     └─低电──▶ LOW_BATTERY ──到桩──▶ CHARGING ──≥resume──▶ RUNNING(剩余)
  任意状态 + cancel ──▶ CANCELLED ──▶ IDLE

接口(均 std_msgs/String,JSON 负载,无自定义 msg 包):
  /patrol/cmd        订阅  JSON:
                       {"op":"start","points":["P1_设备A","P3_设备C"],"return_dock":true}
                       {"op":"cancel"}
                       {"op":"return_dock"}
  /patrol/status     发布  JSON(2Hz,锁存 TRANSIENT_LOCAL),网页唯一回执
  /patrol/scenario   发布  JSON(锁存,启动一次),网页场地标记
  /gimbal/cmd_pos    发布  到点后瞄准设备
  /odom              订阅  实时位姿(用 odom 不查 TF,避开时间戳坑)
  /battery           订阅  实时电量
  /gimbal/state      订阅  云台实际角(判定 settled)
  /gimbal/image_raw/compressed  订阅 拍照(只收最新帧 + 等待斜坡完成)

时间戳铁律:拍照必须等云台斜坡完成(settle)+ 图像 stamp >= t_settle+0.3,
否则可能拍到云台还在转的帧。
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, CompressedImage
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

from robot_sim import scenario
from robot_sim.scenario import CHARGER, CHARGER_RADIUS, PATROL_POINTS, DEVICE_MAP

PHOTOS_DIR = os.path.expanduser('~/ros2_ws/photos')


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')

        # ---- 参数 ----
        self.declare_parameter('low_battery', 20.0)         # 触发回充阈值 %
        self.declare_parameter('resume_battery', 90.0)     # 充到多少继续 %
        self.declare_parameter('gimbal_settle_tol_deg', 2.0)   # 判定稳定的角度容差
        self.declare_parameter('gimbal_timeout', 6.0)      # 云台斜坡最长等待 s
        self.declare_parameter('cam_height', 0.35)         # 相机高度 m
        self.low_batt = self.get_parameter('low_battery').value
        self.resume_batt = self.get_parameter('resume_battery').value
        self.settle_tol = self.get_parameter('gimbal_settle_tol_deg').value
        self.gimbal_to = self.get_parameter('gimbal_timeout').value
        self.cam_h = self.get_parameter('cam_height').value

        # ---- 状态机 ----
        self.state = 'IDLE'              # IDLE/RUNNING/DOCKING/LOW_BATTERY/CHARGING/DONE/CANCELLED
        self.sub = 'IDLE'                # 子状态(NAV/AIM/WAIT_GIMBAL/PHOTO)
        self.queue = []                  # [{'name','x','y','device'}]
        self.idx = 0                     # 当前执行到第几个
        self.return_dock = True
        self.battery = 1.0
        self.rx = self.ry = self.yaw_r = 0.0
        self.gimbal_yaw = self.gimbal_pitch = 0.0
        self.last_img = None
        self.pending_photo = None         # {'point':名, 't_settle':秒}
        self.t_sub0 = 0.0
        self.photos = []                 # 全部已拍照片文件名(全量)
        self.msg = ''                    # 状态附带说明
        self.nav_goal_handle = None
        self.charging_before_battery = None
        self.last_start_t = 0.0
        self.nav_fail_cnt = 0            # 导航连续失败计数(5 次跳点/停机,给 costmap 热身留时间)
        self._retry_timers = []
        os.makedirs(PHOTOS_DIR, exist_ok=True)

        # ---- ROS 接口 ----
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'patrol/status', latched)
        self.scenario_pub = self.create_publisher(String, 'patrol/scenario', latched)
        self.gim_pub = self.create_publisher(Vector3, 'gimbal/cmd_pos', 10)
        self.create_subscription(String, 'patrol/cmd', self.cmd_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(BatteryState, 'battery', self.bat_cb, qos_profile_sensor_data)
        self.create_subscription(Vector3, 'gimbal/state', self.gim_state_cb, 10)
        self.create_subscription(CompressedImage, 'gimbal/image_raw/compressed',
                                 self.img_cb, qos_profile_sensor_data)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(0.1, self.tick)              # 10Hz 状态机
        self.create_timer(0.5, self.publish_status)    # 2Hz 状态发布
        self._publish_scenario()
        self.get_logger().info('patrol_manager 启动, %d 个巡检点, 照片目录:%s'
                               % (len(PATROL_POINTS), PHOTOS_DIR))

    # ============== 订阅回调 ==============
    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.rx, self.ry = p.x, p.y
        self.yaw_r = math.atan2(2*(q.w*q.z + q.x*q.y),
                                1 - 2*(q.y*q.y + q.z*q.z))

    def bat_cb(self, msg):
        self.battery = msg.percentage
        if self.state == 'CHARGING' and self.charging_before_battery is not None:
            # 充电提醒
            pass

    def gim_state_cb(self, msg):
        self.gimbal_yaw, self.gimbal_pitch = msg.x, msg.y

    def img_cb(self, msg):
        self.last_img = msg
        if self.pending_photo is None:
            return
        # 新鲜度门控:必须等斜坡结束 0.3s 之后产生的帧
        img_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t_need = self.pending_photo['t_settle'] + 0.3
        if img_t < t_need:
            return
        self._save_photo(msg)
        self.pending_photo = None
        self._next_point()

    # ============== 场景发布 ==============
    def _publish_scenario(self):
        d = {
            'wall': scenario.WALL,
            'charger': {'x': CHARGER['x'], 'y': CHARGER['y'],
                        'w': CHARGER['w'], 'd': CHARGER['d']},
            'devices': [{'name': d[0], 'x': d[1], 'y': d[2], 'w': d[3], 'd': d[4]}
                        for d in scenario.DEVICES],
            'points': [{'name': p['name'], 'x': p['x'], 'y': p['y'],
                        'device': p['device']} for p in PATROL_POINTS],
        }
        msg = String()
        msg.data = json.dumps(d, ensure_ascii=False)
        self.scenario_pub.publish(msg)

    # ============== 控制入口 ==============
    def cmd_cb(self, msg):
        try:
            o = json.loads(msg.data)
        except Exception:
            self.msg = 'JSON 解析失败'
            return
        op = o.get('op')
        if op == 'start':
            names = o.get('points', [])
            if not names:
                self.msg = 'start 缺少 points'
                return
            unknown = [n for n in names if n not in scenario.POINT_MAP]
            if unknown:
                self.msg = '未知点位: %s' % unknown
                return
            now = time.time()
            if self.state == 'RUNNING' and now - self.last_start_t < 0.5:
                self.get_logger().warn('重复 start 忽略(防抖)')
                return
            self.last_start_t = now
            self.queue = [dict(scenario.POINT_MAP[n]) for n in names]
            self.idx = 0
            self.return_dock = bool(o.get('return_dock', True))
            self.state = 'RUNNING'
            self.sub = 'NAV'
            self.msg = '开始巡检, %d 个点' % len(self.queue)
            self.get_logger().info(self.msg)
            self._nav_to(self.queue[self.idx])
        elif op == 'cancel':
            self._cancel_nav('用户取消')
            self.state = 'CANCELLED'
            self.sub = 'IDLE'
        elif op == 'return_dock':
            if self.state in ('IDLE', 'DONE', 'CANCELLED'):
                self.state = 'DOCKING'
                self.sub = 'NAV'
                self.msg = '返回充电桩'
                self._nav_to({'x': CHARGER['approach']['x'],
                              'y': CHARGER['approach']['y'], 'name': 'CHARGER'})
        else:
            self.msg = '未知 op: %s' % op

    # ============== 状态机主循环 ==============
    def tick(self):
        if self.state == 'RUNNING':
            # 1) 低电监护(最高优先级):当前正在前往的点继续走完,到达后中断回充
            if self.battery * 100.0 < self.low_batt and self.sub == 'IDLE':
                self._abort_to_dock('低电: %d%%' % int(self.battery * 100))
                return
            if self.sub == 'AIM':
                self._aim_gimbal()
                self.sub = 'WAIT_GIMBAL'
                self.t_sub0 = time.time()
            elif self.sub == 'WAIT_GIMBAL':
                if self._gimbal_settled() or time.time() - self.t_sub0 > self.gimbal_to:
                    self.sub = 'PHOTO'
                    self.pending_photo = {
                        'point': self.queue[self.idx]['name'],
                        't_settle': time.time(),
                    }
                    self.t_sub0 = time.time()
            elif self.sub == 'PHOTO':
                # img_cb 触发 _next_point;超时兜底
                if time.time() - self.t_sub0 > 8.0 and self.pending_photo is not None:
                    self.get_logger().warn('拍照超时,跳过')
                    self.pending_photo = None
                    self._next_point()
        elif self.state == 'CHARGING':
            if self._in_dock() and self.battery * 100.0 >= self.resume_batt:
                self.state = 'RUNNING'
                self.sub = 'NAV'
                self.msg = '已充至 %d%%,继续巡检' % int(self.battery * 100)
                self._nav_to(self.queue[self.idx])

    def _in_dock(self):
        return math.hypot(self.rx - CHARGER['x'], self.ry - CHARGER['y']) < CHARGER_RADIUS

    def _gimbal_settled(self):
        tyaw, tpitch = self._north_facing_angles()
        return (abs(self.gimbal_yaw - tyaw) < self.settle_tol
                and abs(self.gimbal_pitch - tpitch) < self.settle_tol)

    def _north_facing_angles(self):
        """世界系朝北(+y,方位角90°)所需云台角:节点右转为正 → 取负"""
        rel = wrap_pi(math.pi / 2 - self.yaw_r)
        tyaw = max(-170.0, min(170.0, -math.degrees(rel)))
        return tyaw, 90.0

    def _aim_angles(self, pt):
        """返回 (gimbal_yaw°, gimbal_pitch°) —— 节点右转为正、抬头为正"""
        device = pt.get('device')
        if device and device in DEVICE_MAP:
            _, dx, dy, _, _, h, insp = DEVICE_MAP[device]
            tx, ty, tz = dx, dy, insp
        else:
            tx, ty, tz = pt['x'], pt['y'], 1.0
        az = math.atan2(ty - self.ry, tx - self.rx)         # 世界方位角
        rel = wrap_pi(az - self.yaw_r)                      # 车体系
        gim_yaw = -math.degrees(rel)
        dist = math.hypot(tx - self.rx, ty - self.ry)
        gim_pitch = math.degrees(math.atan2(tz - self.cam_h, max(dist, 0.1)))
        return gim_yaw, gim_pitch

    def _aim_gimbal(self):
        """用户约定:到点后云台转到世界系面朝北方(+y)、水平俯仰拍一张"""
        tyaw, tpitch = self._north_facing_angles()
        self.gim_pub.publish(Vector3(x=tyaw, y=tpitch, z=0.0))

    # ============== 导航 ==============
    def _nav_to(self, pt):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pt['x']
        goal.pose.pose.position.y = pt['y']
        # 终点 yaw 朝设备(仅引导,DWB 终点朝向不强制)
        if 'name' in pt and pt.get('device') and pt['device'] in DEVICE_MAP:
            _, dx, dy, _, _, _, _ = DEVICE_MAP[pt['device']]
            yaw = math.atan2(dy - pt['y'], dx - pt['x'])
            goal.pose.pose.orientation = yaw_to_quat(yaw)
        else:
            goal.pose.pose.orientation = yaw_to_quat(0.0)
        self.nav_goal_handle = None
        self.get_logger().info('导航 → %s (%.1f, %.1f)'
                               % (pt.get('name', '?'), pt['x'], pt['y']))
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('goal 被拒,跳过该点')
            if self.state in ('RUNNING', 'DOCKING', 'LOW_BATTERY'):
                self._next_point()
            return
        self.nav_goal_handle = gh
        gh.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, fut):
        st = fut.result().status
        if st == 4:                  # SUCCEEDED
            self.nav_fail_cnt = 0
            if self.state == 'RUNNING':
                self.sub = 'AIM'
            elif self.state == 'DOCKING':
                self.state = 'DONE'
                self.sub = 'IDLE'
                self.msg = '已到充电桩,任务结束'
            elif self.state == 'LOW_BATTERY':
                self.state = 'CHARGING'
                self.charging_before_battery = self.battery
                self.msg = '回桩充电中...'
        else:                        # 失败或被取消:重试,超限按状态收尾
            if self.state not in ('RUNNING', 'DOCKING', 'LOW_BATTERY'):
                return               # 已被 cancel,忽略迟到回调
            self.nav_fail_cnt += 1
            self.get_logger().warn('导航未成功 status=%d (第 %d 次)'
                                   % (st, self.nav_fail_cnt))
            if self.nav_fail_cnt >= 5:
                if self.state == 'RUNNING':
                    self.nav_fail_cnt = 0
                    self.msg = '导航连续失败,跳过 %s' % \
                        (self.queue[self.idx]['name'] if self.idx < len(self.queue) else '?')
                    self._next_point()
                else:                # DOCKING / LOW_BATTERY:停机,不再循环
                    self._cancel_nav('回桩导航连续失败')
                    self.state = 'CANCELLED'
                    self.sub = 'IDLE'
                    self.msg = '回桩导航连续失败,已停止'
            else:
                # 1s 后重试(一次性定时器,避免 sleep 堵 executor)
                retry_target = ({'x': CHARGER['approach']['x'],
                                 'y': CHARGER['approach']['y'], 'name': 'CHARGER'}
                                if self.state in ('DOCKING', 'LOW_BATTERY')
                                else (self.queue[self.idx] if self.idx < len(self.queue) else None))
                if retry_target:
                    self.msg = '导航重试中...'
                    tm = self.create_timer(3.0, lambda: self._retry_nav(retry_target))
                    self._retry_timers.append(tm)

    def _retry_nav(self, target):
        """一次性重试定时器回调:重发导航目标"""
        # 取消所有未触发的重试定时器
        for tm in list(self._retry_timers):
            tm.cancel()
        self._retry_timers.clear()
        if self.state not in ('RUNNING', 'DOCKING', 'LOW_BATTERY'):
            return
        self._nav_to(target)

    def _next_point(self):
        self.idx += 1
        if self.idx >= len(self.queue):
            # 全部点完成
            if self.return_dock:
                self.state = 'DOCKING'
                self.sub = 'NAV'
                self.msg = '任务完成,返回充电桩'
                self._nav_to({'x': CHARGER['approach']['x'],
                              'y': CHARGER['approach']['y'], 'name': 'CHARGER'})
            else:
                self.state = 'DONE'
                self.sub = 'IDLE'
                self.msg = '任务完成'
            return
        self.sub = 'NAV'
        self._nav_to(self.queue[self.idx])

    def _abort_to_dock(self, reason):
        self.get_logger().warn('中断任务: %s' % reason)
        self._cancel_nav(reason)
        self.state = 'LOW_BATTERY'
        self.sub = 'NAV'
        self.msg = reason + ', 回桩'
        # 队列记录断点:从 idx+1 处继续(用户期望)
        self._nav_to({'x': CHARGER['approach']['x'], 'y': CHARGER['approach']['y'], 'name': 'CHARGER'})

    def _cancel_nav(self, reason):
        if self.nav_goal_handle is not None:
            try:
                self.nav_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.nav_goal_handle = None
        self.msg = reason

    # ============== 拍照 ==============
    def _save_photo(self, msg):
        name = '%s_%s.jpg' % (self.pending_photo['point'],
                              time.strftime('%Y%m%d_%H%M%S'))
        path = os.path.join(PHOTOS_DIR, name)
        try:
            with open(path, 'wb') as f:
                f.write(bytes(msg.data))
            self.photos.append(name)
            self.get_logger().info('已拍照: %s' % path)
        except Exception as e:
            self.get_logger().error('存照片失败: %s' % e)

    # ============== 状态发布 ==============
    def publish_status(self):
        try:
            people = [{'name': n, 'x': x, 'y': y} for n, x, y, _ in scenario.person_positions(time.time())]
        except Exception:
            people = []
        cur = self.queue[self.idx] if self.idx < len(self.queue) else None
        d = {
            'state': self.state,
            'sub': self.sub,
            'point': cur['name'] if cur else None,
            'index': self.idx,
            'total': len(self.queue),
            'battery': round(self.battery, 3),
            'at_dock': self._in_dock(),
            'msg': self.msg,
            'photos': self.photos[-30:],
            'people': people,
            'pose': {'x': round(self.rx, 2), 'y': round(self.ry, 2),
                     'yaw_deg': round(math.degrees(self.yaw_r), 1),
                     'gimbal_yaw': round(self.gimbal_yaw, 1),
                     'gimbal_pitch': round(self.gimbal_pitch, 1)},
        }
        msg = String()
        msg.data = json.dumps(d, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
