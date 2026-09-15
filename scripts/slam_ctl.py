#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slam_ctl.py -- 真雷达 SLAM/导航 交互式控制台

用法:
    python3 ~/ros2_ws/scripts/slam_ctl.py
    (建议先 source 环境; 脚本内部会自动补 source)

功能: 菜单式启动/停止/建图/存图/加载/健康检查, 不用记命令
"""

import os
import subprocess
import sys
import time

HOME = os.path.expanduser('~')
SCRIPTS = os.path.join(HOME, 'ros2_ws', 'scripts')
LOGDIR = os.path.join(HOME, 'ros2_ws', 'logs')

SRC = ('source /opt/ros/humble/setup.bash && '
       'source ~/ros2_ws/install/setup.bash')

PROCS = []  # 本脚本拉起的后台进程


def sh_bg(cmd, logname):
    """后台启动一条命令, 日志写入 ~/ros2_ws/logs/"""
    os.makedirs(LOGDIR, exist_ok=True)
    log = open(os.path.join(LOGDIR, logname), 'w')
    p = subprocess.Popen(['bash', '-c', SRC + ' && ' + cmd],
                         stdout=log, stderr=subprocess.STDOUT)
    PROCS.append(p)
    print('  已启动 -> %s (日志: logs/%s)' % (cmd.split()[0:3], logname))
    return p


def sh(cmd, timeout=30, show=True):
    """前台执行, 返回输出"""
    r = subprocess.run(['bash', '-c', SRC + ' && ' + cmd],
                       capture_output=True, text=True, timeout=timeout)
    out = (r.stdout + r.stderr).strip()
    if show and out:
        print(out)
    return out


def start_slam():
    print('\n[1/5] 雷达驱动...')
    sh_bg('ros2 launch livox_ros_driver2 msg_MID360s_launch.py', 'livox.log')
    time.sleep(5)
    print('[2/5] IMU 单位换算节点...')
    sh_bg('python3 ~/ros2_ws/scripts/imu_unit_fix.py', 'imu_fix.log')
    time.sleep(2)
    print('[3/5] FAST_LIO...')
    sh_bg('ros2 launch fast_lio mapping.launch.py rviz:=false', 'fastlio.log')
    time.sleep(4)
    print('[4/5] 静态 map->odom TF...')
    sh_bg('ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom',
          'static_tf.log')
    time.sleep(1)
    print('[5/5] map_manager...')
    sh_bg('ros2 run robot_sim map_manager --ros-args '
          '-r livox/lidar:=/cloud_registered', 'map_manager.log')
    time.sleep(5)
    print('\n=== 启动完成, 自动健康检查: ===')
    health()


def start_nav():
    print('\n启动 Web + Nav2 + RViz...')
    sh_bg('ros2 launch robot_sim web_interface.launch.py', 'web.log')
    time.sleep(3)
    sh_bg('ros2 launch robot_sim nav2_sim.launch.py', 'nav2.log')
    time.sleep(3)
    sh_bg('export LIBGL_ALWAYS_SOFTWARE=1 && '
          'rviz2 -d ~/ros2_ws/src/FAST_LIO_ROS2/rviz/fastlio.rviz', 'rviz.log')
    print('完成: Web=http://localhost:8090  demo=http://localhost:8091/demo.html')


def stop_all():
    print('\n停止全部节点...')
    subprocess.run(['bash', os.path.join(SCRIPTS, 'stop_slam.sh')])
    del PROCS[:]


def save_map():
    print('\n保存地图:')
    sh('ros2 service call /map_mode/save example_interfaces/srv/Trigger')


def load_map():
    print('\n加载最新地图为导航图:')
    sh('ros2 service call /map_mode/load_latest '
       'example_interfaces/srv/Trigger')


def reset_mapping():
    print('\n清空地图, 重新建图:')
    sh('ros2 service call /map_mode/start_mapping '
       'example_interfaces/srv/Trigger')


def health():
    print('\n---------- 健康检查 ----------')
    try:
        out = sh('timeout 6 ros2 topic hz /livox/lidar --window 5',
                 timeout=10, show=False)
        line = [l for l in out.splitlines() if 'average' in l]
        hz = line[0].split()[2] if line else '??'
        print('点云频率: %s Hz %s' % (hz, '(正常≈10)' if hz != '??' and
                                      float(hz) > 8 else '(异常! 查丢包/遮挡)'))
    except Exception as e:
        print('点云频率: 查询失败 %s' % e)
    try:
        out = sh('timeout 4 ros2 topic echo /Odometry --once '
                 '--field pose.pose.position', timeout=8, show=False)
        print('当前位姿:\n  %s' % '  '.join(
            l.strip() for l in out.splitlines() if l.strip()))
    except Exception:
        print('当前位姿: FAST_LIO 未运行?')
    out = sh('grep -c "No Effective" ~/ros2_ws/logs/fastlio.log 2>/dev/null',
             show=False)
    print('失明告警累计: %s (持续增长=异常)' % (out or '0'))
    out = sh('tail -1 ~/ros2_ws/logs/map_manager.log 2>/dev/null', show=False)
    print('地图状态: %s' % out)


def status():
    print('\n当前相关进程:')
    out = sh('pgrep -a -f "livox_ros_driver2_node|fastlio_mapping|rviz2|'
             'rosbridge|map_manager|imu_unit_fix|nav2|web_server" '
             '| grep -v slam_ctl', show=False)
    print(out if out else '  (无, 链路未启动)')


MENU = """
========= 巡检机器人 SLAM 控制台 =========
  1. 启动建图链路(雷达+IMU换算+FAST_LIO+TF+map_manager)
  2. 启动导航三件套(Web+Nav2+RViz)
  3. 停止全部
  4. 保存地图
  5. 加载最新地图(切导航模式)
  6. 清空地图重新建图
  7. 健康检查
  8. 查看运行中的进程
  0. 退出(不停止已启动的节点)
=========================================="""


def main():
    while True:
        print(MENU)
        try:
            c = input('选择> ').strip()
        except (EOFError, KeyboardInterrupt):
            print('\n退出(节点保持运行)')
            break
        if c == '1':
            start_slam()
        elif c == '2':
            start_nav()
        elif c == '3':
            stop_all()
        elif c == '4':
            save_map()
        elif c == '5':
            load_map()
        elif c == '6':
            reset_mapping()
        elif c == '7':
            health()
        elif c == '8':
            status()
        elif c == '0':
            print('退出(节点保持运行, 停止请选3或 stop_slam.sh)')
            break
        else:
            print('无效选择')


if __name__ == '__main__':
    main()
