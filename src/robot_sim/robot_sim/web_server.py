#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
web_server —— Web 控制台静态文件服务器

用 Python 标准库 http.server 托管 robot_sim/web 目录,
浏览器访问 http://<工控机IP>:8090 即可打开控制台。
(WebSocket 数据走 rosbridge 的 9090 端口,与这里互不冲突)

扩展:子类化 handler,translate_path 把 /photos/<file> 映射到 ~/ros2_ws/photos/,
realpath 校验阻断 ../ 穿越。
"""
import os
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

PORT = 8090
PHOTOS_DIR = os.path.expanduser('~/ros2_ws/photos')
PHOTOS_REAL = os.path.realpath(PHOTOS_DIR)


class ConsoleHandler(SimpleHTTPRequestHandler):
    """静态文件 + /photos/ 路由(realpath 校验防穿越)"""

    def log_message(self, format, *args):
        # 静默(照片缩略图轮询会刷屏)
        pass

    def translate_path(self, path):
        if path.startswith('/photos/'):
            from urllib.parse import unquote
            rel = unquote(path[len('/photos/'):].split('?')[0].split('#')[0])
            full = os.path.realpath(os.path.join(PHOTOS_REAL, rel))
            if full.startswith(PHOTOS_REAL + os.sep) and os.path.isfile(full):
                return full
            return os.path.join(PHOTOS_REAL, '__missing__')    # → 404
        return super().translate_path(path)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('web_server')

    os.makedirs(PHOTOS_DIR, exist_ok=True)
    web_dir = os.path.join(get_package_share_directory('robot_sim'), 'web')
    handler = partial(ConsoleHandler, directory=web_dir)
    httpd = ThreadingHTTPServer(('0.0.0.0', PORT), handler)

    # 打印所有可用网卡的地址,方便平板/手机访问
    import socket
    ips = ['localhost']
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ips.insert(0, s.getsockname()[0])
        s.close()
    except Exception:
        pass

    node.get_logger().info(
        'Web 控制台已就绪: http://%s:%d  (同网段设备均可访问)  /photos/ -> %s'
        % (ips[0], PORT, PHOTOS_DIR))

    import threading
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
