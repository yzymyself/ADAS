#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
CarlaConnector.py - 仅负责连接 CARLA 服务器
功能：根据 host 和 port 创建 carla.Client 连接，设置超时
"""

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def connect_to_server(host='127.0.0.1', port=2000, timeout=20.0):
    """
    连接到 CARLA 服务器

    参数:
        host: 服务器 IP 地址 (默认 '127.0.0.1')
        port: 服务器端口号 (默认 2000)
        timeout: 超时时间（秒）(默认 20.0)

    返回:
        client: carla.Client 对象
    """
    print(f"Connecting to CARLA server at {host}:{port}...")
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    print("Connected to CARLA server successfully.")
    return client


if __name__ == '__main__':
    # 测试连接
    client = connect_to_server()
    print(f"Server version: {client.get_server_version()}")
    world = client.get_world()
    print(f"Current map: {world.get_map().name}")
