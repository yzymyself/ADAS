#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
MapLoader.py - 仅负责加载地图
功能：通过 client 加载指定地图，获取 world 对象，配置同步模式
"""

import sys

import carla


def load_map(client, map_name='Town05', sync=False):
    """
    加载指定地图并返回 world 对象

    参数:
        client: carla.Client 对象（已连接）
        map_name: 地图名称 (默认 'Town05')
        sync: 是否启用同步模式 (默认 False)

    返回:
        world: carla.World 对象
        original_settings: 原始设置（用于恢复，sync=True 时有效）
        traffic_manager: 交通管理器对象（sync=True 时有效）
    """
    print(f"Loading {map_name} map...")
    client.load_world(map_name)
    world = client.get_world()
    print(f"Map '{map_name}' loaded successfully.")

    original_settings = None
    traffic_manager = None

    if sync:
        original_settings = world.get_settings()
        settings = world.get_settings()
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        print("Synchronous mode enabled.")

    return world, original_settings, traffic_manager


def get_available_maps(client):
    """
    获取可用地图列表

    参数:
        client: carla.Client 对象

    返回:
        list: 可用地图名称列表
    """
    return client.get_available_maps()


if __name__ == '__main__':
    # 测试：需先启动 CARLA 服务器
    from CarlaConnector import connect_to_server

    client = connect_to_server()
    maps = get_available_maps(client)
    print("Available maps:")
    for m in maps:
        print(f"  - {m}")
