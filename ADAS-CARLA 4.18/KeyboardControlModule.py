#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
KeyboardControlModule.py - 仅保留 WASDQE 键盘控制
功能：W油门 / S刹车 / A左转 / D右转 / Q倒档切换 / E交通车
"""

import carla
from traffic import traffic

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_e
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


class KeyboardControl(object):
    """处理键盘控制输入（仅 WASDQE）"""

    def __init__(self, world, start_in_autopilot=False):
        """
        初始化键盘控制器

        参数:
            world: World 对象（包含 player, hud 等）
            start_in_autopilot: 是否默认开启自动驾驶
        """
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, sync_mode):
        """
        解析键盘事件并执行对应操作

        参数:
            client: carla.Client 对象
            world: World 对象
            clock: pygame.time.Clock 对象
            sync_mode: 是否同步模式

        返回:
            bool: 是否应该退出（True=退出）
        """
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                # ---- 退出 ----
                if self._is_quit_shortcut(event.key):
                    return True
                # ---- 倒档切换 (Q) ----
                elif event.key == K_q and not (pygame.key.get_mods() & KMOD_CTRL):
                    if isinstance(self._control, carla.VehicleControl):
                        self._control.gear = 1 if self._control.reverse else -1
                # ---- 交通车生成/销毁 (E) ----
                elif event.key == K_e:
                    result = traffic(client, world.world, world.player)
                    if result is True:
                        world.hud.notification("Traffic vehicles spawned (13 cars, autopilot ON)")
                    elif result is False:
                        world.hud.notification("All traffic vehicles destroyed")

        # ===== 实时按键检测（持续按住） =====
        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # 自动灯光状态更新
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else:
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else:
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights:
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)

            world.player.apply_control(self._control)

        return False

    def _parse_vehicle_keys(self, keys, milliseconds):
        """解析车辆驾驶按键（WASD + 手刹）"""
        # W 油门
        if keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.01, 1.00)
        else:
            self._control.throttle = 0.0

        # S 刹车
        if keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        # A/D 转向
        steer_increment = 5e-4 * milliseconds
        if keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)

    def _parse_walker_keys(self, keys, milliseconds, world):
        """解析行人移动按键"""
        self._control.speed = 0.0
        if keys[K_s]:
            self._control.speed = 0.0
        if keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_w]:
            self._control.speed = world.player_max_speed
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        """判断是否为退出快捷键 (ESC 或 CTRL+Q)"""
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


if __name__ == '__main__':
    print("KeyboardControlModule loaded successfully.")
    print("This module should be used with a running CARLA session.")
