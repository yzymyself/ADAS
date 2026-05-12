# -*- coding: utf-8 -*-
"""
CameraController.py - CARLA 统一摄像头视角控制器

参照 CARLA 官方 CameraManager (VehicleSpawner.py) 的方式，
使用 bounding_box 动态计算摄像头位置，支持 Rigid/SpringArm 两种 attach 模式。

视角说明
    DRIVER    驾驶员（Rigid）
    TOP_DOWN  俯视（SpringArm）

单独测试（需先启动 CARLA）：
    python CameraController.py --host 127.0.0.1 --port 2000 --map Town05

依赖 carla, numpy, pygame, threading, math
"""

import numpy
import carla
import pygame
import math
import threading
import time
import weakref
import argparse
from enum import Enum

# ============================================================
#                    ViewMode 枚举
# [新增] 独立定义视角枚举，与 QtCarlaApp 中的 _MODES 列表对应。
#        原始代码的视角定义散布在 VehicleSpawner.py 的 CameraManager 中，
#        这里将其抽离为独立模块，便于 QtCarlaApp 复用。
# ============================================================

class ViewMode(Enum):
    DRIVER   = 'driver'    # 驾驶员视角：Rigid 刚性跟随，位于车内驾驶位
    TOP_DOWN = 'top_down'  # 俯视：SpringArm 弹簧臂跟随，从车后上方俯瞰

    @classmethod
    def from_string(cls, s):
        """从字符串解析视角枚举，支持名称和值两种匹配方式"""
        s = s.upper()
        for m in cls:
            if m.value.upper() == s or m.name.upper() == s:
                return m
        raise ValueError(f"未知视角: {s!r}")


# ============================================================
#              SpectatorCamera - 跟随摄像头的核心实现
# [新增] 封装 CARLA 的 RGB 传感器，自动 attach 到车辆并跟随。
#        参照官方 VehicleSpawner.py 中 CameraManager 的实现方式，
#        但不依赖 HUD 对象，可直接被 QtCarlaApp 调用。
# ============================================================

class SpectatorCamera:
    """
    RGB 摄像头，attach 到车辆上自动跟随。

    [新增] 与原始 VehicleSpawner.CameraManager 的区别：
    - 不依赖 HUD 对象，画面尺寸由参数直接传入
    - 提供 set_offset() 方法支持运行时切换视角参数
    - 使用线程锁保护帧缓冲区，避免回调与读取竞争

    参照 CARLA 官方 CameraManager 的方式：
    - 使用 bounding_box.extent 动态计算摄像头位置
    - offset 参数为倍数（乘以 bound_x/bound_y/bound_z）
    - 支持 Rigid 和 SpringArm 两种 attach 模式

    参数:
        world           : carla.World
        player          : carla.Vehicle
        width           : 图像宽
        height          : 图像高
        x_offset        : X 轴偏移倍数（负值=车后，正值=车前）
        y_offset        : Y 轴偏移倍数（负值=右侧，正值=左侧）
        z_offset        : Z 轴偏移倍数（正值=上方）
        pitch           : 俯仰角（度）
        fov             : 视野角（度）
        attachment_type : carla.AttachmentType（Rigid 或 SpringArm）
    """

    def __init__(self, world, player,
                 width=1280, height=720,
                 x_offset=-2.0,
                 y_offset=0.0,
                 z_offset=2.0,
                 pitch=8.0,
                 fov=90.0,
                 attachment_type=carla.AttachmentType.SpringArm):
        self._world = world
        self._player = player
        self._width = width
        self._height = height
        self._x_offset = x_offset
        self._y_offset = y_offset
        self._z_offset = z_offset
        self._pitch = pitch
        self._fov = fov
        self._attachment_type = attachment_type

        # [新增] 参照官方 CameraManager，用 bounding_box 动态计算基准尺寸
        # 这样不同车型的摄像头位置会自动适配车身大小
        self._bound_x = 0.5 + player.bounding_box.extent.x
        self._bound_y = 0.5 + player.bounding_box.extent.y
        self._bound_z = 0.5 + player.bounding_box.extent.z

        self._buffer = None       # [新增] 最新的 RGB 帧缓冲（numpy 数组）
        self._frame_id = 0        # [新增] 帧计数器，用于判断是否有新帧
        self._lock = threading.Lock()  # [新增] 线程锁，保护 _buffer 和 _frame_id
        self._sensor = None       # [新增] CARLA 摄像头传感器对象

        self._spawn()

    def _make_relative_transform(self):
        """
        计算相对于车辆的 Transform（参照官方 CameraManager）。

        [新增] 原始 CameraManager 用硬编码的 index 选择预设，
               这里改为用 x/y/z_offset 倍数动态计算，
               使得切换视角时只需调用 set_offset() 传入新参数即可。

        车辆本地坐标：X 向前，Y 向左，Z 向上
        摄像头默认朝前方看（+X 方向）
        x_offset 负值 = 车后方，正值 = 车前方
        """
        return carla.Transform(
            carla.Location(
                x=self._x_offset * self._bound_x,
                y=self._y_offset * self._bound_y,
                z=self._z_offset * self._bound_z
            ),
            carla.Rotation(pitch=self._pitch, yaw=0.0, roll=0.0)
        )

    def _spawn(self):
        """[新增] 创建摄像头传感器并 attach 到车辆，注册回调接收图像帧"""
        bp_lib = self._world.get_blueprint_library()
        bp = bp_lib.find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(self._width))
        bp.set_attribute('image_size_y', str(self._height))
        if bp.has_attribute('gamma'):
            bp.set_attribute('gamma', '2.2')
        if bp.has_attribute('fov'):
            bp.set_attribute('fov', str(self._fov))

        transform = self._make_relative_transform()
        self._sensor = self._world.spawn_actor(
            bp, transform,
            attach_to=self._player,
            attachment_type=self._attachment_type
        )
        # [新增] 使用 weakref 防止回调持有强引用导致对象无法被垃圾回收
        weak_self = weakref.ref(self)
        self._sensor.listen(
            lambda img: SpectatorCamera._on_image(weak_self, img)
        )

    @staticmethod
    def _on_image(weak_self, image):
        """[新增] 异步回调：将 CARLA 返回的 BGRA 图像转为 numpy RGB 数组"""
        self = weak_self()
        if self is None:
            return
        import numpy as np
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        # BGRA → RGB
        rgb = numpy.ascontiguousarray(array[:, :, 2::-1])
        with self._lock:
            self._buffer = rgb
            self._frame_id += 1

    def update(self):
        """[新增] attach 模式下摄像头由 CARLA 引擎自动跟随车辆，无需手动更新位置"""
        pass

    def get_image(self):
        """[新增] 返回 (frame_id, numpy_rgb)，调用方通过比较 frame_id 判断是否有新帧"""
        with self._lock:
            return self._frame_id, self._buffer

    def set_offset(self, x=None, y=None, z=None, pitch=None, attachment_type=None):
        """
        [新增] 切换视角参数：销毁旧传感器，用新偏移量重新创建。
               这比原始 CameraManager 通过 index 切换更灵活，
               可以在运行时动态调整任意参数。
        """
        if x is not None:
            self._x_offset = x
        if y is not None:
            self._y_offset = y
        if z is not None:
            self._z_offset = z
        if pitch is not None:
            self._pitch = pitch
        if attachment_type is not None:
            self._attachment_type = attachment_type
        self.destroy()
        self._spawn()

    def destroy(self):
        """[新增] 销毁摄像头传感器，停止回调并从 CARLA 世界中移除"""
        if self._sensor is not None:
            self._sensor.stop()
            self._sensor.destroy()
            self._sensor = None


# ============================================================
#             CameraController - 统一视角控制器
# [新增] 提供简洁的视角切换接口（set_mode/get_mode/next_view），
#        内部维护视角预设表 _View_PRESETS，替代原始 CameraManager
#        通过数字 index 切换视角的方式。被 QtCarlaApp 直接调用。
# ============================================================

# 视角预设（参照 CARLA 官方 CameraManager 的 _camera_transforms）
# [新增] 原始代码在 CameraManager 中用 index 0~5 硬编码 6 个视角，
#        这里改为用 ViewMode 枚举作 key 的字典，只保留实际使用的 2 个视角
# 参数含义：
#   x: X 轴偏移倍数（负值=车后，正值=车前）× bound_x
#   y: Y 轴偏移倍数（负值=右，正值=左）× bound_y
#   z: Z 轴偏移倍数（正值=上）× bound_z
#   pitch: 俯仰角
#   fov: 视野角
#   attachment: Rigid（刚性跟随，摄像头严格跟随车辆姿态）或 SpringArm（弹簧臂跟随，摄像头有平滑过渡）
_View_PRESETS = {
    ViewMode.DRIVER:  dict(x=+0.5, y=+0.3, z=1.5,  pitch=0.0,  fov=110,  attachment=carla.AttachmentType.Rigid,       desc='驾驶员'),
    ViewMode.TOP_DOWN: dict(x=-2.8, y=0.0,  z=4.6,  pitch=6.0,  fov=50,   attachment=carla.AttachmentType.SpringArm,  desc='俯视'),
}


class CameraController:
    """
    统一的 CARLA 摄像头控制器。

    [新增] 替代原始 VehicleSpawner.py 中的 CameraManager 类。
    - 原始 CameraManager 依赖 HUD 对象和 pygame 显示，无法在 QtCarlaApp 中复用
    - 本控制器不依赖 HUD，直接通过 SpectatorCamera 获取画面
    - 对外提供 set_mode() / get_image() / next_view() 简洁接口

    基于 SpectatorCamera，通过不同偏移参数实现各种视角。
    对外提供简单的 set_mode() / get_image() / update() 接口。

    参数:
        carla_world  : carla.World
        player       : carla.Vehicle
        display_size : tuple (width, height)
    """

    AVAILABLE_MODES = list(_View_PRESETS.keys())  # [新增] 可用视角列表，供 next_view() 循环

    def __init__(self, carla_world, player, display_size=(1280, 720)):
        self._world = carla_world
        self._player = player
        self._width, self._height = display_size

        # [新增] 默认驾驶员视角，初始化时创建 SpectatorCamera
        self._camera = SpectatorCamera(
            carla_world, player,
            width=self._width, height=self._height,
            x_offset=+0.5, y_offset=+0.3, z_offset=1.5,
            pitch=0.0, fov=110.0,
            attachment_type=carla.AttachmentType.Rigid
        )

        self._current_mode = ViewMode.DRIVER  # [新增] 记录当前视角模式
        self.set_mode(ViewMode.DRIVER)

    # ─────────────────────────────────────────────────────────
    #   公共 API
    # ─────────────────────────────────────────────────────────

    def set_mode(self, mode):
        """
        切换视角。

        参数:
            mode : ViewMode 常量或字符串
                   'driver' / 'top_down'
        """
        if isinstance(mode, str):
            mode = ViewMode.from_string(mode)

        if mode not in _View_PRESETS:
            raise ValueError(
                f"未知视角: {mode!r}，可用: {list(_View_PRESETS.keys())}"
            )

        self._current_mode = mode
        preset = _View_PRESETS[mode]
        # [新增] 通过 SpectatorCamera.set_offset() 切换视角，
        #        内部会销毁旧传感器并按新参数重新创建
        self._camera.set_offset(
            x=preset['x'],
            y=preset['y'],
            z=preset['z'],
            pitch=preset['pitch'],
            attachment_type=preset['attachment']
        )

    def get_mode(self):
        """返回当前视角模式"""
        return self._current_mode

    def update(self):
        """[新增] 每帧调用：在 attach 模式下 CARLA 引擎自动跟随，此方法保留为接口一致性"""
        self._camera.update()

    def get_image(self):
        """[新增] 返回 (frame_id, numpy_rgb)，QtCarlaApp 通过比较 frame_id 跳过重复帧"""
        return self._camera.get_image()

    def destroy(self):
        """销毁摄像头"""
        self._camera.destroy()

    # ─────────────────────────────────────────────────────────
    #   调试/辅助
    # ─────────────────────────────────────────────────────────

    def next_view(self):
        """轮换到下一个视角（所有模式循环）"""
        modes = self.AVAILABLE_MODES
        idx = modes.index(self._current_mode)
        self.set_mode(modes[(idx + 1) % len(modes)])

    def get_camera(self):
        """返回内部的 SpectatorCamera 对象"""
        return self._camera


# ============================================================
#          单独测试入口（需要先启动 CARLA 服务器）
# [新增] 提供独立测试功能，不依赖 QtCarlaApp，使用 pygame 渲染。
#        正式使用时通过 QtCarlaApp 调用 CameraController。
# ============================================================

if __name__ == '__main__':
    import numpy

    parser = argparse.ArgumentParser(description='CameraController 单独测试')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--map', default='Town05')
    args = parser.parse_args()

    # 连接 CARLA
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.load_world(args.map)
    print(f"已连接 CARLA，加载地图: {args.map}")

    # 生成一辆测试车
    bp_lib = world.get_blueprint_library()
    bp = bp_lib.filter('vehicle.*')[0]
    bp.set_attribute('role_name', 'hero')
    spawn_points = world.get_map().get_spawn_points()
    player = world.spawn_actor(bp, spawn_points[0])
    print(f"车辆生成: {bp.id}")

    try:
        cam = CameraController(world, player, display_size=(1280, 720))
        cam.set_mode('driver')
        print("CameraController 初始化完成")
        print("提示: 按 V 切换视角 | ESC 退出")

        mode_names = {ViewMode.DRIVER: 'DRIVER 驾驶员', ViewMode.TOP_DOWN: 'TOP_DOWN 俯视'}

        pygame.init()
        display = pygame.display.set_mode((1280, 720))
        pygame.display.set_caption('CameraController 测试')
        clock = pygame.time.Clock()

        running = True
        while running:
            clock.tick(30)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_v:
                        cam.next_view()
                        print(f"切换到: {mode_names[cam.get_mode()]}")

            cam.update()
            frame_id, img = cam.get_image()
            if img is not None:
                surface = pygame.surfarray.make_surface(img.transpose(1, 0, 2))
                display.blit(pygame.transform.scale(surface, (1280, 720)), (0, 0))

            pygame.display.flip()

        pygame.quit()
    finally:
        cam.destroy()
        player.destroy()
        print("资源已清理")
