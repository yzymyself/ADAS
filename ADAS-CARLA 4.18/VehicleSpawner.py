#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
VehicleSpawner.py - 仅负责主车定位与生成
功能：在指定位置生成主车，配置车辆属性，安装传感器
"""

import sys
import random
import weakref
import math
import re

import carla
from carla import ColorConverter as cc

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


# ==============================================================================
# -- 全局辅助函数 --------------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    import re
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


def get_actor_blueprints(world, filter_pattern='vehicle.*', generation='2'):
    bps = world.get_blueprint_library().filter(filter_pattern)
    if generation.lower() == "all":
        return bps
    if len(bps) == 1:
        return bps
    try:
        int_generation = int(generation)
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


def modify_vehicle_physics(actor):
    """修改车辆物理参数，启用扫掠碰撞检测"""
    try:
        physics_control = actor.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        actor.apply_physics_control(physics_control)
    except Exception:
        pass


# ==============================================================================
# -- 传感器类 ------------------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = {}
        for frame, intensity in self.history:
            history[frame] = history.get(frame, 0) + intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        # [修改] hud 可能为 None（QtCarlaApp 模式下传入 hud=None），需做空指针保护
        if self.hud is not None:
            self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        # [修改] hud 可能为 None（QtCarlaApp 模式下传入 hud=None），需做空指针保护
        if self.hud is not None:
            self.hud.notification('Crossed line %s' % ' and '.join(text))


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction=2.2, display_size=None):
        # [修改] 增加 display_size 参数：QtCarlaApp 模式下没有 HUD 对象，
        #        但仍需指定摄像头画面分辨率，所以增加独立的 display_size 参数
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        # [修改] 获取显示尺寸：优先使用传入的 display_size，
        #        否则从 hud.dim 获取，最后回退到默认 1280x720
        if display_size is not None:
            self._display_size = display_size
        elif hud is not None and hasattr(hud, 'dim'):
            self._display_size = hud.dim
        else:
            self._display_size = (1280, 720)  # 默认尺寸
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid)]
        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.CityScapesPalette, 'Camera Instance Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.Raw, 'Camera Instance Segmentation (Raw)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                 'lens_circle_falloff': '3.0',
                 'chromatic_aberration_intensity': '0.5',
                 'chromatic_aberration_offset': '0'}],
            ['sensor.camera.optical_flow', cc.Raw, 'Optical Flow', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(self._display_size[0]))  # [修改] 用 self._display_size 替代 hud.dim
                bp.set_attribute('image_size_y', str(self._display_size[1]))  # [修改] 同上
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify and self.hud is not None:  # [修改] hud 可能为 None，需做空指针保护
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        if self.hud is not None:  # [修改] hud 可能为 None，需做空指针保护
            self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self._display_size) / (2.0 * self.lidar_range)  # [修改] 用 self._display_size 替代 hud.dim
            lidar_data += (0.5 * self._display_size[0], 0.5 * self._display_size[1])  # [修改] 同上
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self._display_size[0], self._display_size[1], 3)  # [修改] 用 self._display_size 替代 hud.dim
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# 延迟导入 pygame（CameraManager.render 需要用到）


# ==============================================================================
# -- RadarSensor --------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5  # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))


# ==============================================================================
# -- World 类（管理世界、地图、车辆生成） ----------------------------------------
# ==============================================================================


class World(object):
    """管理 CARLA 世界、地图、车辆和传感器"""

    # 默认出生点坐标（Town05）
    DEFAULT_SPAWN_LOCATION = (-103.0, -204.0, 14.0)
    DEFAULT_SPAWN_ROTATION = (0, 180, 0)  # pitch, yaw, roll

    def __init__(self, carla_world, hud, args=None,
                 spawn_location=None, spawn_rotation=None,
                 filter='vehicle.*', generation='2',
                 role_name='hero', gamma=2.2, sync=False,
                 display_size=None):  # [修改] 增加 display_size 参数：QtCarlaApp 可传入画面尺寸
        """
        初始化 World

        参数:
            carla_world: carla.World 对象
            hud: HUD 实例（可以为 None）
            args: 命令行参数对象（可选，用于兼容旧接口）
            spawn_location: 自定义出生点 (x, y, z)，默认使用 DEFAULT_SPAWN_LOCATION
            spawn_rotation: 自定义朝向 (pitch, yaw, roll)，默认使用 DEFAULT_SPAWN_ROTATION
            filter: 车辆过滤器 (默认 'vehicle.*')
            generation: 车代数 (默认 '2')
            role_name: 角色名称 (默认 'hero')
            gamma: 伽马校正 (默认 2.2)
            sync: 同步模式 (默认 False)
            display_size: 显示尺寸 tuple (width, height)，默认为 None
        """
        self.world = carla_world
        self.sync = sync
        self.hud = hud
        self._display_size = display_size  # [修改] 保存显示尺寸，供 CameraManager 使用
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

        # 从 args 获取参数（如果提供）
        if args is not None:
            self.actor_role_name = args.rolename
            self._actor_filter = args.filter
            self._actor_generation = args.generation
            self._gamma = args.gamma
        else:
            self.actor_role_name = role_name
            self._actor_filter = filter
            self._actor_generation = generation
            self._gamma = gamma

        # 自定义出生点
        if spawn_location is not None:
            self._spawn_location = spawn_location
        else:
            self._spawn_location = self.DEFAULT_SPAWN_LOCATION

        if spawn_rotation is not None:
            self._spawn_rotation = spawn_rotation
        else:
            self._spawn_rotation = self.DEFAULT_SPAWN_ROTATION

        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

        self.restart()
        if self.hud is not None:  # [修改] hud 可能为 None（QtCarlaApp 模式），避免空指针
            self.world.on_tick(self.hud.on_world_tick)

    def restart(self):
        """重新启动/生成车辆"""
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713

        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # 获取蓝图：默认只生成指定轿车列表
        _SEDAN_LIST = [
            'vehicle.mercedes.coupe_2020',     # 奔驰 E级 Coupe
            'vehicle.lincoln.mkz_2020',         # 林肯 MKZ
            'vehicle.mini.cooper_s_2021',       # Mini Cooper S
            'vehicle.ford.crown',               # 福特 Crown Victoria
            'vehicle.dodge.charger_2020',       # 道奇 Charger
            'vehicle.tesla.cybertruck',         # 特斯拉 Cybertruck
        ]
        blueprints = get_actor_blueprints(self.world, self._actor_filter, 'all')
        # 优先从指定轿车列表中选
        sedans = [bp for bp in blueprints if bp.id in _SEDAN_LIST]
        if sedans:
            blueprint = random.choice(sedans)
        else:
            # 回退到4轮车
            fallback = [bp for bp in blueprints if int(bp.get_attribute('number_of_wheels')) == 4]
            blueprint = random.choice(fallback) if fallback else random.choice(blueprints)
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # 在指定位置生成主车
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            modify_vehicle_physics(self.player)

        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            # 使用自定义出生点坐标
            spawn_point = carla.Transform(
                carla.Location(x=self._spawn_location[0],
                               y=self._spawn_location[1],
                               z=self._spawn_location[2]),
                carla.Rotation(pitch=self._spawn_rotation[0],
                              yaw=self._spawn_rotation[1],
                              roll=self._spawn_rotation[2])
            )
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            modify_vehicle_physics(self.player)

        # 安装传感器
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma, self._display_size)  # [修改] 传入 display_size
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        if self.hud is not None:  # [修改] hud 可能为 None
            self.hud.notification(actor_type)

        # 启动时默认开启手刹
        control = self.player.get_control()
        control.hand_brake = True
        self.player.apply_control(control)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def set_spawn_position(self, location, rotation=None):
        """
        设置自定义出生点位置（下次 restart 生效）

        参数:
            location: tuple (x, y, z) 出生点坐标
            rotation: tuple (pitch, yaw, roll) 朝向角度，默认 (0, 180, 0)
        """
        self._spawn_location = location
        if rotation is not None:
            self._spawn_rotation = rotation
        print(f"Spawn position set to {location}, rotation {self._spawn_rotation}")

    def tick(self, clock):
        """每帧更新"""
        if self.hud is not None:  # [修改] hud 可能为 None
            self.hud.tick(self, clock)

    def render(self, display):
        """渲染画面"""
        if self.camera_manager:
            self.camera_manager.render(display)
        if self.hud is not None:  # [修改] hud 可能为 None
            self.hud.render(display)

    def destroy_sensors(self):
        """销毁传感器"""
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        """销毁所有传感器和玩家"""
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

    # ---- 天气/地图层相关方法 ----

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None


if __name__ == '__main__':
    # 测试：需先启动 CARLA 服务器
    from CarlaConnector import connect_to_server
    from MapLoader import load_map

    client = connect_to_server()
    world_obj, _, _ = load_map(client, map_name='Town05')

    # 简单测试：打印地图信息
    map_data = world_obj.get_map()
    print(f"Map name: {map_data.name}")
    spawn_points = map_data.get_spawn_points()
    print(f"Available spawn points: {len(spawn_points)}")
