# -*- coding: utf-8 -*-
"""
QtCarlaApp.py - CARLA 0.9.13 摄像头显示

界面中间有连接按钮，点击后连接CARLA服务器，连接成功按钮消失。

用法：
    python QtCarlaApp.py                           # 默认 127.0.0.1:2000, Town05
    python QtCarlaApp.py --host 192.168.1.10       # 指定 IP

操作：
    WASD / 方向键  控制车辆
    V            循环切换视角
    C            循环切换视角（同 V）
    ESC          退出
"""

from __future__ import print_function
import sys
import os

# ================================================================
#                    CARLA 环境配置（自动检测路径）
# [新增] 原始代码依赖手动配置 CARLA 路径或从 pygame 版本的 main.py 传入，
#        这里改为自动检测：先从脚本相对路径推断，再尝试硬编码路径，
#        最后回退到 CARLA_ROOT 环境变量。
# ================================================================

# [新增] 获取当前脚本所在目录，用于推断 CARLA 根目录和添加模块搜索路径
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# [新增] 按优先级尝试多种 CARLA 根目录：相对路径 > 硬编码路径 > 环境变量
_CANDIDATE_ROOTS = [
    os.path.join(_SCRIPT_DIR, '..', '..', '..'),  # 从 PythonAPI/ADAS-CARLA 上两级
    r'E:\deep_study\CARLA_0.9.13\WindowsNoEditor',
    os.environ.get('CARLA_ROOT', ''),
]

CARLA_ROOT = None
for _root in _CANDIDATE_ROOTS:
    # [新增] 通过检测 CarlaUE4 目录判断是否为有效的 CARLA 根目录
    if _root and os.path.isdir(os.path.join(_root, 'CarlaUE4')):
        CARLA_ROOT = _root
        break

if CARLA_ROOT is None:
    print("[错误] 无法找到 CARLA 根目录，请确保 CARLA 已安装或设置 CARLA_ROOT 环境变量")
    sys.exit(1)

ADAS_CARLA_DIR = os.path.dirname(os.path.abspath(__file__))  # [新增] 本项目目录，用于导入同目录模块

# [新增] 将 CARLA 的 PythonAPI 路径和本项目目录加入 sys.path，
#        确保能正确导入 carla 包和本地模块（CarlaConnector、VehicleSpawner、CameraController）
for _path in [ADAS_CARLA_DIR, os.path.join(CARLA_ROOT, 'PythonAPI', 'carla', 'dist'),
              os.path.join(CARLA_ROOT, 'PythonAPI')]:
    if _path not in sys.path:
        sys.path.insert(0, _path)


def _setup_dll_path(root):
    """[新增] 配置 DLL 搜索路径，Windows 下 CARLA 需要加载 CarlaUE4 和 Engine 的 DLL"""
    paths = [
        os.path.join(root, r'CarlaUE4\Binaries\Win64'),
        os.path.join(root, r'Engine\Binaries\Win64'),
    ]
    _third_party = os.path.join(root, r'Engine\Binaries\ThirdParty')
    if os.path.isdir(_third_party):
        for entry in os.scandir(_third_party):
            if entry.is_dir():
                for sub in [r'Win64\VS2015', r'Win64']:
                    p = os.path.join(entry.path, sub)
                    if os.path.isdir(p):
                        paths.append(p)
    cur = os.environ.get('PATH', '')
    for p in paths:
        if p and p not in cur:
            cur = p + os.pathsep + cur
    os.environ['PATH'] = cur


_setup_dll_path(CARLA_ROOT)
import carla
import math
import argparse
import datetime

# [新增] 导入同目录下的模块：
#   CarlaConnector - CARLA 服务器连接工具
#   VehicleSpawner - 车辆生成器（World 类），修改后支持 hud=None
#   CameraController - 独立摄像头控制器，替代原始 CameraManager
from CarlaConnector import connect_to_server
from VehicleSpawner import World, get_actor_display_name
from CameraController import CameraController

try:
    from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QVBoxLayout, QPushButton, QMessageBox
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtGui import QImage, QPixmap, QKeyEvent
except ImportError:
    raise RuntimeError('[错误] 无法导入 PyQt5，请运行: pip install PyQt5')


# ================================================================
#                  命令行参数解析
# ================================================================

def _parse_args():
    parser = argparse.ArgumentParser(
        description='CARLA 摄像头显示',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例：
    python QtCarlaApp.py
    python QtCarlaApp.py --host 127.0.0.1 --port 2000
    python QtCarlaApp.py --width 1920 --height 1080
        '''
    )
    parser.add_argument('--host', default='127.0.0.1', help='CARLA 服务器地址 (默认 127.0.0.1)')
    parser.add_argument('--port', type=int, default=2000, help='CARLA 端口 (默认 2000)')
    parser.add_argument('--map', default='Town05', help='地图名称 (默认 Town05)')
    parser.add_argument('--sync', action='store_true', help='开启同步模式')
    parser.add_argument('--width', type=int, default=1280, help='图像宽度 (默认 1280)')
    parser.add_argument('--height', type=int, default=720, help='图像高度 (默认 720)')
    return parser.parse_args()


# ================================================================
#                  QtCameraDisplay - 全屏画面
# [新增] 用 QLabel 显示 CARLA 摄像头画面，替代原始 pygame 渲染方式。
#        pygame 版本通过 HUD.render() 将画面 blit 到 pygame surface，
#        Qt 版本通过 QImage → QPixmap → QLabel.setPixmap() 显示。
# ================================================================

class QtCameraDisplay(QLabel):
    """[新增] 显示 CARLA 摄像头画面（全屏占据整个窗口），替代 pygame 显示"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("""
            background-color: #000000;
            color: #555;
        """)
        self.setText("等待连接...")
        # 关键：不接受焦点，确保键盘事件始终由父窗口（CarlaWindow）处理
        self.setFocusPolicy(Qt.NoFocus)

    def update_image(self, rgb_array):
        """[新增] 将 numpy RGB 数组转为 QPixmap 并显示，自动缩放适配窗口大小"""
        if rgb_array is None:
            return
        try:
            h, w, ch = rgb_array.shape
            qt_img = QImage(rgb_array.data, w, h, ch * w, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_img)
            scaled = pixmap.scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.setPixmap(scaled)
        except Exception:
            pass


# ================================================================
#                  QtVehicleController - 键盘控制
# [新增] 通过 Qt 键盘事件驱动 CARLA 车辆，替代原始 pygame 版本的
#        KeyboardControl 模块。操控逻辑（油门/刹车/转向/倒挡/手刹）
#        与原始 KeyboardControl 保持一致。
# ================================================================

class QtVehicleController:
    """[新增] 通过 Qt keyPressEvent/keyReleaseEvent 驱动 CARLA 车辆，替代 pygame KeyboardControl"""

    def __init__(self, vehicle):
        self._vehicle = vehicle
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0
        self._keys = set()
        self.speed_kmh = 0.0

    def key_press(self, key):
        self._keys.add(key)

    def key_release(self, key):
        self._keys.discard(key)

    def tick(self, dt_ms=50):
        """[新增] 每帧调用，根据当前按下的键计算并应用车辆控制指令"""
        steer_inc = 5e-4 * dt_ms

        # 油门（每帧累加，松开时清零）
        if 'W' in self._keys:
            self._control.throttle = min(self._control.throttle + 0.05, 1.0)
        else:
            self._control.throttle = 0.0

        # 刹车（每帧累加，松开时清零）
        if 'S' in self._keys:
            self._control.brake = min(self._control.brake + 0.3, 1.0)
        else:
            self._control.brake = 0.0

        # 转向
        if 'A' in self._keys:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_inc
        elif 'D' in self._keys:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_inc
        else:
            self._steer_cache = 0.0
        self._steer_cache = max(-0.7, min(0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 2)

        # 倒档
        if 'Q' in self._keys:
            self._control.gear = -1 if self._control.gear >= 0 else 1

        # 手刹
        self._control.hand_brake = ('SPACE' in self._keys)

        self._vehicle.apply_control(self._control)

        # [新增] 计算当前车速（km/h），供状态栏显示
        v = self._vehicle.get_velocity()
        self.speed_kmh = 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


# ================================================================
#                  ConnectWidget - 连接界面（居中）
# [新增] 初始连接界面，包含标题、服务器信息和连接按钮。
#        原始 pygame 版本通过命令行直接连接，无 GUI 连接界面。
#        Qt 版本提供可视化连接按钮，连接成功后自动隐藏。
# ================================================================

class ConnectWidget(QWidget):
    """[新增] 居中显示的连接界面，点击按钮触发连接 CARLA"""

    def __init__(self, args, parent=None):
        super().__init__(parent)
        self._args = args
        self._setup_ui()

    def _setup_ui(self):
        # 主布局
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(20)

        # 标题
        title = QLabel("CARLA 0.9.13")
        title.setStyleSheet("""
            font-size: 32px;
            font-weight: bold;
            color: #2196F3;
        """)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # 连接信息
        info = QLabel(f"服务器: {self._args.host}:{self._args.port}\n地图: {self._args.map}")
        info.setStyleSheet("""
            font-size: 14px;
            color: #666;
        """)
        info.setAlignment(Qt.AlignCenter)
        layout.addWidget(info)

        # 连接按钮
        self._btn_connect = QPushButton("连接 CARLA")
        self._btn_connect.setFixedSize(200, 60)
        self._btn_connect.setStyleSheet("""
            QPushButton {
                font-size: 18px;
                font-weight: bold;
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #1565C0;
            }
        """)
        self._btn_connect.clicked.connect(self._on_connect_clicked)
        layout.addWidget(self._btn_connect)

        # 状态标签
        self._status_label = QLabel("")
        self._status_label.setStyleSheet("""
            font-size: 14px;
            color: #FF9800;
        """)
        self._status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self._status_label)

        self.setLayout(layout)

    def _on_connect_clicked(self):
        """点击连接按钮"""
        self._btn_connect.setEnabled(False)
        self._btn_connect.setText("连接中...")
        self._status_label.setText("正在连接，请稍候...")

        # [新增] 通过 window() 获取父级 CarlaWindow 实例，调用其 do_connect() 方法
        self.window().do_connect()


# ================================================================
#                  CarlaWindow - 主窗口
# [新增] Qt5 主窗口，替代原始 pygame 渲染循环。核心变化：
#   1. 用 QTimer 替代 pygame 的事件循环，分别驱动仿真 tick 和画面刷新
#   2. 用 QtCameraDisplay (QLabel) 替代 pygame surface 显示画面
#   3. 通过 keyPressEvent/keyReleaseEvent 捕获键盘，替代 pygame.event
#   4. 不依赖 HUD 对象，World 初始化时传入 hud=None
# ================================================================

class CarlaWindow(QMainWindow):

    def __init__(self, args):
        super().__init__()
        self.setFocusPolicy(Qt.StrongFocus)

        self._args = args
        self._client = None
        self._world_obj = None
        self._controller = None
        self._camera_ctrl = None
        self._is_connected = False
        self._view_idx = 0   # 0=驾驶员, 1=俯视
        self._view_names = ['驾驶员', '俯视']
        self._MODES = ['driver', 'top_down']

        self._last_frame_id = -1    # [新增] 用于跳过重复帧，避免无效 CPU 开销

        # [新增] 三个定时器替代 pygame 的主循环：
        #   _sim_timer    - 仿真 tick（50ms/20Hz），驱动车辆控制和同步模式
        #   _cam_timer    - 摄像头刷新（33ms/30Hz），从 CameraController 获取最新帧
        #   _status_timer - 状态打印（2s），输出 FPS/速度/位置到控制台
        self._sim_timer = QTimer()
        self._sim_timer.timeout.connect(self._on_sim_tick)
        self._sim_timer.setInterval(50)

        self._cam_timer = QTimer()
        self._cam_timer.timeout.connect(self._on_camera_tick)
        self._cam_timer.setInterval(33)

        self._fps_count = 0
        self._fps_elapsed = 0.0
        self._last_tick = datetime.datetime.now()
        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._print_status)
        self._status_timer.setInterval(2000)

        # [新增] UI 布局：连接界面和摄像头画面叠加放置，
        #        连接成功后隐藏连接界面，显示摄像头画面
        self._camera_display = QtCameraDisplay()
        self._connect_widget = ConnectWidget(args)
        self._stacked_widget = QWidget()
        self._stacked_layout = QVBoxLayout()
        self._stacked_layout.setContentsMargins(0, 0, 0, 0)
        self._stacked_layout.addWidget(self._connect_widget)
        self._stacked_layout.addWidget(self._camera_display)
        self._stacked_container = QWidget()
        self._stacked_container.setLayout(self._stacked_layout)
        self.setCentralWidget(self._stacked_container)
        self._camera_display.hide()  # 初始隐藏

        self.setWindowTitle(f"CARLA 摄像头 — {args.host}:{args.port} / {args.map}")
        self.resize(args.width, args.height)

    # ─────────────────────────────────────────────────────────────

    def do_connect(self):
        """点击连接按钮后执行"""
        self.setWindowTitle(f"连接 {self._args.host}:{self._args.port} ...")
        try:
            self._client = connect_to_server(
                host=self._args.host,
                port=self._args.port,
                timeout=20.0
            )
            print(f"[连接] CARLA {self._client.get_server_version()} 已连接")

            self._client.load_world(self._args.map)
            sim_world = self._client.get_world()
            print(f"[地图] 加载 {self._args.map} 完成")

            # [新增] 同步模式：开启后仿真由客户端驱动 tick，
            #        可保证每帧数据一致，但需要手动调用 world.tick()
            if self._args.sync:
                settings = sim_world.get_settings()
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
                sim_world.apply_settings(settings)
                tm = self._client.get_trafficmanager()
                tm.set_synchronous_mode(True)
                print("[模式] 同步模式已开启")

            # [新增] 生成车辆，传入 hud=None 表示不使用 pygame HUD，
            #        传入 display_size 指定摄像头画面分辨率
            #        原始代码中 CameraManager 依赖 hud.dim 获取画面尺寸，
            #        修改后改为由 display_size 参数独立传入
            self._world_obj = World(
                sim_world,
                None,               # 不需要 HUD
                filter='vehicle.*',
                generation='2',
                role_name='hero',
                gamma=2.2,
                sync=self._args.sync,
                display_size=(self._args.width, self._args.height)
            )
            player = self._world_obj.player
            print(f"[车辆] {get_actor_display_name(player)} 生成完成")

            # 释放手刹（车辆启动时默认锁止）
            control = player.get_control()
            control.hand_brake = False
            player.apply_control(control)

            # [新增] 键盘控制器，替代原始 pygame 版本的 KeyboardControl
            self._controller = QtVehicleController(player)

            # [新增] 摄像头控制器，替代原始 CameraManager，
            #        使用独立模块 CameraController（不依赖 HUD）
            self._camera_ctrl = CameraController(
                sim_world,
                player,
                display_size=(self._args.width, self._args.height)
            )
            self._camera_ctrl.set_mode(self._MODES[self._view_idx])
            print(f"[视角] {self._view_names[self._view_idx]}")

            self._is_connected = True

            # [新增] 切换到摄像头界面（隐藏连接界面）
            self._connect_widget.hide()
            self._camera_display.show()

            # [新增] 启动三个定时器，替代 pygame 的主循环
            self._sim_timer.start()
            self._cam_timer.start()
            self._status_timer.start()
            self._fps_timer_start = datetime.datetime.now()

            self.setWindowTitle(
                f"CARLA 摄像头 — {self._args.host}:{self._args.port} / {self._args.map} | "
                f"{self._view_names[self._view_idx]}")

            print()
            print("=" * 50)
            print("  WASD / 方向键  控制车辆")
            print("  V / C         循环切换视角")
            print("  ESC           退出")
            print("=" * 50)

        except Exception as e:
            self.setWindowTitle("连接失败")
            QMessageBox.critical(self, "连接失败", str(e))
            self._connect_widget._btn_connect.setEnabled(True)
            self._connect_widget._btn_connect.setText("重新连接")
            self._connect_widget._status_label.setText(f"连接失败: {e}")
            print(f"[错误] {e}")

    # ─────────────────────────────────────────────────────────────

    def _on_sim_tick(self):
        """[新增] 仿真 tick 定时器回调（50ms/20Hz），驱动车辆控制和同步模式 tick"""
        if not self._is_connected:
            return
        try:
            if self._args.sync:
                self._world_obj.world.tick()
            if self._controller:
                self._controller.tick(50)
            if self._camera_ctrl:
                self._camera_ctrl.update()

            # [新增] 确保主窗口始终持有焦点（防止被子控件抢走）
            if not self.hasFocus():
                self.setFocus(Qt.OtherFocusReason)

            now = datetime.datetime.now()
            self._fps_elapsed += (now - self._last_tick).total_seconds()
            self._last_tick = now
            self._fps_count += 1
        except Exception as e:
            print(f"[错误] tick: {e}")

    def _on_camera_tick(self):
        """[新增] 摄像头刷新定时器回调（33ms/30Hz），获取最新帧并更新显示"""
        if not self._is_connected or not self._camera_ctrl:
            return
        frame_id, img = self._camera_ctrl.get_image()
        # [新增] 只有新帧才渲染，跳过重复帧避免无效 CPU 开销
        if img is not None and frame_id != self._last_frame_id:
            self._last_frame_id = frame_id
            self._camera_display.update_image(img)

    def _print_status(self):
        """[新增] 状态打印定时器回调（2s），输出 FPS/速度/位置到控制台"""
        if not self._is_connected or self._fps_elapsed == 0:
            return
        fps = self._fps_count / self._fps_elapsed
        speed = self._controller.speed_kmh if self._controller else 0
        t = self._world_obj.player.get_transform()
        print(
            f"  FPS: {fps:.1f}  |  速度: {speed:.1f} km/h  |  "
            f"位置: ({t.location.x:8.2f}, {t.location.y:8.2f})  |  "
            f"朝向: {t.rotation.yaw:6.1f}°  |  {self._view_names[self._view_idx]}",
            end='\r'
        )
        self._fps_count = 0
        self._fps_elapsed = 0.0

    # ─────────────────────────────────────────────────────────────

    def keyPressEvent(self, event: QKeyEvent):
        """[新增] Qt 键盘按下事件，替代 pygame 的 event.get()。映射到 QtVehicleController"""
        if not self._is_connected:
            return
        key = None
        if event.key() == Qt.Key_Escape:
            self.close()
            return
        elif event.key() in (Qt.Key_W, Qt.Key_Up):
            key = 'W'
        elif event.key() in (Qt.Key_S, Qt.Key_Down):
            key = 'S'
        elif event.key() in (Qt.Key_A, Qt.Key_Left):
            key = 'A'
        elif event.key() in (Qt.Key_D, Qt.Key_Right):
            key = 'D'
        elif event.key() in (Qt.Key_Q,):
            key = 'Q'
        elif event.key() in (Qt.Key_Space,):
            key = 'SPACE'
        elif event.key() in (Qt.Key_V, Qt.Key_C):
            self._cycle_view()
            return
        else:
            # 处理普通字母键（从 event.text() 获取）
            text = event.text().upper()
            if text in ('W', 'A', 'S', 'D', 'Q'):
                key = text
            else:
                return
        if key and self._controller:
            self._controller.key_press(key)

    def keyReleaseEvent(self, event: QKeyEvent):
        """[新增] Qt 键盘释放事件，替代 pygame 的 KEYUP 事件"""
        if not self._is_connected:
            return
        key = None
        if event.key() in (Qt.Key_W, Qt.Key_Up):
            key = 'W'
        elif event.key() in (Qt.Key_S, Qt.Key_Down):
            key = 'S'
        elif event.key() in (Qt.Key_A, Qt.Key_Left):
            key = 'A'
        elif event.key() in (Qt.Key_D, Qt.Key_Right):
            key = 'D'
        elif event.key() in (Qt.Key_Q,):
            key = 'Q'
        elif event.key() in (Qt.Key_Space,):
            key = 'SPACE'
        else:
            text = event.text().upper()
            if text in ('W', 'A', 'S', 'D', 'Q'):
                key = text
            else:
                return
        if key and self._controller:
            self._controller.key_release(key)

    def _cycle_view(self):
        """[新增] 循环切换视角，在驾驶员和俯视之间轮换，替代原始的 V/C 键切换"""
        if not self._camera_ctrl:
            return
        self._view_idx = (self._view_idx + 1) % len(self._MODES)
        self._camera_ctrl.set_mode(self._MODES[self._view_idx])
        name = self._view_names[self._view_idx]
        self.setWindowTitle(
            f"CARLA 摄像头 — {self._args.host}:{self._args.port} / {self._args.map} | {name}")
        print(f"\n[视角] {name}")

    def closeEvent(self, event):
        """[新增] 窗口关闭事件，停止定时器并销毁 CARLA 资源"""
        print("\n[退出] 正在清理资源 ...")
        self._sim_timer.stop()
        self._cam_timer.stop()
        self._status_timer.stop()
        if self._camera_ctrl:
            self._camera_ctrl.destroy()
        if self._world_obj:
            self._world_obj.destroy()
        print("[退出] 已关闭")
        event.accept()


# ================================================================
#                       入口
# [新增] 程序入口，替代原始 pygame 版本的 main.py 启动方式。
#        原始版本在 main.py 中通过 pygame 事件循环运行，
#        Qt 版本通过 QApplication 事件循环运行。
# ================================================================

if __name__ == '__main__':
    # 设置 Qt 应用程序属性（支持高DPI）
    os.environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '1'
    os.environ['QT_SCALE_FACTOR_ROUNDING_POLICY'] = 'PassThrough'

    # 设置 Qt 应用程序属性（支持高DPI）—— 必须在 QApplication 之前设置
    os.environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '1'
    os.environ['QT_SCALE_FACTOR_ROUNDING_POLICY'] = 'PassThrough'

    args = _parse_args()

    # 启动提示
    print("=" * 55)
    print("  CARLA 0.9.13 仿真界面")
    print("=" * 55)
    print(f"  服务器: {args.host}:{args.port}")
    print(f"  地图: {args.map}")
    print(f"  分辨率: {args.width}x{args.height}")
    print(f"  同步模式: {'开启' if args.sync else '关闭'}")
    print("=" * 55)
    print("  [提示] 点击 '连接 CARLA' 按钮开始仿真")
    print("=" * 55)
    print()

    try:
        # [新增] AA_EnableHighDpiScaling 必须在 QApplication 构造前设置，
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
        app = QApplication(sys.argv)
        win = CarlaWindow(args)
        win.show()
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("\n[退出] 用户中断")
        sys.exit(0)
