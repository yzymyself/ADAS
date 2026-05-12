# ADASSim — 智能驾驶辅助系统仿真实训平台

基于 CARLA 0.9.13 的桌面级 ADAS 教学仿真系统，支持 ACC/AEB/LKA/BSD 算法仿真、G29 方向盘全物理操作、毫米波/超声波雷达硬件联动、SLAM 实时建图与全自动驾驶演示。

## 核心功能

- **ADAS 算法仿真**：ACC（自适应巡航）、AEB（自动紧急制动）、LKA（车道保持）、BSD（盲区检测）、ACC+LKA 联合模式，全部支持实时参数调节
- **G29 方向盘联动**：Logitech G29 方向盘力反馈同步、手动变速箱（含离合互锁、熄火机制）、踏板操作
- **自动驾驶演示**：基于 CARLA 路径规划的全自动驾驶，方向盘随路径自动同步转动
- **SLAM 建图**：激光雷达实时三维点云建图（kiss-icp），高精地图车道线采集与轨迹记录
- **硬件传感器**：MR20 毫米波雷达（77GHz）、DYP-A05 超声波雷达（8 路）的真实串口数据采集与 BEV 可视化，USB 摄像头车道线识别（YOLOPv2）
- **故障注入教学**：传感器、执行器故障模拟，帮助理解系统边界
- **双屏显示**：主屏仿真画面 + 副屏传感器监控（激光雷达点云、语义分割、深度图、雷达 BEV）

## 系统架构

```plain
┌──────────────┐    ┌──────────────────────────────────────┐
│  CARLA 服务端 │    │            ADASSim 客户端              │
│  (Unreal)    │◄───│  MainWindow ──────── MonitorWindow    │
│  TCP :2000   │    │    │                    │              │
└──────────────┘    │    ├─ pages/ (5个功能页面)              │
                    │    ├─ panels/ (HUD/图表/传感器面板)      │
                    │    ├─ src/adas/ (ACC/AEB/LKA/BSD)      │
                    │    ├─ src/vehicle/ (G29/控制融合)        │
                    │    └─ src/sensors/ (雷达/摄像头/LiDAR)   │
                    └──────────────────────────────────────┘
                              ▲         ▲
                    ┌─────────┘         └─────────┐
                    │ G29 方向盘                    │ 毫米波/超声波雷达
                    └──────────────────────────────┘
```

## 快速开始

### 环境要求

| 类别 | 最低配置 |
|------|---------|
| CPU | Intel i7 第 10 代 / AMD Ryzen 7 5000 |
| 内存 | 16 GB |
| GPU | NVIDIA RTX 2060（6 GB 显存） |
| 存储 | 100 GB SSD |
| 操作系统 | Windows 10/11 64-bit |
| Python | 3.10（conda 环境 torch241_cu124） |

外设：Logitech G29 方向盘、USB 摄像头、CH341 串口适配器（硬件传感器为可选项，系统支持仿真模拟模式）。

### 安装

```bash
# 1. 创建 conda 环境
conda create -n torch241_cu124 python=3.10 -y
conda activate torch241_cu124

# 2. 安装 PyTorch（CUDA 12.4）
pip install torch==2.4.1 torchvision==0.19.1 torchaudio==2.4.1 --index-url https://download.pytorch.org/whl/cu124

# 3. 安装 CARLA Python 客户端（本地 wheel）
pip install carla-0.9.13-cp310-cp310-win_amd64.whl

# 4. 安装其余依赖
pip install -r requirements.txt
```

### 启动

1. 启动 CARLA 服务端：运行 `CarlaUE4.exe`，等待场景加载完成（约 30~60 秒）
2. 双击 `run.bat` 或执行：
```bash
cd E:\ADAS\ADASSim
E:\Anaconda3\envs\torch241_cu124\pythonw.exe -m app.main
```
3. 在主界面选择功能模块，配置参数后点击启动

## 项目结构

```plain
ADASSim/
├── run.bat                     # Windows 启动脚本
├── app/                        # GUI 层（PyQt5）
│   ├── main.py                 # 应用入口
│   ├── main_window.py          # 主窗口核心逻辑
│   ├── monitor_window.py       # 副屏监控窗口
│   ├── launcher.py             # 仿真启动器
│   ├── pages/                  # 功能页面（首页/ADAS/自动驾驶/SLAM/硬件）
│   ├── panels/                 # HUD 覆盖层 / 传感器面板 / 图表
│   ├── widgets/                # 可复用组件（摄像头/雷达视图/小地图等）
│   └── assets/                 # 图标 / 字体 / 引导视频
├── src/                        # 业务逻辑层
│   ├── adas/                   # ADAS 算法（ACC/AEB/LKA/BSD/车道检测）
│   ├── lane_detection/         # YOLOPv2 车道线识别
│   ├── slam/                   # kiss-icp SLAM 建图 + 高精地图采集
│   ├── sensors/                # 毫米波雷达 / 超声波雷达 / LiDAR / 摄像头
│   ├── vehicle/                # G29 方向盘 / 键盘 / 手柄 / 控制融合器
│   ├── core/                   # CARLA 连接 / 场景管理 / 配置加载
│   └── fault_injection/        # 故障注入教学模块
├── config/                     # 配置文件（adas_params.yaml / g29_config.yaml 等）
├── data/                       # 路测视频素材
├── drivers/                    # 驱动安装包（G HUB / CH341）
├── slam_output/                # SLAM 建图输出（点云 / 轨迹 / 高精地图）
├── docs/                       # 使用文档 / 部署文档 / 架构文档
└── third_party/                # CARLA 官方导航 agents
```

## 文档

详细使用说明请参阅 `docs/Userguide.md`，部署指南请参阅 `docs/环境部署文档.md`。
