"""
交通车模块 - 在主车周围生成多台自动驾驶车辆（模拟周围交通）
按E键切换：批量生成 -> 批量销毁 -> 批量生成 ...

车辆分布（共13辆）：
  - 前方同车道   1 辆（固定50m）      — 目标速度   20 km/h
  - 前方其他车道 3 辆（随机50~150m）   — 目标速度 30/40/50 km/h
  - 后方其他车道 3 辆（随机20~100m）   — 目标速度 60/70/80 km/h
  - 后方其他车道 6 辆（随机200~400m）  — 目标速度 100~120 km/h
"""

import carla
import random


# =============================================================================
# 工具函数
# =============================================================================


def _move_along_waypoint(waypoint, distance):
    """
    沿车道Waypoint向前/后移动指定距离，返回目标Waypoint。
    正值=向前，负值=向后。
    """
    remaining = abs(distance)
    current_wp = waypoint
    step_size = 2.0
    max_steps = int(abs(distance) / step_size) + 5
    step = 0
    
    while remaining > 0.1 and step < max_steps:
        step += 1
        if distance > 0:
            next_wps = current_wp.next(min(remaining, step_size))
        else:
            next_wps = current_wp.previous(min(remaining, step_size))
        if not next_wps:
            break
        current_wp = next_wps[0]
        remaining -= step_size
    
    return current_wp


def _get_spawn_transform(waypoint, offset_z=0.3):
    """从Waypoint构建生成Transform。"""
    t = waypoint.transform
    t.location.z += offset_z
    return t


def _kmh_to_mps(kmh):
    """km/h 转 m/s"""
    return kmh / 3.6


def _set_vehicle_speed(tm, actor, target_kmh):
    """
    通过TrafficManager设置车辆目标速度（km/h）。
    CARLA 0.9.13 兼容写法。
    """
    try:
        # 方法1: vehicle_fraction_speed_limit（推荐）
        # 获取车辆最大速度来反算比例
        max_speed = float(actor.get_attribute('speed').recommended_values[-1])
        # recommended_values 最后一个是最高档位对应的最大速度(km/h)
        fraction = min(target_kmh / max_speed, 1.0)
        tm.vehicle_fraction_speed_limit_of_vehicle(actor, fraction)
    except Exception:
        try:
            # 方法2: vehicle_percentage_speed_difference
            # 设定比最大速度慢多少百分比
            max_speed = float(actor.get_attribute('speed').recommended_values[-1])
            pct_diff = ((max_speed - target_kmh) / max_speed) * 100
            pct_diff = max(0.0, pct_diff)
            tm.vehicle_percentage_speed_difference(actor, pct_diff)
        except Exception:
            print(f"[traffic] 无法设置车辆 {actor.id} 的速度为 {target_kmh} km/h")


def _try_spawn_vehicle(world, bp_lib, valid_bps, transform, tm=None, target_kmh=None):
    """
    尝试在指定位置生成一辆随机车辆并开启autopilot。
    可选传入 TrafficManager 和目标速度。
    
    Returns:
        (vehicle_actor, name_str) 或 (None, None)
    """
    for attempt in range(3):
        bp = random.choice(valid_bps)
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
        
        actor = world.try_spawn_actor(bp, transform)
        if actor is not None:
            # 开启自动驾驶，注册到TrafficManager
            if tm is not None:
                actor.set_autopilot(True, tm.get_port())
                if target_kmh is not None:
                    _set_vehicle_speed(tm, actor, target_kmh)
            else:
                actor.set_autopilot(True)
            
            name = bp.id.split('.')[1].replace('_', ' ').title()
            return actor, name
        
        # 偏移重试
        offset = random.uniform(-3.0, 3.0)
        alt = carla.Transform(
            carla.Location(x=transform.location.x + offset,
                           y=transform.location.y,
                           z=transform.location.z),
            transform.rotation
        )
        transform = alt
    
    return None, None


# =============================================================================
# 主函数
# =============================================================================


# 模块级变量：保存所有已生成的交通车列表
_traffic_vehicles = []

# 每辆车的目标速度（km/h），共13个
_TARGET_SPEEDS = [
    20,              # #0  前方同车道
    30, 40, 50,      # #1~3 前方邻道 ×3
    60, 70, 80,      # #4~6 后方邻道(近) ×3 (20~100m)
    100, 104, 108, 112, 116, 120,  # #7~12 后方邻道(远) ×6 (200~400m)
]


def traffic(client, world, player):
    """
    切换交通车的显示状态：
      - 第一次调用：在主车周围批量生成10台自动驾驶车辆（各车不同速度）
      - 再次调用：销毁所有已生成的交通车
      - 以此类推...
    
    Args:
        client: carla.Client 对象
        world:  carla.World 对象
        player: 主车 actor
    
    Returns:
        True  - 成功生成了车辆
        False - 销毁了所有车辆
        None  - 操作失败
    """
    global _traffic_vehicles
    
    # ==================== 销毁模式 ====================
    if _traffic_vehicles:
        destroyed = 0
        for v in _traffic_vehicles:
            try:
                v.set_autopilot(False)
                v.destroy()
                destroyed += 1
            except Exception as e:
                print(f"[traffic] 销毁车辆失败（可忽略）: {e}")
        _traffic_vehicles.clear()
        print(f"[traffic] 已销毁 {destroyed} 辆交通车")
        return False
    
    # ==================== 生成模式 ====================
    try:
        map_ = world.get_map()
        bp_lib = world.get_blueprint_library()
        all_bps = bp_lib.filter('vehicle.*')
        valid_bps = [bp for bp in all_bps if int(bp.get_attribute('number_of_wheels')) == 4]
        if not valid_bps:
            valid_bps = list(all_bps)
        
        # 获取TrafficManager（用于精确控制车速）
        tm = client.get_trafficmanager(port=8000)
        
        player_location = player.get_location()
        base_wp = map_.get_waypoint(player_location)
        if base_wp is None:
            print("[traffic] 无法获取主车Waypoint")
            return None
        
        # 获取相邻车道（左右）—— CARLA 0.9.13 用单数API
        side_lanes = []
        left_lane = base_wp.get_left_lane()
        if left_lane is not None:
            side_lanes.append(left_lane)
        right_lane = base_wp.get_right_lane()
        if right_lane is not None:
            side_lanes.append(right_lane)
        
        if not side_lanes:
            side_lanes = [base_wp]
        
        spawn_tasks = []
        
        # ---------- ① 前方同车道 1辆（固定50m，20km/h） ----------
        fwd_wp_50 = _move_along_waypoint(base_wp, 50.0)
        spawn_tasks.append((_get_spawn_transform(fwd_wp_50), _TARGET_SPEEDS[0]))
        
        # ---------- ② 前方其他车道 3辆（50~150m 随机，30/40/50 km/h） ----------
        for i in range(3):
            dist = random.uniform(50, 150)
            lane_wp = random.choice(side_lanes)
            target_wp = _move_along_waypoint(lane_wp, dist)
            spawn_tasks.append((_get_spawn_transform(target_wp), _TARGET_SPEEDS[1 + i]))
        
        # ---------- ③ 后方其他车道(近) 3辆（20~100m 随机，60/70/80 km/h） ----------
        for i in range(3):
            dist = -random.uniform(20, 100)  # 负数=向后，20~100m
            lane_wp = random.choice(side_lanes)
            target_wp = _move_along_waypoint(lane_wp, dist)
            spawn_tasks.append((_get_spawn_transform(target_wp), _TARGET_SPEEDS[4 + i]))
        
        # ---------- ④ 后方其他车道(远) 6辆（200~400m 随机，100~120 km/h） ----------
        for i in range(6):
            dist = -random.uniform(200, 400)  # 负数=向后，200~400m
            lane_wp = random.choice(side_lanes)
            target_wp = _move_along_waypoint(lane_wp, dist)
            spawn_tasks.append((_get_spawn_transform(target_wp), _TARGET_SPEEDS[4 + i]))
        
        # ---------- 批量生成 ----------
        success_count = 0
        labels = (['前方同车道']
                  + ['前方邻道'] * 3
                  + ['后方邻道(近)'] * 3
                  + ['后方邻道(远)'] * 6)
        
        for i, (tf, speed_kmh) in enumerate(spawn_tasks):
            vehicle, vname = _try_spawn_vehicle(
                world, bp_lib, valid_bps, tf,
                tm=tm, target_kmh=speed_kmh
            )
            if vehicle is not None:
                _traffic_vehicles.append(vehicle)
                success_count += 1
                label = labels[i]
                dist_val = "50m" if i == 0 else f"{abs(spawn_tasks[i][0].location.distance(base_wp.transform.location)):.0f}m"
                print(f"[traffic] [{label}] {vname} → {speed_kmh} km/h")
            else:
                print(f"[traffic] 第{i+1}个位置生成失败（目标{speed_kmh}km/h），跳过")
        
        if success_count == 0:
            print("[traffic] 所有位置均无法生成车辆！")
            return None
        
        print(f"\n[traffic] === 共成功生成 {success_count}/13 辆交通车 ===\n"
              f"  · 前方同车道   1 辆（50m）       →   20 km/h\n"
              f"  · 前方邻道      {min(success_count-1, 3)} 辆（50~150m）→ 30/40/50 km/h\n"
              f"  · 后方邻道(近)  {max(0, min(success_count-4, 3))} 辆（20~100m）→ 60/70/80 km/h\n"
              f"  · 后方邻道(远)  {max(0, success_count-7)} 辆（200~400m）→ 100~120 km/h")
        return True
        
    except Exception as e:
        print(f"[traffic] 生成交通车时发生错误: {e}")
        import traceback
        traceback.print_exc()
        return None
