#!/usr/bin/env python3
"""第一阶段验收 · 建图跑：驱动机器人走遍住宅，SLAM 在线出图。

为什么不用裸 cmd_vel 走固定轨迹：住宅里有家具和 90cm 门洞，开环走必撞，
撞完的激光会把错误障碍写进地图。所以这里用 Nav2 的 NavigateToPose，
避障交给导航栈，SLAM 只负责出图。

记录指标（后续阶段做对照实验的基线）：
    * 每个目标点：成功/失败、耗时
    * 全程路径长度（由 /odom 累积）
    * 最终地图：已知栅格数、自由栅格数、推算面积

产物：experiments/phase1_mapping_<时间戳>.json（机器可读）
      experiments/phase1_mapping_<时间戳>.md  （人可读）

用法：
    python3 phase1_mapping.py            # 默认读 share 里的 apartment.yaml
    python3 phase1_mapping.py --layout /path/to/apartment.yaml
"""

import argparse
import json
import math
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_path
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# 单个目标点最多等多久（软件渲染下仿真比实时慢，留足余量）
GOAL_TIMEOUT_S = 180.0


def normalize_goals(raw):
    """把 nav_goals 的各种写法统一成 [{'name','x','y','yaw'}]。

    布局 YAML 是人手写的，字段形式容易变（dict / list 混用），
    与其在读取处假设一种形状，不如在这里收敛一次并显式报错。
    """
    goals = []
    for i, g in enumerate(raw or []):
        if isinstance(g, dict):
            name = (g.get('label') or g.get('name') or g.get('id')
                    or f'goal_{i}')
            try:
                x, y = float(g['x']), float(g['y'])
            except (KeyError, TypeError) as exc:
                raise ValueError(f'第 {i} 个 nav_goal 缺 x/y: {g!r}') from exc
            yaw = float(g.get('yaw', 0.0))
        elif isinstance(g, (list, tuple)) and len(g) >= 2:
            name, x, y = f'goal_{i}', float(g[0]), float(g[1])
            yaw = float(g[2]) if len(g) > 2 else 0.0
        else:
            raise ValueError(f'无法解析第 {i} 个 nav_goal: {g!r}')
        goals.append({'name': str(name), 'x': x, 'y': y, 'yaw': yaw})
    return goals


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class Metrics:
    """累积 /odom 路径长度与 /map 覆盖率。"""

    def __init__(self, node):
        self.path_m = 0.0
        self._last = None
        self.map_msg = None
        node.create_subscription(Odometry, '/odom', self._on_odom, 20)
        node.create_subscription(OccupancyGrid, '/map', self._on_map, 5)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        cur = (p.x, p.y)
        if self._last is not None:
            self.path_m += math.hypot(cur[0] - self._last[0],
                                      cur[1] - self._last[1])
        self._last = cur

    def _on_map(self, msg):
        self.map_msg = msg

    def map_stats(self):
        m = self.map_msg
        if m is None:
            return {'available': False}
        cells = list(m.data)
        known = sum(1 for v in cells if v >= 0)
        free = sum(1 for v in cells if v == 0)
        occupied = sum(1 for v in cells if v >= 50)
        area = m.info.resolution * m.info.resolution
        return {
            'available': True,
            'width': m.info.width,
            'height': m.info.height,
            'resolution': m.info.resolution,
            'known_cells': known,
            'free_cells': free,
            'occupied_cells': occupied,
            'known_area_m2': round(known * area, 4),
            'free_area_m2': round(free * area, 4),
            'occupied_area_m2': round(occupied * area, 4),
            'known_fraction': round(known / max(1, len(cells)), 4),
        }


def drive_to(nav, goal, timeout_s):
    """送一个目标并等它真正结束，返回 (result, elapsed_s, timed_out)。

    ⚠️ 这里必须用 waitUntilNav2Complete（内部把 result_future 结算掉再回，
    然后才读 getResult），不能自己写「while not isTaskComplete() + 超时
    cancelTask + getResult」。

    为什么自带的那套写法会**伪造成功**：
      * isTaskComplete() 的第一行是 `if not self.result_future: return True`，
        即"取消后 result_future 被置空 → 直接返回完成"；
      * 而 getResult() 读的是 self.status —— **上一次动作的残留值**，
        因为 robot_navigator 里没有任何地方把 self.result 赋成新值。
    两件事叠加的结果：超时 cancelTask() 之后，getResult() 返回的是**上一个
    目标**的 SUCCEEDED。实测踩过：建图日志里厨房一条 180.1 s "SUCCEEDED"，
    180.1 恰好等于 GOAL_TIMEOUT_S=180.0，而机器人从未进过厨房 —— 它一直
    卡在书桌与隔墙之间。全程里程 29.4 m（完整巡游应约 40 m）也印证了这点。

    ⚠️ 另需检查 goToPose() 的返回值：它只等动作服务器"被发现"，
    不等 bt_navigator"激活完成"，所以巡游的第一个点会被
    "Action server is inactive. Rejecting the goal." 拒掉，
    goToPose 返回 False 且**不设** result_future → isTaskComplete() 立刻为真 →
    该点被记成 `TaskResult.UNKNOWN` / 0.0 s。返回第 4 个值 sent 供外层重试。
    """
    if not nav.goToPose(make_pose(nav, goal['x'], goal['y'], goal['yaw'])):
        return TaskResult.UNKNOWN, 0.0, False, False
    t0 = time.time()

    # waitUntilNav2Complete 内部自带阻塞等待与 result_future 结算，
    # 但它没有超时参数，所以外面用 spin_once 驱动、自己数超时。
    while not nav.isTaskComplete():
        rclpy.spin_once(nav, timeout_sec=0.1)
        if time.time() - t0 > timeout_s:
            nav.cancelTask()
            # 取消是异步的：等动作真正结算完再读结果，否则读到的仍是旧值。
            # 同时再给 cancelTask 一个上限，避免"取消也取消不掉"时死等。
            t1 = time.time()
            while nav.result_future is not None and time.time() - t1 < 30.0:
                rclpy.spin_once(nav, timeout_sec=0.1)
                fut = nav.result_future
                if fut is not None and fut.done():
                    break
            return TaskResult.CANCELED, round(time.time() - t0, 2), True, True
        time.sleep(0.4)

    return nav.getResult(), round(time.time() - t0, 2), False, True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--layout', default=None, help='apartment.yaml 路径')
    ap.add_argument('--outdir', default='experiments')
    ap.add_argument('--goal-timeout', type=float, default=GOAL_TIMEOUT_S)
    args = ap.parse_args()

    layout_path = Path(args.layout) if args.layout else (
        get_package_share_path('semnav_bringup') / 'config' / 'apartment.yaml')
    layout = yaml.safe_load(layout_path.read_text(encoding='utf-8'))
    # 优先用 mapping_tour（房间↔走廊交替的巡游路线），它把每段的两端都锚在
    # 走廊这一已知区域上，从而堵住"穿未知墙近路"这条失败路径；
    # 布局里没写 mapping_tour 时退回 nav_goals，行为与旧版一致。
    tour_key = 'nav_goals'
    if layout.get('mapping_tour'):
        tour_key = 'mapping_tour'
    goals = normalize_goals(layout.get(tour_key))
    print(f'[建图] 布局: {layout_path}')
    print(f'[建图] 路线来源: {tour_key}')
    print(f'[建图] 目标点 {len(goals)} 个: '
          + ', '.join(f"{g['name']}({g['x']:.1f},{g['y']:.1f})" for g in goals))

    rclpy.init()
    nav = BasicNavigator()
    metrics = Metrics(nav)

    # 就绪判据必须是 TF 树 + 数据，而不是「action server 是否存在」：
    # Nav2 在 configure 阶段就宣告 action server，那时节点还是 inactive，
    # 送目标会被直接拒绝，日志只留一句 "Goal was rejected!"，看不出根因。
    # （这个坑实际踩过：6 个目标全被拒，用时 0.0s。）
    tf_buffer = Buffer()
    TransformListener(tf_buffer, nav)

    print('[建图] 等待 TF 树 map->base_footprint（即 SLAM 出图）...')
    waited = 0.0
    while waited < 240.0:
        rclpy.spin_once(nav, timeout_sec=0.3)
        if tf_buffer.can_transform('map', 'base_footprint', Time()):
            print(f'  [OK] 用时 {waited:.0f}s')
            break
        waited += 0.3
    else:
        raise RuntimeError('SLAM 迟迟没出 map->odom 变换，请查 slam_toolbox 日志')

    print('[建图] 等待 /navigate_to_pose action server ...')
    waited = 0.0
    while not nav.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
        waited += 5.0
        print(f'  已等 {waited:.0f}s ...')
        if waited > 300.0:
            raise RuntimeError('Nav2 /navigate_to_pose 迟迟未出现，请查 launch 日志')
    # 服务"被发现"≠"已激活"：bt_navigator 在 inactive 时已宣告动作服务器，
    # 此时送目标会被 "Action server is inactive. Rejecting the goal." 拒掉。
    # 等它生命周期真正 active，再静默几秒让 Bond 建稳（否则第一个巡游点必被拒）。
    print('[建图] 等待 bt_navigator 激活 ...')
    waited = 0.0
    while waited < 180.0:
        st = subprocess.run(
            ['ros2', 'lifecycle', 'get', '/bt_navigator'],
            capture_output=True, text=True, timeout=15).stdout.strip()
        if 'active' in st:
            print(f'  [OK] bt_navigator active（用时 {waited:.0f}s）')
            break
        time.sleep(3.0)
        waited += 3.0
    time.sleep(3.0)
    print('[建图] 导航栈就绪，开始巡游')

    records = []
    t_start = time.time()
    for g in goals:
        rec = {'name': g['name'], 'x': g['x'], 'y': g['y'], 'yaw': g['yaw']}
        print(f"\n[建图] -> {g['name']} ({g['x']:.2f}, {g['y']:.2f})")
        # 被拒（bt_navigator 尚未激活完）不算到达失败，重试一次即可。
        for _ in range(2):
            result, elapsed, timed_out, sent = drive_to(nav, g, args.goal_timeout)
            if sent:
                break
            print('   目标被拒（导航栈尚未激活），5s 后重试 ...')
            time.sleep(5.0)
        rec['result'] = str(result)
        rec['accepted'] = bool(sent)
        rec['success'] = bool(result == TaskResult.SUCCEEDED)
        rec['timed_out'] = timed_out
        rec['elapsed_s'] = elapsed
        print(f"   {rec['result']}  用时 {rec['elapsed_s']:.1f}s"
              + ('  ← 超时取消' if timed_out else ''))
        records.append(rec)

    total_s = round(time.time() - t_start, 2)
    stats = metrics.map_stats()
    ok = sum(1 for r in records if r['success'])

    summary = {
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'layout': str(layout_path),
        'goals_total': len(records),
        'goals_succeeded': ok,
        'success_rate': round(ok / max(1, len(records)), 4),
        'wall_clock_s': total_s,
        'path_length_m': round(metrics.path_m, 3),
        'map': stats,
        'goals': records,
    }

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    (outdir / f'phase1_mapping_{stamp}.json').write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')

    md = [f'# 第一阶段建图跑 · {summary["timestamp"]}', '',
          f'- 目标点成功：**{ok}/{len(records)}**',
          f'- 全程耗时：{total_s:.1f} s',
          f'- 路径长度：{summary["path_length_m"]:.2f} m', '']
    if stats.get('available'):
        md += [f'- 地图已知面积：{stats["known_area_m2"]:.2f} m²'
               f'（自由 {stats["free_area_m2"]:.2f} m²）',
               f'- 地图栅格：{stats["width"]}×{stats["height"]}'
               f' @ {stats["resolution"]} m', '']
    md += ['| 目标点 | 坐标 | 结果 | 耗时(s) |', '|---|---|---|---|']
    md += [f'| {r["name"]} | ({r["x"]:.2f}, {r["y"]:.2f}) | {r["result"]} '
           f'| {r["elapsed_s"]:.1f} |' for r in records]
    (outdir / f'phase1_mapping_{stamp}.md').write_text('\n'.join(md) + '\n',
                                                       encoding='utf-8')

    print('\n===== 建图跑结果 =====')
    print(f'目标点成功 {ok}/{len(records)}  耗时 {total_s:.1f}s  '
          f'路径 {metrics.path_m:.2f} m')
    if stats.get('available'):
        print(f'地图已知面积 {stats["known_area_m2"]:.2f} m²  '
              f'自由 {stats["free_area_m2"]:.2f} m²')
    print(f'产物: {outdir}/phase1_mapping_{stamp}.json / .md')

    # ⚠️ 不要在这里调 nav.lifecycleShutdown()。
    #    它内部会**无限等待** /lifecycle_manager_localization/manage_nodes 服务，
    #    而建图跑起的是 mapping.launch.py（SLAM，没有 AMCL/定位生命周期管理器），
    #    该服务根本不存在 —— 实测脚本会卡在这里永不返回，把调用方（run_mapping.sh）
    #    挡在后面的 map_saver 之前，地图永远存不下来。
    #    建图段结束后该由**编排脚本**停栈，这里直接退出即可。
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
