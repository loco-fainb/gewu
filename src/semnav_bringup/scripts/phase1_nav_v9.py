#!/usr/bin/env python3
"""A 模式导航验收（已知地图 + AMCL + Nav2，不跑 SLAM 建图）。

与 phase1_navigation.py 的区别 —— 它只回答"到没到"，本脚本回答
ObjectNav 标准指标所要求的"怎么到的、走了多远"：

  SR   （Success Rate）        终点距目标 <= 0.25 m 且 action 返回 SUCCEEDED
  SPL  （Success weighted by Path Length）
        = (1/N) Σ  S_i · L_i* / max(L_i*, L_i)
        L_i* = 起点到目标**按地图可通行格的最短路径长度**（不是直线距离）
        L_i  = 实际走过的路径长度（对 /odom 积分）
  DTG  （Distance To Goal）     逐目标过程曲线（每 0.5 s 采样一次）

为什么不沿用旧脚本的记法：旧结果只有 elapsed / final_dist，**没有 L_i**，
SPL 因此完全算不出来（EXPERIMENTS.md 第 六 节已记录这个缺口）。这里补上。

轨迹与 rosbag 同时落盘，一并补掉 M4 验收物的另两项缺口。

用法：
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 scripts/phase1_nav_v9.py                  # 全部 13 个目标
    python3 scripts/phase1_nav_v9.py --goals 厨房,客厅
    python3 scripts/phase1_nav_v9.py --no-bag         # 不录 rosbag（省磁盘）
"""

import argparse
import json
import math
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

WS = Path.home() / 'projects' / 'semnav'

GOAL_TIMEOUT_S = 300.0
SAMPLE_DT = 0.5
SUCCESS_DIST_M = 0.25


# --------------------------------------------------------------------------
# 地图与最短路径（用于 SPL 的 L*）
# --------------------------------------------------------------------------
def load_occupancy(map_yaml):
    """读三值 PGm。返回 (occ[nx, ny], res, (ox, oy))，索引约定同 check_reachability。"""
    meta = yaml.safe_load(open(map_yaml, encoding='utf-8'))
    pgm = Path(map_yaml).parent / meta['image']
    raw = open(pgm, 'rb').read()
    parts = raw.split(b'\n', 3)
    if parts[0].strip() != b'P5':
        raise SystemExit(f'只支持 P5 PGM: {parts[0]!r}')
    w, h = map(int, parts[1].split())
    data = np.frombuffer(parts[3][:w * h], dtype=np.uint8).reshape(h, w)
    occ = np.ascontiguousarray(data[::-1].T)
    o = meta['origin']
    return occ, float(meta['resolution']), (float(o[0]), float(o[1]))


def build_passable(occ, res, radius):
    from scipy import ndimage
    occupied = occ < 90
    unknown = (occ >= 90) & (occ < 250)
    dist = ndimage.distance_transform_edt(~occupied) * res
    return (~occupied) & (~unknown) & (dist >= radius)


def geodesic(occ, res, origin, radius, a, b):
    """A* 在可通行格上求 a->b 的最短路径长度（米）。不可达返回 None。

    这是 SPL 的 L* 正确定义 —— 用**可行走距离**而不是欧氏直线距离。
    若用直线距离，绕墙的路线会让 L*/L > 1，SPL 变成 >1 的无意义值。
    """
    import heapq
    from scipy import ndimage
    passable = build_passable(occ, res, radius)
    labels, _ = ndimage.label(passable, structure=np.ones((3, 3)))
    nx, ny = occ.shape
    ox, oy = origin

    def ij(x, y):
        return int((x - ox) / res), int((y - oy) / res)

    si, sj = ij(*a)
    gi, gj = ij(*b)
    for i, j in ((si, sj), (gi, gj)):
        if not (0 <= i < nx and 0 <= j < ny) or not passable[i, j]:
            return None
    if labels[si, sj] != labels[gi, gj]:
        return None

    def h(i, j):
        return math.hypot(i - gi, j - gj)

    g = {(si, sj): 0.0}
    pq = [(h(si, sj), 0.0, si, sj)]
    prev = {}
    steps = [(1, 0), (-1, 0), (0, 1), (0, -1),
             (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while pq:
        _, gc, i, j = heapq.heappop(pq)
        if (i, j) == (gi, gj):
            return gc * res
        if gc > g.get((i, j), 1e18):
            continue
        for di, dj in steps:
            ni, nj = i + di, j + dj
            if not (0 <= ni < nx and 0 <= nj < ny) or not passable[ni, nj]:
                continue
            ng = gc + math.hypot(di, dj)
            if ng < g.get((ni, nj), 1e18):
                g[(ni, nj)] = ng
                prev[(ni, nj)] = (i, j)
                heapq.heappush(pq, (ng + h(ni, nj), ng, ni, nj))
    return None


# --------------------------------------------------------------------------
# 里程计轨迹采样（背景线程，边导航边记）
# --------------------------------------------------------------------------
class OdomTracker:
    """订阅 /odom 累积路径长度与 DTG 曲线。

    ⚠️ 用 rclpy 订阅而不是 `ros2 topic echo` 子进程：后者在软件渲染的
    虚拟机上启动要 1~2 s，采样率会稀到把轨迹抹平。
    """

    def __init__(self, node, target_getter):
        from nav_msgs.msg import Odometry
        self.node = node
        self.target_getter = target_getter
        self.samples = []       # [(t, x, y, dtg)]
        self.path_len = 0.0
        self.last = None
        self.t0 = None
        node.create_subscription(Odometry, '/odom', self._cb, 20)

    def _cb(self, msg):
        p = msg.pose.pose.position
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        if self.last is not None:
            self.path_len += math.hypot(p.x - self.last[0], p.y - self.last[1])
        self.last = (p.x, p.y)
        tgt = self.target_getter()
        dtg = math.hypot(p.x - tgt[0], p.y - tgt[1]) if tgt else float('nan')
        self.samples.append((now - self.t0, p.x, p.y, dtg))

    def reset(self):
        self.samples = []
        self.path_len = 0.0
        self.last = None
        self.t0 = None

    def dtg_curve(self):
        if not self.samples:
            return []
        step = max(1, int(round(SAMPLE_DT / max(
            1e-6, np.median(np.diff([s[0] for s in self.samples]))))))
        return [[round(s[0], 2), round(s[3], 3)] for s in self.samples[::step]]


# --------------------------------------------------------------------------
def wait_ready(node, names, timeout=180.0):
    """等生命周期节点真正 active。

    ⚠️ 不能用 goToPose() 的 wait_for_server() 代替 —— 动作服务器在
    inactive 状态就已宣告存在，这个等待会被骗过，goal 随即被拒。
    """
    from lifecycle_msgs.srv import GetState
    t0 = time.time()
    pending = list(names)
    while pending and time.time() - t0 < timeout:
        still = []
        for n in pending:
            cli = node.create_client(GetState, f'/{n}/get_state')
            if not cli.wait_for_service(timeout_sec=1.0):
                still.append(n)
                continue
            fut = cli.call_async(GetState.Request())
            import rclpy
            rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
            st = fut.result()
            if st is None or st.current_state.label != 'active':
                still.append(n)
        pending = still
        if pending:
            time.sleep(2.0)
    return pending


def main():
    ap = argparse.ArgumentParser(description='A 模式导航验收（SR / SPL / DTG）')
    ap.add_argument('--layout', default=str(WS / 'src/semnav_bringup/config/apartment.yaml'))
    ap.add_argument('--map', default=str(WS / 'maps/apartment_v9_furnished_map.yaml'))
    ap.add_argument('--nav2-params',
                    default=str(WS / 'src/semnav_bringup/config/nav2_params.yaml'))
    ap.add_argument('--goals', default=None, help='只跑子集，逗号分隔目标名')
    ap.add_argument('--goal-timeout', type=float, default=GOAL_TIMEOUT_S)
    ap.add_argument('--no-bag', action='store_true')
    ap.add_argument('--out-dir', default=str(WS / 'experiments'))
    args = ap.parse_args()

    import rclpy
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

    layout = yaml.safe_load(open(args.layout, encoding='utf-8'))
    spawn = layout['spawn']
    start = (float(spawn['x']), float(spawn['y']))

    raw_goals = layout.get('nav_goals') or []
    goals = []
    for g in raw_goals:
        goals.append({'name': g.get('label') or g.get('name') or g.get('id'),
                      'x': float(g['x']), 'y': float(g['y']),
                      'yaw': float(g.get('yaw', 0.0))})
    if args.goals:
        want = {s.strip() for s in args.goals.split(',') if s.strip()}
        goals = [g for g in goals if g['name'] in want]
        if not goals:
            raise SystemExit(f'--goals 未匹配: {args.goals}')

    print(f'[验收] 起点 {start}，目标 {len(goals)} 个: '
          + ', '.join(f"{g['name']}({g['x']:.2f},{g['y']:.2f})" for g in goals))

    # ---- 地图参数（L* 在运行期逐段计算，见下方说明） ----
    occ, res, origin = load_occupancy(args.map)
    params = yaml.safe_load(open(args.nav2_params, encoding='utf-8'))
    radius = float(params['local_costmap']['local_costmap']
                   ['ros__parameters']['robot_radius'])
    print(f'[验收] 地图 {occ.shape[0]}x{occ.shape[1]} @ {res} m, 车体半径 {radius} m')
    # ⚠️ L* 不能统一从 spawn 算。
    #    path_len 是「上一目标终点 -> 本目标」的距离，而这里若用 spawn 当起点，
    #    两者起点不同，除第一个目标外 SPL 全部失真（实测把整体从 0.828 压到
    #    0.630，餐厅/客厅从 1.000/1.000 被算成 0.248/0.164）。
    #    故 L* 在每段开跑前用 leg_start 现算。

    # ---- rosbag（可选） ----
    bag_proc = None
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = WS / 'bags' / f'nav_v9_{stamp}'
    if not args.no_bag:
        bag_dir.parent.mkdir(parents=True, exist_ok=True)
        topics = ['/odom', '/scan', '/amcl_pose', '/cmd_vel', '/tf', '/tf_static',
                  '/initialpose', '/plan', '/local_costmap/costmap']
        cmd = ['ros2', 'bag', 'record', '-o', str(bag_dir), '--topics'] + topics
        try:
            bag_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                        stderr=subprocess.DEVNULL)
            print(f'[验收] rosbag 录制中 -> {bag_dir}')
        except Exception as exc:
            print(f'[验收] rosbag 启动失败（不阻塞验收）: {exc}')

    rclpy.init()
    node = rclpy.create_node('phase1_nav_v9')
    nav = BasicNavigator()

    # 等 bt_navigator 真正 active 再开火（README 记录的"起跑枪太早"坑）
    pending = wait_ready(node, ['bt_navigator', 'controller_server',
                                'planner_server', 'amcl'])
    if pending:
        print(f'[验收] ✗ 这些节点未 active: {pending}')
    else:
        print('[验收] ✓ Nav2 节点全部 active')
    time.sleep(3.0)   # 让 Bond 建稳

    cur = {'t': start}
    tracker = OdomTracker(node, lambda: (cur['t'][0], cur['t'][1]))
    import rclpy.executors
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    results = []
    wall0 = time.time()
    try:
        leg_start = start          # 本段起点：第一个目标是 spawn，之后是上一目标终点
        for g in goals:
            print(f"\n[验收] === {g['name']} ({g['x']:.2f}, {g['y']:.2f}) ===")
            cur['t'] = (g['x'], g['y'])
            tracker.reset()
            g['leg_start'] = [round(leg_start[0], 3), round(leg_start[1], 3)]
            g['L_star'] = geodesic(occ, res, origin, radius,
                                   leg_start, (g['x'], g['y']))
            print(f"   段起点 ({leg_start[0]:.2f}, {leg_start[1]:.2f})  L*="
                  + (f"{g['L_star']:.2f} m" if g['L_star'] else '不可达'))

            from geometry_msgs.msg import PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x = g['x']
            pose.pose.position.y = g['y']
            pose.pose.orientation.z = math.sin(g['yaw'] / 2.0)
            pose.pose.orientation.w = math.cos(g['yaw'] / 2.0)

            t0 = time.time()
            accepted = nav.goToPose(pose)
            if not accepted:
                print('   目标被拒，重试一次')
                time.sleep(2.0)
                accepted = nav.goToPose(pose)
            if not accepted:
                print('   ✗ 两次都被拒')
                fl = tracker.last or (g['x'], g['y'])
                leg_start = (fl[0], fl[1])
                results.append({
                    'name': g['name'], 'x': g['x'], 'y': g['y'],
                    'leg_start': [round(leg_start[0], 3), round(leg_start[1], 3)],
                    'L_star': g.get('L_star'), 'accepted': False,
                    'result': 'REJECTED', 'success': False, 'timed_out': False,
                    'cancelled': False, 'elapsed_s': 0.0,
                    'final_dist_m': None, 'final_pos': None,
                    'path_len_m': 0.0, 'spl': 0.0, 'reached_by_position': False,
                    'dtg_curve': [],
                })
                continue

            timed_out = cancelled = False
            deadline = time.time() + args.goal_timeout
            while not nav.isTaskComplete():
                executor.spin_once(timeout_sec=0.1)
                if time.time() > deadline:
                    print(f'   ⏱ 超时 {args.goal_timeout:.0f}s，取消')
                    nav.cancelTask()
                    timed_out = cancelled = True
                    # ⚠️ cancelTask 后必须等动作**真正结算**再读结果：
                    # isTaskComplete() 在 result_future 被置空时返回 True，
                    # getResult() 会读回**上一个目标**的残留状态（假成功）。
                    t_c = time.time()
                    while time.time() - t_c < 15.0:
                        executor.spin_once(timeout_sec=0.1)
                        if nav.isTaskComplete() and not nav.result_future:
                            break
                    break

            elapsed = time.time() - t0
            # ⚠️ getResult() 会无界阻塞到 result_future 结算。超时路径下 BT 可能
            #    仍在窄处跑恢复行为（实测卡 970 s，把 300s 上限拖成 1288 s），
            #    故超时时改用带 deadline 的轮询，拿不到就按 CANCELED 记。
            if timed_out:
                t_r = time.time()
                settled = False
                while time.time() - t_r < 10.0:
                    executor.spin_once(timeout_sec=0.1)
                    if nav.result_future is not None and nav.result_future.done():
                        settled = True
                        break
                if settled:
                    try:
                        nav_result = nav.getResult()
                    except Exception:
                        nav_result = TaskResult.CANCELED
                else:
                    nav_result = TaskResult.CANCELED
                    print('   （BT 未结算，按 CANCELED 记录，不等）')
            else:
                nav_result = nav.getResult()
            ok_result = (nav_result == TaskResult.SUCCEEDED) and not timed_out

            # 物证：按坐标验（action 返回值可能被状态机伪造）
            od = tracker.last
            final_dist = (math.hypot(od[0] - g['x'], od[1] - g['y'])
                          if od else float('inf'))
            by_pos = final_dist <= SUCCESS_DIST_M
            success = bool(ok_result and by_pos and accepted)

            L = tracker.path_len
            Ls = g.get('L_star')
            spl = (Ls / max(Ls, L)) if (success and Ls and L > 0) else 0.0

            results.append({
                **g,
                'result': str(nav_result),
                'accepted': bool(accepted),
                'timed_out': timed_out,
                'cancelled': cancelled,
                'elapsed_s': round(elapsed, 2),
                'final_dist_m': round(final_dist, 4) if final_dist != float('inf') else None,
                'final_pos': [round(od[0], 3), round(od[1], 3)] if od else None,
                'path_len_m': round(L, 3),
                'spl': round(spl, 4),
                'reached_by_position': bool(by_pos),
                'success': success,
                'dtg_curve': tracker.dtg_curve(),
            })
            # 推进下一段的起点：用**实际终点**（失败也一样，机器人确实在那儿）
            leg_start = ((od[0], od[1]) if od else (g['x'], g['y']))

            mark = '✓' if success else '✗'
            if Ls:
                print(f"   {mark} {nav_result}  {elapsed:.1f}s  终点距目标 "
                      f"{final_dist:.3f} m  走过 {L:.2f} m  "
                      f"(L*={Ls:.2f})  SPL={spl:.3f}")
            else:
                print(f"   {mark} {nav_result}  {elapsed:.1f}s  终点距 "
                      f"{final_dist:.3f} m")

    except KeyboardInterrupt:
        print('\n[验收] 中断')
    finally:
        wall = time.time() - wall0
        if bag_proc:
            bag_proc.send_signal(signal.SIGINT)
            try:
                bag_proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                bag_proc.kill()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    # ---- 汇总 ----
    n = len(results)
    succ = sum(1 for r in results if r['success'])
    spls = [r['spl'] for r in results if r.get('spl') is not None]
    sr = succ / n if n else 0.0
    spl_mean = sum(spls) / len(spls) if spls else 0.0

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    payload = {
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'mode': 'A_known_map (no SLAM)',
        'map': args.map,
        'layout': args.layout,
        'start': list(start),
        'goals_total': n,
        'goals_succeeded': succ,
        'success_rate': round(sr, 4),
        'spl': round(spl_mean, 4),
        'wall_clock_s': round(wall, 2),
        'bag': str(bag_dir) if bag_proc else None,
        'goals': results,
    }
    jf = out / f'phase1_nav_v9_{ts}.json'
    jf.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')

    md = [f'# A 模式导航验收 · {payload["timestamp"]}', '',
          f'- 模式：已知地图 + AMCL + Nav2（不跑 SLAM 建图）',
          f'- 地图：`{args.map}`',
          f'- 起点：{start}',
          f'- **SR = {sr:.1%}**（{succ}/{n}），**SPL = {spl_mean:.3f}**',
          f'- 全程耗时 {wall:.1f} s',
          f'- rosbag：{payload["bag"] or "未录"}', '',
          '| 目标 | 坐标 | 结果 | 耗时(s) | 终点误差(m) | 实际路径(m) | L*(m) | SPL |',
          '|---|---|---|---|---|---|---|---|']
    for r in results:
        fd = '-' if r['final_dist_m'] is None else f"{r['final_dist_m']:.3f}"
        ls = '-' if not r.get('L_star') else f"{r['L_star']:.2f}"
        sp = '-' if not r.get('L_star') else f"{r['spl']:.3f}"
        md.append(
            f"| {r['name']} | ({r['x']:.2f},{r['y']:.2f}) | "
            f"{'✓' if r['success'] else '✗'} {r['result']} | "
            f"{r['elapsed_s']:.1f} | {fd} | {r['path_len_m']:.2f} | {ls} | {sp} |")
    md += ['', '- 判据：终点距目标 ≤ 0.25 m **且** action 返回 SUCCEEDED',
           '- SPL = L*/max(L*, L)，L* 为起点到目标**按可通行格**的最短路径',
           '- DTG 曲线在各目标 `dtg_curve` 字段（[时间s, 距离m]）']
    mf = out / f'phase1_nav_v9_{ts}.md'
    mf.write_text('\n'.join(md) + '\n', encoding='utf-8')

    print(f'\n[验收] SR {sr:.1%} ({succ}/{n})  SPL {spl_mean:.3f}  用时 {wall:.1f}s')
    print(f'[验收] 结果 -> {jf}')
    print(f'[验收]       -> {mf}')
    return 0 if succ == n else 1


if __name__ == '__main__':
    sys.exit(main())

