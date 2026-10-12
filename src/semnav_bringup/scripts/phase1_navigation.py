#!/usr/bin/env python3
"""第一阶段验收 · 导航跑：加载已建地图，AMCL 定位，依次到达各目标坐标。

这对应计划书第一阶段的验收条件「仿真机器人可建图并到达指定坐标」的后半句。

除到达与否，还记录**定位精度**：到达每个目标点时，把 AMCL 的位姿估计
（/amcl_pose）与仿真真值位姿（/odom，本配置下 odom_source=ground_truth）
作差。这才是有意义的结果 —— 「到了」但定位偏 0.5m，和「到了且定位准」
是两回事，只有后者才能支撑后续的语义融合与主动探索。

产物：experiments/phase1_navigation_<时间戳>.json / .md

用法：
    python3 phase1_navigation.py --outdir experiments
    python3 phase1_navigation.py --goals 主卧,厨房        # 只跑子集
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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

GOAL_TIMEOUT_S = 240.0
SETTLE_S = 3.0          # 到达后再等几秒，让 AMCL 收敛后再采样
# 到位容差：验收是"到达指定坐标"，终点与目标的直线距离超过这个值就不算到达。
# 比 nav2 的 xy_goal_tolerance(0.15 m) 略宽，留出 SLAM 漂移与停车余量。
ARRIVAL_TOL_M = 0.25


def normalize_goals(raw):
    """把 nav_goals 统一成 [{'name','x','y','yaw'}]。"""
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


def make_pose(stamp, x, y, yaw, frame='map'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _fmt(v):
    """可选浮点的表格渲染：None 显示 n/a，不抛异常。"""
    return 'n/a' if v is None else f'{v:.3f}'


class Tracker:
    """采样本机最近一次的 AMCL 估计位姿与真值位姿。"""

    def __init__(self, node):
        self.amcl = None
        self.truth = None
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_amcl, 10)
        node.create_subscription(Odometry, '/odom', self._on_odom, 20)

    def _on_amcl(self, msg):
        self.amcl = msg.pose.pose

    def _on_odom(self, msg):
        self.truth = msg.pose.pose

    def error(self):
        """返回 AMCL 估计与真值的 (位置误差 m, 朝向误差 deg)，缺数据返回 None。"""
        if self.amcl is None or self.truth is None:
            return None
        dx = self.amcl.position.x - self.truth.position.x
        dy = self.amcl.position.y - self.truth.position.y
        dpos = math.hypot(dx, dy)
        dyaw = (yaw_of(self.amcl.orientation) - yaw_of(self.truth.orientation)
                + math.pi) % (2 * math.pi) - math.pi
        return round(dpos, 4), round(math.degrees(abs(dyaw)), 3)


def drive_to(nav, pose_stamp, goal, timeout_s):
    """送一个目标并等它真正结束，返回 (result, elapsed_s, timed_out)。

    ⚠️ 不能自己写「while not isTaskComplete() + 超时 cancelTask + getResult」，
    那套写法会**伪造成功**，原因见 phase1_mapping.py 里同名函数的注释：
      * isTaskComplete() 在 result_future 为 None 时直接返回 True（取消后
        就是这种情况）；
      * getResult() 读的 self.status 是**上一次动作的残留值**。
    两者叠加 → 超时取消后读到上一个目标的 SUCCEEDED。

    实测踩过：厨房目标 237 s abort，但同一次运行里前一个目标的状态被读回来，
    报告成"成功"。这种假成功比失败更危险 —— 它会把一个真实存在的规划缺陷
    掩盖掉，让整张验收表看起来是 6/6。

    ⚠️ 还要检查 goToPose() 的**返回值**。它下面第一件事就是
    `while not wait_for_server(...): pass` —— 只等**服务被发现**，不等
    `bt_navigator` **激活完成**。动作服务器在 `inactive` 状态就已经宣告存在，
    所以这个等待会被骗过；紧接着 `send_goal_async` 被 `bt_navigator` 以
    "Action server is inactive. Rejecting the goal." 拒掉，
    `goToPose` 因此 `return False`，而它**不会**设置 `self.result_future`。

    此时 `isTaskComplete()` 第一行 `if not self.result_future: return True`
    立刻返回 True，`getResult()` 读到的又是残留状态 —— 于是这个目标
    被记成 `TaskResult.UNKNOWN`、用时 **0.0 s**。实测踩过（整机的第一个目标
    最容易中：验收脚本在 bt_navigator 激活完成的几秒前就开火了，3 s 后重发就正常）。
    """
    if not nav.goToPose(make_pose(pose_stamp, goal['x'], goal['y'], goal['yaw'])):
        # 被拒 = 根本没跑起来，不能算数。交给外层重试。
        return TaskResult.UNKNOWN, 0.0, False, False
    t0 = time.time()
    while not nav.isTaskComplete():
        rclpy.spin_once(nav, timeout_sec=0.1)
        if time.time() - t0 > timeout_s:
            _abort(nav, t0)
            return TaskResult.CANCELED, round(time.time() - t0, 2), True, True
        time.sleep(0.4)
    return nav.getResult(), round(time.time() - t0, 2), False, True


def _abort(nav, t0):
    """超时后的收尾：取消当前目标，并等它真正结算再返回。

    必须等结算 —— 否则下一个 goToPose 会和"正在取消"的状态撞上被直接拒绝，
    表现为下一个目标 0.0 s 就 FAILED（实测踩过，看着像导航失败，其实是
    驱动脚本的竞态）。
    """
    print(f'   超时，取消（已等 {time.time() - t0:.0f}s）')
    nav.cancelTask()
    t1 = time.time()
    while time.time() - t1 < 30.0:
        rclpy.spin_once(nav, timeout_sec=0.1)
        fut = nav.result_future
        if fut is None or fut.done():
            break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--layout', default=None)
    ap.add_argument('--outdir', default='experiments')
    ap.add_argument('--goal-timeout', type=float, default=GOAL_TIMEOUT_S)
    ap.add_argument('--goals', default=None,
                    help='逗号分隔的目标名子集（按 label / name / id 匹配）')
    args = ap.parse_args()

    layout_path = Path(args.layout) if args.layout else (
        get_package_share_path('semnav_bringup') / 'config' / 'apartment.yaml')
    layout = yaml.safe_load(layout_path.read_text(encoding='utf-8'))
    goals = normalize_goals(layout.get('nav_goals'))

    if args.goals:
        want = {s.strip() for s in args.goals.split(',') if s.strip()}
        goals = [g for g in goals if g['name'] in want]
        if not goals:
            raise SystemExit(f'--goals 未匹配到任何目标: {args.goals}')

    spawn = layout.get('spawn', {})
    sx = float(spawn.get('x', 0.0))
    sy = float(spawn.get('y', 0.0))
    syaw = float(spawn.get('yaw', 0.0))

    print(f'[导航] 布局 : {layout_path}')
    print(f'[导航] 初始位姿（取自布局 spawn）: ({sx}, {sy}, yaw={syaw})')
    print(f'[导航] 目标 {len(goals)} 个: '
          + ', '.join(f"{g['name']}({g['x']:.1f},{g['y']:.1f})" for g in goals))

    rclpy.init()
    nav = BasicNavigator()
    tracker = Tracker(nav)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, nav)

    # 1) 必须先把初始位姿告诉 AMCL —— 不给它，AMCL 会以 (0,0,0) 为起点，
    #    机器人实际在 (-4.2, 0)，于是 map→odom 一开始就是错的。
    #
    # 注意：AMCL 的初始位姿走**话题** /initialpose，不是服务。
    # nav2_simple_commander 这个版本没有 initial_pose_client，
    # setInitialPose() 内部就是 publish（+ 自带等待回执）。
    #
    # ⚠️ 不能只调一次 setInitialPose 就完事。它内部只 publish 一次，
    #    而此刻 AMCL 的订阅可能还没建立（DDS 发现尚未完成），那条消息直接
    #    丢掉、且无任何报错。实测踩过：日志里 AMCL 一次
    #    `initialPoseReceived` 都没有，只有无限循环的
    #    "AMCL cannot publish a pose or update the transform"，
    #    最后以 "AMCL 迟迟没出 map→odom" 告终。
    #
    #    所以这里自己重发：等到确实有订阅者，再连发几次。
    init_pub = nav.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    wait_start = time.time()
    while (init_pub.get_subscription_count() == 0
           and time.time() - wait_start < 60.0):
        rclpy.spin_once(nav, timeout_sec=0.1)
        time.sleep(0.1)
    print(f'[导航] AMCL 订阅就绪（{init_pub.get_subscription_count()} 个订阅者，'
          f'用时 {time.time() - wait_start:.1f}s）')

    # 初始位姿用 PoseWithCovarianceStamped —— 不能用上面的 make_pose()：
    # 那个返回 PoseStamped，其 .pose 是 Pose（只有 position/orientation），
    # 没有 covariance 字段，赋值会 AttributeError。
    init = PoseWithCovarianceStamped()
    init.header.frame_id = 'map'
    init.pose.pose.position.x = float(sx)
    init.pose.pose.position.y = float(sy)
    init.pose.pose.orientation.z = math.sin(syaw / 2.0)
    init.pose.pose.orientation.w = math.cos(syaw / 2.0)
    # 协方差：给小的位置方差 + 中等朝向方差，别让粒子铺得过散
    init.pose.covariance[0] = 0.25
    init.pose.covariance[7] = 0.25
    init.pose.covariance[35] = 0.0685

    def send_init():
        # 时间戳留 0 = "用最新可用变换"。填具体时刻反而会让 AMCL 报
        # "Failed to transform initial pose in time (extrapolation into the
        # future)"，因为那一刻的 map→odom 还不存在。
        init.header.stamp = Time().to_msg()
        init_pub.publish(init)

    for _ in range(8):
        send_init()
        rclpy.spin_once(nav, timeout_sec=0.05)
        time.sleep(0.5)
    print(f'[导航] 已下发初始位姿 {sx, sy, syaw}')

    # 2) 等 AMCL 真正发出 map→base_footprint 变换（这才是"定位可用"的判据；
    #    只看 action server 存在会被"已宣告但未激活"骗过）。
    #
    #    超时给足：AMCL 要在收到的初始位姿上撒粒子再做激光更新，软件渲染下
    #    仿真时间慢于墙钟，太短会误判成"AMCL 坏了"。
    print('[导航] 等待 TF map->base_footprint（AMCL 收敛）...')
    waited = 0.0
    while waited < 240.0:
        rclpy.spin_once(nav, timeout_sec=0.3)
        if tf_buffer.can_transform('map', 'base_footprint', Time()):
            print(f'  [OK] 用时 {waited:.0f}s')
            break
        # 还没收敛就再补发一次初始位姿：AMCL 收到初始位姿后若激光更新失败，
        # 会退回"没有初始位姿"的状态继续等，补发能把这种情况救回来。
        if int(waited) % 30 == 29:
            send_init()
        waited += 0.3
    else:
        raise RuntimeError('AMCL 迟迟没出 map→odom，请查 amcl 日志')

    print('[导航] 等待 /navigate_to_pose action server ...')
    while not nav.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
        pass
    # ⚠️ 服务"被发现"≠"已激活"。bt_navigator 在 inactive 时就已经宣告动作服务器，
    #    此时送目标会被 "Action server is inactive. Rejecting the goal." 拒掉。
    #    所以再等它生命周期真正 active（并且额外静默 3 s 让 Bond 建稳）。
    print('[导航] 等待 bt_navigator 激活 ...')
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
    else:
        print('  ⚠️ bt_navigator 未确认 active，仍继续（首个目标可能被拒，会自动重试）')
    time.sleep(3.0)
    print('[导航] 导航栈就绪，开始导航\n')

    records = []
    t_start = time.time()
    for g in goals:
        rec = {'name': g['name'], 'x': g['x'], 'y': g['y'], 'yaw': g['yaw']}
        print(f"[导航] -> {g['name']} ({g['x']:.2f}, {g['y']:.2f})")
        # 被拒（bt_navigator 还没激活完）不算失败，重试一次即可 ——
        # 不重试的话这个目标会被记成 UNKNOWN/0.0s，看着像导航坏了，
        # 其实是"起跑枪响得太早"。
        for attempt in range(2):
            result, elapsed, timed_out, sent = drive_to(
                nav, nav.get_clock().now().to_msg(), g, args.goal_timeout)
            if sent:
                break
            print(f'   目标被拒（导航栈尚未激活），5s 后重试 ...')
            time.sleep(5.0)
        rec['result'] = str(result)
        rec['accepted'] = bool(sent)
        rec['timed_out'] = timed_out
        # ⚠️ success 由**位置**判定，不看 action 返回值。
        # 验收条件是「到达指定坐标」，那就该按坐标验。这条独立校验存在的
        # 意义是：action 结果本身可能被 libnav2 的陈旧状态污染而谎报成功
        # （见 drive_to 的注释），而"机器人到底在不在目标点附近"是物证，
        # 无法被驱动脚本的状态机伪造。
        rec['cancelled'] = timed_out
        rec['elapsed_s'] = elapsed

        # 到达后等 AMCL 收敛再采样定位误差
        t2 = time.time()
        while time.time() - t2 < SETTLE_S:
            rclpy.spin_once(nav, timeout_sec=0.1)
            time.sleep(0.1)
        err = tracker.error()
        if err:
            rec['loc_error_m'], rec['loc_error_deg'] = err
        # ---- 独立位置校验：真值位姿与目标坐标的直线距离 ----
        if tracker.truth is not None:
            dx = tracker.truth.position.x - g['x']
            dy = tracker.truth.position.y - g['y']
            rec['final_dist_m'] = round(math.hypot(dx, dy), 4)
        else:
            rec['final_dist_m'] = None

        reached = (rec['final_dist_m'] is not None
                   and rec['final_dist_m'] <= ARRIVAL_TOL_M)
        rec['success'] = bool(reached and result == TaskResult.SUCCEEDED)
        rec['reached_by_position'] = bool(reached)

        tag = '✓' if rec['success'] else '✗'
        detail = (f"{rec['final_dist_m']:.3f} m" if rec['final_dist_m']
                  is not None else 'n/a')
        if err:
            print(f"   {tag} {rec['result']}  用时 {rec['elapsed_s']:.1f}s"
                  f"  终点距目标 {detail}"
                  f"   定位误差 {err[0]:.3f} m / {err[1]:.1f}°")
        else:
            print(f"   {tag} {rec['result']}  用时 {rec['elapsed_s']:.1f}s"
                  f"  终点距目标 {detail}"
                  f"   （无 /amcl_pose 或 /odom，未测定位误差）")
        if result == TaskResult.SUCCEEDED and not reached:
            print(f'   ⚠️ action 报 SUCCEEDED 但终点距目标 {detail}，'
                  f'超出到位容差 {ARRIVAL_TOL_M} m —— 判为未到达')
        records.append(rec)

    total_s = round(time.time() - t_start, 2)
    ok = sum(1 for r in records if r['success'])
    errs = [r['loc_error_m'] for r in records if 'loc_error_m' in r]

    summary = {
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'layout': str(layout_path),
        'goals_total': len(records),
        'goals_succeeded': ok,
        'success_rate': round(ok / max(1, len(records)), 4),
        'wall_clock_s': total_s,
        'loc_error_m_mean': round(sum(errs) / len(errs), 4) if errs else None,
        'loc_error_m_max': round(max(errs), 4) if errs else None,
        'goals': records,
    }

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    (outdir / f'phase1_navigation_{stamp}.json').write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')

    md = [f'# 第一阶段导航跑 · {summary["timestamp"]}', '',
          f'- 加载地图：`{summary["layout"]}` 所在工作区已保存地图',
          f'- 目标点成功：**{ok}/{len(records)}**'
          f'（成功率 {summary["success_rate"]:.1%}）',
          f'- 全程耗时：{total_s:.1f} s', '']
    if errs:
        md += [f'- AMCL 定位误差：均值 {summary["loc_error_m_mean"]:.3f} m，'
               f'最大 {summary["loc_error_m_max"]:.3f} m', '']
    md += ['| 目标点 | 坐标 | 结果 | 耗时(s) | 终点距目标(m) | 定位误差(m) |',
           '|---|---|---|---|---|---|']
    md += [f'| {r["name"]} | ({r["x"]:.2f}, {r["y"]:.2f}) | {r["result"]} '
           f'| {r["elapsed_s"]:.1f} '
           f'| {_fmt(r.get("final_dist_m"))} '
           f'| {r.get("loc_error_m", float("nan")):.3f} |' for r in records]
    md += ['', f'- 到位判据：终点距目标 ≤ {ARRIVAL_TOL_M} m 且 action 返回'
               f' SUCCEEDED（两者都满足才算到达；只看 action 返回值会被陈旧'
               f'状态骗过，见脚本内注释）']
    (outdir / f'phase1_navigation_{stamp}.md').write_text(
        '\n'.join(md) + '\n', encoding='utf-8')

    print('\n===== 导航跑结果 =====')
    print(f'目标点成功 {ok}/{len(records)}  ({summary["success_rate"]:.1%})  '
          f'耗时 {total_s:.1f}s')
    if errs:
        print(f'AMCL 定位误差 均值 {summary["loc_error_m_mean"]:.3f} m  '
              f'最大 {summary["loc_error_m_max"]:.3f} m')
    print(f'产物: {outdir}/phase1_navigation_{stamp}.json / .md')

    nav.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
