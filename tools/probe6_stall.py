#!/usr/bin/env python3
"""停滞瞬间把 DWB 的全部候选轨迹摊成表格，判定"为什么选零速度"。

⚠️ 实测语义（别搞反）：
    total 是**代价**，越小越好；DWB 用 total = -1.0 表示非法轨迹（撞障碍/超速）。
    消息里的 best_index 实测 = argmin（best_index_total 恒等于合法轨迹的最小 total）。
    代价构成 = Σ (raw_score/100 × scale)，已用数据验证。

本脚本不做任何 initialpose 注入 —— localization.launch.py 里的 set_initial_pose.py
已经下发过初始位姿。⚠️ 往已定位的机器人再灌一次初始位姿会让 AMCL 发散（踩过）。

流程：等定位收敛 → goToPose(次卧) → goToPose(厨房) → 检测到停滞 → 全量转储 →
      cancel → 退出（**不** shutdown lifecycle，栈可继续用）。
"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from dwb_msgs.msg import LocalPlanEvaluation

GOALS = {'次卧': (1.2, 3.0), '厨房': (2.3, 1.6)}
OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/probe6.json'
STALL_WIN = 12.0      # 观察窗口（秒）
STALL_EPS = 0.05      # 窗口内位移小于此值即判停滞（米）
BUDGET = 240.0        # 单个目标最长等待


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_stamped(x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = rclpy.time.Time().to_msg()   # 0：避免 TF 外推
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class Probe:
    def __init__(self, nav):
        self.nav = nav
        self.amcl = None
        self.odom = None
        self.plan = None
        self.ev = None
        self.n_amcl = 0
        self.n_ev = 0
        nav.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        nav.create_subscription(Odometry, '/odom', self.on_odom, 10)
        nav.create_subscription(Path, '/plan', self.on_plan, 10)
        nav.create_subscription(LocalPlanEvaluation, '/evaluation', self.on_ev, 10)

    def on_amcl(self, m):
        p = m.pose.pose
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.n_amcl += 1

    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_plan(self, m):
        self.plan = [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]

    def on_ev(self, m):
        self.ev = m
        self.n_ev += 1

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self.nav, timeout_sec=0.1)

    def wait_data(self, timeout, labels):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self.nav, timeout_sec=0.2)
            if all(getattr(self, a) is not None for a in labels):
                return True
        return False


def dump(probe, tag):
    m = probe.ev
    tw = list(m.twists)
    def v_of(t):
        return (round(float(t.traj.velocity.x), 3), round(float(t.traj.velocity.theta), 3))
    def crit_of(t):
        return {s.name: round(float(s.raw_score), 2) for s in t.scores}
    def last_of(t):
        ps = list(t.traj.poses)
        return (round(ps[-1].x, 3), round(ps[-1].y, 3)) if ps else None

    rows = [{'v': v_of(t), 'total': round(float(t.total), 3), 'crit': crit_of(t),
             'last': last_of(t)} for t in tw]
    legal = [r for r in rows if r['total'] >= 0]
    if not legal:
        print(f'  [{tag}] 无合法轨迹！')
        return None

    goal = GOALS['厨房']
    print(f'\n===== [{tag}] /evaluation 全量转储 =====')
    if probe.amcl:
        print(f'  机器人 AMCL ({probe.amcl[0]:.3f},{probe.amcl[1]:.3f}) '
              f'yaw {math.degrees(probe.amcl[2]):.1f}°  到厨房直线 '
              f'{math.dist(probe.amcl[:2], goal):.3f} m')
    print(f'  轨迹 {len(rows)} 条，合法 {len(legal)} 条，非法 {len(rows)-len(legal)} 条')
    if probe.plan:
        print(f'  /plan {len(probe.plan)} 点，前 3: '
              + ' '.join(f'({x:.2f},{y:.2f})' for x, y in probe.plan[:3]))

    argmin = min(range(len(rows)), key=lambda i: rows[i]['total'])
    print(f'  消息 best_index={m.best_index}（自算 argmin={argmin}）'
          f'  worst_index={m.worst_index}')

    vxs = sorted({r['v'][0] for r in rows})
    print('\n  每个 vx 档位（对该 vx 取 vth 最优）：')
    print('      vx  合法/总   最优total  该vth  末点            直线距目标  GoalDist.raw')
    bin_best = {}
    for vx in vxs:
        sub = [r for r in rows if r['v'][0] == vx]
        ok = [r for r in sub if r['total'] >= 0]
        if not ok:
            print(f'  {vx:>6.2f}   0/{len(sub):<3d}     全部非法')
            continue
        b = min(ok, key=lambda r: r['total'])
        bin_best[vx] = b
        sd = math.dist(b['last'], goal) if b['last'] else float('nan')
        print(f'  {vx:>6.2f}  {len(ok):>3d}/{len(sub):<3d}  {b["total"]:>8.2f}  '
              f'{b["v"][1]:>6.2f}  {str(b["last"]):<16s} {sd:>7.3f}  '
              f'{b["crit"].get("GoalDist", float("nan"))}')

    zero = [r for r in legal if abs(r['v'][0]) < 1e-9]
    mov = [r for r in legal if r['v'][0] > 1e-9]
    zmin = min(zero, key=lambda r: r['total']) if zero else None
    mmin = min(mov, key=lambda r: r['total']) if mov else None
    print('\n  ★ 判定：')
    if zmin:
        print(f'    vx=0 最优 : total={zmin["total"]:.2f}  vth={zmin["v"][1]:.3f}  '
              f'末点={zmin["last"]}  {zmin["crit"]}')
    if mmin:
        print(f'    vx>0 最优 : total={mmin["total"]:.2f}  vx={mmin["v"][0]:.3f} '
              f'vth={mmin["v"][1]:.3f}  末点={mmin["last"]}  {mmin["crit"]}')
    if zmin and mmin:
        gap = mmin['total'] - zmin['total']
        verdict = ('**并列**（说明 DWB 靠"先到先得"选了 vx=0，属排序/剪枝问题）'
                   if abs(gap) < 1e-6 else
                   f'**前进严格更贵 {gap:.2f}**（说明评分函数本身偏好静止）')
        print(f'    → vx>0 最优 − vx=0 最优 = {gap:+.3f}   {verdict}')
        d = {k: round(mmin['crit'].get(k, 0.0) - zmin['crit'].get(k, 0.0), 2)
             for k in sorted(set(mmin['crit']) | set(zmin['crit']))}
        print(f'    → 两者 critic raw 差值(前进−静止): {d}')
        print('       （若差值同号，则任何正 scale 都无法翻盘；异号才可能靠调 scale 解决）')

    # GoalDist 是否与"直线距目标"单调 —— 判断它是不是墙无关度量
    pairs = [(math.dist(b['last'], goal), b['crit'].get('GoalDist'))
             for b in bin_best.values() if b['last'] and b['crit'].get('GoalDist') is not None]
    pairs.sort()
    print('\n  GoalDist.raw  vs  轨迹末点直线距目标（按距离升序）：')
    for sd, raw in pairs:
        print(f'    距目标 {sd:6.3f} m  ->  GoalDist.raw {raw:7.1f}')
    if len(pairs) >= 3:
        inc = all(pairs[i][1] <= pairs[i + 1][1] + 1e-9 for i in range(len(pairs) - 1))
        print(f'    单调不减？{inc}  → {"GoalDist 与直线距离同向，是墙无关度量" if inc else "不成单调，GoalDist 含更多结构"}')

    return {'amcl': probe.amcl, 'plan': probe.plan, 'rows': rows,
            'zero_best': zmin, 'mov_best': mmin, 'argmin': argmin,
            'msg_best_index': int(m.best_index)}


def drive(probe, name, budget=BUDGET, on_stall=None):
    """开到 name；一旦判定停滞就立刻回调 on_stall（因为 cancel 后 /evaluation 会停发）。"""
    x, y = GOALS[name]
    print(f'\n[probe6] -> {name} ({x},{y})')
    t0 = time.time()
    probe.nav.goToPose(pose_stamped(x, y))
    hist = []
    stalled_at = None
    fired = False
    while time.time() - t0 < budget:
        rclpy.spin_once(probe.nav, timeout_sec=0.1)
        if probe.odom:
            now = time.time()
            hist.append((now, probe.odom[0], probe.odom[1]))
            hist = [h for h in hist if now - h[0] <= STALL_WIN]
        if probe.nav.isTaskComplete():
            r = probe.nav.getResult()
            print(f'   结果 {r}  用时 {time.time()-t0:.1f}s')
            return r, time.time() - t0, stalled_at
        if len(hist) > 5 and hist[-1][0] - hist[0][0] > STALL_WIN - 1.0:
            span = math.dist((hist[0][1], hist[0][2]), (hist[-1][1], hist[-1][2]))
            if span < STALL_EPS and not fired:
                fired = True
                stalled_at = time.time() - t0
                print(f'   ★ 停滞：{STALL_WIN:.0f}s 内净位移 {span:.3f} m'
                      f'（t={stalled_at:.1f}s）立刻抓 /evaluation')
                if on_stall is not None:
                    on_stall(stalled_at)
    print(f'   超时 {budget}s，取消')
    probe.nav.cancelTask()
    t1 = time.time()
    while time.time() - t1 < 20.0:
        rclpy.spin_once(probe.nav, timeout_sec=0.1)
        fut = probe.nav.result_future
        if fut is None or fut.done():
            break
    return TaskResult.CANCELED, time.time() - t0, stalled_at


def main():
    rclpy.init()
    nav = BasicNavigator()
    probe = Probe(nav)

    # ⚠️ 只等 amcl/odom：/plan 与 /evaluation 都要等第一个目标下发后才会出现，
    #    在这里一起等会白等一整个 timeout。
    print('[probe6] 等 /amcl_pose /odom （定位就绪）...')
    if not probe.wait_data(240.0, ['amcl', 'odom']):
        print('[probe6] ★ 定位数据未到，仍继续（大概率会失败）')
    print(f'[probe6] /amcl_pose {probe.n_amcl} 条，/evaluation {probe.n_ev} 条')
    if probe.amcl and probe.odom:
        print(f'[probe6] AMCL ({probe.amcl[0]:.3f},{probe.amcl[1]:.3f}) vs '
              f'ODOM ({probe.odom[0]:.3f},{probe.odom[1]:.3f})  '
              f'差 {math.dist(probe.amcl[:2], probe.odom[:2]):.3f} m')

    dumps = []

    def grab(t):
        if probe.ev is None:
            print('   ★ /evaluation 还没来，抓不到')
            return
        d = dump(probe, f'厨房 leg 停滞瞬间 t={t:.1f}s')
        if d:
            dumps.append(d)

    # 控制腿：先到次卧，证明控制器本身健康
    r1, t1, s1 = drive(probe, '次卧')
    # 关键腿：厨房 —— 停滞即抓，不等超时
    r2, t2, s2 = drive(probe, '厨房', on_stall=grab)

    rec = {'次卧': {'result': str(r1), 'elapsed_s': round(t1, 1), 'stall_at': s1},
           '厨房': {'result': str(r2), 'elapsed_s': round(t2, 1), 'stall_at': s2},
           'dumps': dumps, 'n_amcl': probe.n_amcl, 'n_ev': probe.n_ev}
    json.dump(rec, open(OUT, 'w'), ensure_ascii=False, default=str)
    print(f'\n[probe6] -> {OUT}')
    print('[probe6] 不 shutdown lifecycle，栈保持可用')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
