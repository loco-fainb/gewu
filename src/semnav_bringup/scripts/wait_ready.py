#!/usr/bin/env python3
"""等待仿真与导航栈**真正**就绪。

为什么需要这个脚本 —— 两个会骗人的检查：

    `ros2 topic list` 看到 /map 就以为 SLAM 好了
        -> SLAM 在启动瞬间就宣告 /map 话题，此时一个栅格都没有
    `ros2 action list` 看到 navigate_to_pose 就以为 Nav2 好了
        -> Nav2 在 configure 阶段就宣告 action server，节点还是 inactive，
           此时送目标会被直接拒绝（日志只有一句 "Goal was rejected!"，
           看不出根因）

真正的就绪判据（本脚本逐条验证）：

    1. TF 树完整：odom -> base_footprint 且 map -> base_footprint
       （map->odom 由 SLAM 提供，缺它则全局代价地图永远等不到变换）
    2. /scan 有真实数据（至少一条有限距离）—— 排除 gpu_lidar 静默返回 inf
    3. /map 有非空内容（已知栅格 > 0）
    4. Nav2 关键节点处于 active

退出码 0 = 全部就绪；非 0 = 超时，并打印还差哪一项。

用法：
    python3 wait_ready.py                 # 默认等 240s
    python3 wait_ready.py --timeout 400
    python3 wait_ready.py --no-nav2       # 只要仿真就绪（建图前的检查）
"""

import argparse
import math
import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# Nav2 里必须处于 active 才能接受目标的关键节点
NAV2_NODES = ('bt_navigator', 'controller_server', 'planner_server',
              'behavior_server')


class Readiness(Node):
    def __init__(self):
        super().__init__('semnav_readiness')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan = None
        self.map_msg = None
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 5)
        # 注意别叫 _clients：rclpy 的 Node 内部已占用该名字（是 list 不是 dict），
        # 覆盖它会让 create_client() 在 append 时崩溃。
        self._state_clients = {}

    def _on_scan(self, msg):
        self.scan = msg

    def _on_map(self, msg):
        self.map_msg = msg

    # ---------- 各项判据 ----------

    def tf_ok(self, parent, child):
        try:
            return self.tf_buffer.can_transform(parent, child, rclpy.time.Time())
        except Exception:
            return False

    def scan_ok(self):
        if self.scan is None:
            return False, '无 /scan 消息'
        good = [v for v in self.scan.ranges if math.isfinite(v) and v > 0.01]
        if not good:
            # 这正是 gpu_lidar 在 headless-rendering 下的静默失效形态
            return False, f'/scan 有 {len(self.scan.ranges)} 条但全为 inf/0'
        return True, f'{len(good)}/{len(self.scan.ranges)} 条有效'

    def map_ok(self):
        if self.map_msg is None:
            return False, '无 /map 消息'
        known = sum(1 for v in self.map_msg.data if v >= 0)
        if known == 0:
            return False, '/map 已宣告但内容为空（SLAM 还没建出任何栅格）'
        return True, f'{known} 个已知栅格'

    def nav2_state(self, name):
        """查询生命周期状态；服务不存在时返回 'absent'。"""
        cli = self._state_clients.get(name)
        if cli is None:
            cli = self.create_client(GetState, f'/{name}/get_state')
            self._state_clients[name] = cli
        if not cli.service_is_ready():
            if not cli.wait_for_service(timeout_sec=0.5):
                return 'absent'
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            return 'unknown'
        return fut.result().current_state.label


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=240.0)
    ap.add_argument('--no-nav2', action='store_true',
                    help='只等仿真（scan / odom TF），不等 SLAM 与 Nav2')
    args = ap.parse_args()

    rclpy.init()
    node = Readiness()
    t0 = time.time()
    last_report = 0.0
    pending = []

    try:
        while time.time() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.3)

            checks = [('TF odom->base_footprint',
                       *((True, '') if node.tf_ok('odom', 'base_footprint')
                         else (False, '缺失')))]
            checks.append(('scan', *node.scan_ok()))

            need_map = not args.no_nav2
            if need_map:
                checks.append(('TF map->base_footprint',
                               *((True, '')
                                 if node.tf_ok('map', 'base_footprint')
                                 else (False, '缺失（SLAM 未出图）'))))
                checks.append(('map', *node.map_ok()))

            nav2_states = {}
            if need_map:
                for name in NAV2_NODES:
                    st = node.nav2_state(name)
                    nav2_states[name] = st
                    checks.append((f'nav2 {name}', st == 'active',
                                   st if st != 'active' else ''))

            pending = [(n, d) for n, ok, d in checks if not ok]

            now = time.time()
            if pending and now - last_report > 15.0:
                last_report = now
                print(f'[wait_ready] {now - t0:5.0f}s 仍缺: '
                      + '; '.join(f'{n}({d})' for n, d in pending),
                      flush=True)

            if not pending:
                print(f'\n[wait_ready] 全部就绪（用时 {time.time() - t0:.0f}s）')
                for name, ok, detail in checks:
                    print(f'  ✓ {name:28s} {detail}')
                print('\n[wait_ready] 关键帧与节点：'
                      + ', '.join(f'{k}={v}' for k, v in nav2_states.items()))
                return 0

        print(f'\n[wait_ready] 超时 {args.timeout:.0f}s，仍缺：', file=sys.stderr)
        for name, detail in pending:
            print(f'  ✗ {name:28s} {detail}', file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
