#!/usr/bin/env python3
"""连续记录 DWB /evaluation，用于事后判定"停滞时控制器在纠结什么"。

与 probe5_eval.py 不同，这里不猜排序语义：同时记录 total 的最小/最大、合法/非法
计数，以及 best_index 指向的那条轨迹。DWB 用 total = -1.0 表示"非法轨迹"
（撞障碍/超速等），因此 legal 定义为 total >= 0。

输出 /tmp/eval_rec.jsonl，每行一个采样点（5 s 一次），字段自解释。
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from dwb_msgs.msg import LocalPlanEvaluation

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/eval_rec.jsonl'
PERIOD = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
MAX_LINES = int(os.environ.get('EVAL_REC_MAX', '400'))


class Rec(Node):
    def __init__(self):
        super().__init__('eval_rec')
        self.fh = open(OUT, 'w', buffering=1)
        self.n_lines = 0
        self.last = None
        self.create_subscription(LocalPlanEvaluation, '/evaluation', self.cb, 10)
        self.create_timer(PERIOD, self.dump)

    def cb(self, msg):
        self.last = msg

    def dump(self):
        if self.last is None or self.n_lines >= MAX_LINES:
            return
        m = self.last
        tw = list(m.twists)
        if not tw:
            return
        vals = [t.total for t in tw]
        legal = [t for t in tw if t.total >= 0.0]
        n_illegal = sum(1 for v in vals if v < 0.0)

        def vel(t):
            return [round(float(t.traj.velocity.x), 3),
                    round(float(t.traj.velocity.theta), 3)]

        def crit(t):
            return {s.name: round(float(s.raw_score), 2) for s in t.scores}

        best = tw[m.best_index] if 0 <= m.best_index < len(tw) else None
        worst = tw[m.worst_index] if 0 <= m.worst_index < len(tw) else None
        # 自己按 total 找合法轨迹的两个极端，不依赖消息里的 index 语义
        lo = min(legal, key=lambda t: t.total) if legal else None
        hi = max(legal, key=lambda t: t.total) if legal else None
        # 在所有合法轨迹里，前进速度最大的那条（看"前进"是否真的被判死刑）
        mov = [t for t in legal if float(t.traj.velocity.x) > 1e-6]
        fwd = max(mov, key=lambda t: t.traj.velocity.x) if mov else None

        rec = {
            't': round(time.time(), 1),
            'n': len(tw),
            'n_illegal': n_illegal,
            'n_legal_moving': len(mov),
            'total_range': [round(min(vals), 3), round(max(vals), 3)],
            'best_index': int(m.best_index),
            'best_index_vel': vel(best) if best is not None else None,
            'best_index_total': round(float(best.total), 3) if best is not None else None,
            'best_index_crit': crit(best) if best is not None else None,
            'worst_index_vel': vel(worst) if worst is not None else None,
            'worst_index_total': round(float(worst.total), 3) if worst is not None else None,
            'legal_min_total': round(float(lo.total), 3) if lo is not None else None,
            'legal_min_vel': vel(lo) if lo is not None else None,
            'legal_max_total': round(float(hi.total), 3) if hi is not None else None,
            'legal_max_vel': vel(hi) if hi is not None else None,
            'legal_max_crit': crit(hi) if hi is not None else None,
            'fwd_max_vx': round(float(fwd.traj.velocity.x), 3) if fwd is not None else None,
            'fwd_total': round(float(fwd.total), 3) if fwd is not None else None,
        }
        self.fh.write(json.dumps(rec, ensure_ascii=False) + '\n')
        self.n_lines += 1

    def destroy_node(self):
        try:
            self.fh.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Rec()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    print(f'[eval_rec] 已写 {node.n_lines} 行 -> {OUT}')


if __name__ == '__main__':
    main()
