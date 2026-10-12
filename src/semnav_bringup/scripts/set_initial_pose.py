#!/usr/bin/env python3
"""下发 AMCL 初始位姿 —— 让定位栈无人值守地自己转起来。

为什么必须有这个节点：
    AMCL 在收到初始位姿之前不会发布 map→odom；而没有 map 帧，
    Nav2 的 global_costmap 激活就会**直接失败**（不是重试），
    bt_navigator / planner_server 于是卡在 inactive。
    表现为「节点都在、话题都在，但导航不可用」——很难从现象猜到根因。

    自动下发（而不是人工 ros2 topic pub）的另一个好处：位姿从布局 YAML 的
    spawn 读出，是**单一真源**，改了出生点不用改两处。

用法（由 localization.launch.py 自动拉起）：
    ros2 run semnav_bringup set_initial_pose.py
    ros2 run semnav_bringup set_initial_pose.py --x -4.2 --y 0.0 --yaw 0.0
"""

import argparse
import math
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_path
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--layout', default=None)
    ap.add_argument('--x', type=float, default=None)
    ap.add_argument('--y', type=float, default=None)
    ap.add_argument('--yaw', type=float, default=None, help='弧度')
    ap.add_argument('--repeat', type=int, default=6,
                    help='重发次数：AMCL 可能还没订阅上，发一次会丢')
    ap.add_argument('--interval', type=float, default=1.0)
    ap.add_argument('--wait-sub', type=float, default=30.0,
                    help='最多等多少秒让 AMCL 订阅上来（0 = 不等）')
    args, _ = ap.parse_known_args()

    x, y, yaw = args.x, args.y, args.yaw
    if x is None or y is None:
        layout_path = (args.layout
                       or str(get_package_share_path('semnav_bringup')
                              / 'config' / 'apartment.yaml'))
        spawn = (yaml.safe_load(open(layout_path, encoding='utf-8'))
                 .get('spawn', {}))
        x = float(spawn.get('x', 0.0)) if x is None else x
        y = float(spawn.get('y', 0.0)) if y is None else y
        if yaw is None:
            yaw = float(spawn.get('yaw', 0.0))

    rclpy.init()
    # automatically_declare_parameters_from_overrides 让 launch 传进来的
    # use_sim_time:=true 真正生效，get_clock() 才会跟着 /clock 走。
    node = Node('set_initial_pose',
                automatically_declare_parameters_from_overrides=True)
    pub = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    # 时间戳刻意留 0（"无时间"约定）。TF2 把 0 解释为「用最新的可用变换」，
    # 这正是初始位姿需要的语义。
    #
    # ⚠️ 不要在这里填 node.get_clock().now()：AMCL 会用它把初始位姿从 map
    #    变换到 odom，而该时刻的 TF 往往还没准备好，于是报
    #    "Failed to transform initial pose in time (Lookup would require
    #     extrapolation into the future...)"。
    #    更糟的是若 use_sim_time 没生效，这里填的是**墙钟**（约 1.79e9），
    #    而 TF 时间轴是仿真时间（约 400），差着几个数量级 —— AMCL 永远
    #    等不到那个时刻。
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    # 协方差：告诉 AMCL 我们对初始位置有多大把握。给小的位置方差 +
    # 中等朝向方差，避免它把粒子铺得过散。
    msg.pose.covariance[0] = 0.25     # x
    msg.pose.covariance[7] = 0.25     # y
    msg.pose.covariance[35] = 0.0685  # yaw

    node.get_logger().info(f'下发初始位姿 ({x}, {y}, yaw={yaw:.3f} rad)')

    # ⚠️ 必须等 AMCL 订阅上来，不能"发完就走"。
    #
    # 踩过的坑：本脚本曾是 `for i in range(6): publish(); spin_once(1.0)`，
    # 自以为在「每 1 秒重发一次，共 5 秒」。实测这 6 次全在 **0.04 秒**内发完
    # （日志时间戳 .503 → .543），AMCL 一条都没收到，日志里 0 次
    # initialPoseReceived。原因：节点当时只有发布者、没有任何订阅/定时器，
    # rclpy.spin_once 没有可等待实体就立刻返回 —— 那个 timeout_sec=1.0 是
    # 上限而不是保证，于是循环空转到底。
    #
    # 后果很隐蔽：AMCL 不报错，只是反复打
    #   "AMCL cannot publish a pose or update the transform. Please set the
    #    initial pose..."
    # 而它永远不满足。没有 map→odom 就没有 map 帧，Nav2 的 global_costmap
    # 激活是**一次性**的、不重试，于是整条导航链再无出头之日。
    #
    # 正确做法：等到确实有订阅者（说明 DDS 发现完成），再发；发的时候也用
    # 真实时钟计时，别依赖 spin_once 的返回值。
    if args.wait_sub > 0:
        wait_start = time.time()
        while time.time() - wait_start < args.wait_sub:
            if pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
        waited = time.time() - wait_start
        n_sub = pub.get_subscription_count()
        if n_sub:
            node.get_logger().info(f'AMCL 已订阅，用时 {waited:.1f}s')
        else:
            node.get_logger().warn(
                f'等待 {waited:.1f}s 仍未见订阅者，仍会尝试发送')

    for i in range(args.repeat):
        # header.stamp 保持为 0（见上面注释）；这里只重发，不盖时间戳。
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        # 用 wall clock 计时：spin_once 可能立刻返回，靠它做间隔不可靠
        time.sleep(args.interval)

    node.get_logger().info(f'已发送 {args.repeat} 次，退出')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
