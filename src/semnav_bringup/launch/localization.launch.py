"""第一阶段 · 定位导航：加载已建地图 + AMCL，导航到指定坐标。

与 mapping.launch.py 的区别：
    建图阶段由 SLAM 在线产出 map→odom；
    这里是**已知地图**，由 map_server 提供地图、AMCL 提供 map→odom。
    两者不能同时跑 —— 会争抢 map→odom 并互相打架。

⚠️ 启动顺序是这个文件的核心，不能简化成一句 bringup_launch.py：
    Nav2 的代价地图激活是**一次性**的 —— global_costmap 拿不到 map 帧就
    直接判定激活失败（**不重试**），bt_navigator / planner_server 随之卡在
    inactive。而没有初始位姿，AMCL 根本不发 map→odom。
    所以必须严格分四段：
        sim ──> 定位(map_server+AMCL) ──> 下发初始位姿 ──> Nav2 导航节点
    若把定位与导航一起启动（bringup_launch.py 的默认行为），导航节点会在
    初始位姿到达之前就激活失败。

用法：
    ros2 launch semnav_bringup localization.launch.py
    xvfb-run -a -s "-screen 0 1280x1024x24" \\
        ros2 launch semnav_bringup localization.launch.py headless:=true
"""

import os

import yaml
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_defaults():
    """出生点默认值取自布局 YAML 的 spawn —— 与 sim.launch.py 同源。

    此前这里硬编码 (-5.4, 0.0)，是**旧户型**的坐标。换成 v9 户型后，
    该点在 333x278@0.05 的地图上落在格 (-78, 64)，即地图范围之外
    （地图 x∈[-1.50, 15.15]）—— 机器人会被生成到墙外，且 AMCL 收到
    一个与激光观测完全矛盾的初始位姿。实测同类故障曾让 loc_error
    飙到 6.46 m、6 个目标全部 FAILED。
    """
    try:
        path = get_package_share_path('semnav_bringup') / 'config' / 'apartment.yaml'
        spawn = yaml.safe_load(open(path, encoding='utf-8')).get('spawn', {}) or {}
    except Exception:
        spawn = {}
    return (str(spawn.get('x', 0.0)),
            str(spawn.get('y', 0.0)),
            str(spawn.get('yaw', 0.0)))


def generate_launch_description():
    _spawn_x, _spawn_y, _spawn_yaw = _spawn_defaults()
    share = get_package_share_path('semnav_bringup')
    nav2_share = get_package_share_path('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    autostart = LaunchConfiguration('autostart')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')
    map_file = LaunchConfiguration('map')
    params = str(share / 'config' / 'nav2_params.yaml')

    # ---- 第 1 段：仿真与机器人 ----
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'sim.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'headless': headless,
                          'x_pose': x_pose,
                          'y_pose': y_pose,
                          'yaw_pose': yaw_pose}.items(),
    )

    # ---- 第 2 段：定位（map_server 提供地图，AMCL 提供 map→odom）----
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / 'launch' / 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': map_file,
            'params_file': params,
        }.items(),
    )
    localization_delayed = TimerAction(period=40.0, actions=[localization])

    # ---- 第 3 段：下发初始位姿 ----
    # set_initial_pose.py 用 argparse 读位姿（--x/--y/--yaw），不是 ROS 参数，
    # 所以必须走 Node(arguments=...)，塞进 parameters 会被静默忽略。
    #
    # 曾经只传 parameters=[{'use_sim_time': ...}]，该脚本于是回退到读
    # apartment.yaml 的 spawn 默认值，与本 launch 的 x_pose/y_pose/yaw_pose
    # 完全脱钩。实测机器人 spawn 在 (1.10, 6.32)，AMCL 却被设成 (7.15, 5.50)，
    # 差 6 m，导致 map->odom 错误、global_costmap 永不收敛、Nav2 报
    # "Timed out waiting for transform from base_link to map"。
    initial_pose = TimerAction(
        period=70.0,
        actions=[Node(package='semnav_bringup',
                      executable='set_initial_pose.py',
                      name='set_initial_pose', output='screen',
                      arguments=['--x', x_pose,
                                 '--y', y_pose,
                                 '--yaw', yaw_pose],
                      parameters=[{'use_sim_time': use_sim_time}])],
    )

    # ---- 第 4 段：Nav2 导航节点（必须在 map 帧可用之后）----
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / 'launch' / 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params,
        }.items(),
    )
    navigation_delayed = TimerAction(period=95.0, actions=[navigation])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('headless', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('autostart', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('x_pose', default_value=_spawn_x,
                              description='出生点 x（默认取布局 YAML 的 spawn）'),
        DeclareLaunchArgument('y_pose', default_value=_spawn_y,
                              description='出生点 y（默认取布局 YAML 的 spawn）'),
        DeclareLaunchArgument('yaw_pose', default_value=_spawn_yaw,
                              description='出生点朝向 rad（默认取布局 YAML 的 spawn）'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/projects/semnav/maps/apartment_v9_furnished_map.yaml'),
            description='已保存的地图 YAML（由 map_saver 产出）'),
        sim,
        localization_delayed,
        initial_pose,
        navigation_delayed,
    ])
