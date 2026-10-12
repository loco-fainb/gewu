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

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_path('semnav_bringup')
    nav2_share = get_package_share_path('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    autostart = LaunchConfiguration('autostart')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    map_file = LaunchConfiguration('map')
    params = str(share / 'config' / 'nav2_params.yaml')

    # ---- 第 1 段：仿真与机器人 ----
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'sim.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'headless': headless,
                          'x_pose': x_pose,
                          'y_pose': y_pose}.items(),
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

    # ---- 第 3 段：下发初始位姿（位姿取自布局 YAML 的 spawn，单一真源）----
    initial_pose = TimerAction(
        period=70.0,
        actions=[Node(package='semnav_bringup',
                      executable='set_initial_pose.py',
                      name='set_initial_pose', output='screen',
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
        DeclareLaunchArgument('x_pose', default_value='-4.2'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/projects/semnav/maps/apartment.yaml'),
            description='已保存的地图 YAML（由 map_saver 产出）'),
        sim,
        localization_delayed,
        initial_pose,
        navigation_delayed,
    ])
