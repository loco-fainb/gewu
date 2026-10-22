"""第一阶段 · 建图：在未知住宅里一边建图一边导航。

为什么建图阶段也要起 Nav2：
    住宅里有家具和 90cm 门洞，用裸 cmd_vel 开车必然撞家具，而撞完的激光数据
    会反过来污染地图（障碍物出现在错误位置）。让 Nav2 负责避障、SLAM 负责出图，
    各司其职。

数据流：
    /scan ──> slam_toolbox ──> map→odom TF ──> Nav2 代价地图 ──> /cmd_vel

用法：
    ros2 launch semnav_bringup mapping.launch.py
    xvfb-run -a -s "-screen 0 1280x1024x24" \
        ros2 launch semnav_bringup mapping.launch.py headless:=true
"""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, RegisterEventHandler,
                            TimerAction)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def mapping_params_file(share):
    """产出建图期专用的 Nav2 参数文件：把 planner 的 allow_unknown 改成 true。

    为什么建图期必须允许穿未知区（而导航期必须禁止）
    ------------------------------------------------
    建图的目标点常常还在**没被观测过**的房间里，此时那一片栅格全是"未知"。
    若未知当致命，规划器连一步都规划不出去 —— 越没去过的地方越去不了，
    建图会永远停在第 1 个房间。

    导航期正相反：地图已经建好，未知区就是"没看到过的地方"，把它当可通行
    会诱使规划器穿墙抄近路（代价函数只算路径长度），控制器推不过去。

    所以同一份 nav2_params.yaml 在这两个阶段需要相反的取值。与其维护两份
    几乎相同的 451 行配置（必然漂移），不如在这里**程序化派生**：读共享的
    参数文件，只覆写 allow_unknown 这一个键。基准文件里剩下的所有调参都在
    两个阶段自动保持一致。
    """
    src = share / 'config' / 'nav2_params.yaml'
    params = yaml.safe_load(src.read_text(encoding='utf-8'))
    params['planner_server']['ros__parameters']['GridBased'][
        'allow_unknown'] = True
    tmp = os.path.join(tempfile.gettempdir(), 'semnav_mapping_params.yaml')
    with open(tmp, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(params, fh, allow_unicode=True, sort_keys=False)
    return tmp


def _spawn_defaults():
    """从 config/apartment.yaml 读出生位姿，作为 launch 参数默认值。

    硬编码坐标的老问题：配置改了 launch 没改，机器人被放到地图外。
    （localization.launch.py 曾用 -5.4/0.0，在 v9 地图 x 范围 [-1.50,15.15]
    之外，AMCL 直接判越界。）
    """
    cfg = (get_package_share_path('semnav_bringup')
           / 'config' / 'apartment.yaml')
    try:
        spawn = yaml.safe_load(cfg.read_text(encoding='utf-8'))['spawn']
    except Exception:
        spawn = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    return (str(spawn.get('x', 0.0)), str(spawn.get('y', 0.0)),
            str(spawn.get('yaw', 0.0)))


def generate_launch_description():
    share = get_package_share_path('semnav_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    autostart = LaunchConfiguration('autostart')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')
    _dx, _dy, _dyaw = _spawn_defaults()

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'sim.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'headless': headless,
                          'x_pose': x_pose,
                          'y_pose': y_pose,
                          'yaw_pose': yaw_pose}.items(),
    )

    # ⚠️ 必须用 LifecycleNode，不能只写 Node。
    #
    # async_slam_toolbox_node 是**生命周期节点**：进程起来后停在 unconfigured，
    # 必须显式发 TRANSITION_CONFIGURE 再 TRANSITION_ACTIVATE 才会干活。
    # 只把它拉起来（普通 Node）的后果是"静默不工作"——进程活着、话题列表里
    # 连 /map 都能看到，但 Publisher count 是 0，一个栅格都不出；而且它所有参数
    # （odom_frame / map_frame / base_frame / scan_topic / mode / …）都是在
    # on_configure 里才声明的，没 configure 就会全部报 "Parameter not set"。
    # 这个坑实际踩过：Nav2 全局代价地图一直等不到 map 帧，6 个目标点全被拒。
    #
    # 下面这段 configure → activate 的事件链照抄官方 slam_toolbox 的
    # launch/online_async_launch.py，那是经过验证的写法。
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[str(share / 'config' / 'slam_toolbox.yaml'),
                    # use_lifecycle_manager=false 让 autostart 生效；
                    # 若有外部生命周期管理器接管，则改由它驱动状态转换。
                    {'use_lifecycle_manager': False,
                     'use_sim_time': use_sim_time}],
    )

    slam_configure = EmitEvent(
        event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                          transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart),
    )

    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam),
                transition_id=Transition.TRANSITION_ACTIVATE))],
        ),
        condition=IfCondition(autostart),
    )

    # 用 navigation_launch.py 而不是 bringup_launch.py：后者会带上 map_server
    # 与 AMCL，而建图阶段地图由 SLAM 在线产出，两者会争抢 map→odom。
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path('nav2_bringup') / 'launch'
                / 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            # 派生版参数：仅 allow_unknown 与基准不同（建图期=true）
            'params_file': mapping_params_file(share),
        }.items(),
    )

    # 延迟启动 Nav2：它的 global_costmap 激活时要拿到 map→base_footprint，
    # 拿不到就会**直接判定激活失败**（不是重试）。软件渲染下 Gazebo 加载世界
    # + spawn 机器人 + SLAM 出首帧图要 40-60s，所以等够再起。
    nav2_delayed = TimerAction(period=45.0, actions=[nav2])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('headless', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('autostart', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('x_pose', default_value=_dx),
        DeclareLaunchArgument('y_pose', default_value=_dy),
        DeclareLaunchArgument('yaw_pose', default_value=_dyaw),
        sim,
        slam,
        slam_configure,
        slam_activate,
        nav2_delayed,
    ])
