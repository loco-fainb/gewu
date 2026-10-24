"""semnav 住宅仿真启动：三室一厅一卫一厨场景 + semnav_robot 机器人 + ros_gz 桥。

设计取舍：
  * 机器人侧（URDF / 传感器 / 桥配置）由本仓库的 **semnav_robot** 包提供 ——
    semnav 是自包含工作区，不依赖任何外部工作区（出处见 src/semnav_robot/NOTICE）；
    只把世界换成 semnav_bringup 生成的语义住宅。
  * `GZ_SIM_RESOURCE_PATH` 在 launch 里显式声明，不依赖 shell 手工 export ——
    否则换个终端或换台机器就会「模型找不到」这种难查的故障。
  * headless 不传 `--headless-rendering`：那条路径会让 gpu_lidar 的 360 条射线
    全部读到 inf（上游实测结论）。headless 由调用方用 xvfb-run 提供虚拟显示，
    再配合软件 GL，雷达渲染路径才正常。

用法：
    ros2 launch semnav_bringup sim.launch.py
    ros2 launch semnav_bringup sim.launch.py x_pose:=1.0 y_pose:=-2.5
    ros2 launch semnav_bringup sim.launch.py rviz:=true
    xvfb-run -a -s "-screen 0 1280x1024x24" \\
        ros2 launch semnav_bringup sim.launch.py headless:=true
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_MODEL = 'semnav_robot'


def _spawn_defaults():
    """出生点 (x, y, yaw) 的默认值取自布局 YAML 的 spawn —— 单一真源。

    踩过的坑：此前 x_pose/y_pose 的默认值硬编码在 launch 文件里，改户型后
    与 spawn 不一致；而 set_initial_pose.py 读的正是 spawn，于是机器人被
    生成在 A 点、AMCL 却被告知在 B 点，粒子当场发散。改为同源读取后，
    改 spawn 一处即全链路生效。
    """
    try:
        path = get_package_share_path('semnav_bringup') / 'config' / 'apartment.yaml'
        spawn = yaml.safe_load(open(path, encoding='utf-8')).get('spawn', {}) or {}
    except Exception:
        spawn = {}
    return (str(spawn.get('x', 0.0)),
            str(spawn.get('y', 0.0)),
            str(spawn.get('yaw', 0.0)))
SENSOR_ARGS = ('enable_lidar', 'enable_ranges', 'enable_tof',
               'enable_cameras', 'enable_depth', 'enable_imu')


def _make_nodes(context):
    def cfg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    render_engine = cfg('render_engine')
    x_pose = cfg('x_pose')
    y_pose = cfg('y_pose')
    yaw_pose = cfg('yaw_pose')
    headless = cfg('headless').lower() == 'true'
    software_gl = cfg('software_gl').lower() == 'true'
    use_sim_time = cfg('use_sim_time').lower() == 'true'
    rviz = cfg('rviz').lower() == 'true'
    visual_only = cfg('visual_only').lower() == 'true'
    odom_source = cfg('odom_source')

    share_robot = get_package_share_path(ROBOT_MODEL)
    urdf_path = share_robot / 'urdf' / 'robot.urdf.xacro'
    bridge_cfg = share_robot / 'config' / 'gz_bridge.yaml'
    world_path = get_package_share_path('semnav_bringup') / 'worlds' / 'apartment.world'

    mappings = {'odom_source': odom_source}
    for name in SENSOR_ARGS:
        mappings[name] = 'false' if visual_only else cfg(name)
    robot_description = xacro.process_file(str(urdf_path), mappings=mappings).toxml()

    # 家具模型全部来自本仓库的 semnav_models（world 里的 18 个 model:// 引用
    # 均为 sem_*），不依赖任何外部模型库。
    resource_dirs = [
        str(get_package_share_path('semnav_models') / 'models'),
    ]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing:
        resource_dirs.append(existing)

    print(f'[semnav] world  : {world_path}')
    print(f'[semnav] urdf   : {urdf_path}')
    print(f'[semnav] odom   : {odom_source}')

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resource_dirs)),
        # Ubuntu 24.04 的 GNOME Wayland + Xwayland 下，Qt 的 MIT-SHM 重绘会
        # 造成 Gazebo GUI 闪烁；固定走 XCB 且关闭共享内存路径。
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        SetEnvironmentVariable('QT_XCB_GL_INTEGRATION', 'xcb_glx'),
    ]
    if headless or software_gl:
        # 虚拟机无硬件 GL。软件光栅器保证 ogre2 拿到 OpenGL 3.3+，
        # 雷达与相机才有渲染路径可走。
        actions += [
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
            SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
        ]

    # server-only 开关：headless 时不启 GUI。
    # 注意与 --headless-rendering 的区别：后者会关掉渲染管线，导致 gpu_lidar
    # 全读 inf（见本文件头部说明）；`-s` 只是不创建 GUI 进程，server 端仍完整
    # 渲染，传感器正常。实测 GUI 在无头场景白占 ~88% CPU。
    server_only = '-s ' if headless else ''
    gz_args = (f'{server_only}-r -v 4 --render-engine {render_engine} {world_path}')

    gz_sim_launch = get_package_share_path('ros_gz_sim') / 'launch' / 'gz_sim.launch.py'
    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_sim_launch)),
            launch_arguments={'gz_args': gz_args,
                              'on_exit_shutdown': 'true'}.items(),
        ),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['--ros-args', '-p', f'config_file:={bridge_cfg}']),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'robot_description': robot_description}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', ROBOT_MODEL, '-topic', 'robot_description',
                        '-timeout', '180', '-x', x_pose, '-y', y_pose,
                        '-Y', yaw_pose, '-g', ' ',
                        '-allow_renaming', 'false']),
    ]

    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                            parameters=[{'use_sim_time': use_sim_time}]))

    return actions


def generate_launch_description():
    _spawn_x, _spawn_y, _spawn_yaw = _spawn_defaults()
    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'render_engine', default_value='ogre',
            description='gz sim render engine: ogre works on this GPU-less VM; '
                        'ogre2 works on hosts with a real GPU'),
        DeclareLaunchArgument('x_pose', default_value=_spawn_x,
                              description='机器人出生点 x（默认取布局 YAML 的 spawn）'),
        DeclareLaunchArgument('y_pose', default_value=_spawn_y,
                              description='机器人出生点 y（默认取布局 YAML 的 spawn）'),
        DeclareLaunchArgument('yaw_pose', default_value=_spawn_yaw,
                              description='机器人出生点朝向 rad（默认取布局 YAML 的 spawn）'),
        DeclareLaunchArgument('headless', default_value='false',
                              choices=['true', 'false'],
                              description='软件 GL；需由调用方提供显示（xvfb-run）'),
        DeclareLaunchArgument('software_gl', default_value='true',
                              choices=['true', 'false'],
                              description='强制 Mesa 软件 GL（GUI 崩溃时用）'),
        DeclareLaunchArgument('rviz', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument(
            'visual_only', default_value='false', choices=['true', 'false'],
            description='稳定可视化模式：关闭 Gazebo 传感器渲染，只显示房屋和机器人'),
        DeclareLaunchArgument('odom_source', default_value='ground_truth',
                              choices=['ground_truth', 'robot_wheels'],
                              description='哪一个里程计拥有 /odom 与 /tf。'
                                          'ground_truth 无滑移，适合先建立基线；'
                                          'robot_wheels 带编码器滑移，用于验证 SLAM 鲁棒性'),
    ]
    for name, default, desc in (
        ('enable_lidar', 'true', '2D 激光 /scan'),
        ('enable_ranges', 'true', '侧向测距 /range_left|right'),
        ('enable_tof', 'true', '前向 ToF /tof_front/points'),
        ('enable_cameras', 'false', '双目 RGB（语义导航需要，默认关）'),
        ('enable_depth', 'false', '深度相机（语义导航定位用，默认关）'),
        ('enable_imu', 'true', 'IMU /imu'),
    ):
        declared.append(DeclareLaunchArgument(name, default_value=default,
                                              choices=['true', 'false'],
                                              description=desc))

    declared.append(OpaqueFunction(function=_make_nodes))
    return LaunchDescription(declared)
