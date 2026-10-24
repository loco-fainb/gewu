"""语义目标定位 launch（v3）。

用法：
  ros2 launch semnav_bringup semantic.launch.py target_class:=refrigerator

前提：仿真已启动且 enable_depth:=true。
坐标系 auto 时按 map -> odom 探测（A 模式无 AMCL，只有 odom）。

门槛含义：
  min_score     尺寸匹配误差分上限（越小越严；冰箱实测 0.029）
  min_margin    最优类与次优类的分数差下限（区分度）
  max_planarity 拟合平面残差上限（冰箱 0.005，墙面 0.19~0.28）
  ground_z_*    物体最低点允许范围（落地柜体 ≈0.07；只看上半段会 >0.35）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    from ament_index_python.packages import get_package_share_path

    layout = str(get_package_share_path('semnav_bringup') / 'config' / 'apartment.yaml')

    args = [
        DeclareLaunchArgument('target_class', default_value='refrigerator'),
        DeclareLaunchArgument('min_score', default_value='0.15'),
        DeclareLaunchArgument('min_margin', default_value='0.02'),
        DeclareLaunchArgument('max_planarity', default_value='0.05'),
        DeclareLaunchArgument('ground_z_min', default_value='-0.15'),
        DeclareLaunchArgument('ground_z_max', default_value='0.35'),
        DeclareLaunchArgument('min_area_px', default_value='2500'),
        DeclareLaunchArgument('min_depth', default_value='0.15'),
        DeclareLaunchArgument('max_depth', default_value='4.5'),
        DeclareLaunchArgument('normal_z_tol', default_value='0.45'),
        DeclareLaunchArgument('depth_jump', default_value='0.06'),
        DeclareLaunchArgument('target_frame', default_value='auto'),
        DeclareLaunchArgument('max_hz', default_value='2.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('layout_yaml', default_value=layout),
    ]

    params = {a.name: LaunchConfiguration(a.name)
              for a in args if a.name != 'layout_yaml'}
    params['layout_yaml'] = LaunchConfiguration('layout_yaml')

    return LaunchDescription(args + [
        Node(
            package='semnav_bringup',
            executable='semantic_locator.py',
            name='semantic_locator',
            output='screen',
            parameters=[params],
        ),
    ])
