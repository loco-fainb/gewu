cmake_minimum_required(VERSION 3.8)
project(semnav_robot)

find_package(ament_cmake REQUIRED)

# URDF/xacro 与 ros_gz 桥配置都是纯数据，原样 install 即可。
# 不在这里校验 URDF 语法：xacro 展开需要 gz 侧的参数映射（odom_source、
# enable_* 开关），由 sim.launch.py 在启动时喂进去，构建期展开没有意义。
install(DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES LICENSE NOTICE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
