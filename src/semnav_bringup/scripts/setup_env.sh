#!/usr/bin/env bash
# semnav 环境入口。
#
# semnav 是**自包含**工作区：机器人描述（URDF/xacro）与 ros_gz 桥配置都在本仓库的
# src/semnav_robot 里，不依赖任何外部工作区。因此这里只需 source ROS 与 semnav 自身。
#
# （历史：早期版本曾 source 另一个工作区以复用其机器人包。那会让两个独立项目在
#   运行时耦合 —— 对方改动或移动目录都会影响 semnav。现已改为把所需资产复制进来，
#   出处与改动记录见 src/semnav_robot/NOTICE。）
#
# 用法：
#   source src/semnav_bringup/scripts/setup_env.sh
#
# 可用环境变量覆盖工作区位置：
#   SEMNAV_WS（默认 ~/projects/semnav）

export SEMNAV_WS="${SEMNAV_WS:-$HOME/projects/semnav}"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "[semnav] 错误：未找到 /opt/ros/jazzy —— 先安装 ros-jazzy-desktop" >&2
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/jazzy/setup.bash

if [ -f "$SEMNAV_WS/install/setup.bash" ]; then
  source "$SEMNAV_WS/install/setup.bash"
else
  echo "[semnav] 警告：未找到 $SEMNAV_WS/install/setup.bash" >&2
  echo "[semnav]       请先执行：cd $SEMNAV_WS && colcon build --symlink-install" >&2
fi

# GUI 模式保留 VMware 的 OpenGL；无 GUI/headless 模式由启动命令显式设置软件 GL。
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER MESA_LOADER_DRIVER_OVERRIDE
export QT_XCB_GL_INTEGRATION=xcb_glx

echo "[semnav] 环境就绪  工作区=$SEMNAV_WS"
