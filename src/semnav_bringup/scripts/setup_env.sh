#!/usr/bin/env bash
# semnav 环境入口：按依赖顺序 source 各工作区。
#
# 为什么需要这个文件：
#   semnav 复用上游 oomwoo_one 的机器人模型（URDF / 传感器 / ros_gz 桥配置），
#   而它位于**另一个**工作区。ROS 2 的 underlay 机制要求按顺序 source，
#   漏掉任何一个都会在启动时报 "package 'xxx' not found" —— 这类报错只看
#   现象很难猜到根因，所以把它固化成脚本，而不是靠记忆敲 export。
#
# 用法：
#   source src/semnav_bringup/scripts/setup_env.sh
#
# 可用环境变量覆盖工作区位置：
#   SEMNAV_WS（默认 ~/projects/semnav）、OOMWOO_WS（默认 ~/oomwoo-project）

export SEMNAV_WS="${SEMNAV_WS:-$HOME/projects/semnav}"
export OOMWOO_WS="${OOMWOO_WS:-$HOME/oomwoo-project}"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "[semnav] 错误：未找到 /opt/ros/jazzy —— 先安装 ros-jazzy-desktop" >&2
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/jazzy/setup.bash

if [ -f "$OOMWOO_WS/install/setup.bash" ]; then
  source "$OOMWOO_WS/install/setup.bash"
else
  echo "[semnav] 警告：未找到上游工作区 $OOMWOO_WS/install/setup.bash" >&2
  echo "[semnav]       机器人模型 oomwoo_one 将不可用。" >&2
  echo "[semnav]       请先在 $OOMWOO_WS 执行 source /opt/ros/jazzy/setup.bash && colcon build" >&2
fi

if [ -f "$SEMNAV_WS/install/setup.bash" ]; then
  source "$SEMNAV_WS/install/setup.bash"
else
  echo "[semnav] 警告：未找到 $SEMNAV_WS/install/setup.bash" >&2
  echo "[semnav]       请先执行：cd $SEMNAV_WS && colcon build --symlink-install" >&2
fi

# 虚拟机无硬件 GL：软件光栅器同时保证 gpu_lidar / 相机的渲染路径可用。
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

echo "[semnav] 环境就绪  工作区=$SEMNAV_WS  上游=$OOMWOO_WS"
