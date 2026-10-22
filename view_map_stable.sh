#!/usr/bin/env bash
# VMware Ubuntu 稳定可视化入口。
#
# 已知环境限制：本虚拟机的 vmwgfx / VMware SVGA3D 未提供硬件加速
# （glxinfo -B: Accelerated: no，显存约 1 MB）。GNOME Wayland + Xwayland
# 与 Gazebo Ogre2/EGL/传感器渲染并行时会出现 GUI 闪烁，甚至触发 Ogre2
# 渲染线程段错误。这里固定使用 XCB/GLX 软件光栅，并把 server 与 GUI 分离。
# 需要稳定查看地图时必须调用本脚本；不要用普通桌面 launch 作为稳定性测试。
# 传感器或 SLAM 测试请使用 xvfb-run 的 headless/server-only 模式。
set -eo pipefail

WS="${SEMNAV_WS:-$HOME/projects/semnav}"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

export GZ_SIM_RESOURCE_PATH="$WS/install/semnav_models/share/semnav_models/models:$WS/install/semnav_bringup/share/semnav_bringup/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export QT_XCB_GL_INTEGRATION=xcb_glx
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

WORLD="$WS/install/semnav_bringup/share/semnav_bringup/worlds/apartment.world"
SERVER_LOG="${TMPDIR:-/tmp}/semnav_gz_server.log"

gz sim -s -r -v 2 "$WORLD" >"$SERVER_LOG" 2>&1 &
server_pid=$!
trap 'kill "$server_pid" 2>/dev/null || true' EXIT INT TERM
sleep 5

echo "Gazebo server running (pid $server_pid). Starting stable GUI..."
exec gz sim -g -v 2
