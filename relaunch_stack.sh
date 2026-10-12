#!/usr/bin/env bash
# 干净重启仿真栈（仿真 + 定位 + Nav2），等到 AMCL 真正给出位姿为止。
# 关键坑（都踩过）：
#   1) 必须在 nohup xvfb-run **之前** source 环境，否则 xvfb-run 起的新 shell 找不到 ros2
#   2) 本脚本里绝不能 pkill xvfb —— 会连带杀掉自己刚拉起的 xvfb-run
#   3) ROS 的 setup.bash 在 set -u 下会因未绑定的 AMENT_TRACE_SETUP_FILES 报错
set -Eeuo pipefail

WS="$HOME/projects/semnav"
LOG=/tmp/relaunch_stack.log
: > "$LOG"
exec > >(tee -a "$LOG") 2>&1

source_ros() { set +u; source "$1"; set -u; }

echo "=== 1) 停旧栈 ==="
# 先停 nav2/定位节点（正常退出，让 lifecycle 收尾）
for n in controller_server planner_server bt_navigator behavior_server smoother_server \
         velocity_smoother amcl map_server lifecycle_manager; do
  pkill -f "/opt/ros/jazzy/lib/.*/$n" 2>/dev/null || true
done
sleep 3
# 再停仿真
pkill -f 'gz sim' 2>/dev/null || true
pkill -f 'ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz' 2>/dev/null || true
sleep 4
# 只杀掉上一次的 xvfb-run 包装进程，不碰别的 xvfb
pkill -f 'xvfb-run -a -s -screen 0 1280x1024x24' 2>/dev/null || true
pkill -f 'localization.launch.py' 2>/dev/null || true
sleep 4
echo "  残留: gz=$(pgrep -c -f 'gz sim' 2>/dev/null || echo 0) ctrl=$(pgrep -c -f controller_server 2>/dev/null || echo 0)"

echo "=== 2) 起新栈 ==="
cd "$WS"
source_ros "$WS/src/semnav_bringup/scripts/setup_env.sh"
echo "  ros2 = $(command -v ros2)"

setsid nohup xvfb-run -a -s "-screen 0 1280x1024x24" \
  ros2 launch semnav_bringup localization.launch.py headless:=true \
  < /dev/null > /tmp/relaunch_nav.log 2>&1 &
echo "  launcher pid $!"

echo "=== 3) 等 gz 起来 ==="
for i in $(seq 1 40); do
  g=$(pgrep -c -f 'gz sim' 2>/dev/null || echo 0)
  if [ "$g" -ge 3 ]; then echo "  gz=$g (${i}0s)"; break; fi
  sleep 10
done
if [ "$(pgrep -c -f 'gz sim' 2>/dev/null || echo 0)" -lt 3 ]; then
  echo "  ✗ gz 没起来，日志尾部："; tail -30 /tmp/relaunch_nav.log; exit 1
fi

echo "=== 4) 等 AMCL 真正发布位姿（最长 300s）==="
source_ros /opt/ros/jazzy/setup.bash
source_ros "$WS/install/setup.bash"
ok=0
for i in $(seq 1 30); do
  st=$(timeout 10 ros2 lifecycle get /amcl 2>&1 | head -1)
  n=$(timeout 12 ros2 topic echo /amcl_pose --once --field pose.pose.position 2>/dev/null | tr -d ' \n' || true)
  echo "  [${i}] amcl=${st}  pose=${n:-<无>}"
  if echo "$st" | grep -q active && [ -n "$n" ]; then ok=1; break; fi
  sleep 10
done
if [ "$ok" = 1 ]; then
  echo "✓ AMCL 已激活并定位"
else
  echo "✗ AMCL 未就绪，日志尾部："; tail -40 /tmp/relaunch_nav.log
fi
echo "=== 日志 $LOG  栈日志 /tmp/relaunch_nav.log ==="
