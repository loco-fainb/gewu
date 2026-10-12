#!/usr/bin/env bash
# 第一阶段端到端验收：建图 → 存图 → 复定位 → 6 目标导航
#
# 两段都跑，因为**巡游本身也是用 Nav2 驱动的**：换控制器不能只看导航段，
# 建图段同样是 Nav2 在跟路径。
#
# 踩过的坑（都已在本脚本内规避）：
#   1) 必须在 nohup xvfb-run **之前** source 环境，否则新 shell 找不到 ros2
#   2) 本脚本内绝不 pkill xvfb —— 会杀掉自己刚拉起的 xvfb-run
#   3) ROS setup.bash 在 set -u 下会因未绑定的 AMENT_TRACE_SETUP_FILES 报错
#   4) 不能往"已定位"的机器人身上再灌出生点初始位姿 —— AMCL 会发散（实测 6.46 m）
#      所以导航段必须建立在**刚重启**的栈上
set -Eeuo pipefail

WS="$HOME/projects/semnav"
LOG=/tmp/e2e.log
: > "$LOG"
exec > >(tee -a "$LOG") 2>&1

source_ros() { set +u; source "$1"; set -u; }

stop_stack() {
  echo "--- 停栈 ---"
  for n in controller_server planner_server bt_navigator behavior_server smoother_server \
           velocity_smoother amcl map_server slam_toolbox lifecycle_manager; do
    pkill -f "/opt/ros/jazzy/lib/.*/$n" 2>/dev/null || true
  done
  sleep 3
  pkill -f 'gz sim' 2>/dev/null || true
  pkill -f 'ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz' 2>/dev/null || true
  sleep 3
  pkill -f 'xvfb-run -a -s -screen 0 1280x1024x24' 2>/dev/null || true
  pkill -f 'mapping.launch.py' 2>/dev/null || true
  pkill -f 'localization.launch.py' 2>/dev/null || true
  sleep 4
  echo "  残留 gz=$(pgrep -c -f 'gz sim' 2>/dev/null || true)"
}

wait_gz() {
  for i in $(seq 1 40); do
    n=$(pgrep -c -f 'gz sim' 2>/dev/null || true); n=${n:-0}
    if [ "$n" -ge 3 ]; then echo "  gz 起来 $n 个进程 (${i}0s)"; return 0; fi
    sleep 10
  done
  echo "  ✗ gz 没起来"; return 1
}

cd "$WS"
source_ros "$WS/src/semnav_bringup/scripts/setup_env.sh"
source_ros /opt/ros/jazzy/setup.bash
source_ros "$WS/install/setup.bash"

echo "############ 阶段 A：建图 ############"
stop_stack
rm -f maps/apartment.pgm maps/apartment.yaml maps/apartment.data maps/apartment.posegraph

setsid nohup xvfb-run -a -s "-screen 0 1280x1024x24" \
  ros2 launch semnav_bringup mapping.launch.py headless:=true \
  < /dev/null > /tmp/e2e_map_stack.log 2>&1 &
echo "  建图栈 launcher pid $!"
wait_gz || { tail -30 /tmp/e2e_map_stack.log; exit 1; }

echo "--- 等 SLAM + Nav2（150s）---"
sleep 150
python3 - <<'PY'
import yaml
d = yaml.safe_load(open('/tmp/semnav_mapping_params.yaml', encoding='utf-8'))
print('  建图期 planner allow_unknown =',
      d['planner_server']['ros__parameters']['GridBased']['allow_unknown'])
print('  建图期 controller =',
      d['controller_server']['ros__parameters']['FollowPath']['plugin'])
PY

echo "--- 巡游 15 个点 ---"
set +e
python3 src/semnav_bringup/scripts/phase1_mapping.py \
  --layout src/semnav_bringup/config/apartment.yaml \
  --outdir experiments > /tmp/e2e_tour.out 2>&1
echo "  巡游退出码 $?"
set -e
tail -25 /tmp/e2e_tour.out

echo "--- 存图 ---"
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/apartment \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=30.0 2>&1 | tail -6
ls -l maps/

echo "############ 阶段 B：复定位 + 6 目标导航 ############"
stop_stack
setsid nohup xvfb-run -a -s "-screen 0 1280x1024x24" \
  ros2 launch semnav_bringup localization.launch.py headless:=true \
  < /dev/null > /tmp/e2e_nav_stack.log 2>&1 &
echo "  导航栈 launcher pid $!"
wait_gz || { tail -30 /tmp/e2e_nav_stack.log; exit 1; }

echo "--- 等 AMCL 激活（最长 300s）---"
# ⚠️ 这里必须 `set +e`：ros2 lifecycle get 在节点还没起来时**返回非零**，
#    叠加上面的 `set -e` + `pipefail`，脚本会在第一次轮询就静默退出
#    （实测：栈明明健康、AMCL 稍后就 active，但编排脚本已经死了）。
set +e
for i in $(seq 1 30); do
  st=$(timeout 10 ros2 lifecycle get /amcl 2>&1 | head -1)
  echo "  [$i] amcl=$st"
  case "$st" in *active*) break;; esac
  sleep 10
done
set -e
sleep 20

echo "--- 6 目标验收 ---"
set +e
python3 src/semnav_bringup/scripts/phase1_navigation.py \
  --layout src/semnav_bringup/config/apartment.yaml \
  --outdir experiments --goal-timeout 180 > /tmp/e2e_accept.out 2>&1
echo "  验收退出码 $?"
set -e
tail -20 /tmp/e2e_accept.out

echo "############ 汇总 ############"
python3 - <<'PY'
import glob, json, os
def newest(pat):
    f = sorted(glob.glob(pat), key=os.path.getmtime)
    return f[-1] if f else None
m = newest(os.path.expanduser('~/projects/semnav/experiments/phase1_mapping_*.json'))
n = newest(os.path.expanduser('~/projects/semnav/experiments/phase1_navigation_*.json'))
if m:
    d = json.load(open(m))
    ws = d.get('goals') or []
    print(f"建图 {os.path.basename(m)}: 到达 {len(ws)} 个点，success_rate={d.get('success_rate')} "
          f"墙钟 {d.get('wall_clock_s')}s 路径长 {d.get('path_length_m')}m")
    st = d.get('map') or {}
    if st.get('available'):
        print(f"  地图 {st.get('width')}×{st.get('height')} @ {st.get('resolution')} m，"
              f"已知 {st.get('known_area_m2')} m²（自由 {st.get('free_area_m2')} m²）")
else:
    print("建图：无结果文件")
if n:
    d = json.load(open(n))
    print(f"\n导航 {os.path.basename(n)}: {d.get('goals_succeeded')}/{d.get('goals_total')} "
          f"= {d.get('success_rate')}  墙钟 {d.get('wall_clock_s')}s")
    print(f"  定位误差 均值 {d.get('loc_error_m_mean')} m / 最大 {d.get('loc_error_m_max')} m")
    for g in d['goals']:
        ok = '✓' if g['success'] else '✗'
        print(f"    {ok} {g['name']:<8} {str(g['result']):<20} {g['elapsed_s']:>6.1f}s  "
              f"终点误差 {g['final_dist_m']:.3f} m")
else:
    print("导航：无结果文件")
PY
echo "############ E2E 结束 ############"
