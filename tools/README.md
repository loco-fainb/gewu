# tools/ 排查工具

这些是**诊断用**脚本，不属于运行时链路，也不进 `CMakeLists.txt` 的 install 列表。
都是第一阶段排查"厨房走廊死锁"时写的，留下来是因为**换别的控制器/改地图后可能还需要**。

## `probe6_stall.py` — 抓控制器停滞瞬间的决策全貌

```bash
source src/semnav_bringup/scripts/setup_env.sh
python3 tools/probe6_stall.py /tmp/probe6.json
```

用 `/odom` 的 12 s 滑动窗口净位移判定"停滞"，**停滞瞬间立刻**转储 `/evaluation`，
再把 819 条候选按 `vx` 分档打印最优 `total` 与 critic 明细 ——
"哪一项在惩罚前进"一眼可见。

⚠️ **两个前提**：
- 只对 **DWB** 有意义：`/evaluation`（`dwb_msgs/msg/LocalPlanEvaluation`）是
  `dwb_core` 专有话题，换成 RPP 后**不再发布**。要用它请先切回 DWB。
- 必须**立刻**抓：任务被取消后该话题即停发，晚一步就没了（脚本内是回调式触发，
  不是等超时）。

## `mapwin.py` — 地图窗口 ASCII 打印 + A* 叠加

```bash
python3 tools/mapwin.py --x0 1.5 --x1 4.6 --y0 -0.3 --y1 3.2 \
        --start 2.31,0.33 --goal 2.30,1.60
```

把地图某一块以 `#`已占 / `.`自由 / `?`未知 / `o`A*路径 打印出来，
并给出路径分段方向。用来回答"墙到底在不在""规划器到底会不会绕过它"这类问题 ——
比看 RViz 截图可复制、可 diff。`--unknown-ok` 可切换未知格是否算可通行
（便于复现"路径穿未知区"的故障）。

## `geom.py` — 通道净空实测

量测走廊车道净空 vs 路径所在位置，用于**排除**"几何不通"这一假设。
第一阶段结论：走廊车道净空 0.30~0.35 m > 车体半径 0.1745 m，
所以死锁**不是**几何问题（真正的根因见 README「走廊死锁」一节）。

## 顶层编排脚本

- `e2e_phase1.sh` — 端到端验收：建图 → 存图 → 复定位 → 6 目标，一键跑完，
  最后打印汇总表。**跑验收请用这个**，它保证了"导航段建立在刚重启的栈上"
  这一前提（否则 AMCL 会发散，见 README）。
- `relaunch_stack.sh` — 干净重启仿真栈并等到 AMCL 真正 `active`。
- `src/semnav_bringup/scripts/` 下的才是进 install 的正式脚本
  （`phase1_mapping.py` / `phase1_navigation.py` / `evaluate_map.py` /
  `check_tour.py` / `check_reachability.py` / `generate_world.py` / `set_initial_pose.py`）。
