# semnav —— 室内轮式机器人主动语义探索与目标导航

配套《室内轮式机器人主动语义探索与目标导航项目计划书》的实现工作区。
场景：**三室一厅一卫一厨**住宅，机器人：上游 `oomwoo_one`（Proscenic M6 Pro 同构，圆盘 Ø349mm）。

## 与上游项目的关系（重要）

本工作区**独立**，不修改上游 `~/oomwoo-project` 的任何文件。两者是复用关系：

| 复用什么 | 来源 | 为什么 |
|---|---|---|
| 机器人 URDF / 传感器 / ros_gz 桥配置 | `oomwoo_one` 包 | 已调通的 360° 激光 + IMU + 双 RGB + 前向 ToF，重写一遍没有收益 |
| Gazebo 启动装配方式 | `oomwoo_gazebo` 的 launch（仅作参考） | 保留其已验证的 spawn / bridge 写法 |

**不复用**上游的场景与家具：上游的 `Sofa`/`CoffeeTable`/`TVStand` 等模型**没有或只有残缺的碰撞体**，
机器人会直接穿过去 —— 这对建图与导航验证是致命的。因此本工作区自建
`semnav_models`（单一包围盒，碰撞体与视觉严格一致，且 1 模型 = 1 语义实例）。

### 依赖顺序（必须）

`oomwoo_one` 在另一个工作区，ROS 2 的 underlay 机制要求按序 source。
漏掉会报 `package 'oomwoo_one' not found` —— 只看现象很难猜到根因，所以固化成脚本：

```bash
source src/semnav_bringup/scripts/setup_env.sh
```

（该脚本依次 source ROS 2 → 上游工作区 → 本工作区，并设好软件 GL 环境变量。
可用 `SEMNAV_WS` / `OOMWOO_WS` 覆盖路径。）

## 构建

```bash
cd ~/projects/semnav
source src/semnav_bringup/scripts/setup_env.sh   # 首次需先手动 source 上游 + ROS
colcon build --symlink-install
source install/setup.bash
```

**世界与模型在构建期生成**，不是提交进仓库的静态文件。理由：

1. 世界与语义真值清单**同源**产出（同一份 `apartment.yaml`）。真值必须与世界一致，
   否则 Phase 2 的「地图位置 vs 真值误差」评测失去意义。
2. 生成器内置**布局校验**：插墙 / 越界 / 物体重叠 / 门洞净空 / 承托关系（z 轴判断）。
   校验失败即构建失败 —— 把错误拦在生成阶段，而不是「启动 40 秒 + 肉眼看 Gazebo」。
3. 尺寸定义只有一处，不会出现「改了 YAML 忘了重新生成」。

## 运行

### 一阶段验收：一条命令跑完全链路

```bash
cd ~/projects/semnav
bash e2e_phase1.sh          # 建图 → 存图 → 复定位 → 6 目标，约 13 min
```

按顺序做四件事，完成后打印汇总表：① 起建图栈走完 15 点巡游 → ② `map_saver_cli`
存图 → ③ **完整停栈**后起导航栈（保证机器人回到出生点，见下方"静默发散"一节）→
④ 6 个目标逐个验收。日志 `/tmp/e2e.out`，结果 JSON/MD 落在 `experiments/`。

### 分步跑

```bash
# 仿真（虚拟机无 GPU，需软件 GL；headless 由调用方提供虚拟显示）
xvfb-run -a -s "-screen 0 1280x1024x24" \
  ros2 launch semnav_bringup sim.launch.py headless:=true

# 带桌面窗口（虚拟机内直接看）
source src/semnav_bringup/scripts/setup_env.sh
ros2 launch semnav_bringup sim.launch.py

# 覆盖出生点 / 附加 RViz / 切里程计源
ros2 launch semnav_bringup sim.launch.py x_pose:=2.0 y_pose:=-2.0 rviz:=true
ros2 launch semnav_bringup sim.launch.py odom_source:=robot_wheels
```

### ⚠️ headless 不能用 `--headless-rendering`

上游实测结论：那条路径会移除 `gpu_lidar` 用的渲染上下文，导致 **360 条射线全部读到
`inf`**（雷达"瞎掉"，但进程照常运行、话题照常有输出——属于静默失效）。
正确做法是 `xvfb-run` 提供虚拟显示 + `LIBGL_ALWAYS_SOFTWARE=1`，保留完整渲染路径。

### ⚠️ Gazebo DiffDrive 会锁存最后一条 `/cmd_vel`

发布者退出后机器人**仍按最后速度继续走**（上游实测又走 2.27 m 撞墙）。
停车必须**持续发零速度**——"命令过期"是危险信号，不是安全信号。

## 场景定义

`src/semnav_bringup/config/apartment.yaml` 是唯一的场景真源，含：

- `rooms`：6 个房间的矩形边界
- `doors`：每个房间通向走廊的门（90cm 宽）
- `objects`：19 个语义对象，含**类别 / 中文标签 / 精确位姿 / 真实尺寸 / 朝向**
- `spawn` / `nav_goals`：出生点与各房间导航目标点

生成物：

- `worlds/apartment.world` —— Gazebo 世界
- `config/apartment_truth.json` —— **语义真值清单**（Phase 2 目标检测与语义地图评测的对照基准）

改动布局后重新生成：

```bash
python3 src/semnav_bringup/scripts/generate_world.py \
  --layout src/semnav_bringup/config/apartment.yaml \
  --out-world /tmp/apartment.world --out-truth /tmp/truth.json   # 先干跑校验
```

## ⚠️ `allow_unknown` 在建图期与导航期必须取相反的值

这是本项目踩过代价最大的一个坑，单独成节。

`planner_server.GridBased.allow_unknown` 决定 NavFn 把**未知**栅格当可通行还是当致命。
两个阶段的需求正好相反：

| 阶段 | 取值 | 原因 |
|---|---|---|
| 建图 | `true` | 下一个目标点常常还在**没被观测过**的房间里，那一片全是"未知"。未知当致命则规划器一步都规划不出去 —— 越没去过的地方越去不了，建图永远停在第 1 个房间。 |
| 导航 | `false` | 地图已建好，未知区就是"从没看到过的地方"。当可通行会诱使规划器**穿墙抄近路**（代价函数只算路径长度，穿墙比绕门短），控制器物理上推不过去。 |

**实测症状**：次卧(1.20,3.00) → 厨房(2.30,1.60)。x=+1.6 的隔墙在 y∈[0.8,1.8] 一段被次卧
书桌挡住（书桌 x[0.94,1.54] 正好贴到墙 x=1.54），2D 激光从次卧一侧**永远看不到它**，
栅格值一直是"未知"。规划器给出 3.6 m 穿墙直线（绕厨房门要 8.2 m），其中 63.9% 的
路径点落在未知栅格上。机器人卡死在书桌与隔墙之间，`Failed to make progress` ×15 后
abort，耗时 237 s。

**为什么没有维护两份参数文件**：`nav2_params.yaml` 有 451 行，复制成两份必然漂移
（改了一处忘另一处，且很难发现）。改成**程序化派生**：`mapping.launch.py` 读共享基准，
只覆写 `allow_unknown` 这一个键写到 `/tmp/semnav_mapping_params.yaml`。基准文件里其余
所有调参在两个阶段自动保持一致。

**建图巡游路线也必须配合**：`apartment.yaml` 的 `mapping_tour` 严格「房间 ↔ 走廊」交替。
原因是只把 `allow_unknown` 设对还不够 —— 建图期它**必须**是 `true`，所以"房间→房间"
直达时穿墙近路依然可能出现。把每段两端都锚在走廊（已知区域）上，穿墙点就被已知墙挡住。
旧路线 `nav_goals` 里 次卧→厨房 正是"房间→房间"直达的例子。

**开跑前的预检**：`check_tour.py` 用 A\* 在建好的地图上逐段复算，对照世界真值数穿墙次数，
两种语义都给结果。几秒出结果，能在烧掉十几分钟仿真之前把路线缺陷挑出来：

```bash
python3 install/semnav_bringup/lib/semnav_bringup/check_tour.py \
  --map maps/apartment.yaml \
  --layout src/semnav_bringup/config/apartment.yaml \
  --truth install/semnav_bringup/share/semnav_bringup/config/apartment_truth.json
```

退出码 0 = 各段均不穿墙。

## ⚠️ `nav2_simple_commander` 超时取消会**伪造成功**

验收脚本一度把一次 237 s 的真实失败报成 `SUCCEEDED`，`elapsed` 恰好等于
`GOAL_TIMEOUT_S`。根因是 `BasicNavigator` 的状态机有两处设计如此的行为：

- `isTaskComplete()` 第一行是 `if not self.result_future: return True` —— 取消后
  `result_future` 被置空，于是直接返回"完成"；
- `getResult()` 读的是 `self.status`，而这个值**只在结果真正到达时才被赋值** ——
  取消路径下读到的是**上一个目标**的残留状态。

两者叠加：`cancelTask()` 之后 `getResult()` 返回上一个目标的 `SUCCEEDED`。
`cancelTask()` 本身也不清 `result_future`、不等终态。

**修法**：超时后等动作**真正结算**再读结果，并显式把该段标记为取消。

**更根本的一道防线**：验收条件是「到达指定坐标」，那就该**按坐标验**。
`phase1_navigation.py` 因此额外断言终点（`/odom`，`odom_source=ground_truth` 无滑移）
与目标坐标的直线距离 ≤ 0.25 m，与 action 返回值**同时**满足才算到达。
"机器人到底在不在目标点附近"是物证，无法被驱动脚本的状态机伪造 ——
这类假成功比失败更危险，它会把真实存在的规划缺陷掩盖成一张 6/6 的漂亮表。

## ⚠️ 第三个假结果：被拒绝的目标会被记成 `UNKNOWN / 0.0s`

`goToPose()` 会先 `wait_for_server()`，但那只等动作服务器**被发现**，
不等 `bt_navigator` **激活完成** —— 动作服务器在 `inactive` 状态就已经宣告存在。
这个等待因此被骗过，紧接着 `send_goal_async` 被 `bt_navigator` 拒掉：

```
[bt_navigator]: Action server is inactive. Rejecting the goal.
[basic_navigator]: Goal to -4.0 2.5 was rejected!
```

而 `goToPose()` 此时 **`return False`** —— 它**不会**设置 `self.result_future`。
于是 `isTaskComplete()` 的第一行 `if not self.result_future: return True` 立刻为真，
`getResult()` 读回残留状态，这个目标就被记成 **`TaskResult.UNKNOWN`、用时 0.0 s**。

实测：整机的**第一个**目标最容易中（验收脚本在 `bt_navigator` 激活完成前几秒开火），
同一轮里后面 5 个目标全部正常 —— 因为那时栈已经活了。
一次运行因此报 `5/6`，看着像导航退化，其实是**起跑枪响得太早**。

**修法（两处，都已落地）**：
1. 等 `bt_navigator` 生命周期真正 `active`（`ros2 lifecycle get`），再静默 3 s 让 Bond 建稳；
2. `drive_to()` 检查 `goToPose()` 的返回值，被拒则**重试一次**，
   并把 `accepted` 记进结果 JSON —— 不再把"没跑起来"混成"跑失败了"。

## ⚠️ 两个"静默卡死/静默发散"的运维坑（都踩过）

### 1) `phase1_mapping.py` 曾卡在 `nav.lifecycleShutdown()` 永不返回

`BasicNavigator.lifecycleShutdown()` 内部会**无限等待**
`/lifecycle_manager_localization/manage_nodes` 服务。而建图段起的是
`mapping.launch.py`（SLAM，没有 AMCL / 定位生命周期管理器），该服务根本不存在 ——
脚本卡死，把调用方（`run_mapping.sh`）挡在后面的 `map_saver_cli` **之前**，
于是**地图永远存不下来**，而且没有任何报错。

结果 JSON 其实早就写完了，所以从产物看"一切正常"，只有地图文件的时间戳不更新。
**已在 `phase1_mapping.py` 移除该调用**：建图段结束后由**编排脚本**负责停栈，脚本直接退出。

### 2) 不要往"已定位"的机器人身上重灌出生点初始位姿

`phase1_navigation.py` 会把出生点 `(-4.2, 0.0)` 作为初始位姿发给 AMCL（这是必要的，
否则 AMCL 从 `(0,0,0)` 起步，一开始就错）。但它假设**机器人此刻确实在出生点**。

实测踩过两次：先用别的脚本把机器人开到别处（如厨房），再跑验收脚本 ——
AMCL 收到一个与激光观测严重矛盾的初始位姿，粒子当场发散，
`loc_error` 飙到 **6.46 m**，6 个目标全部 `FAILED`（每个只跑 17 s 就放弃）。

**规矩**：跑 `phase1_navigation.py` 之前，栈必须是**刚重启**的（机器人回到出生点）。
`e2e_phase1.sh` 就是按这个顺序编排的（建图段结束后完整停栈 → 再起导航段）。

### 3) 编排脚本里 `set -e` + `pipefail` 会因**轮询命令**返回非零而静默退出

```bash
st=$(timeout 10 ros2 lifecycle get /amcl 2>&1 | head -1)   # 节点没起来 → 非零 → 脚本死掉
```

栈其实是健康的、AMCL 稍后就 `active`，但编排脚本已经在第一次轮询时静默退出了，
表现为"卡在等 AMCL"、`/tmp/e2e.out` 不再增长。等 AMCL 的那一段必须 `set +e` 包住
（`e2e_phase1.sh` 已如此）。

## ⚠️ 走廊死锁：DWB 的代价架构会奖励"原地不动"，已换装 RPP

修掉 `allow_unknown`（上一节）之后，厨房那一段**仍然**失败：场景是从走廊去厨房，
机器人停在 (2.30, 0.21)，目标 (2.30, 1.60) 就在正北 1.39 m —— 但被走廊北墙隔开，
真正能走的路是先**向东**绕到门口再北转。现象是原地微转、10 s 后被判
`Failed to make progress`。**几何不是原因**：同一张地图上这段走廊车道的净空实测
0.30~0.35 m，大于机器人半径 0.1745 m。

### 先纠正一个读错的字段语义（踩过，别重复）

`dwb_msgs/msg/LocalPlanEvaluation.total` 是**代价，越小越好**，不是得分；
`best_index` 实测是 **argmin**。`-1.0` 是"非法轨迹"哨兵值。已用数据核对：
`total = Σ(raw_score/100 × scale)` —— 静止轨迹 `GoalDist 75 × 0.24 = 18.0`；
前进轨迹 `GoalDist 94×0.24 + PathAlign 13×0.32 + PathDist 9×0.32 = 29.6`。
（早先按"越大越好"读，据此删掉 `GoalAlign` 并宣称修好 —— **该结论已实测推翻**：
换回重跑，厨房 leg 依旧 `CANCELED @ 240.3 s`、终点距目标 1.397 m。）

### 真正的原因：评分函数里没有任何一项奖励"沿路径前进"

在停滞瞬间抓 `/evaluation` 全量转储（819 条候选，**全部合法**），按 `vx` 分档取最优：

| `vx` | 最优 `total` | 轨迹末点 | 末点直线距目标 | `GoalDist.raw` |
| --- | --- | --- | --- | --- |
| **0.00** | **16.56** ← 获胜 | (2.345, 0.186) | 1.415 m | **69** |
| 0.032 | 17.04 | (2.392, 0.185) | 1.418 m | 71 |
| 0.08 | 18.00 | (2.471, 0.184) | 1.426 m | 75 |
| 0.20 | 20.16 | (2.645, 0.182) | 1.459 m | 84 |

两条最优轨迹的 critic 明细**逐项**对比（前进 − 静止）：

```
GoalDist      +2.0        ← total 的 +0.48 差值全部来自这一项
PathAlign      0.0
PathDist       0.0
BaseObstacle   0.0
Oscillation    0.0
RotateToGoal   0.0
```

原因有两层，都不是调参能解决的：

1. `PathAlign` / `PathDist` 是**绝对误差**项。静止的机器人恰好落在路径上、
   机头也与路径切向对齐，两项 raw 恒为 **0.0** —— 被**完美满足**，无从惩罚。
   `GoalDist` 也不奖励前进：厨房在墙后，全局路径实测必须向东绕
   （`/plan` 前 3 点 `(2.32,0.22) (2.35,0.22) (2.38,0.22)`，全长 2.25 m），
   于是**沿路径每前进一点，直线距目标的 `GoalDist` 反而单调变大**（69 → 84）。
2. 因此"前进 − 静止"的各项差值**全部非负且同号**。由于 `total` 是
   `Σ(raw/100 × scale)` 且取 argmin，**任何正 `scale` 组合都无法让前进获胜**
   （把差别项调小只是让差值变小，符号不变；调成负数才能翻盘，而那会让该项失去意义）。

**结论：这是控制器架构问题，不是参数问题。** DWB 现有 critic 集合里没有任何一项
奖励"沿路径前进"（`PreferForwardCritic` 也只区分前进/后退，不区分走得远近）。
静止同时满足全部误差项 —— 它是评分函数的一个真实极小点。

### 修法：换成几何跟踪器 Regulated Pure Pursuit

RPP 直接跟踪路径上 lookahead 处的 carrot 点，**不存在"候选轨迹打分取最小代价"这一环**，
因此没有"原地不动即最优"的退化解。已在 `nav2_params.yaml` 把
`controller_server.FollowPath.plugin` 从 `dwb_core::DWBLocalPlanner` 换成
`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`。

RPP 的参数名已逐个在 `libnav2_regulated_pure_pursuit_controller.so` 里核对过
（该库的参数字符串带**前导点**，如 `.desired_linear_vel`，用 `grep -x` 会全部漏掉，
必须用 `grep -a -o`）。注意：

- `use_velocity_scaled_lookahead_dist: true` 时 `lookahead_dist` **不被使用**，
  实际 lookahead = `clamp(|vx| × lookahead_time, min, max)`。
- `xy_goal_tolerance` **不是** RPP 的参数（它在 `.so` 里出现 0 次），
  由 `controller_server.goal_checker`（`SimpleGoalChecker`）统一提供。
- `use_rotate_to_heading: true` 与 `allow_reversing: true` **不能同时为真**，
  插件会直接拒绝参数（错误信息见二进制字符串）。
- 撞上障碍时 RPP 抛 `nav2_core::NoValidControl`，由 `controller_server` 的
  `failure_tolerance` 机制处理，行为与 DWB 不同。

**排查手法（可复用）**：`probe6_stall.py` 用 `/odom` 的 12 s 滑动窗口净位移判定停滞，
**停滞瞬间立刻**转储 `/evaluation`（取消任务后该话题即停发，晚一步就抓不到），
并把 819 条候选按 `vx` 分档打印最优 `total` 与 critic 明细 ——
"哪一项在惩罚前进"一眼可见。这类评分僵局用日志看不出来：Nav2 全程只有 13 条
`ERROR`、**0 条** `illegal trajectory`、0 条 `No valid trajectories`，
DWB 从不报错，它只是**安静地**选了零速度。

## 现状

已验证（第一阶段：**仿真机器人可建图并到达指定坐标**）：

- [x] 住宅世界 + 18 个自建语义家具模型，构建期生成
- [x] 布局校验（插墙 / 重叠 / 门洞净空 / 承托）
- [x] 仿真启动：Gazebo 加载世界，机器人出生于走廊西端 (-4.2, 0.0)
- [x] 激光健康：360/360 有效射线
- [x] **建图**：SLAM Toolbox 在线出图，15 个巡游点全部到达
      （`phase1_mapping_20261012_032140.json`：402.5 s 墙钟 / 36.14 m 路径，
      地图 190×158 格 @ 0.05 m，已知 54.73 m² / 自由 49.41 m²），
      保存为 `maps/apartment.pgm`
- [x] **导航**：AMCL 复定位 + Nav2 逐个到达 6 个房间目标点，
      **6/6 全部成功**（`phase1_navigation_20261012_032930.json`，351.2 s 墙钟，
      终点误差 0.104~0.132 m，均 < 0.25 m 判据；AMCL 定位误差均值 0.033 m / 最大 0.051 m）
- [x] 预检工具：`check_tour.py`（巡游穿墙：15 段全部 0% 穿墙）、
      `check_reachability.py`（新地图 6 目标全部可达，厨房净空 0.600 m）
- [x] 端到端编排：`e2e_phase1.sh`（建图 → 存图 → 复定位 → 6 目标，一键跑完）

建图质量评测（`evaluate_map.py`）实测：对真值配准偏移 `(+0.00,+0.00)`、
幽灵障碍 **0 格**、语义对象检出 **11/19 = 57.9%**。
（更早一版地图曾测到 14/19 = 73.7% —— 检出率对**巡游路径是否恰好扫到该物体**
敏感，低矮/贴墙物体尤其如此，不是地图缺陷；幽灵障碍 0 格与配准无偏移才是不变量。）

**第一阶段验收通过。** 换装 RPP 前后同一套 `nav_goals` 的对照：

| 目标 | DWB（换装前，旧地图） | RPP（旧地图） | RPP（新地图） | RPP 端到端（新地图） |
| --- | --- | --- | --- | --- |
| 主卧 | 22.2 s / 0.118 m ✓ | 27.3 s / 0.122 m ✓ | 27.9 s / 0.125 m ✓ | 28.4 s / 0.120 m ✓ |
| 次卧 | 58.6 s / 0.123 m ✓ | 66.7 s / 0.137 m ✓ | 69.3 s / 0.130 m ✓ | 74.0 s / 0.117 m ✓ |
| **厨房** | **CANCELED 240.4 s / 1.294 m ✗** | **70.3 s / 0.152 m ✓** | **64.3 s / 0.179 m ✓** | **84.1 s / 0.126 m ✓** |
| 客厅 | 36.4 s / 0.108 m ✓ | 59.8 s / 0.109 m ✓ | 62.7 s / 0.095 m ✓ | 66.4 s / 0.121 m ✓ |
| 卫生间 | 41.9 s / 0.104 m ✓ | 52.0 s / 0.088 m ✓ | 49.5 s / 0.102 m ✓ | 50.7 s / 0.132 m ✓ |
| 走廊东端 | 21.2 s / 0.105 m ✓ | 26.9 s / 0.116 m ✓ | 27.8 s / 0.174 m ✓ | 29.4 s / 0.104 m ✓ |
| **合计** | **5/6 = 83.3%** | **6/6 = 100%** | **6/6 = 100%** | **6/6 = 100%** |

RPP 每段略慢（曲率调速 + 接近目标减速），但代价换来的是**不再有死锁**。
四列都是**独立实测**：后两列跑在**全新重跑的地图**上，
最右列是 `e2e_phase1.sh` 一键跑完「建图 → 存图 → 复定位 → 导航」全链路的结果 ——
结论不依赖某一次建图，也不依赖某个手工步骤。

未做（属后续阶段，**不要提前拉进来**）：

- [ ] 语义地图 / 目标检测（阶段二、三）
- [ ] frontier 自主探索（**阶段四**）
- [ ] 自然语言指令 → 语义导航（阶段五）

## 已知约束

- **8 GB 显存上限**：只够推理（YOLO / Grounding DINO tiny / SAM 单帧），**不能训练**。
- **激光 5 Hz**（URDF 声明值，上游实测仅 3.89 Hz）：SLAM 时线速度需压低。
- 虚拟机无 GPU，纯软件渲染，仿真实时率有限（一轮建图约 5.6 min 墙钟）。
- **2D 激光只能看到扫描平面上的东西**：低于/高于扫描面的家具（电视 22%、
  咖啡机 36%）检出率天然偏低，这是物理限制而非建图缺陷。语义层（阶段二）
  正好用来补这个缺口。

