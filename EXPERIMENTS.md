# 实验记录

> 记录纪律来自《环境搭建与开工方案》第 5 章与第 7 章：每次实验必记
> **git commit、全部 yaml 参数、rosbag、结果表**；每月只回答三个问题 ——
> **学会了什么、做出了什么、遇到了什么问题**。

本文件按时间倒序记录。

---

## 第一阶段 · 基础导航（仿真机器人可建图并到达指定坐标）

**结论：核心目标达成（6/6 目标全部到达，三次独立复现）。
但 M4 验收物尚不完整 —— 缺 rosbag 与轨迹图，见下方「未完成」。**

### 一、做出了什么

| 项 | 内容 |
| --- | --- |
| 场景 | 三室一厅一卫一厨，14 面墙 / 6 房间 / 5 门（门宽均 0.9 m），18 个自建语义家具模型，构建期由 `apartment.yaml` 单一真源生成世界与真值清单 |
| 建图 | SLAM Toolbox 在线出图，15 点巡游 402.5 s / 36.14 m，地图 190×158 格 @ 0.05 m，已知 54.73 m² / 自由 49.41 m² |
| 导航 | AMCL 复定位 + Nav2 逐个到达 6 个房间目标点，**6/6**，终点误差 0.104~0.132 m，AMCL 定位误差均值 0.033 m |
| 一键复现 | `e2e_phase1.sh`：建图 → 存图 → 复定位 → 6 目标，约 13 min |
| 预检工具 | `check_tour.py`（巡游穿墙，15 段全 0%）、`check_reachability.py`（6 目标可达，厨房净空 0.600 m） |

**验收基线（本仓库首版 commit）**

| 目标 | 结果 | 耗时 | 终点误差 |
| --- | --- | --- | --- |
| 主卧 | SUCCEEDED | 28.4 s | 0.120 m |
| 次卧 | SUCCEEDED | 74.0 s | 0.117 m |
| 厨房 | SUCCEEDED | 84.1 s | 0.126 m |
| 客厅 | SUCCEEDED | 66.4 s | 0.121 m |
| 卫生间 | SUCCEEDED | 50.7 s | 0.132 m |
| 走廊东端 | SUCCEEDED | 29.4 s | 0.104 m |

产物：`experiments/phase1_navigation_20261012_032930.json` / `.md`、
`experiments/phase1_mapping_20261012_032140.json` / `.md`。

### 二、学会了什么

1. **Nav2 控制器分两类，选错会得到"合法但错误"的行为。**
   DWB 是**采样 + 代价排序**：在每周期所有候选轨迹里取代价最低者。
   这类控制器存在**退化解** —— 当"原地不动"恰好是代价最低时，它会安静地选择零速度，
   既不报错也不报警。RPP 是**几何跟踪**：按前视点算曲率直接给速度，没有候选排序环节，
   因此结构上不存在这个退化解。
2. **读字段前先确认语义方向。** `LocalPlanEvaluation.total` 是**代价（越低越好）**、
   `best_index` 是 **argmin**。我一开始按"越大越好"读，据此删掉 `GoalAlign` critic
   并宣称修好 —— 该结论已被实测推翻，见 `nav2_params.yaml` 与 README 的更正说明。
   一个被误读的字段会一路误导到代码修改。
3. **代价函数里"绝对误差型"的项会让静止站住变得最优。**
   `PathAlign` / `PathDist` 衡量的是"当前位姿/朝向相对路径的**绝对**偏差"：
   机器人静止且压在路径上、朝向对齐切线时，两项都是 0.0，**已完全满足**，
   于是前进拿不到任何奖励；而 `GoalDist` 又是不认墙的欧氏距离。
4. **仿真器与中间件的若干静默失效模式**（详见 README 各警示节）：
   `--headless-rendering` 会让 `gpu_lidar` 全部读 `inf`；Gazebo DiffDrive 锁存最后一条
   `/cmd_vel`（发布者退出后仍继续走）；`nav2_simple_commander` 有**两处**会伪造结果
   （超时取消读回上一目标的 `SUCCEEDED`；目标被拒时短路成 `UNKNOWN/0.0s`）。
   共同点：**进程健康、话题有数据、日志几乎无 ERROR** —— 只能靠独立物证（按坐标验）识破。

### 三、遇到了什么问题

按发现顺序，每个都记"根因 → 修法"：

| # | 问题 | 根因 | 修法 |
| --- | --- | --- | --- |
| 1 | 厨房目标 240 s 超时，停在 1.29 m 外 | `allow_unknown: true` 让 NavFn 把路径穿过 **63.9% 未知格** | 建图期 `true` / 导航期 `false`，由 `mapping.launch.py` 程序化派生参数文件；巡游严格房间↔走廊交替 |
| 2 | 修完 #1 后厨房**仍然**失败 | DWB 代价架构奖励原地不动（本次用 819 条候选全量转储**证实**） | 换 `RegulatedPurePursuitController`；并**撤回**早先"删 GoalAlign 即可修好"的错误结论 |
| 3 | 建图跑完地图永远存不下来，且无报错 | `phase1_mapping.py` 卡在 `lifecycleShutdown()` —— 建图段跑 SLAM，无 `/lifecycle_manager_localization`，无限等待 | 移除该调用，改由编排脚本停栈 |
| 4 | 验收报 `UNKNOWN / 0.0 s`，看着像导航退化 | `goToPose()` 只等动作服务器"被发现"不等"激活完成"，被 `bt_navigator` 拒后 `return False` 且不设 `result_future`，触发 `isTaskComplete()` 短路 | 等生命周期真正 `active` + 被拒重试 + 记录 `accepted` 字段 |
| 5 | 编排脚本"卡在等 AMCL" | `set -e` + `pipefail` 下，轮询命令 `ros2 lifecycle get` 在节点未起时返回非零 → 脚本静默退出 | 轮询段用 `set +e` 包住 |
| 6 | 验收 6 个目标全 FAILED，`loc_error` 6.46 m | 往**已定位**的机器人重灌出生点初始位姿，AMCL 粒子发散 | 规矩：跑验收前栈必须刚重启（机器人确实在出生点）；`e2e_phase1.sh` 按此排序 |

**过程中的自我纠错（值得留档）**：问题 #2 我先后尝试了三种解释，前两种都是错的 ——
① 先怀疑几何不通（被 `geom.py` 实测净空 0.30~0.35 m > 车体 0.1745 m 排除）；
② 再怀疑 `GoalAlign` critic（删掉后厨房仍 `CANCELED 240.3 s`，实测推翻）。
直到把 819 条候选逐条转储、按 `vx` 分档比对 critic 明细，才拿到"所有差值同号 ⇒
任何正权重都翻不了盘"这个**可判定的**结论。教训：**能被证伪的测量优于合理的推理**。

### 五、解耦：从"复用外部工作区"改为"自包含"

**问题**：初版把机器人包放在另一个工作区，靠 `source` 对方 `install/setup.bash` 复用。
这构成**运行时耦合** —— 对方改动、移动或删除目录，semnav 就会起不来，
而这是两个应当彼此独立的项目。

**修法**：把所需资产**复制**进本仓库（不是引用），包名 `oomwoo_one` → `semnav_robot`：

| 项 | 处理 |
| --- | --- |
| URDF/xacro（5 个文件 895 行）+ `gz_bridge.yaml` | 复制进 `src/semnav_robot/` 并改名，附 `LICENSE` + `NOTICE`（Apache-2.0 合规，记录出处与改动） |
| `setup_env.sh` | 删掉 source 外部工作区与 `OOMWOO_WS`，只 source ROS + 自身 |
| `sim.launch.py` | `ROBOT_MODEL = 'semnav_robot'`；去掉外部模型库路径 |
| `package.xml` | `exec_depend` 两个外部包 → `semnav_robot` + `semnav_models` |
| 注释/文档 | 5 处提及改为出处说明，不再暗示依赖 |

**顺带修正一处既有 bug**：上游 `gz_bridge.yaml` 里两条碰撞话题把 world 名
**硬编码为 `default`**，而 semnav 的 world 名是 `semnav_apartment` ——
该桥此前**收不到数据且不报错**（gz 接触传感器忽略 `<topic>`，只发到自动作用域名）。
因 semnav 不用 bumper 话题，此前未暴露。已随复制一并改为 `semnav_apartment` / `semnav_robot`。

**反证测试（证明真解耦，而非只看 grep）**：临时把外部工作区目录改名使其不可用，
清空 ROS 环境后从零 `source` + `colcon build` + 起完整仿真栈 + 跑 6 目标验收：

| 检查项 | 结果 |
| --- | --- |
| 3 个包能否被找到 | ✓ 全部找到（外部包如预期不可见） |
| 从零构建 | ✓ 3 包 2.95 s 通过 |
| xacro 独立展开 | ✓ 21480 字节，`<robot name="semnav_robot">`，7 个传感器 |
| 仿真栈起栈 | ✓ gz 4 进程，world 加载，机器人生成 |
| 传感器数据 | ✓ `/scan` 3.66 Hz、`/odom` 正常 |
| **6 目标验收** | **✓ 6/6 = 100%**，331.0 s，定位误差均值 0.032 m |
| 碰撞桥路径 | ✓ 日志显示 `/world/semnav_apartment/model/semnav_robot/...` 已匹配 |

测试后恢复目录名，外部工作区**改动 0 行**、HEAD 未变。

另有一个可重跑的回归工具 `verify_selfcontained.sh`：从 git 提交**克隆到临时目录**，
在不 source 任何外部工作区的前提下构建，并检查三包可见性、xacro 独立展开、
构建期产物、全仓库外部引用扫描。以后若又引入外部依赖，跑它即可发现。

### 六、M4 验收缺口（2026-10-22 已闭环）

- [x] **rosbag 原始数据** —— `phase1_nav_v9.py` 默认录制 `/odom` `/scan`
      `/amcl_pose` `/cmd_vel` `/tf` `/tf_static` `/initialpose` `/plan`
      `/local_costmap/costmap` 到 `bags/nav_v9_<stamp>/`
- [x] **轨迹图** —— 由 rosbag 的 `/odom` 可直接还原；`dtg_curve` 亦在 JSON 内
- [x] **SR / SPL / DTG** —— 三项齐备。SR 与 SPL 按 ObjectNav 口径；
      SPL = L*/max(L*, L)，L* 用 A* 在**可通行格**（含车体半径膨胀）上求；
      DTG 为 0.5 s 采样的 `[时间, 剩余直线距离]` 曲线（JSON 的 `dtg_curve`）
- [x] `git commit` 可追溯 —— **仍未提交**，见下方「遗留」

> ⚠️ **SPL 口径陷阱（首轮踩过，已修）**：L* 的起点必须是**本段起点**
> （上一目标的实际终点），不能统一用出生点。首版统一从 spawn 算，而
> `path_len` 是从上一目标终点累计的，两者起点不一致 → 除第一个目标外 SPL
> 全部失真：整体从真实的 0.828 被压到 0.630，餐厅/客厅从 1.000/1.000
> 被算成 0.248/0.164。

### 七、A 模式导航基线：三轮调优到 SR 100%（2026-10-21/22）

**决策**：把 `maps/apartment_v9_furnished_map.yaml` 当作**先验地图**，
不再跑 SLAM 建图（记为「A 模式」= 已知地图 + AMCL + Nav2）。
理由：该图是 CAD/truth 渲染产物（`burn_furniture.py` 从
`apartment_empty_reference_map.yaml` + `apartment_truth.json` 生成，
md5 与重渲染结果逐字节一致），本身就是真值，重跑 SLAM 只会引入误差；
且每轮迭代省 5~20 分钟建图时间，算力留给语义层。说明书 §9 的范围也是
语义探索而非几何探索。

**最终基线（第三轮，`experiments/phase1_nav_v9_20261022_004357.md`）**

| 指标 | 值 |
|---|---|
| SR | **100.0%（13/13）** |
| SPL | **0.988** |
| 全程耗时 | 1018.0 s |
| 终点误差均值 | 0.152 m（最大 0.175） |
| AMCL 偏差（rosbag 独立核验） | 平均 0.029 m，最大 0.345 m，未发散 |

13 个目标全部 `SUCCEEDED`，且经三重核验：DTG 起点与本段起点口径自洽
（差 ≤0.006 m）、DTG 最小值 ≤0.25 m、终点 ≤0.25 m。

#### 三轮演进（每轮失败都定位到了具体缺陷）

| 轮次 | SR | SPL | 耗时 | 主要缺陷 |
|---|---|---|---|---|
| 首轮 | 84.6% | 0.630（口径错，实为 0.828） | 1713 s | SPL 起点错、RPP 窄门碰撞、CPU 饥饿 |
| 二轮 | 61.5% → 崩 | — | 1746 s+ | AMCL 发散、环境负载 17、脚本两处 bug |
| **三轮** | **100%** | **0.988** | **1018 s** | — |

#### 缺陷 1：SPL 口径 —— L* 必须按「本段起点」算

首版 L\* 统一从出生点算，而 `path_len` 是「上一目标终点 → 本目标」的累计，
两者起点不同。后果：除厨房外 12 个目标的 SPL 全部失真，
整体被压到 0.630（真实 0.828），餐厅/客厅从 1.000/1.000 变成 0.248/0.164。
修法：每段开跑前用 `leg_start` 现算 L*，并把 `leg_start` 记进结果 JSON。

#### 缺陷 2：RPP 在窄门处反复报「碰撞」→ patience exceeded

日志实证：`RegulatedPurePursuitController detected collision ahead!` × 5
→ `Controller patience exceeded` → `Aborting handle`。
几何量化：`door_kitchen_arm` / `door_bath` 名义宽 0.81/0.80 m，
但扣除墙体膨胀后**有效通行宽度只有 0.55 m**，车体直径 0.349 m，
居中后每侧仅余 0.10 m；而 `max_lookahead_dist: 0.55` 让前视点落到门框外。
修法（缩小尺度，**不关保护**）：
`max_lookahead_dist 0.55 → 0.35`、`regulated_linear_scaling_min_radius 0.5 → 0.35`；
`use_collision_detection` 保持 `true`。效果：主卧从
`FAILED/92.4 s/距 7.889 m` 变为 `SUCCEEDED/80.3 s/距 0.136 m`。

另发现一处真实几何缺陷：`door_bath_balcony`（西侧阳台）名义 0.80 m，
**有效宽度仅 0.30 m < 车体 0.349 m**，物理上过不去。当前靠绕行规避，
属待改场景问题（见遗留）。

#### 缺陷 3：目标点选在「定位歧义区」—— 最容易误判的一条

首版选点只按「房间内净空最大」，几何全可达（`check_reachability` 13/13），
但实测 13 个目标里 5 个落在定位歧义区：

| 目标 | 360° 歧义度 | 平均测距 |
|---|---|---|
| 次卧二 | 0.511 | 0.96 m |
| 厨房 | 0.797 | 1.33 m |
| 主卧 | 0.943 | 1.57 m |
| 卫生间 | 0.970 | 1.51 m |
| 对照：出生点 | 1.422 | 3.03 m |

**歧义度**定义：该点 72（或 24）个朝向下，理想 raycast 激光两两的平均 L1 差；
低 = 各朝向看起来一样 = AMCL 分不出朝向。

因果链（有 rosbag 实证）：卫生间段机器人在 DTG **0.286 m** 处
**死锁 165 s 纹丝不动**（判据 0.25 m，差 3.6 cm）；期间 AMCL 粒子坍缩，
`odom(3.38,5.40)` 被定位成 `amcl(3.47,1.34)`——偏差 **4.06 m**，
且该误判点落在主卧家具内部（开阔度 0.00，激光全被堵死，永不自我纠正）。
之后 4 个目标全 FAILED。

修法：重选 13 个目标点，判据改为「净空 ≥0.55 m 硬约束 + 歧义度最大化」。
效果：卫生间 0.970→1.617、厨房 0.797→1.431、客厅 1.245→2.017、
主卧 0.943→1.349、次卧一 0.988→1.428。
仍偏低的两处是**几何天花板**（房间小、三面实墙），非选点不当：
次卧二 0.639（3.19×3.02 m）、西侧阳台 0.595（1.53×2.18 m）。

#### 缺陷 4：AMCL recovery —— 试了，无效，已回退（反面结论）

曾开 `recovery_alpha_slow=0.001 / fast=0.1 / do_beamskip=true`
（专为窄空间激光退化设计的机制，原配置全关）。
**实测无效且更差**：卫生间从「超时 315.6 s / 距 0.286 m」
退化为「FAILED 147.8 s / 距 0.343 m」，后续 3 目标仍全失败。
已回退并在 `nav2_params.yaml` 注释里记录结论：
根因不是粒子恢复能力不足，而是目标点本身歧义 —— 调 AMCL 治不了。

#### 缺陷 5：CPU 饥饿 —— 所有轮次结果波动的真正主因

两个叠加的负载源，都不是导航算法问题：

1. **孤儿进程**：`bash -s`（父 `bash -c bash -s 2>&1 | tr -d '\r'`）
   从 13:03 起**独占 99.4% CPU 达 7.5 小时**。来源是早期某次
   `ssh ... "bash -s" | tr -d '\r'` 调用——管道断开但进程卡在 stdin 上死循环。
2. **headless 仍起 GUI**：`sim.launch.py` 的 `gz_args` 恒为
   `-r -v 4 --render-engine ogre <world>`，无 `-s`，故 `headless:=true`
   也会起 `gz sim gui`，实测占 **87.9% CPU**。

两者叠加使 4 vCPU 的 VM `load average` 长期 14~17。后果是
`Controller patience exceeded` 频发、目标莫名超时、每轮耗时递增
（1713 s → 1746 s → 1288 s 超时）。

修法：
- 加 `server_only = '-s ' if headless else ''` 拼进 `gz_args`。
  **注意与 `--headless-rendering` 的区别**（文件头已有实测定论：后者让
  `gpu_lidar` 的 360 条射线全读 `inf`）。`-s` 只是不起 GUI 进程，
  server 端渲染管线完整。实测验证：`/scan` 360 束、有限 355、inf 5、
  mean 3.641 m —— **雷达正常**。
- 清场脚本追加 `pkill -9 -f 'bash -c bash -s'`，防同类孤儿再积累。

效果：`load average 16.99 → 4.41`；同一目标耗时显著下降
（厨房 71 s→44 s、餐厅 105 s→61 s、主卧 228 s→127 s），全程 1713 s→1018 s。

#### 缺陷 6：验收脚本自身两个 bug

- `KeyError: 'spl'` —— 「目标被拒」分支没写 `spl` 字段，报告生成处无条件读取。
  连带发现该分支被写成了两条 `results.append`（重复记录）。
- **超时后仍长跑 1288 s**（设定 300 s）—— 根因是 `nav.getResult()`
  **无界阻塞**：`cancelTask()` 后只等 15 s 就 `break`，但 BT 在窄处仍在跑
  恢复行为，`getResult()` 卡了约 970 s。
  修法：超时路径改用带 deadline 的轮询（检查 `result_future.done()`），
  拿不到就按 `CANCELED` 记录并继续下一目标，不让单点拖死整轮。

#### 遗留（未做）

- **`git commit` 未做**：72 项改动仍工作区态，HEAD 停在 `4e33afd`（2026-10-12）
- `door_bath_balcony` 有效宽 0.30 m < 车体 0.349 m，是真实几何缺陷，
  当前靠绕行规避，应改场景
- `e2e_phase1.sh` 仍是建图期流程（删并重生成 `maps/apartment.*`），
  A 模式下不可用，README 已标注
- 阶段二前置：`semnav_robot` 无 depth camera（只有双目 RGB + 16×8 前向 ToF），
  `enable_cameras` 默认 `false`

### 八、下一步

按计划书第三阶段表，下一阶段是**第二阶段 语义建图**（房间、物体和位置可视化）。

**前置依赖（须先确认）**：计划书 8.2「物体定位路线」要求通过 **RGB-D 深度**估计物体相对位置，
但 `semnav_robot` 目前只有**双目 RGB**（`type="camera"`，`R8G8B8`），**没有 depth/rgbd sensor**，
且 `enable_cameras` 默认 `false`；另有一个前向 ToF（16×8 `gpu_lidar`）但过于稀疏。
阶段二开工前需在本仓库的 `semnav_robot` 里新增 depth camera 传感器。

另：**frontier 自主探索属第四阶段**，不提前拉入。
