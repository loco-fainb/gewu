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

### 四、解耦：从"复用外部工作区"改为"自包含"

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

### 五、未完成 / 待补（属 M4 验收缺口）

- [ ] **rosbag 原始数据** —— M4 验收物要求三项，目前只满足"误差/耗时表"一项
- [ ] **轨迹图** —— 现有 `frames_*.pdf` 是 TF 帧关系图，**不是**机器人轨迹图
- [ ] **SR / SPL / DTG** —— 开工方案第 6.2 节要求用 ObjectNav 标准指标而非自造指标。
      当前只记 `success_rate` / `elapsed_s` / `final_dist_m`；**SPL 完全无法计算**
      （未记录逐目标实际路径长度），DTG 只记终点未记过程曲线
- [ ] `git commit` 可追溯 —— 本次建仓后满足

### 六、下一步

按计划书第三阶段表，下一阶段是**第二阶段 语义建图**（房间、物体和位置可视化）。

**前置依赖（须先确认）**：计划书 8.2「物体定位路线」要求通过 **RGB-D 深度**估计物体相对位置，
但 `semnav_robot` 目前只有**双目 RGB**（`type="camera"`，`R8G8B8`），**没有 depth/rgbd sensor**，
且 `enable_cameras` 默认 `false`；另有一个前向 ToF（16×8 `gpu_lidar`）但过于稀疏。
阶段二开工前需在本仓库的 `semnav_robot` 里新增 depth camera 传感器。

另：**frontier 自主探索属第四阶段**，不提前拉入。
