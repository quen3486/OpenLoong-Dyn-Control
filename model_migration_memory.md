# OpenLoong 机型迁移记忆（重构版）

更新时间：2026-04-08  
当前目标：在保留 `AzureLoong` 全量链路和 demo 的前提下，新增 `ref/v4` 的 `speedbot`（以下称 `speedbot_v4`）并支持 A/B 对照。

---

## 0. 文档用途与边界

1. 这是“迁移执行记忆”，不是历史流水日志。
2. 只保留可复用结论：`模型/初始化/接触定义/动力学尺度/MPC权重/低层控制接口` 六层。
3. 迁移原则：
   - 不替换现有 `AzureLoong` 文件与 demo。
   - 所有 speedbot 改动走并行新增（新文件、新目标、新配置）。
   - 先最小改动打通，再做性能调优。

---

## 1. 前序迁移复盘（speed160）可复用结论

### 1.1 成功经验

1. **并行实现而非覆盖**：新增 `*_speed160.*` 接口/控制文件，避免影响 Azure。
2. **先站稳再行走**：先做站立稳定和状态链路正确性，再开步态。
3. **接触体独立设计**：足底使用规则几何碰撞体 + touch 传感器，而非直接 mesh 碰撞。
4. **自动闭环有效**：采用“实验 -> 指标分析 -> 定向调参 -> 回归”的脚本化循环。
5. **长时程测试必须单列**：10s 通过不代表 20~30s 稳定。

### 1.2 高风险坑位

1. 关节名称/顺序错位导致控制输出错轴。
2. `DataBus` 基座状态写入不完整导致 `updateQ()` 问题。
3. 触地定义不一致导致步态切换条件异常（`phi` 卡死或切换抖动）。
4. 直接大改 MPC/WBC 核心逻辑会引入跨机型回归。
5. GUI 手感与无 GUI bench 不一致时，要先排查“二进制版本/参数注入/按键链路”。

---

## 2. speedbot_v4 现状证据链（用于迁移前基线）

> 数据来源：`ref/v4/urdf/speedbot/speedbot_v4.urdf`、`ref/v4/speedbot/speedbot_config.py`、`models/AzureLoong.xml`。

### 2.1 模型层（Model）

1. `speedbot_v4.urdf` 结构统计：
   - `joint_total=30`
   - `revolute=12`（双腿各 6）
   - `fixed=18`（含腰/臂/足端标记等）
2. 腿部 12 个控制关节名称与 `speed160` 风格一致（`left_hip_roll_joint` ... `right_ankle_roll_joint`）。
3. 从 URDF inertial 累计质量：`total_mass=59.11532 kg`。

### 2.2 初始化层（Initialization）

1. 参考 `speedbot_config.py`：
   - 初始基座高度：`z=1.003`
   - 典型腿部初始角：`hip_pitch=-0.26, knee=-0.52, ankle_pitch=0.26`。
2. 该初值可作为 OpenLoong demo 的 `qIni/qStand` 初始模板。

### 2.3 接触定义层（Contact Definition）

1. Azure 经验：`models/AzureLoong.xml` 中双脚使用 `box + condim=4 + touch site(sensor)`。
2. `speedbot_v4.urdf` 已带足端固定标记链路：
   - `left/right_foot_front/mid/hind_joint`
   - 偏移分别约 `0.15 / 0.05 / -0.06`（x），`z=-0.052`
3. `speedbot_v4.urdf` 的 ankle_roll 已含部分 box collision 片段，可作为足底碰撞设计参考。

### 2.4 动力学尺度层（Dynamics Scale）

1. 质量尺度与 Azure 不同，不能直接沿用同一前馈假设。
2. 关节 effort/velocity limit 与 OpenLoong 当前控制输出能力未必匹配，需保守对齐。

### 2.5 MPC 权重层（MPC Weights）

1. speed160 经验：先用稳定保守权重，再逐步放开速度和步态激进参数。
2. 首轮不改算法结构，只做机型级参数表和开关。

### 2.6 低层控制接口层（Low-Level Interface）

1. 需新增 speedbot 专用关节映射和 PVT 参数，不复用 Azure 的 31DoF 逻辑。
2. 必须验证 `JointName/motorName/json配置/URDF关节顺序` 四者一致。

---

## 3. speedbot_v4 六层最小修改方案（兼容 Azure）

## 3.1 模型层

最小修改：

1. 新增目录 `models/speedbot_v4/`，放置 URDF/XML/meshes，不改 `models/AzureLoong*`。
2. 从 `ref/v4/urdf/speedbot/speedbot_v4.urdf` 生成 MuJoCo XML 与 scene。
3. 保留 visual geom；碰撞体按“足底规则体优先”落地。

不做事项：

1. 不覆盖 Azure 模型文件。
2. 不在首轮引入大规模 mesh 碰撞调参。

验收：

1. `mj_loadXML` 成功。
2. `nq/nv/nu/nsensor` 与预期一致（12 电机）。

## 3.2 初始化层

最小修改：

1. 新增 speedbot demo 初始化参数（`qIni/qStand`）对齐 `speedbot_config.py`。
2. 增加 `openLoopCtrTime` 保护窗口，先站稳再切控制闭环。

不做事项：

1. 不在首轮引入复杂状态估计重构。

验收：

1. 无外部命令下稳定站立 10s。
2. 切换到闭环后无 2~3s 内突发发散。

## 3.3 接触定义层

最小修改：

1. 在 XML 明确左右足底主碰撞体（box/capsule），设置 `condim`、摩擦、`margin`。
2. 增加 `lf/rf` touch site + sensor。
3. 补齐相邻连杆 `contact exclude`，避免自碰撞噪声。

不做事项：

1. 不依赖脚部复杂网格直接接触地面作为主接触。

验收：

1. 触地信号在双支撑/单支撑切换时有清晰变化。
2. 原地踏步测试不出现持续抖脚“粘地”。

## 3.4 动力学尺度层

最小修改：

1. 为 speedbot 单独配置质量/惯量相关参数入口（默认保守，不改 Azure 默认值）。
2. 必要时提供环境变量开关启用机型质量注入（与 speed160 做法一致）。

不做事项：

1. 首轮不重写 MPC 动力学方程。

验收：

1. 开关关闭时不影响 Azure。
2. 开关开启后 speedbot 不出现明显提前失稳。

## 3.5 MPC 权重层

最小修改：

1. 复制一套 speedbot 参数组（`Q/R`、步态时序、步高、速度限幅）。
2. 先保守：低速、小步长、较长摆动时间。
3. 只调参数，不改求解器主流程。

不做事项：

1. 不把 speed160 调好的数值直接硬套。

验收：

1. 低速行走（`|vx|<=0.06`）可连续 10s。
2. `qp status` 长时无连续异常。

## 3.6 低层控制接口层

最小修改：

1. 新增并行文件：
   - `sim_interface/MJ_interface_speedbot_v4.*`
   - `common/PVT_ctrl_speedbot_v4.*`
   - `common/joint_ctrl_config_speedbot_v4.json`
   - `algorithm/pino_kin_dyn_speedbot_v4.*`
   - `algorithm/wbc_priority_speedbot_v4.*`
2. 新增 demo 目标：
   - `demo/walk_wbc_speedbot_v4_joystick.cpp`
   - `demo/walk_mpc_wbc_speedbot_v4_joystick.cpp`
   - `demo/bench_walk_mpc_wbc_speedbot_v4.cpp`
3. `CMakeLists.txt` 仅新增 target，不修改 Azure 目标。

不做事项：

1. 不在 Azure 专用类中混入 speedbot 分支判断。

验收：

1. speedbot 可独立编译运行。
2. Azure 原 demo 与 bench 行为不变。

---

## 4. 自动闭环验证计划（必须执行）

## 4.1 闭环流程

1. 实验：按固定场景批量运行。
2. 分析：自动输出统一指标（构型无关）。
3. 调整：每轮仅改 1~2 类参数（避免耦合污染）。
4. 验证：回归 baseline + 新参数，对比同指标。
5. 循环：直到满足退出条件。

## 4.2 场景分级（由易到难）

1. `stand`：静态站立。
2. `single`：单腿承重短窗口验证（自然切回双支撑）。
3. `inplace`：原地踏步。
4. `vslow`：极低速行走。
5. `walk`：常速行走。
6. `fast`：较高速行走（最后验证）。

## 4.3 核心指标（建议沿用 speed160 bench 框架）

1. 稳定性：`stable`, `fall_time`, `min_base_z`, `roll/pitch RMS`。
2. 有效行走：`x_travel`, `vx_abs_mean`, `step_peak_mean`, `leg_switches`。
3. 接触质量：`stance_slip_rms`, `touch时序一致性`。
4. 求解健康：`qp_bad`, `mpc_qp_bad`。

## 4.4 通过标准

1. 短时门槛：`stand/single/inplace/vslow/walk` 均 `stable=1 && walk_effective=1`（10s）。
2. 长时门槛：`walk` 连续 30s 不摔倒，且 `x_travel` 持续增长。
3. A/B 门槛：Azure 所有基线场景无回归。

---

## 5. 执行顺序（最小风险）

1. **阶段 A（模型+初始化）**：只打通加载与静态站立。
2. **阶段 B（接触+低层接口）**：打通触地、WBC、PVT 链路。
3. **阶段 C（MPC参数）**：低速步行可用。
4. **阶段 D（自动闭环）**：10s 全场景通过。
5. **阶段 E（长时程）**：30s 行走 + Azure A/B 回归通过。

---

## 6. 最小改动清单（供实施时逐项打勾）

1. `[ ]` 新增 `models/speedbot_v4/*`，不改 Azure 模型文件。
2. `[ ]` 新增 speedbot_v4 专用接口与控制文件（命名并行）。
3. `[ ]` 新增 speedbot_v4 三个 demo/bench 目标。
4. `[ ]` 新增 speedbot_v4 关节控制 json。
5. `[ ]` 新增/复用自动闭环脚本入口（支持 speedbot_v4 profile）。
6. `[ ]` 完成六场景 10s 回归与 30s walk 回归。
7. `[ ]` 完成 Azure A/B 回归并记录结果。

---

## 7. 迁移时的硬约束（再次强调）

1. 不删除、不替换 Azure 现有文件与 demo。
2. 不在主干类里堆大量 `if(robot_type)` 分支；优先并行类。
3. 每轮改动必须可回滚、可复现实验数据。
4. 先证据后调参：没有指标数据不做“感觉式”修改。

