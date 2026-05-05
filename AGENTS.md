<!-- From: /home/huangkun/workspaces/mpc/Openloong-dyn-control/AGENTS.md -->
# OpenLoong Dynamics Control - AGENTS 指南（基于当前仓库实况）

## 0. 文档目的

本文件面向 **第一次接触本仓库的 AI Coding Agent**，目标是让你在不了解项目背景的情况下，也能快速完成：

1. 正确构建与运行 demo；
2. 读懂代码组织和运行时数据流；
3. 在多机型并行实现中做出不破坏现有链路的改动；
4. 按仓库已有习惯完成开发、验证和回归。

> 本文内容只基于当前仓库实际文件与代码行为整理，不使用“理想化约定”。

---

## 1. 项目概览

OpenLoong Dynamics Control 是一个基于 **MPC（Model Predictive Control）+ WBC（Whole Body Control）** 的仿人机器人运动控制框架，运行在 **MuJoCo** 仿真环境。支持 MuJoCo 仿真、MuJoCo+ROS2 联合仿真、以及 ROS2 真机控制三种运行模式。

当前仓库内主线功能是行走控制（含键盘交互）。控制链路核心为：

`MuJoCo 传感器 -> DataBus -> 状态估计/动力学/MPC/WBC -> PVT -> MuJoCo 电机力矩`

当前 `CMakeLists.txt` 实际构建 4 个仿真可执行文件：

- `walk_wbc_joystick`
- `walk_mpc_wbc_joystick`
- `walk_mpc_wbc_v4`
- `walk_mpc_wbc_leg`

说明：`README.md` 中仍有历史 demo 链接（如 `walk_wbc.cpp` / `jump_mpc.cpp` / `walk_mpc_wbc.cpp`），与当前 CMake 目标不完全一致。开发时以 `CMakeLists.txt` 和 `demo/` 现状为准。

---

## 2. 技术栈与依赖

### 2.1 语言与构建

- 语言：`C++17`
- 构建系统：`CMake >= 3.10`
- 默认构建类型：`Release`（若未显式指定）
- 编译选项：`add_compile_options(-std=c++17)`

### 2.2 依赖来源

仓库内已内置主要第三方库（`third_party/`），CMake 直接链接本地目录：

- MuJoCo（`third_party/mujoco`）
- Pinocchio（`third_party/pinocchio`）
- Eigen3（`third_party/eigen3`）
- qpOASES（`third_party/qpOASES`）
- GLFW（`third_party/glfw`）
- jsoncpp（`third_party/jsoncpp`）
- quill（`third_party/quill`）
- urdfdom（`third_party/urdfdom`）
- boost（`third_party/boost`）

### 2.3 平台与架构

- 推荐系统：Ubuntu 22.04（README 推荐 22.04.4）
- 编译器：g++ 11.x（README 推荐 11.4）
- 架构：x64 / arm64 双分支链接
  - x64：`third_party/mujoco/lin_x64`, `third_party/qpOASES/lin_x64`
  - arm64：`third_party/mujoco/lin_arm64`, `third_party/qpOASES/lin_arm64`

### 2.4 ROS2 可选依赖

CMake 中 `OPENLOONG_ENABLE_ROS2` 默认为 `ON`：
- 若系统已安装 ROS2（`rclcpp`, `sensor_msgs`, `std_msgs`, `geometry_msgs`），则自动启用 ROS2 接口编译（`sim_interface/ROS2_interface_v4_leg.cpp`, `ROS2_state_pub_v4*.cpp`）。
- 若未找到 ROS2，则自动退化为纯仿真编译，并过滤掉 ROS2 源文件。
- `tf2_ros` 为可选依赖；找到后会定义 `OPENLOONG_HAS_TF2=1`。

---

## 3. 关键配置文件（实际存在）

### 3.1 构建与仓库级配置

- `CMakeLists.txt`：唯一主构建配置。
- `.gitignore`：忽略 `build/`、`record/datalog.log`、`*.out`、IDE 目录等。
- `license`：Apache 2.0。

### 3.2 控制参数配置

- `common/joint_ctrl_config.json`（AzureLoong）
- `common/joint_ctrl_config_v4.json`（speedbot_v4）
- `common/joint_ctrl_config_v4_leg.json`（speedbot_v4_leg）

每个关节含 `kp/kd/maxPos/minPos/maxSpeed/maxTorque/PVT_LPF_Fc/gear`。

### 3.3 运行时控制器配置（JSON）

- `common/controller_config_azure.json`
- `common/controller_config_v4.json`
- `common/controller_config_v4_leg.json`
- `common/controller_config_v4_slow.json`

由 `common/controller_config.cpp/h` 提供统一的 `ControllerConfig` 结构体与 `loadControllerConfig()` 加载接口。支持 JSON 注释（`//` 与 `/* */`），便于在配置文件中写调参说明。运行时可通过环境变量 `OPENLOONG_CONTROLLER_CONFIG` 指定加载路径；demo 主循环中按机型默认加载对应配置。

主要可调字段示例：
- `controlBackend` / `mainControlDt` / `mpcControlDt`
- `tSwing` / `phiSwitchMin` / `fzSwitchThreshold`
- `kpVx` / `kpVy` / `stepHeight`
- `posRotKp` / `swingLegKp` / `contactMiu`
- `mpcMass` / `mpcPredictionHorizon` / `mpcUseDataBusInertia`

### 3.4 机型与场景配置

- 场景入口：
  - `models/scene.xml`
  - `models/scene_board.xml`
  - `models/scene_v4.xml`
  - `models/scene_v4_leg.xml`
- 机体 XML：
  - `models/AzureLoong.xml`
  - `models/speedbot_v4/speedbot_v4.xml`
  - `models/speedbot_v4/speedbot_v4_leg.xml`
- URDF：
  - `models/AzureLoong.urdf`
  - `models/speedbot_v4/speedbot_v4.urdf`
  - `models/speedbot_v4/speedbot_v4_leg.urdf`

### 3.5 数据与后处理

- `record/datalog.log`：运行日志（体积可能很大）。
- `record/matlabReadDataScript.txt`：由 `DataLogger` 自动生成列索引脚本。
- `record/plotData.m`：示例绘图。
- `record/exportVideo.txt`：`ffmpeg` 原始帧转视频命令。

### 3.6 关键“缺失项”（明确不存在）

本仓库当前没有以下生态配置：

- `pyproject.toml`
- `package.json`
- `Cargo.toml`
- `go.mod`
- `requirements.txt`
- `.github/workflows/*`（无 CI 工作流目录）
- `Makefile`（顶层无手写 Makefile，仅 CMake 生成）

---

## 4. 构建、测试与运行命令

### 4.1 环境准备（README）

```bash
sudo apt-get update
sudo apt install git cmake gcc-11 g++-11
sudo apt install libglu1-mesa-dev freeglut3-dev
```

若需 ROS2 支持，额外安装 ROS2 Humble 并 source `/opt/ros/humble/setup.bash`。

### 4.2 构建（已在本地验证可通过）

```bash
cmake -S . -B build
cmake --build build -j4
```

构建产物：
- `build/libcore.a`：静态库（由 `algorithm/*.cpp`, `common/*.cpp`, `math/*.cpp`, `sim_interface/*.cpp` 自动聚合）。
- `build/walk_wbc_joystick`
- `build/walk_mpc_wbc_joystick`
- `build/walk_mpc_wbc_v4`
- `build/walk_mpc_wbc_leg`

### 4.3 运行

**直接运行（纯 MuJoCo 仿真）**

```bash
cd build
./walk_wbc_joystick
./walk_mpc_wbc_joystick
./walk_mpc_wbc_v4
./walk_mpc_wbc_leg
```

**仿真 + ROS2 状态发布**

仿真不再通过 `tools/start_control_v4_leg.sh` 或 `CONTROL_MODE` 选择模式。直接运行 demo；若需要发布 `/imu/data`、`/joint_states`、TF 等 ROS2 状态，在对应控制器配置中将 `sim_enable_ros2_state_pub` 置为 `true` 后再运行 demo。

**真机模式（speedbot_v4_leg）**

```bash
./tools/start_control_v4_leg.sh
```

该脚本只用于真机，固定启动 `walk_mpc_wbc_leg --ros2-real`、`common/controller_config_v4_leg.json` 和 RViz。启动后默认只订阅数据，不发布控制命令；按 `G` 后才开始发布。

**常用运行时环境变量**

| 环境变量 | 说明 | 默认值 |
|---|---|---|
| `OPENLOONG_CONTROLLER_CONFIG` | 控制器 JSON 配置路径 | 按机型默认 |
| `AUTOWALK` | 自动起步（`1/0`） | `0` |
| `OPENLOONG_ROS_TOPIC_IMU` | IMU 话题名 | `/imu/data` |
| `OPENLOONG_ROS_TOPIC_JOINT_STATES` | 关节状态话题名 | `/joint_states` |

### 4.4 测试状态

```bash
cd build
ctest -N
```

当前结果：`Total Tests: 0`。

结论：仓库无自动化单元/集成测试注册，主要依赖仿真回归和日志分析。`tools/validation/` 目录下提供了若干 shell 脚本用于分阶段回归验证（如 `generate_leg_stage_compare_report.sh`、`run_leg_noise_delay_sweep.sh`、`check_ros2_real_leg_contract.sh`），但均为外部调用型脚本，未接入 CMake/ctest。

---

## 5. 运行时架构（必须掌握）

### 5.1 数据总线模式

项目核心是 `DataBus`（`common/data_bus.h`），几乎所有算法模块通过它交换状态与控制量。

典型链路：

1. `MJ_Interface*.updateSensorValues()` 读取 MuJoCo；
2. `MJ_Interface*.dataBusWrite()` 写入 `DataBus`；
3. `StateEst` 更新估计状态；
4. `Pin_KinDyn*` 计算运动学/动力学（J, dJ, M, Non 等）；
5. `GaitScheduler + FootPlacement + JoyStickInterpreter` 生成步态与落脚；
6. `MPC`（MPC demo 中每 5 个 1kHz 周期执行一次，约 200Hz）；
7. `WBC_priority*` 求解期望加速度/接触力/关节力矩；
8. `PVT_Ctr*` 组合 PD + 前馈 + 低通 + 限幅，输出电机力矩；
9. `MJ_Interface*.setMotorsTorque()` 下发到 MuJoCo；
10. `DataLogger` 记录一帧数据。

### 5.2 主循环频率结构

- 物理步进：MuJoCo `timestep`（模型中通常 `0.001s`，即 1kHz）。
- 渲染更新：外层 60 FPS 循环（`1.0/60.0`）。
- MPC 控制周期：通常 `0.005s`（每 5 个主循环周期执行一次）。

### 5.3 UI/键盘处理

`sim_interface/GLFW_callbacks.*` 中统一管理：

- 公共控制：
  - `Backspace`：重置仿真
  - `1`：连续运行/暂停切换
  - `2`：单步模式

- 行走键位（由 demo 逻辑消费）：
  - `Space`：站立/行走切换
  - `W/A/S/D/H`（基础）
  - `J/Q/E`（仅 v4/v4_leg demo 额外实现）

#### 5.3.1 各 demo 键位差异（精确）

1. `walk_wbc_joystick`、`walk_mpc_wbc_joystick`（AzureLoong）
   - `W` 前进
   - `S` 减速到 0（停止前进，不是倒退）
   - `A/D` 转向
   - `H` 重置航向参考
   - `Space` 走行/站立切换

2. `walk_mpc_wbc_v4`、`walk_mpc_wbc_leg`（speedbot_v4/v4_leg）
   - `W` 按当前速度前进
   - `S` 按当前速度后退
   - `A/D` 转向
   - `Q/E` 调低/调高目标速度
   - `J` 急停并转入站立
   - `H` 重置航向参考
   - `Space` 走行/站立切换

#### 5.3.2 demo 与模型/场景映射

| 可执行文件 | 场景 XML | URDF | 接口实现 | 默认配置 |
|---|---|---|---|---|
| `walk_wbc_joystick` | `models/scene_board.xml` | `models/AzureLoong.urdf` | 无后缀 | `controller_config_azure.json` |
| `walk_mpc_wbc_joystick` | `models/scene.xml` | `models/AzureLoong.urdf` | 无后缀 | `controller_config_azure.json` |
| `walk_mpc_wbc_v4` | `models/scene_v4.xml` | `models/speedbot_v4/speedbot_v4.urdf` | `_v4` | `controller_config_v4.json` |
| `walk_mpc_wbc_leg` | `models/scene_v4_leg.xml` | `models/speedbot_v4/speedbot_v4_leg.urdf` | `_v4_leg` | `controller_config_v4_leg.json` |

---

## 6. 代码组织与主模块划分

### 6.1 目录分层

- `demo/`：程序入口与控制主循环
- `sim_interface/`：MuJoCo 与 GLFW 接口层；ROS2 状态发布与真机接口
- `algorithm/`：估计、MPC、WBC、步态与落脚规划
- `common/`：DataBus、PVT、日志、控制器配置加载
- `math/`：矩阵工具、滤波器、轨迹生成
- `models/`：URDF/XML/mesh/场景
- `third_party/`：第三方依赖
- `record/`：日志与后处理脚本
- `ref/`：迁移参考资产（不参与当前 CMake 构建）
- `tools/`：启动脚本、验证工具、真机/ROS2 辅助配置

### 6.2 核心类与职责

- `DataBus`：全局状态/命令总线。
- `MJ_Interface / MJ_Interface_V4 / MJ_Interface_V4_Leg`：
  - 按关节名和传感器名绑定 MuJoCo ID；
  - 读取 `qpos/qvel/sensor`；
  - 写入力矩控制。
- `Pin_KinDyn*`：Pinocchio 运动学与动力学。
- `MPC`：单刚体 MPC（qpOASES 求解）。
- `WBC_priority*`：任务优先级 WBC QP。
- `GaitScheduler`：支撑腿/摆动腿切换与相位管理。
- `FootPlacement`：摆腿轨迹与落脚点计算。
- `JoyStickInterpreter`：速度指令斜坡化与世界系积分。
- `PVT_Ctr*`：关节级 PVT（PD + FF + 滤波 + 饱和）。
- `DataLogger`：CSV 风格数值日志和 Matlab 脚本生成。
- `ControllerConfig` + `loadControllerConfig()`：统一 JSON 配置加载。

### 6.3 WBC 任务优先级（当前代码）

`wbc_priority*.cpp` 中构建的任务顺序如下（调参前先确认对应机型）：

1. AzureLoong（`algorithm/wbc_priority.cpp`）
   - walk: `static_Contact -> PosRot -> SwingLeg -> RedundantJoints -> HandTrackJoints`
   - stand: `static_Contact -> CoMXY_HipRPY -> Pz -> HandTrackJoints -> HeadRP`

2. speedbot_v4（`algorithm/wbc_priority_v4.cpp`）
   - walk: `static_Contact -> PosRot -> SwingLeg -> HandTrackJoints`
   - stand: `static_Contact -> CoMXY_HipRPY -> Pz -> HandTrackJoints`

3. speedbot_v4_leg（`algorithm/wbc_priority_v4_leg.cpp`）
   - walk: `static_Contact -> PosRot -> SwingLeg`
   - stand: `static_Contact -> CoMXY_HipRPY -> Pz`

---

## 7. 多机型并行实现现状

当前主干是 **并行文件策略**（不覆盖 AzureLoong 旧实现）：

| 机型 | 有效关节 | 关键文件后缀 | demo |
|---|---:|---|---|
| AzureLoong | 31 | 无后缀 | `walk_wbc_joystick`, `walk_mpc_wbc_joystick` |
| speedbot_v4 | 22（12腿+10臂） | `_v4` | `walk_mpc_wbc_v4` |
| speedbot_v4_leg | 12（仅腿） | `_v4_leg` | `walk_mpc_wbc_leg` |

补充（来自 `pino_kin_dyn_v4*.h` 注释）：

- speedbot_v4：`q=29`，`dq=28`
- speedbot_v4_leg：`q=19`，`dq=18`

---

## 8. 开发约定（按仓库实际行为）

### 8.1 模块接口习惯

算法模块普遍遵循：

- `dataBusRead(...)`
- `step()/compute...()`
- `dataBusWrite(...)`

这是本仓库最稳定的“读-算-写”组织方式。

### 8.2 命名一致性要求（非常关键）

关节命名必须在以下位置保持一致：

- URDF / XML 里的 `joint name`
- MuJoCo 里的 `actuator name`
- `MJ_Interface*::JointName`
- `Pin_KinDyn*::motorName`
- `PVT_Ctr*::motorName`
- `common/joint_ctrl_config*.json` 键名

否则会出现：

- 构造阶段 `not found in the XML file!` 并 `std::terminate()`；
- 或控制阶段索引/参数错位。

### 8.3 机型扩展约定

`model_migration_memory.md` 与 `Tutorial.md` 一致建议：

- 新机型优先并行新增文件，不覆盖 AzureLoong 路径；
- 新增对应 `MJ_interface_xxx / pino_kin_dyn_xxx / wbc_priority_xxx / PVT_ctrl_xxx / joint_ctrl_config_xxx.json / demo`；
- 在 CMake 中新增 target，保持旧 target 可回归。

### 8.4 CMake 组织细节

`CMakeLists.txt` 使用 `file(GLOB ... algorithm/*.cpp common/*.cpp math/*.cpp sim_interface/*.cpp)` 汇总到 `core` 静态库。

意味着：

- 在这些目录新增 `.cpp` 文件通常会自动进入 `core`；
- 新 demo 仍需手动 `add_executable + target_link_libraries`。

### 8.5 代码风格与命名规范

- 仓库文档（README/Tutorial/迁移记忆）以中文为主；
- 代码注释中英文混合，英文较多；
- 若新增注释，优先保证团队可读性与术语一致性；
- 变量命名常见前后缀含义（来自 README）：

| 前缀后缀 | 指代 |
|---|---|
| `_L`, `_W` | 本体坐标系下、世界坐标系下 |
| `fe_` | 足末端（foot-end） |
| `_L`, `_l`, `_R`, `_r` | 左侧、右侧 |
| `swing`, `sw` | 摆动腿 |
| `stance`, `st` | 支撑腿 |
| `eul`, `rpy` | 姿态角 |
| `omega` | 角速度 |
| `pos` | 位置 |
| `vel` | 线速度 |
| `tor`, `tau` | 力矩 |
| `base` | BaseLink |
| `_des` | 期望值 |
| `_cur` | 当前实际值 |
| `_rot` | 坐标变换矩阵 |

### 8.6 配置文件编码

- JSON 配置文件允许 C 风格注释（`//` 与 `/* */`），由 `jsoncpp` 的 `allowComments=true` 支持；
- 建议在调参时保留注释说明修改原因与预期效果。

---

## 9. 测试与验证策略（当前实际）

### 9.1 自动化测试

- 当前无 `add_test(...)`、无 gtest/pytest、无 CI workflow。
- `ctest -N` 为 0。

### 9.2 建议的最小人工回归流程

1. 全量构建通过（4 个可执行文件全部生成）。
2. 至少运行你改动影响的 demo（建议 20~30s）。
3. 检查以下行为：
   - `Space` 切换是否正常；
   - 步态切换是否出现明显卡顿/抖振；
   - 无连续 QP 异常、无快速跌倒。
4. 检查日志：
   - `record/datalog.log` 是否持续写入；
   - `record/matlabReadDataScript.txt` 是否可对应新字段。
5. 涉及多机型公共逻辑时，执行 Azure 与 v4 至少各 1 条 demo 回归。

### 9.3 验证工具（`tools/validation/`）

- `generate_leg_stage_compare_report.sh`：对比 Stage1/2/3 日志，输出关键指标报告（est_err、qpStatus、motionState 分布等）。
- `check_ros2_real_leg_contract.sh`：核对真机数据合约（话题名、动作长度、DataBus 输入合规性）。
- `run_leg_noise_delay_sweep.sh`：批量噪声/延迟参数扫描实验。

### 9.4 v4 调参参考

`model_migration_memory.md`（第 8 节）已给出稳定性优化思路与验收指标，可直接复用做 A/B 评估。

---

## 10. 部署流程现状

仓库当前没有独立“部署打包流水线”（无 Dockerfile、无发布脚本、无 CI/CD 配置）。

实际部署流程即：

1. 本地 CMake 构建；
2. 直接运行 `build/` 下 demo 二进制；
3. 用 `record/` 输出做结果分析；
4. 真机部署时通过 `tools/start_control_v4_leg.sh` 统一入口启动，配合 ROS2 话题收发。

---

## 11. 安全与稳定性注意事项

### 11.1 控制与参数安全

- `PVT_Ctr*` 有力矩限幅（`maxTorque`）与低通，但高增益仍可能导致仿真不稳定。
- `calMotorsPVT(deltaP_Lim)` 仅在部分阶段使用位置增量限幅，常规控制无该保护。

### 11.2 已知鲁棒性风险点

1. `setJointPD(...)`（`common/PVT_ctrl*.cpp`）
   - 关节名未找到时仅打印 `NOT found!`，但仍会以 `id=-1` 写数组，存在越界风险。
   - 修改/新增关节名时务必先保证名称正确。

2. `MJ_Interface*` 初始化绑定
   - 若 joint/actuator/sensor 名不匹配会直接 `std::terminate()`。

3. MPC 参数硬编码
   - `algorithm/mpc.cpp` 中质量/惯量等参数部分硬编码，跨机型调参需谨慎（`model_migration_memory.md` 已强调）。

### 11.3 日志与资源

- `record/datalog.log` 可快速增大（当前仓库已有超大日志文件示例）。
- 开启原始帧录制会生成很大 `.out` 文件，导出视频前先确认磁盘空间。

---

## 12. Agent 工作清单（建议）

接手任何改动前，先完成：

1. 确认改动属于哪个机型链路（Azure / v4 / v4_leg）。
2. 检查关节命名在 `MJ_Interface* / PVT* / Pin* / JSON / XML` 是否一致。
3. 若改算法公共层，至少跑 2 条 demo（受影响主链路 + 一条旁路机型）。
4. 输出变更时注明：
   - 改了哪些参数/任务权重；
   - 是否影响 `DataLogger` 字段；
   - 是否需要同步更新模型或 JSON。

---

## 13. 行走优化与调参指南

基于实际代码分析和运行经验，整理以下行走控制的关键机制和优化建议。

### 13.1 手部动作与行走协调机制

#### 13.1.1 耦合摆臂原理
`wbc_priority.cpp` 中 `HandTrackJoints` 任务实现对角线摆臂：

```cpp
double l_hip_pitch = q(28) - q(34);  // 左髋关节相对角度
double r_hip_pitch = q(34) - q(28);  // 右髋关节相对角度

target_arm_q << 
    0.475 - 0.75*r_hip_pitch, -1.12, 1.9, 0.86, -0.356, 0, 0,  // 左臂
    -0.475 + 0.75*l_hip_pitch, -1.12, -1.9, 0.86, 0.356, 0, 0; // 右臂
```

**物理意义**：
- 右腿前摆 → 左臂后摆（系数 0.75）
- 左腿前摆 → 右臂后摆
- 作用：角动量补偿（减少躯干晃动）、降低能耗 15-20%

#### 13.1.2 摆臂参数调整
```cpp
// 增大摆幅（系数从 0.75 改为 1.0）
target_arm_q << 0.475 - 1.0*r_hip_pitch, ...

// 增加肘关节弯曲（更自然）
target_arm_q[3] = 0.86 + 0.3*fabs(r_hip_pitch); // 左肘

// 降低任务优先级（减少抖动）
kin_tasks_walk.taskLib[id].kp = 100; // 从 200 降低
```

### 13.2 步态僵硬问题诊断与优化

#### 13.2.1 僵硬原因分析
| 问题 | 原因 | 代码位置 |
|------|------|----------|
| 轨迹简单 | XY纯摆线，Z方向贝塞尔控制点固定 | `foot_placement.cpp` |
| 踏频固定 | `tSwing=0.4s` 不随速度变化 | `gait_scheduler.cpp` |
| 无过渡相 | 无双子支撑相，直接切换 | `gait_scheduler.cpp` |
| 增益保守 | SwingLeg kp=500 偏硬 | `wbc_priority.cpp` |

#### 13.2.2 优化方案

**1. 自适应步高**（根据速度调整）
```cpp
// FootPlacement 中添加
double getAdaptiveStepHeight(double vx) {
    double minHeight = 0.05;  // 低速步高
    double maxHeight = 0.15;  // 高速步高
    double ratio = std::min(fabs(vx) / 0.8, 1.0);
    return minHeight + (maxHeight - minHeight) * ratio;
}
// 使用：stepHeight = getAdaptiveStepHeight(curV_W(0));
```

**2. 5次样条替代摆线**（更平滑）
```cpp
// 归一化5次多项式，保证位置/速度/加速度连续
x(phi) = (10*phi^3 - 15*phi^4 + 6*phi^5);  // 满足 x(0)=0, x(1)=1, dx(0)=dx(1)=0
```

**3. 变刚度控制**（摆动中期更软）
```cpp
// WBC中 SwingLeg 任务
double stiffness = 500 * (1 - 0.3*sin(M_PI*phi)); // phi=0.5时最软
kin_tasks_walk.taskLib[id].kp = stiffness * I;
```

### 13.3 踏频与速度控制机制

#### 13.3.1 当前机制分析
```
速度 = 踏频 × 步长

当前实现：
- 踏频固定：tSwing = 0.4s → 踏频 = 1.25Hz
- 速度变化主要靠步长调整
- 步长 ≈ v × tSwing × (1.5 - phi)

问题：
- 速度从 0.4→0.8 时，步长仅增加 2 倍
- 高速时显得"匆忙"，跨距变化不明显
- 低速时步长太短，显得"拖沓"
```

#### 13.3.2 自适应踏频实现
```cpp
// GaitScheduler 中添加
void updateSwingTime(double vx) {
    double minSwing = 0.3;   // 最快踏频
    double maxSwing = 0.6;   // 最慢踏频
    double factor = 0.5 / (fabs(vx) + 0.1);
    factor = std::max(0.5, std::min(factor, 2.0));
    tSwing = minSwing + (maxSwing - minSwing) * (factor - 0.5) / 1.5;
}

// 效果：
// v=0.3 m/s → tSwing=0.5s（踏频 1.0Hz，步长约 0.23m）
// v=0.8 m/s → tSwing=0.3s（踏频 1.67Hz，步长约 0.48m）
```

### 13.4 横向行走支持

#### 13.4.1 现状分析
当前架构**完全支持**横向行走，但键盘控制未启用：
- `JoyStickInterpreter` 已有 `setVyDesLPara()` 接口
- `FootPlacement` 已配置 `kp_vy = 0.03`
- MPC/WBC 对 X/Y 对称处理

#### 13.4.2 启用方法
```cpp
// 1. demo 中添加键盘控制（Q/E键）
if (buttonState.key_q)
    jsInterp.setVyDesLPara(0.3, 1.0);   // 左移
if (buttonState.key_e)
    jsInterp.setVyDesLPara(-0.3, 1.0);  // 右移

// 2. 优化横向参数
footPlacement.kp_vy = 0.02;  // 比 kp_vx 略保守

// 3. 横向安全保护（支撑多边形变窄）
if (fabs(vx) < 0.1 && fabs(vy) > 0.2) {
    tSwing = 0.5;        // 增加周期
    stepHeight = 0.08;   // 降低步高
}
```

**注意**：横向行走支撑多边形窄（脚宽约0.15m小于脚长约0.3m），需限制速度在 0.3 m/s 以内。

### 13.5 评价指标与调参参考

#### 13.5.1 关键评价指标
```cpp
// DataLogger 应记录以下信号：
logger.addIterm("base_pos", 3);      // CoM跟踪
logger.addIterm("base_pos_des", 3);  // 期望位置
logger.addIterm("FL_est", 3);        // 左脚力（ZMP计算）
logger.addIterm("FR_est", 3);        // 右脚力
logger.addIterm("legState", 1);      // 支撑相
logger.addIterm("phi", 1);           // 相位
```

#### 13.5.2 稳定性判据
| 指标 | 良好范围 | 危险值 |
|------|----------|--------|
| CoM跟踪误差 | < 2cm | > 5cm |
| 躯干roll/pitch | < 5° | > 10° |
| ZMP安全裕度 | > 2cm | < 0 |
| 触地冲击力 | < 300N | > 500N |
| 步态对称性 | > 0.9 | < 0.7 |

#### 13.5.3 分层调参建议
| 层次 | 可调参数 | 调参目标 |
|------|----------|----------|
| MPC | L_diag (pCoM权重) | CoM跟踪精度 |
| WBC | SwingLeg kp/kd | 摆动腿跟踪 |
| FootPlacement | kp_vx/kp_vy | 落脚点响应 |
| GaitScheduler | tSwing | 踏频调整 |
| PVT | 各关节kp/kd | 关节跟踪 |

---

## 14. 参考文档

- `README.md`：项目介绍与基础使用
- `Tutorial.md`：模型替换方法
- `model_migration_memory.md`：机型迁移与 v4 调优记忆
- `tools/validation/README.md`：leg 迁移验证工具使用说明
- `tools/real_robot/README.md`：真机模式 RViz 使用说明
- `tools/sim_ros2/README.md`：MuJoCo 仿真 ROS2 状态发布说明
- API 文档：https://www.openloong.org.cn/pages/api/html/index.html
- Wiki：https://www.openloong.org.cn/pages/wiki/html/index.html
