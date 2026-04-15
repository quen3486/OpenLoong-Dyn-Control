# OpenLoong 本地记忆（P0）

## 目标
先完成 P0（观测与链路正确性），并记录可复现的数据证据。

## 时间线
- 2026-04-15 14:11:39 +0800：创建本地记忆文件，进入 P0 实施。

## P0 任务清单
- [x] P0-1：修复 `MJ_interface*` 到 `DataBus` 的 `basePos/baseLinVel` 写回。
- [x] P0-2：接入触地传感器 `lf-touch/rf-touch`，写入 `fL/fR`（至少 Z 分量）。
- [x] P0-3：扩展 demo 日志字段（`phi/tSwing/js_vel_des/js_omega_des/swingDesPosCur_W/swingDesPosFinal_W/qpStatus_MPC`）。
- [x] P0-4：补齐 `DataBus` 相关字段初始化，避免未定义日志值。
- [x] P0-5：编译验证并记录结果。

## 基线数据（改动前）
- `record/datalog.log` 列数：`209`。
- `baseLinVel` 对应列（183~185）绝对值最大值：`0`（当前日志中恒为 0）。
- 现有日志字段可见 `FL_est/FR_est`、`legState`、`motionState`，但缺少 `phi/tSwing` 与摇杆目标显式记录。

## 风险备注
- 当前代码中 `f3d` 未见更新来源逻辑，`fL/fR` 可信度不足；P0 将优先接入 MuJoCo touch 传感器。
- 若运行环境无图形界面，可能无法直接跑 demo 采集新日志；此时先完成代码与编译验证，再说明采集限制。

## 进展更新
- 2026-04-15 14:24:21 +0800：
  - 已在 `MJ_interface.cpp`、`MJ_interface_v4.cpp`、`MJ_interface_v4_leg.cpp` 接入 `lf-touch/rf-touch`，并把 `basePos/baseLinVel` 写回 `DataBus`。
  - 已在 `common/data_bus.h` 补齐关键字段初始化（含 `phi/tSwing/swingDesPos*`、`qpStatus_MPC` 等）。
  - 已在 `walk_mpc_wbc_joystick.cpp`、`walk_mpc_wbc_v4.cpp`、`walk_mpc_wbc_leg.cpp` 扩展日志字段。
  - `walk_mpc_wbc_v4.cpp` 增加自动化测试入口：`OPENLOONG_AUTOWALK=1`、`OPENLOONG_SIM_END=<seconds>`。
- 2026-04-15 14:35:38 +0800：
  - 编译通过：`cmake -S . -B build && cmake --build build -j4`（4 个可执行目标全部成功链接）。
  - 发现并修复自动测试触发时序问题：`OPENLOONG_AUTOWALK=1` 下，原逻辑会在 `simTime∈[3.0, 3.002)` 被强制 `Stand` 覆盖，导致 `phi` 全程为 0。现已将自动触发条件改为 `simTime > openLoopCtrTime + 0.01`。
  - 完成 v4 自动仿真采集：`OPENLOONG_AUTOWALK=1 OPENLOONG_SIM_END=14 ./walk_mpc_wbc_v4`。
  - 已归档基线数据：
    - `record/baseline_p0/datalog_v4_autowalk_20260415_143538.log`
    - `record/baseline_p0/matlabReadDataScript_v4_autowalk_20260415_143538.txt`
    - `record/baseline_p0/README.md`（记录运行命令与核心指标）
  - 新日志规模：`rows=14008`，`cols=224`，时长约 `14.008s`。
  - 关键可用性检查：
    - `phi_range=[0.0, 0.9125]`（不再全 0，步态相位链路有效）
    - `tSwing_range=[0.4, 0.4]`（当前踏频参数固定）
    - `legState` 分布：`LSt=5651, RSt=5347, DSt=3010`
    - `motionState` 分布：`Stand=3010, Walk=10998`
  - t>=10s 行走段指标（作为后续优化对照基线）：
    - 速度跟踪：`vx MAE=0.0510 m/s, RMSE=0.0614 m/s`（目标 `0.4 m/s`）
    - 姿态平顺：`base_rpy RMS(roll/pitch/yaw)=1.3646°/0.8895°/0.7280°`
    - 速度平顺：`|d(vx)/dt| RMS=1.326 m/s², peak=3.764 m/s²`
    - 触地冲击斜率：`peak |dFz/dt|` 左右脚约 `5.42e5/5.52e5 N/s`
    - 步频估计（按同侧支撑复现周期）：`LSt/RSt` 周期约 `0.716s`，对应约 `1.397 Hz`（单侧）/ `2.79 step/s`（双步频）。
- 2026-04-15 14:46:10 +0800：
  - 完成“步态规划关键参数”源码级梳理，覆盖 `GaitScheduler / FootPlacement / JoyStickInterpreter / MPC / WBC`。
  - 关键结论：
    - 目前踏频主参数为 `tSwing=0.4s`（固定），并且 MPC 相位外推内部也写死 `/0.4`（需联动改造后才可真正自适应）。
    - 步态切换触发由 `phi>=0.6 + 估计触地力阈值280N` 主导，停止触地阈值为 `200N`；切换阈值应做配置化。
    - 摆腿轨迹关键参数（`stepHeight`、速度反馈增益 `kp_vx/kp_vy/kp_wz`、足端偏置 `x/y/zOff`、末端下压 `zStretch`）均为硬编码。
    - 平顺度受 `WBC` 的 `PosRot/SwingLeg` 高增益（典型 `kp=500,kd=20`）强影响；建议纳入 gait profile 配置。
- 2026-04-15 15:10:49 +0800：
  - 已落地 gait profile 配置化框架：
    - 新增 `common/gait_profile.h/.cpp`（JSON 配置读取，含默认值与基础限幅）
    - 新增 `common/gait_profile_v4.json`、`common/gait_profile_v4_leg.json`、`common/gait_profile_azure.json`
  - 已接入算法模块：
    - `GaitScheduler`：`tSwing/phiSwitchMin/fzSwitchThreshold/fzStopThreshold` 可配置
    - `FootPlacement`：`kp_vx/kp_vy/kp_wz/stepHeight/xOffsetL/yOffsetL/zOffsetW/swingTrajectoryPhase/swingTrajectoryWindow/zStretch*` 可配置
    - `MPC`：相位外推从硬编码 `/0.4` 改为使用 `Data.tSwing`
    - `WBC_priority*`：`PosRot/SwingLeg` 的误差限幅与 `kp/kd` 改为可配置
  - 已接入 4 个 demo：
    - `walk_mpc_wbc_v4`、`walk_mpc_wbc_leg`、`walk_mpc_wbc_joystick`、`walk_wbc_joystick` 均加载对应 gait profile 并应用参数
  - 验证结果：
    - `cmake -S . -B build && cmake --build build -j4` 编译通过（4 个 demo 全部成功链接）
    - 冒烟仿真：`OPENLOONG_AUTOWALK=1 OPENLOONG_SIM_END=8 ./walk_mpc_wbc_v4`
      - 日志 `rows=8007, cols=224, duration=8.007s`
      - `phi_range=[0.0, 0.9125]`，`tSwing_range=[0.4, 0.4]`
    - `motionState` 含 `Stand` 与 `Walk`，链路正常
- 2026-04-15 15:17:22 +0800：
  - 根据用户要求，为配置文件增加中文注释说明：
    - `common/gait_profile_v4.json`
    - `common/gait_profile_v4_leg.json`
    - `common/gait_profile_azure.json`
  - 为避免解析差异，`common/gait_profile.cpp` 显式启用 `allowComments=true`（支持 `//` 与 `/* */` 注释）。
  - 验证：
    - 增量编译通过：`cmake --build build -j4`
    - 仿真冒烟：`OPENLOONG_AUTOWALK=1 OPENLOONG_SIM_END=4.2 ./walk_mpc_wbc_v4` 正常启动并进入自动行走。
- 2026-04-15 16:23:39 +0800：
  - 完成 MPC 质量参数配置化落地：common/gait_profile.h/.cpp 已支持 mpc_mass 读取与限幅；demo/walk_mpc_wbc_joystick.cpp、demo/walk_mpc_wbc_v4.cpp、demo/walk_mpc_wbc_leg.cpp 均已调用 MPC_solv.setRobotMass(gaitProfile.mpcMass)。
  - 三套机型配置已加入带注释的 mpc_mass：
    - gait_profile_azure.json = 77.35
    - gait_profile_v4.json = 59.12
    - gait_profile_v4_leg.json = 59.12
  - 机体质量依据来自模型惯量质量求和：AzureLoong 77.352584kg，speedbot_v4/v4_leg 59.115320kg。
- 2026-04-15 16:24:35 +0800：
  - 验证通过：cmake --build build -j4 成功，walk_mpc_wbc_joystick 与 walk_mpc_wbc_leg 新增 setRobotMass 调用无编译回归。
  - 仿真冒烟：cd build && OPENLOONG_AUTOWALK=1 OPENLOONG_SIM_END=4.2 ./walk_mpc_wbc_v4 正常启动并进入自动行走。
  - 已完成机型相关硬编码排查：MPC 内仍存在惯量 Ic、摩擦系数 miu、足底支撑多边形 delta_foot、力/力矩上下限 max/min 等固定值；demo 层存在 Fr_ff、stand_legLength、foot_height、width_hips、WBC 构造入参摩擦系数 0.7 等机型强相关常量。
- 2026-04-15 18:01:24 +0800：
  - 完成“m/miu 等参数统一配置 + MPC/WBC 单一数据源”落地：
    - `common/gait_profile.h/.cpp` 新增并接入：`contact_miu`、`mpc_delta_foot_front/rear/left/right`、`mpc_force_max_xy`、`mpc_fz_max_scale`、`mpc_torque_max_x/y/z`。
    - `algorithm/mpc.*` 新增接口：`setFrictionCoeff`、`setFootSupportPolygon`、`setWrenchLimits`，并将 `Fz_max` 由固定 `3*m*|g|` 改为 `mpc_fz_max_scale * m * |g|`。
    - `algorithm/wbc_priority*.{h,cpp}` 新增 `setContactMiu`，统一使用 gait profile 的 `contact_miu`。
    - `demo/walk_mpc_wbc_joystick.cpp`、`demo/walk_mpc_wbc_v4.cpp`、`demo/walk_mpc_wbc_leg.cpp`、`demo/walk_wbc_joystick.cpp` 已统一从 gait profile 下发摩擦/质量/接触约束。
  - 三套机型配置文件均已补齐并带中文注释：
    - `common/gait_profile_azure.json`
    - `common/gait_profile_v4.json`
    - `common/gait_profile_v4_leg.json`
  - 编译验证：`cmake --build build -j4` 成功（4 个 demo 全部链接通过）。
  - 仿真与采集：`cd build && OPENLOONG_AUTOWALK=1 OPENLOONG_SIM_END=6 ./walk_mpc_wbc_v4`
    - 归档目录：`record/experiments/config_unified_mpc_wbc/`
    - 文件：
      - `datalog_v4_autowalk_20260415_180124.log`
      - `matlabReadDataScript_v4_autowalk_20260415_180124.txt`
      - `summary_v4_autowalk_20260415_180124.md`
    - 摘要指标：`rows=6001`、`cols=224`、`phi_range=[0,0.775]`、`tSwing=0.4` 固定、`vx误差MAE=0.0455`（t>=3s）。
- 2026-04-15 19:31:58 +0800：
  - 针对“`mpc.cpp` 中 m/Ic/miu 仍写死”完成二次核对与修正：
    - `algorithm/mpc.cpp` 删除固定惯量赋值 `Ic << ...`，改为：
      - 默认使用 `DataBus.inertia`（由 `Pin_KinDyn*` 每周期写入）；
      - 若 DataBus 惯量不可用，则回退到 gait profile 配置惯量。
    - 新增 MPC 接口：
      - `setBodyInertia(const Eigen::Matrix3d&)`
      - `setUseDataBusInertia(bool)`
    - 构造函数中的 `m/miu` 改为中性安全默认（`m=1.0, miu=0.7`），运行期由 demo 从 gait profile 显式下发，不再保留机型硬编码常量。
  - 配置侧新增惯量项并注释（3机型）：
    - `mpc_use_databus_inertia`（默认 true）
    - `mpc_inertia_xx/xy/xz/yy/yz/zz`（用于回退或固定惯量模式）
  - demo 侧接线完成（3个 MPC demo）：
    - 从 gait profile 组装惯量矩阵并调用 `setBodyInertia(...)`
    - 调用 `setUseDataBusInertia(gaitProfile.mpcUseDataBusInertia)`
  - 验证：
    - 编译通过：`cmake --build build -j4`
    - 仿真冒烟：先清空日志再跑 `OPENLOONG_AUTOWALK=1 OPENLOONG_SIM_END=4.2 ./walk_mpc_wbc_v4`
    - 日志：`rows=4216, cols=224, phi_range=[0,0.7675], tSwing=0.4`，链路正常。
- 2026-04-15 20:03:24 +0800：
  - 按用户要求完成“足底支撑域半尺寸 vs XML”一致性核对（Azure / v4 / v4_leg）。
  - 对照方法：读取各机型脚底接触 `geom`（`type=box, condim=4, friction=1 1 1`）的 `pos/size`，换算
    - `front = pos_x + size_x`
    - `rear = size_x - pos_x`
    - `left = pos_y + size_y`
    - `right = size_y - pos_y`
  - 结论：当前三套配置的 `mpc_delta_foot_front/rear/left/right = 0.073/0.125/0.025/0.025` 均与 XML 不一致：
    - Azure XML 平均约 `0.1700 / 0.0750 / 0.0408 / 0.0394`
    - v4 XML 约 `0.1585 / 0.0665 / 0.0400 / 0.0400`
    - v4_leg XML 约 `0.1500 / 0.0600 / 0.0400 / 0.0400`
  - 差异特征：`front` 明显偏小、`rear` 偏大、`left/right` 偏小约 `0.015m`。
- 2026-04-15 20:38:50 +0800：
  - 完成 `x_offset_l` 与 `foot_kp_vx` 源码级语义核对（`algorithm/foot_placement.cpp`）：
    - `x_offset_l`：
      - 含义：摆脚目标落点在机体系 x 方向的固定偏置（左腿符号基准），经当前航向 `yawCur` 旋转后叠加到世界系落脚点。
      - 代码路径：`xOff_L -> (xOff_W, yOff_W) -> posDes_W(0/1)`。
      - 作用：改变名义落脚前后位置（步态“前探/后撤”偏置），对躯干前后俯仰、步态对称性、稳定裕度有直接影响。
    - `foot_kp_vx`：
      - 含义：落脚点前后速度反馈增益，构造在 `KP(0,0)`，参与 `-KP*(desV_W-curV_W)` 项。
      - 代码路径：`KP(0,0)=kp_vx`，并按 `yawCur` 旋转至世界系后用于 `posDes_W` 计算。
      - 作用：调节“速度误差 -> 前后落脚修正”的强度；值越大，速度误差下的步长修正越激进，响应更快但冲击/抖动风险更高。

## 双支撑状态可配置化改造计划（新增，待实施）

### 现状结论（源码核对）
- `GaitScheduler` 目前行走主链路为 `LSt <-> RSt` 直接切换，注释已明确 “no double-support here”。
- `DSt` 主要在 `Walk2Stand` 末段出现；行走过程无显式双支撑驻留时长控制。
- `MPC` 约束侧已经支持 `DSt`（双脚受力/约束分配）。
- `WBC_priority*` 的受力约束对 `DSt` 具备兼容性，但 `computeDdq/dataBusRead` 当前对 `DSt` 仍会落到单摆腿逻辑分支（需要修正）。

### 改造目标
- 在连续行走中新增“双支撑过渡相（DSt）”。
- 双支撑时长可配置（按机型 profile 配置）。
- 不破坏现有 `Walk2Stand`、MPC/WBC/PVT 主链路。

### 优先级与实施项

#### P0（必须先做，打通链路）
- P0-1：配置项扩展（`common/gait_profile.*` + 三套 json）
  - 新增建议：
    - `enable_double_support`（bool）
    - `t_double_support`（s）
    - `phi_ds_enter_min`（允许进入 DSt 的最小摆动相位，防早切）
    - `fz_ds_threshold`（进入 DSt 的触地阈值）
  - 增加安全限幅：`t_double_support >= 0.0`，建议上限 `<= 0.5`。
- P0-2：`GaitScheduler` 状态机重构为三态轮转
  - 轮转：`LSt -> DSt -> RSt -> DSt -> LSt ...`
  - 关键变量：
    - `legStatePrevSS`（前一单支撑）
    - `legStateNextSS`（双支撑结束后目标单支撑）
    - `dsTimeCur`（双支撑已运行时间）
  - 进入条件：`phi >= phi_ds_enter_min && swingFootFz >= fz_ds_threshold`。
  - 退出条件：`dsTimeCur >= t_double_support`（可附加力阈值保护）。
- P0-3：`DataBus` 增补双支撑相信息
  - 新增建议：`tDoubleSupport`、`phiDS`、`isDoubleSupport`。
  - `phi` 保持“单支撑摆动相位”语义，避免 FootPlacement/MPC 误解。
- P0-4：demo 接线
  - 三个 MPC demo + `walk_wbc_joystick` 均下发新参数到 `GaitScheduler`。
  - DataLogger 新增：`phiDS/isDoubleSupport/tDoubleSupport`。

#### P1（稳定性关键，紧随 P0）
- P1-1：`WBC_priority*` 明确 `DSt` 分支
  - `dataBusRead` 中对 `legStateCur==DSt` 不再默认落到“右支撑”分支。
  - `computeDdq` 中新增 `Walk-DS` 任务簇（建议：`static_Contact(双脚) + PosRot + (可选)HandTrack`，去掉 `SwingLeg`）。
  - 三个机型实现保持同构：`wbc_priority.cpp / _v4.cpp / _v4_leg.cpp` 同步。
- P1-2：`MPC` 相位外推兼容 DS 时长
  - 当前外推使用 `Data.phi + i*dt/Data.tSwing`，需在 `DSt` 下改为参考 `tDoubleSupport` + `phiDS`。
  - 保持 `legStateNext` 与 scheduler 一致，避免预测接触相错位。

#### P2（效果优化与回归）
- P2-1：FootPlacement 在 DSt 段冻结/旁路（避免无效 swing 轨迹写入）。
- P2-2：参数扫描（`t_double_support`）并建立 A/B 数据对比。

### 风险点（需重点盯防）
- `WBC` 若未完成 `DSt` 分支，双支撑段可能仍执行单摆腿任务，导致突变力矩。
- `MPC` 若仍按 `tSwing` 推相，DS 段预测接触会滞后/超前，引发接触力抖动。
- `StateEst::getTrustRegion_wt_h()` 中存在 `legState == DataBus::Stand` 判断（类型语义不一致），建议在本轮并行修正为 `legState == DataBus::DSt`。

### 验收与数据指标（用于判断优化正/负向）
- 触地冲击：`peak |dFz/dt|` 下降为正向。
- 姿态平顺：`roll/pitch` RMS 下降为正向。
- 速度平顺：`|d(vx)/dt|` RMS/peak 下降为正向。
- 跟踪性能：`vx MAE` 不恶化（允许小幅波动）。
- 步态相时序：日志应出现稳定 `LSt-DSt-RSt-DSt` 轮转，`DSt` 持续时间接近配置值。
