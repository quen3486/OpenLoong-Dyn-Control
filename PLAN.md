# v4_leg 闭环 PVT 配置化、参数扫测与换相突变继续优化计划

## Summary

- 本轮只改 `walk_mpc_wbc_leg` / `speedbot_v4_leg` 主链路，AzureLoong 和 `speedbot_v4` 暂不动。
- 将 `demo/walk_mpc_wbc_leg.cpp` 里闭环阶段硬编码的 `pvtCtr.setJointPD(...)` 改为读取 `common/joint_ctrl_config_v4_leg.json` 中新增的闭环 PVT 参数。
- 用 MuJoCo 直行回归扫测 ankle PVT、`fz_switch_threshold/fz_stop_threshold`，选出一组默认参数。
- 继续围绕换相突变分析：重点区分 PVT PD 项、WBC 前馈项、接触确认时序和摆动脚轻微擦地。

## Key Changes

- 在 `joint_ctrl_config_v4_leg.json` 的 12 个腿部关节中新增：
  - `closedLoopKp`
  - `closedLoopKd`
- 初始值先等于当前 hardcode 行为，保证重构后 baseline 不变：
  - hip_roll `400/15`
  - hip_yaw `200/10`
  - hip_pitch `300/10`
  - knee `300/14`
  - ankle_pitch `300/18`
  - ankle_roll `300/16`
- 扩展 `PVT_Ctr_V4_Leg`：
  - 构造时读取 `closedLoopKp/closedLoopKd`。
  - 若字段缺失则回退到原 `kp/kd`。
  - 新增 `applyClosedLoopPD()`，一次性把 12 个关节闭环增益写入 `pvt_Kp/pvt_Kd`。
- 替换 `walk_mpc_wbc_leg.cpp` 闭环分支：
  - 删除当前每周期硬编码 `setJointPD(...)` 列表。
  - 改为 `pvtCtr.applyClosedLoopPD(); pvtCtr.calMotorsPVT();`
- 真机安全 PVT 合力矩估算同步使用 `closedLoopKp/closedLoopKd`，字段缺失时回退 `kp/kd`，保证仿真闭环和真机安全判断一致。
- 更新现有 `check_ros2_real_leg_contract.sh`：
  - 检查 12 个腿部关节都有 `closedLoopKp/closedLoopKd`。
  - 检查闭环 PVT 不再使用 hardcoded `setJointPD(400...)` 这类固定值。

## Parameter Sweep

- 固定已有换相保护：
  - `phase_transition_blend_time_sec = 0.02`
  - `contact_confirm_time_sec = 0.02`
  - `contact_force_blend_time_sec = 0.03`
- 第一组确认重构等价：
  - `closedLoopKp/Kd` 使用当前 hardcode 等价值。
  - 指标应接近当前 baseline：left/right ankle pitch 满额饱和约 `60/18ms`。
- 第二组只扫 ankle，hip/knee 保持当前值：
  - A0：ankle_pitch `300/18`，ankle_roll `300/16`
  - A1：ankle_pitch `260/18`，ankle_roll `260/16`
  - A2：ankle_pitch `240/18`，ankle_roll `240/16`
  - A3：ankle_pitch `220/18`，ankle_roll `220/16`
  - A4：ankle_pitch `220/20`，ankle_roll `220/18`
  - A5：ankle_pitch `200/20`，ankle_roll `200/18`
- 在 ankle 最优组基础上扫 `fz`：
  - F0：`280/200`
  - F1：`240/180`
  - F2：`220/160`
  - F3：`200/160`
  - F4：`180/160`
- 若 ankle 调参后 knee 的 `0.5*maxTorque` 占用仍明显偏高，再追加 hip/knee 轻量扫测：
  - H0：当前值
  - H1：hip_pitch/knee `280/10`、`280/14`
  - H2：hip_pitch/knee `260/10`、`260/14`

## Selection Rule

- 每组跑 3 次正常直行回归：
  - `cd build && ./walk_mpc_wbc_leg`，手动执行 `F -> Space -> W -> J`
- 只统计 `motionState=Walk` 段。
- 先剔除不合格组：
  - 行走段不足 `15.5s`
  - 换腿次数明显异常
  - ankle pitch 最大速度 `>= 6.7rad/s`
  - ankle pitch 满额 `42Nm` 饱和 `>= 100ms`
  - knee 出现满额饱和
  - 行走提前摔倒或 QP 连续异常
- 合格组按以下顺序选最优：
  - ankle pitch 左右满额饱和总时长最小。
  - ankle pitch `0.5*maxTorque` 占用时长最小。
  - knee `0.5*maxTorque` 占用不高于当前 baseline。
  - 换相后 `0..80ms` 内 `q_des/tau_ff/tau_out/dq_cur` 峰值更小。
  - 若指标接近，优先选择更高 `kp`，保留跟踪刚度。

## 换相突变继续分析

- 使用现有日志，不新增脚本文件；用临时解析命令输出报告。
- 每次换相统计：
  - 新支撑腿 `FL_est/FRest` 达阈值时间。
  - MuJoCo touch truth 首次超过 `20/100/280N` 时间。
  - 触地时足端高度、足端竖直速度。
  - 换相后 `0..80ms` ankle/knee 的 `q_des` 跳变、`tau_ff` 跳变、`tau_out` 饱和时长、`dq_cur` 峰值。
- 对突变来源做判定：
  - 若 `tau_ff` 跳变大且 `q_des` 平滑，优先调 WBC/contact force transition。
  - 若 `q_des` 跳变大，优先调命令过渡或 swing foot trajectory。
  - 若 touch truth 早于估计力较多，优先降低 `fz_switch_threshold` 或调整 confirm 时间。
  - 若摆动脚在支撑相中 `touch > 20N` 占比仍高，优先测试 `z_stretch_step` 减小或关闭、`step_height` 小幅增加。
- 若 PVT 和 `fz` 扫测后 ankle `0.5*maxTorque` 占用仍长，追加轻量轨迹扫测：
  - T0：当前 `step_height=0.06`，`z_stretch_step=-0.005`
  - T1：`step_height=0.07`，`z_stretch_step=-0.003`
  - T2：`step_height=0.07`，`z_stretch_step=0.0`

## Test Plan

- 构建与静态检查：
  - `cmake --build build -j4`
  - `git diff --check`
  - `./tools/validation/check_ros2_real_leg_contract.sh`
- 功能回归：
  - 重构后 baseline 必须与当前 hardcode 结果接近，确认配置化没有改变行为。
  - 参数扫测全部使用临时 config，不污染仓库；只把最终胜出参数写回 `joint_ctrl_config_v4_leg.json` 和必要的 `controller_config_v4_leg.json`。
- 最终验收：
  - ankle pitch 最大速度 `< 6.7rad/s`
  - 左右 ankle pitch 满额饱和 `< 100ms`
  - knee 无满额饱和
  - knee `0.5*maxTorque` 占用不高于当前 baseline
  - 行走稳定，换腿次数稳定
  - 输出最终参数表和 baseline/最优组对比表

## Assumptions

- 本轮只优化 `speedbot_v4_leg`。
- 不新增启动脚本，不新增长期维护的分析脚本。
- 不放宽 URDF/joint limit，也不放宽真机安全阈值。
- `kp/kd` 继续作为开环/默认 PVT 参数；`closedLoopKp/closedLoopKd` 专门用于闭环 walking PVT。
