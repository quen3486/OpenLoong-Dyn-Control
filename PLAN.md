## Leg 链路 MuJoCo 传感等价化与真机迁移计划

### Summary
- 已核对 `StateEst` I/O：输入来自 IMU（`rpy/baseAcc/baseAngVel`）、步态相位（`phi/legState`）、足端运动学（`fe_*`）和动力学+关节力矩（`dyn_* + motors_tor_cur`）；输出回写 `base_pos/base_vel/base_rpy/base_omega_W` 与 `q/dq`，并输出 `FL_est/FR_est`。
- 已核对 `MPC` I/O：输入是估计后的 `base_rpy/q/dq`、`js_*`、足端位姿、`legState/legStateNext/phi/tSwing`、惯量；输出 `Fr_ff/des_ddq/des_dq/des_delta_q/base_*_des` 和 QP 状态。
- 已核对 `WBC` I/O：输入是全身动力学雅可比、足端/髋/CoM状态、`Fr_ff + des_*` 与步态状态；输出 `wbc_delta_q_final/wbc_dq_final/wbc_tauJointRes/wbc_FrRes` 和 QP 状态。
- 已核对 Python 驱动脚本 topic：订阅 `/imu/data`、`/joint_states`，发布 `/rl_motion_control_command`（29 维，前 12 维为下肢顺序关节位置）；与当前 leg ROS2 接口默认 topic 一致。

### Public Interfaces / Contracts
- 修改 [MJ_interface_v4_leg.cpp](/home/huangkun/workspaces/mpc/Openloong-dyn-control/sim_interface/MJ_interface_v4_leg.cpp)：`dataBusWrite` 的“仿真传感输入契约”改为真机等价，仅写入 IMU + 关节位置/速度 + 关节力矩；`basePos/baseLinVel/fL/fR` 置零，不再作为控制输入真值来源。
- 修改 [PVT_ctrl_v4_leg.cpp](/home/huangkun/workspaces/mpc/Openloong-dyn-control/common/PVT_ctrl_v4_leg.cpp)：`dataBusWrite` 不再覆盖 `motors_tor_cur`，保留接口层写入的“测得力矩语义”。
- 修改 [walk_mpc_wbc_leg.cpp](/home/huangkun/workspaces/mpc/Openloong-dyn-control/demo/walk_mpc_wbc_leg.cpp)：将“状态估计+动力学更新”前置到状态机之前，使 `applyLegControlStateMachine` 使用估计态而非接口原始 `q/rpy`。

### Implementation Changes
- 在 [MJ_interface_v4_leg.h](/home/huangkun/workspaces/mpc/Openloong-dyn-control/sim_interface/MJ_interface_v4_leg.h) 与 [MJ_interface_v4_leg.cpp](/home/huangkun/workspaces/mpc/Openloong-dyn-control/sim_interface/MJ_interface_v4_leg.cpp) 增加并填充关节力矩缓存，来源使用 `mj_data->qfrc_actuator[jntId_qvel[i]]`（对应关节实际执行力矩）。
- 在 [walk_mpc_wbc_leg.cpp](/home/huangkun/workspaces/mpc/Openloong-dyn-control/demo/walk_mpc_wbc_leg.cpp) 拆分控制流程为两个明确阶段：`StateEst+PinDyn(+setF)` 阶段与 `Joystick/Gait/FootPlacement/MPC/WBC` 阶段；`mujoco` 与 `ros2_real` 两条 backend 都采用相同时序。
- 保持 ROS 话题与消息形状不变：`/imu/data`、`/joint_states`、`/rl_motion_control_command`，并保持 12 关节顺序与当前映射一致（不改接口兼容性）。

### Test Plan
- 编译回归：`cmake -S . -B build && cmake --build build -j4`，确认 `walk_mpc_wbc_leg` 可执行。
- MuJoCo 功能回归（30s）：运行 `walk_mpc_wbc_leg`，验证 `Space/WASD/QE/J/H` 行为正常，`qpStatus_MPC` 无持续异常，步态切换无明显抖振/跌倒。
- 数据契约验证（日志）：确认控制前端输入满足“真机等价”约束（`basePos/baseLinVel/fL/fR` 不再作为控制有效输入）；`base_pos_est/base_vel_est/base_rpy` 连续可用并驱动后续模块。
- 力矩语义验证：确认 `motors_tor_cur` 来源为接口层测得力矩（MuJoCo `qfrc_actuator`），不被 PVT 回写覆盖；`FL_est/FR_est` 能持续输出。
- ROS 兼容验证（仿真桥）：开启 `SIM_ENABLE_ROS2_STATE_PUB` 后检查 `/imu/data`、`/joint_states` 字段和关节顺序与 Python 脚本预期一致。

### Assumptions / Defaults
- 本阶段仅改 leg 链路，不改 Azure/v4 非 leg 可执行。
- 保持真机输出接口为位置命令（`/rl_motion_control_command` 29 维）不变。
- 真机暂不依赖足底触地传感，接触估计继续由状态估计/动力学链路提供。
- `ros2_real` backend 在同一时序重构后仅做一致性核对，不引入新的通信协议改动。
