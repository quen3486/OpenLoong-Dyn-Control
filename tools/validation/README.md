# Leg 迁移验证工具

本目录用于 `walk_mpc_wbc_leg` 的分步迁移回归和真机接口一致性核对。

## 1. 生成 Stage1/2/3 对比报告

脚本：`tools/validation/generate_leg_stage_compare_report.sh`

```bash
cd /home/huangkun/workspaces/mpc/Openloong-dyn-control

./tools/validation/generate_leg_stage_compare_report.sh \
  --stage1 record/datalog_stage1.log \
  --stage2 record/datalog_stage2.log \
  --stage3 record/datalog_stage3.log
```

- 默认输出：`record/leg_stage_compare_YYYYmmdd_HHMMSS.md`
- 可用 `--output record/leg_stage_compare_manual.md` 指定输出路径。

统计口径（默认）：
- warmup 后总段：`t >= 2s`
- 分段：
  - `open-loop`: `2-5s`
  - `closed-stand`: `5-10s`
  - `walk`: `10-26s`
- 指标：`est_err_pos/vel/yaw` 均值和最大值、`qpStatus_MPC` 非零占比、`motionState` 分布与 walk 时长。

## 2. 核对 ros2_real 数据合约

脚本：`tools/validation/check_ros2_real_leg_contract.sh`

```bash
cd /home/huangkun/workspaces/mpc/Openloong-dyn-control
./tools/validation/check_ros2_real_leg_contract.sh
```

检查项：
- 配置话题是否为 `/imu/data`、`/joint_states`、`/rl_motion_control_command_with_torque`
- 真机默认启动是否固定为 `ros2_real + leg + RViz + AUTOWALK=0`
- G 键发布门控、默认不发布、超限停发逻辑是否存在
- `joint_ctrl_config_v4_leg.json` 与 URDF 关节位置/速度/力矩限制是否一致
- `real_pvt_torque_limit_scale` 是否配置为默认 `0.5`
- 发布动作长度是否为 `58 (29位置+29力矩前馈)`
- DataBus 输入是否仅使用真机可得量（IMU + joint pos/vel/effort）
- 是否禁用 `basePos/baseLinVel/fL/fR` 真值注入
- `ros2_real` 主循环是否为统一时序：
  `dataBusWrite -> Estimation -> StateMachine -> MPC/WBC -> setMotorsCommand`
