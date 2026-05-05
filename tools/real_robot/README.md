# 真机模式使用说明（speedbot_v4_leg）

真机控制只保留一个统一入口：`tools/start_control_v4_leg.sh`。脚本固定启动 `ros2_real + leg + RViz`，启动后必须按 `G` 才发布控制命令。

## 1. 启动

```bash
cd /home/huangkun/workspaces/mpc/Openloong-dyn-control
./tools/start_control_v4_leg.sh
```

- 控制入口：`build/walk_mpc_wbc_leg`
- 配置文件：`common/controller_config_v4_leg.json`
- 关节限制与 PVT 安全估算参数：`common/joint_ctrl_config_v4_leg.json`
- 启动后默认只订阅数据，不发布控制命令；按 `G` 才开始发布，再按 `G` 立即停发。
- 按 `G` 开始发布后仍处于开环站姿；确认落地稳定后再按 `F` 进入闭环站立。

## 2. 话题约定

- IMU：`/imu/data`
- 关节状态：`/joint_states`
- 动作命令：`/rl_motion_control_command_with_torque`

动作命令类型为 `std_msgs/msg/Float64MultiArray`，长度固定 `58`：前 `29` 位为位置目标，后 `29` 位为力矩前馈。当前 leg 控制器只写入前 12 个腿部位置和 `29..40` 的腿部 WBC 力矩前馈，上肢位置与力矩均填 `0.0`。

## 3. 安全策略

安全检查覆盖 IMU 姿态/角速度、关节位置、关节速度、命令跳变，以及真机底层等效 PVT 合力矩：

```text
tau_est = kp * (q_des - q_cur) + kd * (0 - dq_cur) + tau_ff
```

`kp/kd/minPos/maxPos/maxSpeed/maxTorque` 统一来自 `common/joint_ctrl_config_v4_leg.json`。PVT 合力矩安全阈值为：

```text
abs(tau_est) <= maxTorque * real_pvt_torque_limit_scale
```

`real_pvt_torque_limit_scale` 在 `common/controller_config_v4_leg.json` 中配置，当前为 `0.6`。

任一安全检查失败后，控制器立即停发动作命令，不发布保持姿态或冻结姿态。恢复方式是人工停止进程、检查现场和日志后重新启动。
