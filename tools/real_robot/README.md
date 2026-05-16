# 真机控制模式说明（speedbot_v4_leg）

MPC 项目只负责真机反馈订阅和控制命令发布，不再启动 RViz、`robot_state_publisher`，也不再发布可视化用的 `/robot_description`、`/tf` 或仿真 `/imu/data`、`/joint_states`。RViz 可视化统一由控制器工程提供。

## 启动

```bash
cd /home/huangkun/workspaces/mpc/speedbot-dyn-control
./tools/start_control_v4_leg.sh
```

- 控制入口：`build/walk_mpc_wbc_leg --ros2-real`
- 配置文件：`common/controller_config_v4_leg.json`
- 启动后默认只订阅数据，不发布控制命令；按 `G` 才开始发布，再按 `G` 停发。
- 按 `G` 开始发布后仍处于开环站姿；确认稳定后再按 `F` 进入闭环站立。

## 话题约定

- 输入 IMU：`/imu/data`
- 输入关节状态：`/joint_states`
- 输出动作命令：`/rl_motion_control_command`

动作命令类型为 `std_msgs/msg/Float64MultiArray`，长度固定 `69`，布局为 `[pos23][vel23][torque23]`。当前临时发布模式下 leg 版本写入前 12 个腿部关节，`waist_yaw_joint` 和 10 个手臂关节使用默认角/零速度/零前馈力矩。

## 安全策略

安全检查覆盖 IMU 姿态/角速度、关节位置、关节速度、命令跳变，以及真机底层等效 PVT 合力矩：

```text
tau_est = kp * (q_des - q_cur) + kd * (dq_des - dq_cur) + tau_ff
```

`kp/kd/minPos/maxPos/maxSpeed/maxTorque` 统一来自 `common/joint_ctrl_config_v4_leg.json`。PVT 合力矩安全阈值为：

```text
abs(tau_est) <= maxTorque * real_pvt_torque_limit_scale
```

任一安全检查失败后，MPC 立即停发动作命令。恢复方式是人工停止进程、检查现场和日志后重新启动。
