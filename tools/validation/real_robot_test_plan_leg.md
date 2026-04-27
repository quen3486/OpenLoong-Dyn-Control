# speedbot_v4_leg 真机测试计划

本文目标：使用唯一真机启动流程验证输入链路、G 发布门控、闭环站立和低速行走安全性。

控制流基线：
`/imu/data + /joint_states -> ROS2_interface_v4_leg::dataBusWrite -> StateEst/Pin_KinDyn -> 状态机 -> MPC/WBC -> /rl_motion_control_command_with_torque`

## 阶段 R0：静态预检

```bash
cd /home/huangkun/workspaces/mpc/Openloong-dyn-control
cmake --build build -j4
./tools/validation/check_ros2_real_leg_contract.sh
```

通过门槛：
- 构建成功。
- 合约检查 PASS，包含 58 长度动作话题、G 门控、默认停发、PVT 合力矩安全阈值、JSON/URDF limit 一致性。

## 阶段 R1：只订阅启动

```bash
./tools/start_control_v4_leg.sh
```

脚本固定为 `ros2_real + leg + RViz + AUTOWALK=0`。启动后不要按 `G`。

检查：
- `ros2 topic hz /imu/data`
- `ros2 topic hz /joint_states`
- `/rl_motion_control_command_with_torque` 不应有新的控制命令发布。
- RViz 姿态显示与现场姿态一致。

## 阶段 R2：开环发布与落地

1. 机器人悬吊或安全支撑，电机处于低风险状态。
2. 按 `G` 开始发布动作命令。
3. 确认输出为 58 长度，初始开环站姿下力矩前馈为 0。
4. 缓慢落地轻载，确认关节方向、IMU 朝向、双脚接触和姿态稳定。

若出现位置、速度、PVT 合力矩或姿态超限，控制器会立即停发控制消息；此时停止进程并人工检查，不做在线恢复。

## 阶段 R3：闭环站立

在 R2 稳定后按 `F` 进入闭环站立，保持 10~20 秒。

通过门槛：
- 无快速跌倒、无明显高频抖振。
- `qpStatus_MPC` 非零占比 <= 1%。
- `base_rpy` 建议保持在 |roll|、|pitch| < 5 deg。
- PVT 合力矩安全检查不触发。

## 阶段 R4：低速行走

在闭环站立稳定后按 `Space` 进入行走，再依次小幅测试 `W/A/D/Q/E/H/J`。

通过门槛：
- `motionState` 正确流转：`Stand -> Walk -> Walk2Stand -> Stand`。
- 无持续 MPC QP 异常。
- 任何异常均以停发控制消息或人工急停结束，本轮不做自动恢复。

## 统一安全规则

1. 全程可触达急停。
2. 每次只改变一个变量。
3. 未完成 R1/R2，不进入闭环站立。
4. 未通过 R3，不进入行走。
5. 任一超限停发后，必须停止进程、检查现场和日志，再重新启动。
