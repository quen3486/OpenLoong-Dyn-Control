# 真机控制模式说明（speedbot_v4_leg）

MPC 项目只负责真机反馈订阅和控制命令发布，不再启动 RViz、`robot_state_publisher`，也不再发布可视化用的 `/robot_description`、`/tf` 或仿真 `/imu/data`、`/joint_states`。RViz 可视化统一由控制器工程提供。

## 启动

```bash
cd /home/huangkun/workspaces/mpc/speedbot-dyn-control
./tools/start_control_v4_leg.sh
```

- 控制入口：`build/walk_mpc_wbc_leg --ros2-real`
- 配置文件：`common/controller_config_v4_leg.json`
- 启动后默认只订阅数据，不发布控制命令；按 `P` 才开始发布，再按 `P` 停发。
- 按 `P` 开始发布后仍处于开环站姿；确认稳定后再按 `F` 进入闭环站立。

## 话题约定

- 输入 IMU：`/imu/data`
- 输入关节状态：`/joint_states`
- 输出动作命令：`/rl_motion_control_command`

动作命令类型为 `std_msgs/msg/Float64MultiArray`，长度固定 `69`，布局为 `[pos23][vel23][torque23]`。当前临时发布模式下 leg 版本写入前 12 个腿部关节，V4 版本写入 12 个腿部关节和 10 个手臂关节；`waist_yaw_joint` 固定为 0，速度和前馈力矩段全部临时置 0。

## 单关节/多关节位置测试

可使用 `tools/real_robot/publish_joint_position_command.py` 直接按真机控制协议发布位置命令：

```bash
cd /home/huangkun/workspaces/mpc/speedbot-dyn-control
python3 tools/real_robot/publish_joint_position_command.py
```

直接在脚本顶部修改 `JOINT_ANGLE_DEG` 字典，单位为度。脚本发布 `/rl_motion_control_command` 的 69 维命令，格式为 `[pos23][zero_vel23][zero_torque23]`；发布到 ROS2 的 `pos23` 仍按控制器协议自动转换为 rad。默认从 `/joint_states` 读取当前 23 个关节位置作为插值起点，500Hz 发布，3 秒平滑过渡，随后保持 10 秒。

常用选项：

```bash
# 只检查 JOINT_ANGLE_DEG、关节名和限位，不发布
python3 tools/real_robot/publish_joint_position_command.py --dry-run

# 过渡 5 秒并持续发布，直到 Ctrl+C
python3 tools/real_robot/publish_joint_position_command.py --ramp 5 --hold -1

# 交互点动模式：启动后输入 joint <关节名/编号> 选关节，输入 + 或 - 点动
python3 tools/real_robot/publish_joint_position_command.py --jog --joint right_shoulder_pitch_joint --step 3

# 右臂顺序诊断：每次点动后打印右臂 5 个反馈变化量
python3 tools/real_robot/publish_joint_position_command.py --jog --diagnose --joint right_shoulder_pitch_joint --step 3
```

`--jog` 模式不会应用脚本顶部的 `JOINT_ANGLE_DEG`，而是从当前 `/joint_states` 位置开始持续发布。交互命令需要回车确认：

```text
list                         # 显示 command index、EtherCAT position、目标位置、反馈位置
order                        # 显示右臂命令 data[18..22] 与 EtherCAT 20..24 对应关系
joint 18                     # 按命令 data index 选择 right_shoulder_pitch_joint
ec 20                        # 按 EtherCAT position 选择 right_shoulder_pitch_joint
joint ec:20                  # 同上，适合 --joint ec:20
joint right_elbow_joint      # 按关节名选择
+                            # 当前关节目标位置增加 step
-                            # 当前关节目标位置减少 step
step 1                       # 修改点动步长(deg)
set 30                       # 直接设置当前关节目标位置(deg)
show                         # 显示当前选中关节
q                            # 退出
```

如果 `--diagnose` 输出中最大反馈变化关节不是当前选择的关节，说明命令 topic 到真实电机之间存在右臂物理映射错位，需要按诊断结果修正控制器工程里的 EtherCAT 站号/关节名映射。

注意：`joint 20` 表示命令数组 `data[20]`，也就是 `right_shoulder_yaw_joint`；不是 EtherCAT position 20。若要按控制器配置注释中的 EtherCAT position 20 选择，请输入 `ec 20` 或直接使用关节名。

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
