# speedbot_v4_leg 真机测试计划（分阶段控制流核对）

本文目标：在 `ros2_real` 模式下，分阶段验证控制流正确、输入输出一致、稳定性达标，并给出故障排查路径。

控制流基线：
`/imu/data + /joint_states -> ROS2_interface_v4_leg::dataBusWrite -> StateEst/Pin_KinDyn -> 状态机 -> MPC/WBC -> /rl_motion_control_command`

## 阶段 R0：静态与环境预检（上机前）

目标：确认代码与接口合约无偏差。

执行：
1. `cmake --build build -j4`
2. `./tools/validation/check_ros2_real_leg_contract.sh`
3. 确认配置话题：`/imu/data`、`/joint_states`、`/rl_motion_control_command`

通过门槛：
- 构建成功。
- 合约检查 `PASS`。

失败排查：
- 话题不匹配：先查 `common/controller_config_v4_leg.json` 与现场驱动配置。
- 合约脚本失败：先修复接口，再上机。

## 阶段 R1：真机输入链路核对（电机不上使能/安全支撑）

目标：验证传感输入被控制器正确接收，且指令输出格式正确。

执行：
1. 启动控制（安全模式）：
   `CONTROL_MODE=ros2_real AUTOWALK=0 START_RVIZ=0 ./tools/start_control_v4_leg.sh`
2. 另开终端检查 topic：
   - `ros2 topic hz /imu/data`
   - `ros2 topic hz /joint_states`
   - `ros2 topic echo /rl_motion_control_command --once`
3. 检查输出向量长度：应为 `29 (12+17)`。

通过门槛：
- `/imu/data`、`/joint_states` 频率稳定（建议 >= 200 Hz，目标 400~1000 Hz）。
- `/rl_motion_control_command` 连续发布，长度 29。
- 控制进程无崩溃、无异常退出。

失败排查：
- 无动作输出：检查 `isReady/hasFreshData`，常见是关节名不匹配或数据超时。
- 长度异常：检查 `ROS2_interface_v4_leg.cpp` 的 `msg.data.assign(29, 0.0)`。

## 阶段 R2：闭环站立验证（5s open-loop + 10s closed stand）

目标：验证状态机切换与站立稳定，不引入异常控制量。

执行：
1. 上机前准备：安全绳、急停、软限位确认。
2. 运行控制：`CONTROL_MODE=ros2_real AUTOWALK=0 ./tools/start_control_v4_leg.sh`
3. 人工按键：
   - 初始 open-loop 约 5s。
   - 按 `F` 切换闭环站立，保持 10s。
4. 记录 `record/datalog.log`。

通过门槛：
- 无快速跌倒、无明显高频抖振。
- `qpStatus_MPC` 非零占比 <= 1%。
- `base_rpy` 在可控范围（建议 |roll|、|pitch| < 5 deg）。

失败排查：
- 切换瞬间冲击大：检查 `StateEst.init` 对齐时刻与 `setIniPos` 时序。
- 站立发散：先核对 IMU 朝向、关节顺序、关节力矩方向。

## 阶段 R3：低速行走控制流验证（15~20s）

目标：验证从站立到行走的整链路与命令响应。

执行：
1. 在 R2 通过后进行。
2. 按键序列：`Space -> W -> A -> D -> Q -> E -> H -> J`。
3. 行走持续 15~20s，随后 `J` 急停回站立。
4. 记录日志与现场视频。

通过门槛：
- `motionState` 正确流转：`Stand -> Walk -> Walk2Stand -> Stand`。
- 无持续 MPC QP 异常（`qpStatus_MPC` 非零占比 <= 1%）。
- 指令响应与按键一致，无明显延迟或反向。

失败排查：
- 转向/速度方向不对：查 `JoyStickInterpreter` 与驱动坐标约定。
- 行走中抖振：先降速（`Q`），再看 `wbc_swing_kp/kd` 与关节 PD。

## 阶段 R4：扰动与鲁棒性验证

目标：验证小扰动恢复能力和超时保护。

执行：
1. 闭环站立/慢走时施加小扰动（人工轻推）。
2. 人工制造短时传感中断（可控条件下），观察 stale 保护。

通过门槛：
- 扰动后可恢复，不出现持续发散。
- 传感中断时进入保护行为（保持安全站立命令），恢复后可继续。

失败排查：
- 恢复慢：优先检查 IMU 延迟、`ros_data_timeout_sec` 与控制周期抖动。

## 阶段 R5：长时运行与回归封版

目标：验证可持续运行，形成封版基线。

执行：
1. 连续运行 5~10 分钟（站立+慢走）。
2. 保存日志并归档：`record/datalog_*.log`。
3. 记录版本信息（提交号、配置文件、驱动版本）。

通过门槛：
- 无崩溃、无内存异常、无控制流中断。
- 关键指标稳定，无明显漂移。

## 统一安全规则（所有阶段）

1. 必须全程可触达急停。
2. 每次仅改一个变量（参数或流程），禁止多变量同时修改。
3. 任一阶段出现异常，回退到上一个通过阶段重测。
4. 未通过 R2，不进入 R3。

## 现场快速命令清单

```bash
# 1) 启动控制
cd /home/huangkun/workspaces/mpc/Openloong-dyn-control
CONTROL_MODE=ros2_real AUTOWALK=0 START_RVIZ=0 ./tools/start_control_v4_leg.sh

# 2) topic 频率检查
ros2 topic hz /imu/data
ros2 topic hz /joint_states

# 3) 动作输出检查
ros2 topic echo /rl_motion_control_command --once

# 4) 日志核对（示例）
awk -F',' '{if($176!=0) bad++; n++} END{printf("qp_nonzero_ratio=%.6f (%d/%d)\n", bad/n, bad, n)}' record/datalog.log
```
