# ROS2 机器人控制流程分析

> 文档生成时间：2026-05-13  
> 工作空间：`/home/huangkun/workspaces/ros2_ws`

---

## 一、整体架构

```
[rl_deploy Python 推理节点]
        │
        │  /rl_motion_control_command  (std_msgs/Float64MultiArray)
        ↓
[mit_controller  (ros2_control Controller Plugin)]
        │
        │  HardwareInterface: kp / kd / position / velocity / effort
        ↓
[EthercatDriver  (ethercat_driver/EthercatDriver)]
        │
        │  EtherCAT PDO  @ 500 Hz
        ↓
[物理电机  (TI CRA，MIT 力控模式，13 个关节)]
        │
        │  position / velocity / effort 状态回读
        ↓
[joint_state_broadcaster → /joint_states]
        ↑
        │  /joint_states  +  /imu/data
        │
[rl_deploy Python 推理节点]  ← 形成闭环
```

---

## 二、启动流程

### 2.1 ec_driver.sh

```bash
sudo /etc/init.d/ethercat stop
sudo /etc/init.d/ethercat start
ros2 launch ethercat_motor_drive motor_drive.launch.py
```

### 2.2 motor_drive.launch.py 启动的节点

| 节点 | 说明 |
|---|---|
| `ros2_control_node` | 加载机器人描述（xacro）+ controllers.yaml，500 Hz 控制循环 |
| `robot_state_publisher` | 发布 TF 变换 |
| `joint_state_broadcaster` | 将 state interface 广播为 `/joint_states` |
| `mit_controller` | 订阅 RL 命令，写入 command interface |

### 2.3 硬件描述层

文件：`description/ros2_control/motor_drive.ros2_control.xacro`

每个关节声明：

| 类型 | 接口名 |
|---|---|
| command_interface | `kp`, `kd`, `position`, `velocity`, `effort` |
| state_interface | `position`, `velocity`, `effort` |

底层插件：`ethercat_generic_plugins/EcCiA402Drive`（MIT 力控电机驱动）  
配置文件：`config/slave_config/ti/ti_cra_config-pt.yaml`

---

## 三、mit_controller 详细流程

### 3.1 关节列表（controllers.yaml 顺序）

```
index  0: left_hip_roll_joint      (EtherCAT ID 7)
index  1: left_hip_yaw_joint       (EtherCAT ID 8)
index  2: left_hip_pitch_joint     (EtherCAT ID 9)
index  3: left_knee_joint          (EtherCAT ID 10)
index  4: left_ankle_pitch_joint   (EtherCAT ID 11)
index  5: left_ankle_roll_joint    (EtherCAT ID 12)
index  6: right_hip_roll_joint     (EtherCAT ID 1)
index  7: right_hip_yaw_joint      (EtherCAT ID 2)
index  8: right_hip_pitch_joint    (EtherCAT ID 3)
index  9: right_knee_joint         (EtherCAT ID 4)
index 10: right_ankle_pitch_joint  (EtherCAT ID 5)
index 11: right_ankle_roll_joint   (EtherCAT ID 6)
index 12: waist_yaw_joint          (EtherCAT ID 13)
```

### 3.2 生命周期

```
on_init()
  └─ 读取 joints 参数（13 个关节名）

on_configure()
  ├─ 订阅 /rl_motion_control_command  → rlMotionControlCallback()
  ├─ 订阅 /imu/data                   → imuCallback()
  └─ 加载 RL 配置（各关节 stiffness / damping，来自 controllers.yaml）

on_activate()
  ├─ 绑定 command interfaces（kp / kd / position / velocity / effort）
  ├─ 绑定 state interfaces（position / velocity / effort）
  └─ 写入初始 kp / kd

update() @ 500 Hz
  ├─ 广播 TF（base_link，IMU 四元数姿态）
  │
  ├─ [DEFAULT 模式] 平滑过渡阶段
  │    目标：从当前关节位置线性插值到 default_angles
  │    最大角速度限制：0.6 rad/s × 0.005 s = 0.003 rad/步
  │    插值总步数 = max(|Δθ_i| / 0.003)，各关节同步完成
  │    插值完成后自动切换到 RL_MOTION_CONTROL 模式
  │    打印日志："Controller is ready!!!"
  │
  └─ [RL_MOTION_CONTROL 模式] 正常运行阶段
       update() 本身为空循环
       命令值由 rlMotionControlCallback() 异步写入

rlMotionControlCallback()  ← 异步回调（由 ROS2 executor 触发）
  ├─ 只接受长度 69 的 Float64MultiArray：[pos23][vel23][torque23]
  ├─ 检查所有数据 finite、接口已绑定、控制器已进入 RL_MOTION_CONTROL
  └─ 对 23 个控制关节 i 写入 command interface：
       joint_pos_cmd_if_[i]  = msg->data[i]       （目标位置，rad）
       joint_vel_cmd_if_[i]  = msg->data[23 + i]  （目标速度，rad/s）
       joint_eff_cmd_if_[i]  = msg->data[46 + i]  （前馈力矩，N*m）
       joint_kp_cmd_if_[i]   = stiffness[joint_names_[i]]
       joint_kd_cmd_if_[i]   = damping[joint_names_[i]]
     当前 MPC 临时发布模式下，速度和前馈力矩段全部为 0。
```

### 3.3 电机端扭矩计算（MIT 力控公式）

命令由 EtherCAT 电机驱动器内部执行：

$$\tau = kp \times (pos_{des} - pos_{actual}) + kd \times (vel_{des} - vel_{actual}) + \tau_{ff}$$

当前 MPC 真机开环链路先只验证位置顺序，发布的 $vel_{des}$ 和 $\tau_{ff}$ 临时为 0；底层仍按 MIT 形式执行。

### 3.4 controllers.yaml 关键 KP/KD 值

| 关节组 | KP | KD |
|---|---|---|
| 髋 roll（左/右） | 220.0 | 2.0 |
| 髋 yaw（左/右） | 150.0 | 2.0 |
| 髋 pitch（左/右） | 150.0 | 2.0 |
| 膝（左/右） | 180.0 | 4.0 |
| 踝 pitch（左/右） | 10.0 | 5.0 |
| 踝 roll（左/右） | 10.0 | 5.0 |
| 腰 yaw | 500.0 | — |

---

## 四、`/rl_motion_control_command` 数据结构

**消息类型**：`std_msgs/Float64MultiArray`  
**发布节点**：`walk_mpc_wbc_leg --ros2-real`、`walk_mpc_wbc_leg --sim-real-openloop`、`walk_mpc_wbc_v4 --sim-real-openloop`  
**订阅节点**：`mit_controller`（`JointGroupMITController`）  
**总长度**：**69 个 float64**

### 4.1 数据布局

```
msg.data = [pos23] + [vel23] + [torque23]
```

| 索引范围 | 内容 | 说明 |
|---|---|---|
| [0..11] | 12 个腿关节目标位置 | 单位 rad |
| [12] | waist_yaw 目标位置 | 当前固定 0 |
| [13..17] | 左臂 5 关节目标位置 | leg 版本为 0，V4 版本来自 MPC/WBC |
| [18..22] | 右臂 5 关节目标位置 | leg 版本为 0，V4 版本来自 MPC/WBC |
| [23..45] | 23 关节目标速度 | 当前临时全 0 |
| [46..68] | 23 关节前馈力矩 | 当前临时全 0 |

23 个控制关节顺序固定为：

```
left_hip_roll_joint, left_hip_yaw_joint, left_hip_pitch_joint,
left_knee_joint, left_ankle_pitch_joint, left_ankle_roll_joint,
right_hip_roll_joint, right_hip_yaw_joint, right_hip_pitch_joint,
right_knee_joint, right_ankle_pitch_joint, right_ankle_roll_joint,
waist_yaw_joint,
left_shoulder_pitch_joint, left_shoulder_roll_joint, left_shoulder_yaw_joint,
left_elbow_joint, left_wrist_roll_joint,
right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint,
right_elbow_joint, right_wrist_roll_joint
```

### 4.2 MPC 目标生成过程

`runControlPipeline()` 生成：

```cpp
RobotState.motors_pos_des = integrateDIY(q, wbc_delta_q_final)[7..18];
RobotState.motors_vel_des = wbc_dq_final;
RobotState.motors_tor_des = wbc_tauJointRes;
```

随后 `ROS2_Interface_V4_Leg::setMotorsCommand()` 固定发布 69 维：

```cpp
// leg: q_des[0..11] -> msg.data[0..11], 其余位置段为 0
// V4:  q_des[0..11] -> msg.data[0..11]
//      q_des[12..16] -> msg.data[13..17]
//      q_des[17..21] -> msg.data[18..22]
// msg.data[12] 固定为 waist_yaw=0
// msg.data[23..68] 当前临时全 0
```

### 4.3 观测向量组成

**单帧维度**：45，**历史长度**：6，**总观测维度**：270

```
single_obs[ 0: 3]  = omega * 0.25            IMU 角速度（滑动窗口均值滤波，窗口=10）
single_obs[ 3: 6]  = gravity_orientation     重力方向向量（由四元数计算）
single_obs[ 6: 9]  = cmd * [2.0, 2.0, 0.25] 速度指令（vx, vy, yaw_rate）
single_obs[ 9:21]  = (qj - default) * 1.0   关节位置偏差（12关节）
single_obs[21:33]  = dqj * 0.05             关节速度（12关节）
single_obs[33:45]  = last_action            上一步网络输出动作（12维）
```

重力方向向量计算公式（将世界系重力向量转到机体系）：

```python
gravity_orientation[0] = 2 * (-qz*qx + qw*qy)
gravity_orientation[1] = -2 * (qz*qy + qw*qx)
gravity_orientation[2] = 1 - 2 * (qw*qw + qz*qz)
```

---

---

## 六、/joint_states → rl_deploy 状态回读

### 6.1 关节顺序映射

`/joint_states` 发布全量关节（顺序由 hardware 层决定），Python 通过索引映射提取腿部关节：

```python
# controller_ros2_amp3.py 中的完整 ros_joint_names 列表（29个关节）
ros_joint_names = [
    "left_ankle_pitch_joint",   # 0
    "left_ankle_roll_joint",    # 1
    "right_hip_yaw_joint",      # 2
    "right_hip_pitch_joint",    # 3
    "right_ankle_roll_joint",   # 4
    "WAIST_R",                  # 5
    "left_hip_yaw_joint",       # 6
    "left_hip_pitch_joint",     # 7
    "right_knee_joint",         # 8
    "R_SHOULDER_P",             # 9
    "L_SHOULDER_P",             # 10
    "L_WRIST_Y",                # 11
    "left_knee_joint",          # 12
    "L_SHOULDER_Y",             # 13
    "R_SHOULDER_Y",             # 14
    "left_hip_roll_joint",      # 15
    "waist_yaw_joint",          # 16
    ...                         # 其余手臂关节
]

# 目标顺序（控制顺序）
joint_names = [
    'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_hip_pitch_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_hip_pitch_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint'
]

# 构建重排索引
joint_indices[i] = ros_joint_names.index(joint_names[i])

# 提取状态（自动完成顺序重排）
qj     = msg.position[joint_indices]   # 关节位置（rad）
dqj    = msg.velocity[joint_indices]   # 关节速度（rad/s）
effort = msg.effort[joint_indices]     # 实际力矩（Nm，用于监控/前馈计算）
```

---

## 七、关键参数汇总

### 7.1 taihu_real_amp.yaml

| 参数 | 值 | 说明 |
|---|---|---|
| `control_hz` | 70 Hz | RL 推理发布频率 |
| `amp_action_scale` | 0.25 | 网络输出动作缩放系数 |
| `amp_ang_vel_scale` | 0.25 | 角速度观测缩放 |
| `amp_dof_pos_scale` | 1.0 | 关节位置观测缩放 |
| `amp_dof_vel_scale` | 0.05 | 关节速度观测缩放 |
| `amp_obs_history_len` | 6 帧 | 观测历史长度 |
| `amp_single_num_obs` | 45 | 单帧观测维度 |
| `amp_default_angles` | 全 0 | 关节默认角度（rad） |
| `amp_kps` | [150,150,150,240,60,40] × 2 | Python 侧 PD（仅用于前馈力矩计算，不写入硬件） |
| `amp_kds` | [2.5,2.5,2.5,4,2,2] × 2 | 同上 |
| `period` | 0.8 s | 步态相位周期 |
| `omega_history_len` | 10 | 角速度滑动均值窗口 |
| `cur_action_weight` | 1.0 | 动作平滑系数（1.0 = 不平滑） |

### 7.2 controllers.yaml（硬件侧，实际生效）

| 参数 | 值 |
|---|---|
| `update_rate` | 500 Hz |
| `cpu_affinity` | 8（绑定 CPU 核） |
| `thread_priority` | 90 |
| `lock_memory` | true（实时性保障） |

> **重要区分**：  
> - `taihu_real_amp.yaml` 的 `amp_kps/kds` 是 Python 推理节点内部参数，用于计算前馈力矩（仅 `controller_ros2_amp2.py` 使用）  
> - `controllers.yaml` 的 `stiffness/damping` 才是**实际写入电机驱动器的 KP/KD**，由 `mit_controller` 读取并通过 command interface 下发

---

## 八、数据流时序

```
t=0ms  EtherCAT 周期开始（2ms = 500Hz）
       ├─ 读取各关节 PDO：position / velocity / effort
       └─ 写入上一周期的 kp/kd/pos/vel/eff 命令到驱动器

t≈0ms  joint_state_broadcaster 广播 /joint_states

t≈1ms  walk_mpc_wbc_leg 收到 /imu/data 与 /joint_states
       ├─ 按关节名重排 12 个腿关节 position / velocity / effort
       ├─ StateEst / Pin_KinDyn 更新状态估计和动力学量
       ├─ MPC/WBC 生成 motors_pos_des / motors_vel_des / motors_tor_des
       └─ 按 P 门控发布 /rl_motion_control_command（69个float：[pos23][vel23][torque23]）

t≈1ms  mit_controller 回调触发（异步）
       └─ 写入 command interfaces：pos/vel/eff/kp/kd
          （在下一个2ms EtherCAT 周期生效）
```

---

*文档基于源码分析生成，如代码更新请同步修订。*

#--sim-real- 和 --ros2-real 