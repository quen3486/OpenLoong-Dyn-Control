# 真机模式 RViz 使用说明（speedbot_v4_leg）

当前已统一为单一启动脚本：`tools/start_control_v4_leg.sh`。

## 1. 统一入口

```bash
cd /home/huangkun/workspaces/mpc/Openloong-dyn-control
CONTROL_MODE=ros2_real START_RVIZ=1 ./tools/start_control_v4_leg.sh
```

- 控制入口仍是原 demo：`build/walk_mpc_wbc_leg`
- 默认配置统一使用：`common/controller_config_v4_leg.json`
- 不再依赖独立的 `*_real.json` 启动配置

## 2. 话题约定（与真机驱动一致）

- IMU：`/imu/data`
- 关节状态：`/joint_states`
- 动作命令：`/rl_motion_control_command`

## 3. RViz 单独启动

```bash
cd /home/huangkun/workspaces/mpc/Openloong-dyn-control
OPENLOONG_ROS_TOPIC_IMU=/imu/data \
OPENLOONG_ROS_TOPIC_JOINT_STATES=/joint_states \
./tools/real_robot/start_rviz_real_leg.sh
```

## 4. 常用环境变量

- `CONTROL_MODE`：`ros2_real|mujoco|mujoco_ros2`
- `ROBOT_VARIANT`：`v4|leg`（`mujoco` 与 `mujoco_ros2` 均支持；`ros2_real` 目前仅支持 `leg`）
- `TARGET`：兼容旧变量（建议迁移到 `ROBOT_VARIANT`）
- `OPENLOONG_CONTROLLER_CONFIG`：配置路径（默认 `common/controller_config_v4_leg.json`）
- `START_RVIZ`：是否启动 RViz（`1/0`）
- `AUTOWALK`：真机自动起步（默认 `0`，安全默认）
- `OPENLOONG_ROS_TOPIC_IMU`：IMU 话题
- `OPENLOONG_ROS_TOPIC_JOINT_STATES`：关节状态话题
- `OPENLOONG_ROBOT_URDF`：URDF 路径
- `OPENLOONG_RVIZ_CONFIG`：RViz 配置路径
