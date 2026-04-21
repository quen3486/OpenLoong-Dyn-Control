# MuJoCo 仿真 ROS2 状态发布 + RViz 可视化（v4 / v4_leg）

当前已统一为单一启动脚本：`tools/start_control_v4_leg.sh`。

## 一键启动（推荐）

```bash
cd /home/huangkun/workspaces/mpc/Openloong-dyn-control
CONTROL_MODE=mujoco_ros2 START_RVIZ=1 ./tools/start_control_v4_leg.sh
```

- 默认机型：`ROBOT_VARIANT=v4`（可切到 `leg`）
- v4 示例：`CONTROL_MODE=mujoco_ros2 ROBOT_VARIANT=v4 START_RVIZ=1 ./tools/start_control_v4_leg.sh`
- leg 示例：`CONTROL_MODE=mujoco_ros2 ROBOT_VARIANT=leg START_RVIZ=1 ./tools/start_control_v4_leg.sh`
- 通过环境变量切到 `mujoco + ROS2状态发布`，避免重复配置文件漂移
- 可用 `START_RVIZ=0` 只跑仿真+ROS2发布

## 发布话题

- IMU：`/imu/data`（`sensor_msgs/Imu`）
- 关节状态：`/joint_states`（`sensor_msgs/JointState`）
- TF：`/tf`（`world -> base_link`）

## 常用环境变量

- `CONTROL_MODE`：`mujoco_ros2|mujoco|ros2_real`
- `ROBOT_VARIANT`：`v4|leg`（`mujoco` 与 `mujoco_ros2` 均支持；`ros2_real` 目前仅支持 `leg`）
- `TARGET`：兼容旧变量（建议迁移到 `ROBOT_VARIANT`）
- `OPENLOONG_CONTROLLER_CONFIG`：配置路径（随机型默认切换：`v4 -> controller_config_v4.json`，`leg -> controller_config_v4_leg.json`）
- `SIM_ROS_PUBLISH_DT`：ROS2 发布周期（默认 `0.01`）
- `AUTOWALK`：自动起步（默认 `0`，用配置默认速度）
- `START_RVIZ`：是否启动 RViz（`1/0`）

## 话题连通性检查

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | rg "/imu/data|/joint_states|/tf"
ros2 topic hz /joint_states
```
