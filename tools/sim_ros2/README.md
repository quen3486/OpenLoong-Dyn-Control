# MuJoCo 仿真 ROS2 状态发布 + RViz 可视化（v4 / v4_leg）

`tools/start_control_v4_leg.sh` 只用于真机 `ros2_real` 链路。仿真请直接运行对应 demo。

## 直接运行仿真

```bash
cd /home/huangkun/workspaces/mpc/Openloong-dyn-control/build

# v4
./walk_mpc_wbc_v4

# v4_leg
./walk_mpc_wbc_leg
```

需要发布 ROS2 状态时，在 `common/controller_config_v4.json` 或
`common/controller_config_v4_leg.json` 中将 `sim_enable_ros2_state_pub` 置为 `true`，
再直接运行 demo。

## RViz

另开终端启动 RViz。v4_leg 使用默认配置：

```bash
cd /home/huangkun/workspaces/mpc/Openloong-dyn-control
./tools/real_robot/start_rviz_real_leg.sh
```

v4 使用完整机型配置：

```bash
cd /home/huangkun/workspaces/mpc/Openloong-dyn-control
RVIZ_CONFIG=tools/real_robot/speedbot_v4.rviz \
ROBOT_URDF=models/speedbot_v4/speedbot_v4.urdf \
./tools/real_robot/start_rviz_real_leg.sh
```

## 发布话题

- IMU：`/imu/data`（`sensor_msgs/Imu`）
- 关节状态：`/joint_states`（`sensor_msgs/JointState`）
- TF：`/tf`（`world -> base_link`）

## 话题连通性检查

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | rg "/imu/data|/joint_states|/tf"
ros2 topic hz /joint_states
```
