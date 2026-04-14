/*
 * MJ_Interface for speedbot_v4 robot.
 * Adapted from MJ_interface.h for OpenLoong.
 */
#pragma once

#include <mujoco/mujoco.h>
#include "data_bus.h"
#include <string>
#include <vector>

class MJ_Interface_V4 {
public:
    int jointNum{0};
    std::vector<double> motor_pos;
    std::vector<double> motor_pos_Old;
    std::vector<double> motor_vel;
    double rpy[3]{0};
    double yaw_simgle;
    int    yaw_N = 0;
    double baseQuat[4]{0};
    double f3d[3][2]{0};
    double basePos[3]{0};
    double baseAcc[3]{0};
    double baseAngVel[3]{0};
    double baseLinVel[3]{0};
    // Joint names in pinocchio order (URDF tree traversal):
    // leg_l(6), leg_r(6), waist(1), arm_l(5), arm_r(5) = 23 joints
    const std::vector<std::string> JointName={
        "left_hip_roll_joint", "left_hip_yaw_joint", "left_hip_pitch_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_yaw_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint"};
    const std::string baseName="base_link";
    const std::string orientationSensorName="baselink-quat";
    const std::string velSensorName="baselink-velocity";
    const std::string gyroSensorName="baselink-gyro";
    const std::string accSensorName="baselink-baseAcc";

    MJ_Interface_V4(mjModel *mj_modelIn, mjData *mj_dataIn);
    void updateSensorValues();
    void setMotorsTorque(std::vector<double> &tauIn);
    void dataBusWrite(DataBus &busIn);

private:
    mjModel *mj_model;
    mjData  *mj_data;
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl;

    int orientataionSensorId;
    int velSensorId;
    int gyroSensorId;
    int accSensorId;
    int baseBodyId;

    double timeStep{0.001};
    bool isIni{false};
};
