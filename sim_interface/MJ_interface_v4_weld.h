/*
 * MJ_Interface for speedbot_v4_weld robot.
 * Adapted from MJ_interface.h for OpenLoong.
 */
#pragma once

#include <mujoco/mujoco.h>
#include "data_bus.h"
#include <string>
#include <vector>

class MJ_Interface_V4_Weld {
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
    // Right-arm-only weld demo joint order.
    const std::vector<std::string> JointName={
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint", "welding_gun_joint"};
    const std::string baseName="base_link";

    MJ_Interface_V4_Weld(mjModel *mj_modelIn, mjData *mj_dataIn);
    void updateSensorValues();
    void setMotorsPos(std::vector<double> &posIn);
    void dataBusWrite(DataBus &busIn);

private:
    mjModel *mj_model;
    mjData  *mj_data;
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl;

    int baseBodyId;

    double timeStep{0.001};
    bool isIni{false};
};
