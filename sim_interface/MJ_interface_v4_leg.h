/*
 * MJ_Interface for speedbot_v4 leg-only robot.
 * Adapted from MJ_interface_v4.h for OpenLoong.
 */
#pragma once

#include <mujoco/mujoco.h>
#include "data_bus.h"
#include <string>
#include <vector>

class MJ_Interface_V4_Leg {
public:
    int jointNum{0};
    std::vector<double> motor_pos;
    std::vector<double> motor_pos_Old;
    std::vector<double> motor_vel;
    std::vector<double> motor_tor_mea_link;
    double rpy[3]{0};
    double yaw_simgle;
    int    yaw_N = 0;
    double baseQuat[4]{0};
    double f3d[3][2]{0};
    double basePos[3]{0};
    double baseAcc[3]{0};
    double baseAngVel[3]{0};
    double baseLinVel[3]{0};
    // Joint names in pinocchio order: leg_l(6), leg_r(6) = 12 joints.
    const std::vector<std::string> JointName={
        "left_hip_roll_joint", "left_hip_yaw_joint", "left_hip_pitch_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};
    const std::string baseName="base_link";
    const std::string orientationSensorName="baselink-quat";
    const std::string velSensorName="baselink-velocity";
    const std::string gyroSensorName="baselink-gyro";
    const std::string accSensorName="baselink-baseAcc";
    const std::string touchSensorLName="lf-touch";
    const std::string touchSensorRName="rf-touch";

    MJ_Interface_V4_Leg(mjModel *mj_modelIn, mjData *mj_dataIn);
    void updateSensorValues();
    void setMotorsTorque(std::vector<double> &tauIn);
    void dataBusWrite(DataBus &busIn);
    void getTruthSnapshot(double basePosOut[3],
                          double baseLinVelOut[3],
                          double rpyOut[3],
                          std::vector<double> &jointTorOut) const;

private:
    mjModel *mj_model;
    mjData  *mj_data;
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl;

    int orientataionSensorId;
    int velSensorId;
    int gyroSensorId;
    int accSensorId;
    int touchSensorLId;
    int touchSensorRId;
    int baseBodyId;
    int floorGeomId{-1};
    int leftFootBodyId{-1};
    int rightFootBodyId{-1};

    double timeStep{0.001};
    bool isIni{false};
};
