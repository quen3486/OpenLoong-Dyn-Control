/*
 * PVT_Ctr_V4: PVT controller for speedbot_v4 robot.
 * Adapted from PVT_ctrl.h for OpenLoong.
 */
#pragma once
#include <fstream>
#include "json/json.h"
#include <string>
#include "LPF_fst.h"
#include <vector>
#include <cmath>
#include "data_bus.h"

class PVT_Ctr_V4 {
public:
    int jointNum;
    std::vector<double> motor_pos_cur;
    std::vector<double> motor_pos_des_old;
    std::vector<double> motor_vel;
    std::vector<double> motor_tor_out_link;
    std::vector<double> motor_tor_out_motor;
    PVT_Ctr_V4(double timeStepIn, const char * jsonPath);
    void calMotorsPVT();
    void calMotorsPVT(double deltaP_Lim);
    void enablePV();
    void disablePV();
    void enablePV(int jtId);
    void disablePV(int jtId);
    void setJointPD(double kp, double kd, const char * jointName);
    void dataBusRead(DataBus &busIn);
    void dataBusWrite(DataBus &busIn);

    std::vector<double> motor_pos_des;
    std::vector<double> motor_vel_des;
    std::vector<double> motor_tor_des;

    std::vector<double> pvt_Kp;
    std::vector<double> pvt_Kd;
    std::vector<double> maxTor;
    std::vector<double> maxVel;
    std::vector<double> maxPos;
    std::vector<double> minPos;
    std::vector<double> gear;

private:
    std::vector<LPF_Fst> tau_out_lpf;
    std::vector<int> PV_enable;
    double sign(double in);
    // Pinocchio order: leg_l(6), leg_r(6), waist(1), arm_l(5), arm_r(5) = 23 joints
    const std::vector<std::string> motorName={
        "left_hip_roll_joint","left_hip_yaw_joint","left_hip_pitch_joint",
        "left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
        "right_hip_roll_joint","right_hip_yaw_joint","right_hip_pitch_joint",
        "right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
        "waist_yaw_joint",
        "left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint",
        "left_elbow_joint","left_wrist_roll_joint",
        "right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
        "right_elbow_joint","right_wrist_roll_joint"};
};
