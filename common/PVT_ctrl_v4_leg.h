/*
 * PVT_Ctr_V4_Leg: PVT controller for speedbot_v4 leg-only robot.
 * Adapted from PVT_ctrl_v4.h for OpenLoong.
 */
#pragma once
#include <fstream>
#include "json/json.h"
#include <string>
#include "LPF_fst.h"
#include <vector>
#include <cmath>
#include "data_bus.h"

class PVT_Ctr_V4_Leg {
public:
    int jointNum;
    std::vector<double> motor_pos_cur;
    std::vector<double> motor_pos_des_old;
    std::vector<double> motor_vel;
    std::vector<double> motor_tor_out_link;
    std::vector<double> motor_tor_out_motor;
    PVT_Ctr_V4_Leg(double timeStepIn, const char * jsonPath);
    void calMotorsPVT();
    void calMotorsPVT(double deltaP_Lim);
    void enablePV();
    void disablePV();
    void enablePV(int jtId);
    void disablePV(int jtId);
    void setJointPD(double kp, double kd, const char * jointName);
    void applyClosedLoopPD();
    void dataBusRead(DataBus &busIn);
    void dataBusWrite(DataBus &busIn);

    std::vector<double> motor_pos_des;
    std::vector<double> motor_vel_des;
    std::vector<double> motor_tor_des;

    std::vector<double> pvt_Kp;
    std::vector<double> pvt_Kd;
    std::vector<double> closedLoopKp;
    std::vector<double> closedLoopKd;
    std::vector<double> maxTor;
    std::vector<double> maxVel;
    std::vector<double> maxPos;
    std::vector<double> minPos;
    std::vector<double> gear;

private:
    std::vector<LPF_Fst> tau_out_lpf;
    std::vector<int> PV_enable;
    double sign(double in);
    // Pinocchio order: leg_l(6), leg_r(6) = 12 joints.
    const std::vector<std::string> motorName={
        "left_hip_roll_joint","left_hip_yaw_joint","left_hip_pitch_joint",
        "left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
        "right_hip_roll_joint","right_hip_yaw_joint","right_hip_pitch_joint",
        "right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint"};
};
