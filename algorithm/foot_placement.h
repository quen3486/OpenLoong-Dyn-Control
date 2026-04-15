/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#pragma once

#include <Eigen/Dense>
#include "data_bus.h"

class FootPlacement
{
public:
    double kp_vx{0}, kp_vy{0}, kp_wz{0};
    double legLength{1};
    double stepHeight{0.1};
    double xOffsetL{-0.07};
    double yOffsetL{0.04};
    double zOffsetW{-0.035};
    double swingTrajectoryPhase{0.2};
    double swingTrajectoryWindow{1.4};
    double zStretchStartPhi{0.98};
    double zStretchStep{-0.002};
    double zStretchMin{-0.05};
    double phi{0};      // phase varialbe for trajectory generation, must between 0 and 1
    double tSwing{0.4}; // swing time
    Eigen::Vector3d posStart_W, posDes_W, hipPos_W, STPos_W;
    Eigen::Vector3d desV_W, curV_W;
    double desWz_W;
    Eigen::Vector3d base_pos;
    double Trajectory(double phase, double des1, double des2);
    void getSwingPos();
    void dataBusRead(DataBus &robotState);
    void dataBusWrite(DataBus &robotState);
    DataBus::LegState legState;

private:
    double pDesCur[3]{0};
    double yawCur;
    double theta0;
    double omegaZ_W;
    double hip_width;
    bool stretchLeg{false};
    double zStretch{0};
    bool finish_Stretch;
};
