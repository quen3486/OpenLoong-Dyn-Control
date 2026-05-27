/*
 * Fixed-base 6-DoF kinematics helper for the speedbot_v4_weld right-arm demo.
 */
#pragma once

#include "json/json.h"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <Eigen/Dense>
#include <string>
#include <vector>

class Pin_KinDyn_V4_Weld
{
public:
    struct IkRes
    {
        int status{-1};
        int itr{0};
        Eigen::VectorXd err{Eigen::VectorXd::Zero(6)};
        Eigen::VectorXd jointPosRes{Eigen::VectorXd::Zero(6)};
    };

    const std::vector<std::string> motorName = {
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint", "welding_gun_joint"};

    std::vector<bool> motorReachLimit;
    Eigen::VectorXd motorMaxPos;
    Eigen::VectorXd motorMinPos;

    pinocchio::Model model_biped_fixed;
    int model_nv{6};
    pinocchio::FrameIndex welding_tcp_frame_fixed{0};

    explicit Pin_KinDyn_V4_Weld(std::string urdf_pathIn);

    Eigen::VectorXd clampRightArmQ(const Eigen::VectorXd &rightArmQ) const;
    pinocchio::SE3 computeRightHandPoseFixed(const Eigen::VectorXd &rightArmQ);
    Eigen::Vector3d computeRightHandPosFixed(const Eigen::VectorXd &rightArmQ);
    IkRes computeRightHandPoseIK(const Eigen::Matrix3d &Rdes_R,
                                 const Eigen::Vector3d &Pdes_R,
                                 const Eigen::VectorXd &qSeedFixed);

private:
    pinocchio::Data data_biped_fixed;
};
