/*
 * Pin_KinDyn for speedbot_v4 robot.
 * Adapted from pino_kin_dyn.h for OpenLoong.
 *
 * Pinocchio joint ordering (URDF tree traversal):
 *   leg_l: 0-5, leg_r: 6-11, arm_l: 12-16, arm_r: 17-21
 *   q:  [base(7), leg_l(6), leg_r(6), arm_l(5), arm_r(5)] = 29
 *   dq: [base(6), leg_l(6), leg_r(6), arm_l(5), arm_r(5)] = 28
 */
#pragma once

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "data_bus.h"
#include <string>
#include "json/json.h"
#include <vector>

class Pin_KinDyn_V4
{
public:
    std::vector<bool> motorReachLimit;
    // Joint names in pinocchio order: leg_l(6), leg_r(6), arm_l(5), arm_r(5)
    const std::vector<std::string> motorName = {
        "left_hip_roll_joint", "left_hip_yaw_joint", "left_hip_pitch_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint"};
    Eigen::VectorXd motorMaxTorque;
    Eigen::VectorXd motorMaxPos;
    Eigen::VectorXd motorMinPos;

    Eigen::VectorXd tauJointOld;
    std::string urdf_path;
    pinocchio::Model model_biped;
    pinocchio::Model model_biped_fixed;
    int model_nv;
    pinocchio::JointIndex r_ankle_joint, l_ankle_joint, base_joint, r_hip_joint, l_hip_joint, r_hip_roll_joint, l_hip_roll_joint;
    pinocchio::JointIndex r_ankle_joint_fixed, l_ankle_joint_fixed, r_hip_joint_fixed, l_hip_joint_fixed;
    pinocchio::JointIndex r_hand_joint, l_hand_joint, r_hand_joint_fixed, l_hand_joint_fixed;
    Eigen::VectorXd q, dq, ddq;
    Eigen::Matrix3d Rcur;
    Eigen::Quaternion<double> quatCur;
    Eigen::Matrix<double, 6, -1> J_r, J_l, J_hd_r, J_hd_l, J_base, J_hip_link, J_r_body, J_l_body;
    Eigen::Matrix<double, 6, -1> dJ_r, dJ_l, dJ_hd_r, dJ_hd_l, dJ_base, dJ_hip_link;
    Eigen::Matrix<double, 3, -1> Jcom;
    Eigen::Vector3d fe_r_pos, fe_l_pos, base_pos;
    Eigen::Vector3d fe_r_pos_body, fe_l_pos_body;
    Eigen::Vector3d hd_r_pos, hd_l_pos;
    Eigen::Vector3d fe_r_vel_body, fe_l_vel_body;
    Eigen::Vector3d hd_r_pos_body, hd_l_pos_body;
    Eigen::Vector3d hip_r_pos, hip_l_pos, hip_link_pos;
    Eigen::Vector3d hip_r_pos_body, hip_l_pos_body;
    Eigen::Matrix3d hip_link_rot;
    Eigen::Matrix3d fe_r_rot, fe_l_rot, base_rot;
    Eigen::Matrix3d fe_r_rot_body, fe_l_rot_body;
    Eigen::Matrix3d hd_r_rot, hd_l_rot;
    Eigen::Matrix3d hd_r_rot_body, hd_l_rot_body;
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_G, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_Non;
    Eigen::Vector3d CoM_pos;
    Eigen::Matrix3d inertia;
    enum legIdx
    {
        left,
        right
    };
    struct IkRes
    {
        int status;
        int itr;
        Eigen::VectorXd err;
        Eigen::VectorXd jointPosRes;
    };

    Pin_KinDyn_V4(std::string urdf_pathIn);
    void dataBusRead(DataBus const &robotState);
    void dataBusWrite(DataBus &robotState);
    void computeJ_dJ();
    void computeDyn();
    IkRes computeInK_Leg(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R, const Eigen::Vector3d &Pdes_R);
    IkRes computeInK_Hand(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R, const Eigen::Vector3d &Pdes_R);
    Eigen::VectorXd integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI);
    static Eigen::Quaterniond intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w);
    void workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT);

private:
    pinocchio::Data data_biped, data_biped_fixed;
};
