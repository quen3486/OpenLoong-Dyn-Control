/*
 * WBC_priority_V4: Whole-body controller for speedbot_v4 robot.
 * Adapted from wbc_priority.h for OpenLoong.
 */
#pragma once

#include "qpOASES.hpp"
#include <algorithm>
#include <Eigen/Dense>
#include "data_bus.h"
#include "useful_math.h"
#include "priority_tasks.h"
#include "pino_kin_dyn_v4.h"
#include <iostream>
#include <iomanip>

class WBC_priority_V4 {
public:
    int model_nv;
    Eigen::Vector3d tau_upp_stand_L, tau_low_stand_L;
    Eigen::Vector3d tau_upp_walk_L, tau_low_walk_L;
    double f_z_low{0},f_z_upp{0};
    DataBus::LegState legStateCur;
    DataBus::MotionState motionStateCur;
    WBC_priority_V4(int model_nv_In, int QP_nvIn, int QP_ncIn, double miu_In, double dt);
    void setContactMiu(double miuIn);
    double miu{0.5};
    double cfg_pos_err_clamp_xy{0.02};
    double cfg_pos_err_clamp_z{0.005};
    double cfg_posrot_kp{500.0};
    double cfg_posrot_kd{10.0};
    double cfg_posrot_kp_x{100.0};
    double cfg_posrot_kp_pitch{800.0};
    double cfg_posrot_kd_pitch{10.0};
    double cfg_swing_kp{500.0};
    double cfg_swing_kd{20.0};
    double cfg_weld_hand_kp{120.0};
    double cfg_weld_hand_kd{24.0};
    double cfg_weld_arm_delta_q_limit{0.018};
    double cfg_weld_arm_dq_limit{0.60};
    double cfg_weld_arm_ddq_limit{10.0};
    double cfg_weld_left_arm_kp{20.0};
    double cfg_weld_left_arm_kd{4.0};
    double cfg_weld_ang_momentum_damping{0.0};
    double cfg_weld_left_arm_momentum_gain{0.02};
    double cfg_weld_left_arm_vel_limit{0.10};
    double cfg_weld_right_arm_recover_kp{8.0};
    double cfg_weld_right_arm_recover_kd{2.0};
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_Non;
    Eigen::MatrixXd Jc, dJc, Jfe, dJfe, Jfe_L, Jfe_R;
    Eigen::MatrixXd J_hd_l, J_hd_r, dJ_hd_l, dJ_hd_r;
    Eigen::MatrixXd Jsw, dJsw;
    Eigen::Matrix3d fe_rot_sw_W;
    Eigen::Vector3d fe_pos_sw_W;
    Eigen::Vector3d hd_l_pos_cur_W, hd_r_pos_cur_W;
    Eigen::Matrix3d hd_l_rot_cur_W, hd_r_rot_cur_W;
    Eigen::Vector3d fe_l_pos_des_W, fe_r_pos_des_W;
    Eigen::Matrix3d fe_l_rot_des_W, fe_r_rot_des_W;
    Eigen::Vector3d fe_l_pos_cur_W, fe_r_pos_cur_W;
    Eigen::Matrix3d fe_l_rot_cur_W, fe_r_rot_cur_W;
    Eigen::VectorXd q, dq, ddq;
    Eigen::VectorXd Fr_ff;
    Eigen::VectorXd delta_ddq;
    Eigen::VectorXd delta_Fr;
    Eigen::VectorXd eigen_xOpt;
    Eigen::VectorXd eigen_ddq_Opt;
    Eigen::VectorXd eigen_fr_Opt, eigen_tau_Opt;
    Eigen::MatrixXd Q1;
    Eigen::MatrixXd Q2;
    Eigen::VectorXd delta_q_final_kin, dq_final_kin, ddq_final_kin, tauJointRes;
    Eigen::Matrix3d fe_L_rot_L_off, fe_R_rot_L_off;
    double l_shoulder_pitch = 0;
    double r_shoulder_pitch = 0;
    Eigen::Vector3d pCoMDes, pCoMCur;

    PriorityTasks kin_tasks_walk, kin_tasks_stand, kin_tasks_weld;
    void setQini(const Eigen::VectorXd &qIniDes, const Eigen::VectorXd &qIniCur);
    void setJointTorqueLimits(const Eigen::VectorXd &tauLowIn, const Eigen::VectorXd &tauUppIn);
    void computeTau();
    void dataBusRead(const DataBus &robotState);
    void dataBusWrite(DataBus &robotState);
    void computeDdq(Pin_KinDyn_V4 &pinKinDynIn);
private:
    double timeStep{0.001};
    qpOASES::QProblem QP_prob;
    Eigen::MatrixXd Sf;
    Eigen::MatrixXd St_qpV1, St_qpV2;

    qpOASES::int_t nWSR=100, last_nWSR{0};
    qpOASES::real_t cpu_time=0.1, last_cpu_time{0};
    int qpStatus{0};
    int QP_nv;
    int QP_nc;
    void copy_Eigen_to_real_t(qpOASES::real_t* target, const Eigen::MatrixXd &source, int nRows, int nCols);
    Eigen::MatrixXd J_base, dJ_base, Jcom;
    Eigen::MatrixXd J_hip_link;
    Eigen::VectorXd tauJointLow, tauJointUpp;
    bool jointTorqueLimitEnabled{false};
    Eigen::Vector3d base_pos_des, base_pos, base_rpy_des, base_rpy_cur, hip_link_pos;
    Eigen::Matrix3d hip_link_rot, base_rot;
    Eigen::VectorXd swing_fe_pos_des_W, swing_fe_rpy_des_W;
    Eigen::Vector3d stance_fe_pos_cur_W;
    Eigen::Matrix3d stance_fe_rot_cur_W;
    Eigen::Vector3d stanceDesPos_W;
    Eigen::Vector3d weld_tcp_pos_des_W;
    Eigen::Matrix3d weld_tcp_rot_des_W;
    Eigen::Vector3d weld_tcp_linear_vel_des_W;
    Eigen::Vector3d weld_tcp_angular_vel_des_W;
    Eigen::Vector3d weld_tcp_linear_acc_des_W;
    Eigen::Vector3d weld_tcp_angular_acc_des_W;
    Eigen::VectorXd weld_left_arm_balance_vel;
    double weld_prepare_phase{0.0};
    double weld_recover_phase{0.0};
    bool weld_active{false};
    Eigen::VectorXd des_ddq, des_dq, des_delta_q, des_q;
    Eigen::VectorXd qIniDes, qIniCur;

    static const int QP_nv_des=18;
    static const int QP_nc_des=44;

    qpOASES::real_t qp_H[QP_nv_des*QP_nv_des];
    qpOASES::real_t qp_A[QP_nc_des*QP_nv_des];
    qpOASES::real_t qp_g[QP_nv_des];
    qpOASES::real_t qp_lbA[QP_nc_des];
    qpOASES::real_t qp_ubA[QP_nc_des];
    qpOASES::real_t xOpt_iniGuess[QP_nv_des];
};
