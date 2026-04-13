/*
 * WBC_priority_V4_Leg: whole-body controller for speedbot_v4 leg-only robot.
 */
#pragma once

#include "qpOASES.hpp"
#include <Eigen/Dense>
#include "data_bus.h"
#include "useful_math.h"
#include "priority_tasks.h"
#include "pino_kin_dyn_v4_leg.h"

class WBC_priority_V4_Leg {
public:
    int model_nv;
    Eigen::Vector3d tau_upp_stand_L, tau_low_stand_L;
    Eigen::Vector3d tau_upp_walk_L, tau_low_walk_L;
    double f_z_low{0}, f_z_upp{0};
    DataBus::LegState legStateCur;
    DataBus::MotionState motionStateCur;

    WBC_priority_V4_Leg(int model_nv_In, int QP_nvIn, int QP_ncIn, double miu_In, double dt);

    double miu{0.5};
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_Non;
    Eigen::MatrixXd Jc, dJc, Jfe, dJfe;
    Eigen::MatrixXd Jsw, dJsw;
    Eigen::Matrix3d fe_rot_sw_W;
    Eigen::Vector3d fe_pos_sw_W;
    Eigen::Vector3d fe_l_pos_des_W, fe_r_pos_des_W;
    Eigen::Matrix3d fe_l_rot_des_W, fe_r_rot_des_W;
    Eigen::Vector3d fe_l_pos_cur_W, fe_r_pos_cur_W;
    Eigen::Matrix3d fe_l_rot_cur_W, fe_r_rot_cur_W;
    Eigen::VectorXd q, dq, ddq;
    Eigen::VectorXd Fr_ff;
    Eigen::VectorXd eigen_xOpt;
    Eigen::VectorXd eigen_ddq_Opt;
    Eigen::VectorXd eigen_fr_Opt, eigen_tau_Opt;
    Eigen::MatrixXd Q1;
    Eigen::MatrixXd Q2;
    Eigen::VectorXd delta_q_final_kin, dq_final_kin, ddq_final_kin, tauJointRes;
    Eigen::Matrix3d fe_L_rot_L_off, fe_R_rot_L_off;
    Eigen::Vector3d pCoMDes, pCoMCur;

    PriorityTasks kin_tasks_walk, kin_tasks_stand;

    void setQini(const Eigen::VectorXd &qIniDes, const Eigen::VectorXd &qIniCur);
    void computeTau();
    void dataBusRead(const DataBus &robotState);
    void dataBusWrite(DataBus &robotState);
    void computeDdq(Pin_KinDyn_V4_Leg &pinKinDynIn);

private:
    double timeStep{0.001};
    qpOASES::QProblem QP_prob;
    Eigen::MatrixXd Sf;
    Eigen::MatrixXd St_qpV1, St_qpV2;

    qpOASES::int_t nWSR = 100, last_nWSR{0};
    qpOASES::real_t cpu_time = 0.1, last_cpu_time{0};
    int qpStatus{0};
    int QP_nv;
    int QP_nc;

    void copy_Eigen_to_real_t(qpOASES::real_t *target, const Eigen::MatrixXd &source, int nRows, int nCols);

    Eigen::MatrixXd J_base, dJ_base, Jcom;
    Eigen::MatrixXd J_hip_link;
    Eigen::Vector3d base_pos_des, base_pos, base_rpy_des, base_rpy_cur, hip_link_pos;
    Eigen::Matrix3d hip_link_rot, base_rot;
    Eigen::VectorXd swing_fe_pos_des_W, swing_fe_rpy_des_W;
    Eigen::Vector3d stance_fe_pos_cur_W;
    Eigen::Matrix3d stance_fe_rot_cur_W;
    Eigen::Vector3d stanceDesPos_W;
    Eigen::VectorXd des_ddq, des_dq, des_delta_q, des_q;
    Eigen::VectorXd qIniDes, qIniCur;

    static const int QP_nv_des = 18;
    static const int QP_nc_des = 22;

    qpOASES::real_t qp_H[QP_nv_des * QP_nv_des];
    qpOASES::real_t qp_A[QP_nc_des * QP_nv_des];
    qpOASES::real_t qp_g[QP_nv_des];
    qpOASES::real_t qp_lbA[QP_nc_des];
    qpOASES::real_t qp_ubA[QP_nc_des];
    qpOASES::real_t xOpt_iniGuess[QP_nv_des];
};
