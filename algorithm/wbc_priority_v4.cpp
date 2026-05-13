/*
 * WBC_priority_V4 implementation for speedbot_v4 robot.
 * Adapted from wbc_priority.cpp for OpenLoong.
 *
 * speedbot_v4 pinocchio dq ordering (model_nv=28):
 *   dq[0..5]:  floating base (pos xyz, rot xyz)
 *   dq[6..11]: leg_l (hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll)
 *   dq[12..17]: leg_r (hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll)
 *   dq[18..22]: arm_l (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
 *   dq[23..27]: arm_r (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
 *
 * q = dq + 1 for joints (q[7..] = joints, q[0..6] = base pos + quaternion)
 */
#include "wbc_priority_v4.h"
#include "iostream"
#include <algorithm>
#include <cmath>
#include <limits>

// QP_nvIn=18, QP_ncIn=22 (same as AzureLoong: 6 delta_b + 12 contact forces)
WBC_priority_V4::WBC_priority_V4(int model_nv_In, int QP_nvIn, int QP_ncIn, double miu_In, double dt) : QP_prob(QP_nvIn,
                                                                                                          QP_ncIn)
{
    timeStep = dt;
    model_nv = model_nv_In;
    miu = miu_In;
    QP_nc = QP_ncIn;
    QP_nv = QP_nvIn;
    Sf = Eigen::MatrixXd::Zero(6, model_nv);
    Sf.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    St_qpV2 = Eigen::MatrixXd::Zero(model_nv, model_nv - 6);
    St_qpV2.block(6, 0, model_nv - 6, model_nv - 6) = Eigen::MatrixXd::Identity(model_nv - 6, model_nv - 6);

    St_qpV1 = Eigen::MatrixXd::Zero(model_nv, 6);
    St_qpV1.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);

    f_z_low = 10;
    f_z_upp = 1400;

    tau_upp_stand_L << 15, 30, 40;
    tau_low_stand_L << -15, -30, -40;

    tau_upp_walk_L << 15, 40, 40;
    tau_low_walk_L << -15, -40, -40;

    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    QP_prob.setOptions(options);

    eigen_xOpt = Eigen::VectorXd::Zero(QP_nv);
    eigen_ddq_Opt = Eigen::VectorXd::Zero(model_nv);
    eigen_fr_Opt = Eigen::VectorXd::Zero(12);
    eigen_tau_Opt = Eigen::VectorXd::Zero(model_nv - 6);
    tauJointLow = Eigen::VectorXd::Constant(model_nv - 6, -1.0e10);
    tauJointUpp = Eigen::VectorXd::Constant(model_nv - 6, 1.0e10);

    delta_q_final_kin = Eigen::VectorXd::Zero(model_nv);
    dq_final_kin = Eigen::VectorXd::Zero(model_nv);
    ddq_final_kin = Eigen::VectorXd::Zero(model_nv);

    base_rpy_cur = Eigen::VectorXd::Zero(3);
    weld_left_arm_balance_vel = Eigen::VectorXd::Zero(5);

    //  WBC task defined and order build
    ///------------ walk --------------
    kin_tasks_walk.addTask("static_Contact");
    kin_tasks_walk.addTask("Roll_Pitch_Yaw_Pz");
    kin_tasks_walk.addTask("PxPy");
    kin_tasks_walk.addTask("SwingLeg");
    kin_tasks_walk.addTask("HandTrackJoints");
    kin_tasks_walk.addTask("PosRot");

    std::vector<std::string> taskOrder_walk;

    taskOrder_walk.emplace_back("static_Contact");
    taskOrder_walk.emplace_back("PosRot");
    taskOrder_walk.emplace_back("SwingLeg");
    taskOrder_walk.emplace_back("HandTrackJoints");

    kin_tasks_walk.buildPriority(taskOrder_walk);

    ///-------- stand ------------
    kin_tasks_stand.addTask("static_Contact");
    kin_tasks_stand.addTask("CoMTrack");
    kin_tasks_stand.addTask("HandTrackJoints");
    kin_tasks_stand.addTask("HipRPY");
    kin_tasks_stand.addTask("Pz");
    kin_tasks_stand.addTask("CoMXY_HipRPY");
    kin_tasks_stand.addTask("Roll_Pitch_Yaw");
    // No HeadRP task for v4 (no head joints)

    std::vector<std::string> taskOrder_stand;

    taskOrder_stand.emplace_back("static_Contact");
    taskOrder_stand.emplace_back("CoMXY_HipRPY");
    taskOrder_stand.emplace_back("Pz");
    taskOrder_stand.emplace_back("HandTrackJoints");
    // No HeadRP in priority order

    kin_tasks_stand.buildPriority(taskOrder_stand);

    ///-------- weld ------------
    kin_tasks_weld.addTask("static_Contact");
    kin_tasks_weld.addTask("CoMXY_HipRPY");
    kin_tasks_weld.addTask("Pz");
    kin_tasks_weld.addTask("AngularMomentumDamping");
    kin_tasks_weld.addTask("RightHandWeld");
    kin_tasks_weld.addTask("RightArmRecover");
    kin_tasks_weld.addTask("LeftArmBalance");

    std::vector<std::string> taskOrder_weld;
    taskOrder_weld.emplace_back("static_Contact");
    taskOrder_weld.emplace_back("CoMXY_HipRPY");
    taskOrder_weld.emplace_back("Pz");
    taskOrder_weld.emplace_back("AngularMomentumDamping");
    taskOrder_weld.emplace_back("RightHandWeld");
    taskOrder_weld.emplace_back("RightArmRecover");
    taskOrder_weld.emplace_back("LeftArmBalance");
    kin_tasks_weld.buildPriority(taskOrder_weld);
}

void WBC_priority_V4::setContactMiu(double miuIn)
{
    miu = std::clamp(miuIn, 0.01, 2.0);
}

void WBC_priority_V4::setJointTorqueLimits(const Eigen::VectorXd &tauLowIn, const Eigen::VectorXd &tauUppIn)
{
    if (tauLowIn.size() != model_nv - 6 || tauUppIn.size() != model_nv - 6)
    {
        jointTorqueLimitEnabled = false;
        tauJointLow = Eigen::VectorXd::Constant(model_nv - 6, -1.0e10);
        tauJointUpp = Eigen::VectorXd::Constant(model_nv - 6, 1.0e10);
        std::cerr << "[WBC_V4] invalid torque limit size, disabling torque constraints." << std::endl;
        return;
    }

    tauJointLow = tauLowIn;
    tauJointUpp = tauUppIn;
    for (int i = 0; i < model_nv - 6; ++i)
    {
        if (!std::isfinite(tauJointLow(i)) || !std::isfinite(tauJointUpp(i)) ||
            tauJointLow(i) >= tauJointUpp(i))
        {
            jointTorqueLimitEnabled = false;
            tauJointLow = Eigen::VectorXd::Constant(model_nv - 6, -1.0e10);
            tauJointUpp = Eigen::VectorXd::Constant(model_nv - 6, 1.0e10);
            std::cerr << "[WBC_V4] invalid torque limit value, disabling torque constraints." << std::endl;
            return;
        }
    }
    jointTorqueLimitEnabled = true;
}

void WBC_priority_V4::dataBusRead(const DataBus &robotState)
{
    fe_L_rot_L_off = robotState.fe_L_rot_L_off;
    fe_R_rot_L_off = robotState.fe_R_rot_L_off;

    base_rpy_des = robotState.base_rpy_des;
    base_rpy_cur << robotState.rpy[0], robotState.rpy[1], robotState.rpy[2];
    base_pos_des = robotState.base_pos_des;
    swing_fe_pos_des_W = robotState.swing_fe_pos_des_W;
    swing_fe_rpy_des_W = robotState.swing_fe_rpy_des_W;
    stance_fe_pos_cur_W = robotState.stance_fe_pos_cur_W;
    stance_fe_rot_cur_W = robotState.stance_fe_rot_cur_W;
    stanceDesPos_W = robotState.stanceDesPos_W;
    weld_tcp_pos_des_W = robotState.weld_tcp_pos_des_W;
    weld_tcp_rot_des_W = robotState.weld_tcp_rot_des_W;
    weld_tcp_linear_vel_des_W = robotState.weld_tcp_linear_vel_des_W;
    weld_tcp_angular_vel_des_W = robotState.weld_tcp_angular_vel_des_W;
    weld_tcp_linear_acc_des_W = robotState.weld_tcp_linear_acc_des_W;
    weld_tcp_angular_acc_des_W = robotState.weld_tcp_angular_acc_des_W;
    weld_prepare_phase = robotState.weld_prepare_phase;
    weld_recover_phase = robotState.weld_recover_phase;
    weld_active = robotState.weld_active;
    hd_l_pos_cur_W = robotState.hd_l_pos_W;
    hd_r_pos_cur_W = robotState.hd_r_pos_W;
    hd_l_rot_cur_W = robotState.hd_l_rot_W;
    hd_r_rot_cur_W = robotState.hd_r_rot_W;
    fe_l_pos_cur_W = robotState.fe_l_pos_W;
    fe_r_pos_cur_W = robotState.fe_r_pos_W;
    fe_l_rot_cur_W = robotState.fe_l_rot_W;
    fe_r_rot_cur_W = robotState.fe_r_rot_W;
    des_ddq = robotState.des_ddq;
    des_dq = robotState.des_dq;
    des_delta_q = robotState.des_delta_q;
    des_q = robotState.des_q;

    J_base = robotState.J_base;
    dJ_base = robotState.dJ_base;
    base_rot = robotState.base_rot;
    base_pos = robotState.base_pos;
    hip_link_pos = robotState.hip_link_pos;
    hip_link_rot = robotState.hip_link_rot;
    J_hip_link = robotState.J_hip_link;

    Jfe = Eigen::MatrixXd::Zero(12, model_nv);
    Jfe.block(0, 0, 6, model_nv) = robotState.J_l;
    Jfe.block(6, 0, 6, model_nv) = robotState.J_r;
    dJfe = Eigen::MatrixXd::Zero(12, model_nv);
    dJfe.block(0, 0, 6, model_nv) = robotState.dJ_l;
    dJfe.block(6, 0, 6, model_nv) = robotState.dJ_r;
    J_hd_l = robotState.J_hd_l;
    J_hd_r = robotState.J_hd_r;
    dJ_hd_l = robotState.dJ_hd_l;
    dJ_hd_r = robotState.dJ_hd_r;
    Fr_ff = robotState.Fr_ff;
    dyn_M = robotState.dyn_M;
    dyn_M_inv = robotState.dyn_M_inv;
    dyn_Ag = robotState.dyn_Ag;
    dyn_dAg = robotState.dyn_dAg;
    dyn_Non = robotState.dyn_Non;
    dq = robotState.dq;
    q = robotState.q;
    legStateCur = robotState.legState;
    motionStateCur = robotState.motionState;

    if (legStateCur == DataBus::LSt)
    {
        Jc = robotState.J_l;
        dJc = robotState.dJ_l;
        Jsw = robotState.J_r;
        dJsw = robotState.dJ_r;
        fe_pos_sw_W = robotState.fe_r_pos_W;
        fe_rot_sw_W = robotState.fe_r_rot_W;
    }
    else if (legStateCur == DataBus::RSt)
    {
        Jc = robotState.J_r;
        dJc = robotState.dJ_r;
        Jsw = robotState.J_l;
        dJsw = robotState.dJ_l;
        fe_pos_sw_W = robotState.fe_l_pos_W;
        fe_rot_sw_W = robotState.fe_l_rot_W;
    }
    else
    {
        Jc = Jfe;
        dJc = dJfe;
        Jsw = Eigen::MatrixXd::Zero(6, model_nv);
        dJsw = Eigen::MatrixXd::Zero(6, model_nv);
        fe_pos_sw_W = 0.5 * (robotState.fe_l_pos_W + robotState.fe_r_pos_W);
        fe_rot_sw_W = robotState.fe_l_rot_W;
    }

    Jcom = robotState.Jcom_W;
    pCoMCur = robotState.pCoM_W;
}

void WBC_priority_V4::dataBusWrite(DataBus &robotState)
{
    robotState.wbc_ddq_final = eigen_ddq_Opt;
    robotState.wbc_tauJointRes = tauJointRes;
    robotState.wbc_FrRes = eigen_fr_Opt;
    robotState.qp_cpuTime = cpu_time;
    robotState.qp_nWSR = nWSR;
    robotState.qp_status = qpStatus;

    robotState.wbc_delta_q_final = delta_q_final_kin;
    robotState.wbc_dq_final = dq_final_kin;
    robotState.wbc_ddq_final = ddq_final_kin;

    robotState.qp_status = qpStatus;
    robotState.qp_nWSR = nWSR;
    robotState.qp_cpuTime = cpu_time;

    robotState.weld_tcp_pos_cur_W = hd_r_pos_cur_W;
    robotState.weld_tcp_pos_err_W = weld_tcp_pos_des_W - hd_r_pos_cur_W;
    if (q.size() >= 24)
    {
        robotState.weld_left_arm_q = q.block<5, 1>(19, 0);
    }
    else
    {
        robotState.weld_left_arm_q = Eigen::VectorXd::Zero(5);
    }
    robotState.weld_left_arm_balance_vel = (weld_left_arm_balance_vel.size() == 5)
                                               ? weld_left_arm_balance_vel
                                               : Eigen::VectorXd::Zero(5);
    robotState.weld_left_arm_vel_norm = robotState.weld_left_arm_balance_vel.norm();
    Eigen::Matrix3d weldDesRot = weld_tcp_rot_des_W;
    robotState.weld_tcp_rot_err_W = diffRot(hd_r_rot_cur_W, weldDesRot);
    if (dyn_Ag.rows() >= 6 && dyn_Ag.cols() == model_nv && dq.size() == model_nv)
    {
        robotState.weld_ang_momentum = dyn_Ag.block(3, 0, 3, model_nv) * dq;
        robotState.weld_h_ang_norm = robotState.weld_ang_momentum.norm();
    }
    else
    {
        robotState.weld_ang_momentum.setZero();
        robotState.weld_h_ang_norm = 0.0;
    }
    if (jointTorqueLimitEnabled && tauJointRes.size() == model_nv - 6)
    {
        double tauMargin = std::numeric_limits<double>::infinity();
        for (int i = 0; i < model_nv - 6; ++i)
        {
            tauMargin = std::min(tauMargin, std::min(tauJointUpp(i) - tauJointRes(i),
                                                     tauJointRes(i) - tauJointLow(i)));
        }
        robotState.weld_tau_margin = std::isfinite(tauMargin) ? tauMargin : 0.0;
    }
    else
    {
        robotState.weld_tau_margin = 0.0;
    }
}

void WBC_priority_V4::computeTau()
{
    Eigen::MatrixXd eigen_qp_A1 = Eigen::MatrixXd::Zero(6, QP_nv);
    eigen_qp_A1.block<6, 6>(0, 0) = Sf * dyn_M * St_qpV1;
    eigen_qp_A1.block<6, 12>(0, 6) = -Sf * Jfe.transpose();

    Eigen::VectorXd eqRes = Eigen::VectorXd::Zero(6);
    eqRes = -Sf * dyn_M * ddq_final_kin - Sf * dyn_Non + Sf * Jfe.transpose() * Fr_ff;

    Eigen::Matrix3d Rfe;
    if (motionStateCur == DataBus::Stand || motionStateCur == DataBus::WeldPrepare ||
        motionStateCur == DataBus::Weld || motionStateCur == DataBus::WeldHold ||
        motionStateCur == DataBus::WeldRecover)
    {
        Rfe = fe_l_rot_cur_W;
    }
    else
    {
        Rfe = stance_fe_rot_cur_W;
    }

    Eigen::Matrix<double, 12, 12> Mw2b;
    Mw2b.setZero();
    Mw2b.block(0, 0, 3, 3) = Rfe.transpose();
    Mw2b.block(3, 3, 3, 3) = Rfe.transpose();
    Mw2b.block(6, 6, 3, 3) = Rfe.transpose();
    Mw2b.block(9, 9, 3, 3) = Rfe.transpose();

    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(16, 12);
    W(0, 0) = 1;
    W(0, 2) = sqrt(2) / 2.0 * miu;
    W(1, 0) = -1;
    W(1, 2) = sqrt(2) / 2.0 * miu;
    W(2, 1) = 1;
    W(2, 2) = sqrt(2) / 2.0 * miu;
    W(3, 1) = -1;
    W(3, 2) = sqrt(2) / 2.0 * miu;
    W.block<4, 4>(4, 2) = Eigen::MatrixXd::Identity(4, 4);
    W.block<8, 6>(8, 6) = W.block<8, 6>(0, 0);
    W = W * Mw2b;

    Eigen::VectorXd f_low = Eigen::VectorXd::Zero(16);
    Eigen::VectorXd f_upp = Eigen::VectorXd::Zero(16);
    Eigen::Vector3d tau_upp_fe, tau_low_fe;
    if (motionStateCur == DataBus::Stand || motionStateCur == DataBus::WeldPrepare || motionStateCur == DataBus::Weld)
    {
        tau_upp_fe = tau_upp_stand_L;
        tau_low_fe = tau_low_stand_L;
    }
    else
    {
        tau_upp_fe = tau_upp_walk_L;
        tau_low_fe = tau_low_walk_L;
    }

    f_upp.block<8, 1>(0, 0) << 1e10, 1e10, 1e10, 1e10,
        f_z_upp, tau_upp_fe(0), tau_upp_fe(1), tau_upp_fe(2);
    f_upp.block<8, 1>(8, 0) = f_upp.block<8, 1>(0, 0);
    f_low.block<8, 1>(0, 0) << 0, 0, 0, 0,
        f_z_low, tau_low_fe(0), tau_low_fe(1), tau_low_fe(2);
    f_low.block<8, 1>(8, 0) = f_low.block<8, 1>(0, 0);

    if (motionStateCur == DataBus::Walk || motionStateCur == DataBus::Walk2Stand)
    {
        if (legStateCur == DataBus::LSt)
        {
            f_upp(12) = 0; f_upp(13) = 0; f_upp(14) = 0; f_upp(15) = 0;
            f_low(12) = 0; f_low(13) = 0; f_low(14) = 0; f_low(15) = 0;
            f_low(8) = -1e-7; f_low(9) = -1e-7; f_low(10) = -1e-7; f_low(11) = -1e-7;
        }
        else if (legStateCur == DataBus::RSt)
        {
            f_upp(4) = 0; f_upp(5) = 0; f_upp(6) = 0; f_upp(7) = 0;
            f_low(4) = 0; f_low(5) = 0; f_low(6) = 0; f_low(7) = 0;
            f_low(0) = -1e-7; f_low(1) = -1e-7; f_low(2) = -1e-7; f_low(3) = -1e-7;
        }
    }

    Eigen::MatrixXd eigen_qp_A2 = Eigen::MatrixXd::Zero(16, QP_nv);
    eigen_qp_A2.block<16, 12>(0, 6) = W;
    Eigen::VectorXd neqRes_low = Eigen::VectorXd::Zero(16);
    Eigen::VectorXd neqRes_upp = Eigen::VectorXd::Zero(16);

    neqRes_low = f_low - W * Fr_ff;
    neqRes_upp = f_upp - W * Fr_ff;

    Eigen::MatrixXd eigen_qp_A_final = Eigen::MatrixXd::Zero(QP_nc, QP_nv);
    Eigen::VectorXd eigen_qp_lbA = Eigen::VectorXd::Constant(QP_nc, -1.0e10);
    Eigen::VectorXd eigen_qp_ubA = Eigen::VectorXd::Constant(QP_nc, 1.0e10);

    eigen_qp_A_final.block(0, 0, 6, QP_nv) = eigen_qp_A1;
    eigen_qp_A_final.block(6, 0, 16, QP_nv) = eigen_qp_A2;
    eigen_qp_lbA.block<6, 1>(0, 0) = eqRes;
    eigen_qp_lbA.block<16, 1>(6, 0) = neqRes_low;
    eigen_qp_ubA.block<6, 1>(0, 0) = eqRes;
    eigen_qp_ubA.block<16, 1>(6, 0) = neqRes_upp;

    const int torqueStartRow = 22;
    const int torqueRows = std::max(0, std::min(model_nv - 6, QP_nc - torqueStartRow));
    const bool enforceJointTorqueLimits = jointTorqueLimitEnabled && motionStateCur == DataBus::Weld;
    if (torqueRows > 0 && enforceJointTorqueLimits)
    {
        Eigen::MatrixXd tauA = Eigen::MatrixXd::Zero(model_nv - 6, QP_nv);
        tauA.block(0, 0, model_nv - 6, 6) = dyn_M.block(6, 0, model_nv - 6, model_nv) * St_qpV1;
        tauA.block(0, 6, model_nv - 6, 12) = -Jfe.transpose().block(6, 0, model_nv - 6, 12);

        const Eigen::VectorXd tauNom =
            (dyn_M * ddq_final_kin + dyn_Non - Jfe.transpose() * Fr_ff).block(6, 0, model_nv - 6, 1);
        eigen_qp_A_final.block(torqueStartRow, 0, torqueRows, QP_nv) = tauA.topRows(torqueRows);
        eigen_qp_lbA.segment(torqueStartRow, torqueRows) =
            (tauJointLow - tauNom).head(torqueRows);
        eigen_qp_ubA.segment(torqueStartRow, torqueRows) =
            (tauJointUpp - tauNom).head(torqueRows);
    }

    Eigen::MatrixXd eigen_qp_H = Eigen::MatrixXd::Zero(QP_nv, QP_nv);
    Q2 = Eigen::MatrixXd::Identity(6, 6);
    Q1 = Eigen::MatrixXd::Identity(12, 12);
    if (motionStateCur == DataBus::Stand || motionStateCur == DataBus::WeldPrepare || motionStateCur == DataBus::Weld){
        eigen_qp_H.block<6, 6>(0, 0) = Q2 * 2.0 * 1e7;
        eigen_qp_H.block<12, 12>(6, 6) = Q1 * 2.0 * 1e1;
        eigen_qp_H(9,9) *= 100;
        eigen_qp_H(10,10) *= 100;
        eigen_qp_H(15,15) *= 100;
        eigen_qp_H(16,16) *= 100;
    }
    else{
        eigen_qp_H.block<6, 6>(0, 0) = Q2 * 2.0 * 1e7;
        eigen_qp_H.block<12, 12>(6, 6) = Q1 * 2.0 * 1e1;
    }

    copy_Eigen_to_real_t(qp_H, eigen_qp_H, eigen_qp_H.rows(), eigen_qp_H.cols());
    copy_Eigen_to_real_t(qp_A, eigen_qp_A_final, eigen_qp_A_final.rows(), eigen_qp_A_final.cols());
    copy_Eigen_to_real_t(qp_lbA, eigen_qp_lbA, eigen_qp_lbA.rows(), eigen_qp_lbA.cols());
    copy_Eigen_to_real_t(qp_ubA, eigen_qp_ubA, eigen_qp_ubA.rows(), eigen_qp_ubA.cols());

    qpOASES::returnValue res;
    for (int i = 0; i < QP_nv; i++)
    {
        xOpt_iniGuess[i] = 0;
        qp_g[i] = 0;
    }
    eigen_xOpt.setZero();
    nWSR = 200;
    cpu_time = timeStep;
    res = QP_prob.init(qp_H, qp_g, qp_A, NULL, NULL, qp_lbA, qp_ubA, nWSR, &cpu_time, xOpt_iniGuess);
    qpStatus = qpOASES::getSimpleStatus(res);

    qpOASES::real_t xOpt[QP_nv];
    QP_prob.getPrimalSolution(xOpt);
    if (res == qpOASES::SUCCESSFUL_RETURN)
        for (int i = 0; i < QP_nv; i++)
            eigen_xOpt(i) = xOpt[i];

    eigen_ddq_Opt = ddq_final_kin;
    eigen_ddq_Opt.block<6, 1>(0, 0) += eigen_xOpt.block<6, 1>(0, 0);
    eigen_fr_Opt = Fr_ff + eigen_xOpt.block<12, 1>(6, 0);

    if (qpStatus != 0)
    {
        Eigen::MatrixXd A_x;
        Eigen::VectorXd xOpt_iniGuess_m(QP_nv, 1);
        for (int i = 0; i < QP_nv; i++)
            xOpt_iniGuess_m(i) = xOpt_iniGuess[i];
    }

    Eigen::VectorXd tauRes;
    tauRes = dyn_M * eigen_ddq_Opt + dyn_Non - Jfe.transpose() * eigen_fr_Opt;

    tauJointRes = tauRes.block(6, 0, model_nv - 6, 1);

    last_nWSR = nWSR;
    last_cpu_time = cpu_time;
}

/*
 * computeDdq: All index references remapped for speedbot_v4.
 *
 * q index reference (nq=29):
 *   q[0..6]: base (x,y,z, qx,qy,qz,qw)
 *   q[7..12]: leg_l  (hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll)
 *   q[13..18]: leg_r (hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll)
 *   q[19..23]: arm_l (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
 *   q[24..28]: arm_r (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
 *
 * dq index reference (model_nv=28):
 *   dq[0..5]: floating base
 *   dq[6..11]: leg_l
 *   dq[12..17]: leg_r
 *   dq[18..22]: arm_l
 *   dq[23..27]: arm_r
 */
void WBC_priority_V4::computeDdq(Pin_KinDyn_V4 &pinKinDynIn)
{
    /// -------- walk -------------
    {
        const bool inDoubleSupport = (legStateCur == DataBus::DSt);
        const int contactTaskDim = inDoubleSupport ? 12 : 6;
        Eigen::MatrixXd contactJ = inDoubleSupport ? Jfe : Jc;
        Eigen::MatrixXd contactdJ = inDoubleSupport ? dJfe : dJc;

        int id = kin_tasks_walk.getId("static_Contact");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(contactTaskDim);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(contactTaskDim);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(contactTaskDim);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(contactTaskDim);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(contactTaskDim, contactTaskDim) * 0;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(contactTaskDim, contactTaskDim) * 0;
        kin_tasks_walk.taskLib[id].J = contactJ;
        kin_tasks_walk.taskLib[id].dJ = contactdJ;
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("Roll_Pitch_Yaw_Pz");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(4);
        Eigen::Matrix3d desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(0, 0) = diffRot(base_rot, desRot);
        kin_tasks_walk.taskLib[id].errX(3) = base_pos_des(2) - q(2);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(4);
        kin_tasks_walk.taskLib[id].derrX.block<3, 1>(0, 0) = -dq.block<3, 1>(3, 0);
        kin_tasks_walk.taskLib[id].derrX(3) = 0 - dq(2);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(4);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(4);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(4, 4) * 100;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(4, 4) * 10;
        Eigen::MatrixXd taskMap = Eigen::MatrixXd::Zero(4, 6);
        taskMap(0, 3) = 1;
        taskMap(1, 4) = 1;
        taskMap(2, 5) = 1;
        taskMap(3, 2) = 1;
        kin_tasks_walk.taskLib[id].J = taskMap * J_base;
        kin_tasks_walk.taskLib[id].dJ = taskMap * dJ_base;
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("PxPy");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(2);
        kin_tasks_walk.taskLib[id].errX = des_dq.block(0, 0, 2, 1) * timeStep;
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(2);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(2, 2) * 100;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(2, 2) * 50;
        taskMap = Eigen::MatrixXd::Zero(2, 6);
        taskMap(0, 0) = 1;
        taskMap(1, 1) = 1;
        kin_tasks_walk.taskLib[id].J = taskMap * J_base;
        kin_tasks_walk.taskLib[id].dJ = taskMap * dJ_base;
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("PosRot");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].errX.block(0, 0, 3, 1) = base_pos_des - q.block(0, 0, 3, 1);
        if (fabs(kin_tasks_walk.taskLib[id].errX(0)) >= cfg_pos_err_clamp_xy)
            kin_tasks_walk.taskLib[id].errX(0) = cfg_pos_err_clamp_xy * sign(kin_tasks_walk.taskLib[id].errX(0));
        if (fabs(kin_tasks_walk.taskLib[id].errX(1)) >= cfg_pos_err_clamp_xy)
            kin_tasks_walk.taskLib[id].errX(1) = cfg_pos_err_clamp_xy * sign(kin_tasks_walk.taskLib[id].errX(1));
        if (kin_tasks_walk.taskLib[id].errX(2) > cfg_pos_err_clamp_z){
            kin_tasks_walk.taskLib[id].errX(2) = cfg_pos_err_clamp_z;
        }
        desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_walk.taskLib[id].errX.block<3, 1>(3, 0) = diffRot(base_rot, desRot);
        kin_tasks_walk.taskLib[id].errX(4) -= 0.05 * dq(4);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(6, 6) * cfg_posrot_kp;
        kin_tasks_walk.taskLib[id].kp.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * cfg_posrot_kp;
        kin_tasks_walk.taskLib[id].kp(0,0) = cfg_posrot_kp_x;
        kin_tasks_walk.taskLib[id].kp(4,4) = cfg_posrot_kp_pitch;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(6, 6) * cfg_posrot_kd;
        kin_tasks_walk.taskLib[id].kd(4,4) = cfg_posrot_kd_pitch;
        kin_tasks_walk.taskLib[id].J = J_base;
        kin_tasks_walk.taskLib[id].dJ = dJ_base;
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_walk.getId("SwingLeg");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(6);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(6);
        if (!inDoubleSupport)
        {
            kin_tasks_walk.taskLib[id].errX.block<3, 1>(0, 0) = swing_fe_pos_des_W - fe_pos_sw_W;
            desRot = eul2Rot(swing_fe_rpy_des_W(0), swing_fe_rpy_des_W(1), swing_fe_rpy_des_W(2));
            kin_tasks_walk.taskLib[id].errX.block<3, 1>(3, 0) = diffRot(fe_rot_sw_W, desRot);
            kin_tasks_walk.taskLib[id].errX(4) *= 2;
            kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(6, 6) * cfg_swing_kp;
            kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(6, 6) * cfg_swing_kd;
            kin_tasks_walk.taskLib[id].J = Jsw;
            kin_tasks_walk.taskLib[id].dJ = dJsw;
        }
        else
        {
            kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Zero(6, 6);
            kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Zero(6, 6);
            kin_tasks_walk.taskLib[id].J = Eigen::MatrixXd::Zero(6, model_nv);
            kin_tasks_walk.taskLib[id].dJ = Eigen::MatrixXd::Zero(6, model_nv);
        }
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        // HandTrackJoints walk: 10 DoF (arm_l 5 + arm_r 5), non-contiguous
        // left_hip_pitch at q[9], right_hip_pitch at q[15]
        double l_hip_pitch = q(9) - q(15);
        double r_hip_pitch = q(15) - q(9);
        Eigen::VectorXd target_arm_q;
        target_arm_q.resize(10);
        // arm_l: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll
        // arm_r: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll
        target_arm_q << 0.3 + 0.75*r_hip_pitch, 1.4, 0, -1.4, 0,
                       -0.3 - 0.75*l_hip_pitch, -1.4, 0, 1.4, 0;

        id = kin_tasks_walk.getId("HandTrackJoints");
        kin_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(10);
        // arm_l at q[19..23], arm_r at q[24..28]
        kin_tasks_walk.taskLib[id].errX.block<5,1>(0,0) = target_arm_q.block<5,1>(0,0) - q.block<5, 1>(19, 0);
        kin_tasks_walk.taskLib[id].errX.block<5,1>(5,0) = target_arm_q.block<5,1>(5,0) - q.block<5, 1>(24, 0);
        kin_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(10);
        kin_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(10);
        kin_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(10);
        kin_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(10, 10) * 200;
        kin_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(10, 10) * 10;
        kin_tasks_walk.taskLib[id].J = Eigen::MatrixXd::Zero(10, model_nv);
        kin_tasks_walk.taskLib[id].J.block(0, 18, 5, 5) = Eigen::MatrixXd::Identity(5, 5); // arm_l dq[18..22]
        kin_tasks_walk.taskLib[id].J.block(5, 23, 5, 5) = Eigen::MatrixXd::Identity(5, 5); // arm_r dq[23..27]
        kin_tasks_walk.taskLib[id].dJ = Eigen::MatrixXd::Zero(10, model_nv);
        kin_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
    }

    /// -------- stand -------------
    {
        int id = kin_tasks_stand.getId("static_Contact");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(12);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(12);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(12);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(12);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(12, 12) * 0;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(12, 12) * 0;
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(12, model_nv);
        Eigen::MatrixXd taskCtMap = Eigen::MatrixXd::Zero(3, 3);
        taskCtMap(0, 0) = 0;
        taskCtMap(1, 1) = 1;
        taskCtMap(2, 2) = 1;
        taskCtMap = fe_l_rot_cur_W * taskCtMap * fe_l_rot_cur_W.transpose();
        kin_tasks_stand.taskLib[id].J = Jfe;
        kin_tasks_stand.taskLib[id].J.block(3, 0, 3, model_nv) = taskCtMap * kin_tasks_stand.taskLib[id].J.block(3, 0, 3, model_nv);
        kin_tasks_stand.taskLib[id].J.block(9, 0, 3, model_nv) = taskCtMap * kin_tasks_stand.taskLib[id].J.block(9, 0, 3, model_nv);
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(12, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        // HipRPY: exclude arms (cols 18-22 arm_l, cols 23-27 arm_r)
        id = kin_tasks_stand.getId("HipRPY");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        Eigen::Matrix3d desRot = eul2Rot(0, 0, 0);
        kin_tasks_stand.taskLib[id].errX.block<3, 1>(0, 0) = diffRot(hip_link_rot, desRot);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 1000;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 50;
        Eigen::MatrixXd taskMapRPY = Eigen::MatrixXd::Zero(3, 6);
        taskMapRPY(0, 3) = 1;
        taskMapRPY(1, 4) = 1;
        taskMapRPY(2, 5) = 1;
        kin_tasks_stand.taskLib[id].J = taskMapRPY * J_hip_link;
        kin_tasks_stand.taskLib[id].J.block(0, 18, 3, 5).setZero();  // exclude arm_l
        kin_tasks_stand.taskLib[id].J.block(0, 23, 3, 5).setZero();  // exclude arm_r
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(3, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_stand.getId("Pz");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].errX(0) = base_pos_des(2) - q(2);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(1);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(1, 1) * 2000;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(1, 1) * 10;
        Eigen::MatrixXd taskMap = Eigen::MatrixXd::Zero(1, 6);
        taskMap(0, 2) = 1;
        kin_tasks_stand.taskLib[id].J = taskMap * J_base;
        kin_tasks_stand.taskLib[id].dJ = taskMap * dJ_base;
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        // CoMTrack: exclude arms (cols 18-22 arm_l, cols 23-27 arm_r)
        id = kin_tasks_stand.getId("CoMTrack");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].errX = pCoMDes.block(0, 0, 2, 1) - pCoMCur.block(0, 0, 2, 1);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(2);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(2, 2) * 2000;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(2, 2) * 100;
        kin_tasks_stand.taskLib[id].J = Jcom.block(0, 0, 2, model_nv);
        kin_tasks_stand.taskLib[id].J.block(0, 18, 2, 5).setZero(); // exclude arm_l
        kin_tasks_stand.taskLib[id].J.block(0, 23, 2, 5).setZero(); // exclude arm_r
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(2, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        // CoMXY_HipRPY: exclude arms from hip RPY rows
        id = kin_tasks_stand.getId("CoMXY_HipRPY");
        taskMapRPY = Eigen::MatrixXd::Zero(3, 6);
        taskMapRPY(0, 3) = 1;
        taskMapRPY(1, 4) = 1;
        taskMapRPY(2, 5) = 1;
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(5);
        kin_tasks_stand.taskLib[id].errX.block(0, 0, 2, 1) = pCoMDes.block(0, 0, 2, 1) - pCoMCur.block(0, 0, 2, 1);
        desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_stand.taskLib[id].errX.block<3, 1>(2, 0) = diffRot(hip_link_rot, desRot);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(5);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(5);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(5);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(5, 5) * 250;
        kin_tasks_stand.taskLib[id].kp.block(2, 2, 3, 3) = Eigen::MatrixXd::Identity(3,3)*1000;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(5, 5) * 10;
        kin_tasks_stand.taskLib[id].kd.block(2, 2, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * 10;
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(5, model_nv);
        kin_tasks_stand.taskLib[id].J.block(0, 0, 2, model_nv) = Jcom.block(0, 0, 2, model_nv);
        kin_tasks_stand.taskLib[id].J.block(2, 0, 3, model_nv) = taskMapRPY * J_hip_link;
        kin_tasks_stand.taskLib[id].J.block(2, 18, 3, 5).setZero(); // exclude arm_l
        kin_tasks_stand.taskLib[id].J.block(2, 23, 3, 5).setZero(); // exclude arm_r
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(5, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        // HandTrackJoints stand: 10 DoF (arm_l 5 + arm_r 5)
        Eigen::VectorXd target_arm_q;
        target_arm_q.resize(10);
        target_arm_q << 0.3, 1.4, 0, -1.4, 0, -0.3, -1.4, 0, 1.4, 0;
        if (qIniDes.size() >= 29)
        {
            target_arm_q.block<5, 1>(0, 0) = qIniDes.block<5, 1>(19, 0);
            target_arm_q.block<5, 1>(5, 0) = qIniDes.block<5, 1>(24, 0);
        }

        id = kin_tasks_stand.getId("HandTrackJoints");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(10);
        kin_tasks_stand.taskLib[id].errX.block<5,1>(0,0) = target_arm_q.block<5,1>(0,0) - q.block<5, 1>(19, 0);
        kin_tasks_stand.taskLib[id].errX.block<5,1>(5,0) = target_arm_q.block<5,1>(5,0) - q.block<5, 1>(24, 0);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(10);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(10);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(10);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(10, 10) * 2000;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(10, 10) * 100;
        kin_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(10, model_nv);
        kin_tasks_stand.taskLib[id].J.block(0, 18, 5, 5) = Eigen::MatrixXd::Identity(5, 5); // arm_l
        kin_tasks_stand.taskLib[id].J.block(5, 23, 5, 5) = Eigen::MatrixXd::Identity(5, 5); // arm_r
        kin_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(10, model_nv);
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        // No HeadRP task for v4

        id = kin_tasks_stand.getId("Roll_Pitch_Yaw");
        kin_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_stand.taskLib[id].errX = diffRot(base_rot, desRot);
        kin_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].derrX = -dq.block<3, 1>(3, 0);
        kin_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 2000;
        kin_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 100;
        taskMap = Eigen::MatrixXd::Zero(3, 6);
        taskMap(0, 3) = 1;
        taskMap(1, 4) = 1;
        taskMap(2, 5) = 1;
        kin_tasks_stand.taskLib[id].J = taskMap * J_base;
        kin_tasks_stand.taskLib[id].dJ = taskMap * dJ_base;
        kin_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

    }

    /// -------- weld -------------
    {
        int id = kin_tasks_weld.getId("static_Contact");
        kin_tasks_weld.taskLib[id].errX = Eigen::VectorXd::Zero(12);
        kin_tasks_weld.taskLib[id].derrX = Eigen::VectorXd::Zero(12);
        kin_tasks_weld.taskLib[id].ddxDes = Eigen::VectorXd::Zero(12);
        kin_tasks_weld.taskLib[id].dxDes = Eigen::VectorXd::Zero(12);
        kin_tasks_weld.taskLib[id].kp = Eigen::MatrixXd::Identity(12, 12) * 0;
        kin_tasks_weld.taskLib[id].kd = Eigen::MatrixXd::Identity(12, 12) * 0;
        Eigen::MatrixXd taskCtMap = Eigen::MatrixXd::Zero(3, 3);
        taskCtMap(1, 1) = 1;
        taskCtMap(2, 2) = 1;
        taskCtMap = fe_l_rot_cur_W * taskCtMap * fe_l_rot_cur_W.transpose();
        kin_tasks_weld.taskLib[id].J = Jfe;
        kin_tasks_weld.taskLib[id].J.block(3, 0, 3, model_nv) = taskCtMap * kin_tasks_weld.taskLib[id].J.block(3, 0, 3, model_nv);
        kin_tasks_weld.taskLib[id].J.block(9, 0, 3, model_nv) = taskCtMap * kin_tasks_weld.taskLib[id].J.block(9, 0, 3, model_nv);
        kin_tasks_weld.taskLib[id].dJ = Eigen::MatrixXd::Zero(12, model_nv);
        kin_tasks_weld.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_weld.getId("CoMXY_HipRPY");
        Eigen::MatrixXd taskMapRPY = Eigen::MatrixXd::Zero(3, 6);
        taskMapRPY(0, 3) = 1;
        taskMapRPY(1, 4) = 1;
        taskMapRPY(2, 5) = 1;
        kin_tasks_weld.taskLib[id].errX = Eigen::VectorXd::Zero(5);
        kin_tasks_weld.taskLib[id].errX.block(0, 0, 2, 1) = pCoMDes.block(0, 0, 2, 1) - pCoMCur.block(0, 0, 2, 1);
        Eigen::Matrix3d desRot = eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kin_tasks_weld.taskLib[id].errX.block<3, 1>(2, 0) = diffRot(hip_link_rot, desRot);
        kin_tasks_weld.taskLib[id].derrX = Eigen::VectorXd::Zero(5);
        kin_tasks_weld.taskLib[id].ddxDes = Eigen::VectorXd::Zero(5);
        kin_tasks_weld.taskLib[id].dxDes = Eigen::VectorXd::Zero(5);
        kin_tasks_weld.taskLib[id].kp = Eigen::MatrixXd::Identity(5, 5) * 250;
        kin_tasks_weld.taskLib[id].kp.block(2, 2, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * 1000;
        kin_tasks_weld.taskLib[id].kd = Eigen::MatrixXd::Identity(5, 5) * 10;
        kin_tasks_weld.taskLib[id].kd.block(2, 2, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * 10;
        kin_tasks_weld.taskLib[id].J = Eigen::MatrixXd::Zero(5, model_nv);
        kin_tasks_weld.taskLib[id].J.block(0, 0, 2, model_nv) = Jcom.block(0, 0, 2, model_nv);
        kin_tasks_weld.taskLib[id].J.block(2, 0, 3, model_nv) = taskMapRPY * J_hip_link;
        kin_tasks_weld.taskLib[id].J.block(0, 18, 2, 5).setZero();
        kin_tasks_weld.taskLib[id].J.block(0, 23, 2, 5).setZero();
        kin_tasks_weld.taskLib[id].J.block(2, 18, 3, 5).setZero();
        kin_tasks_weld.taskLib[id].J.block(2, 23, 3, 5).setZero();
        kin_tasks_weld.taskLib[id].dJ = Eigen::MatrixXd::Zero(5, model_nv);
        kin_tasks_weld.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_weld.getId("Pz");
        kin_tasks_weld.taskLib[id].errX = Eigen::VectorXd::Zero(1);
        kin_tasks_weld.taskLib[id].errX(0) = base_pos_des(2) - q(2);
        kin_tasks_weld.taskLib[id].derrX = Eigen::VectorXd::Zero(1);
        kin_tasks_weld.taskLib[id].ddxDes = Eigen::VectorXd::Zero(1);
        kin_tasks_weld.taskLib[id].dxDes = Eigen::VectorXd::Zero(1);
        kin_tasks_weld.taskLib[id].kp = Eigen::MatrixXd::Identity(1, 1) * 2000;
        kin_tasks_weld.taskLib[id].kd = Eigen::MatrixXd::Identity(1, 1) * 10;
        Eigen::MatrixXd taskMap = Eigen::MatrixXd::Zero(1, 6);
        taskMap(0, 2) = 1;
        kin_tasks_weld.taskLib[id].J = taskMap * J_base;
        kin_tasks_weld.taskLib[id].dJ = taskMap * dJ_base;
        kin_tasks_weld.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        Eigen::Vector3d hAng = Eigen::Vector3d::Zero();
        Eigen::MatrixXd AgAng = Eigen::MatrixXd::Zero(3, model_nv);
        Eigen::MatrixXd dAgAng = Eigen::MatrixXd::Zero(3, model_nv);
        if (dyn_Ag.rows() >= 6 && dyn_Ag.cols() == model_nv)
        {
            AgAng = dyn_Ag.block(3, 0, 3, model_nv);
            hAng = AgAng * dq;
        }
        if (dyn_dAg.rows() >= 6 && dyn_dAg.cols() == model_nv)
        {
            dAgAng = dyn_dAg.block(3, 0, 3, model_nv);
        }

        id = kin_tasks_weld.getId("AngularMomentumDamping");
        kin_tasks_weld.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        kin_tasks_weld.taskLib[id].derrX = -hAng;
        kin_tasks_weld.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_weld.taskLib[id].dxDes = hAng;
        kin_tasks_weld.taskLib[id].kp = Eigen::MatrixXd::Zero(3, 3);
        kin_tasks_weld.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * cfg_weld_ang_momentum_damping;
        if (cfg_weld_ang_momentum_damping > 1.0e-9 &&
            (motionStateCur == DataBus::Weld || motionStateCur == DataBus::WeldHold ||
             motionStateCur == DataBus::WeldRecover))
        {
            kin_tasks_weld.taskLib[id].J = AgAng;
            kin_tasks_weld.taskLib[id].dJ = dAgAng;
        }
        else
        {
            kin_tasks_weld.taskLib[id].derrX.setZero();
            kin_tasks_weld.taskLib[id].J = Eigen::MatrixXd::Zero(3, model_nv);
            kin_tasks_weld.taskLib[id].dJ = Eigen::MatrixXd::Zero(3, model_nv);
        }
        kin_tasks_weld.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_weld.getId("RightHandWeld");
        kin_tasks_weld.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        Eigen::Vector3d weldPosErr = weld_tcp_pos_des_W - hd_r_pos_cur_W;
        const double weldPosErrNorm = weldPosErr.norm();
        const bool isWeldPrepare = (motionStateCur == DataBus::WeldPrepare);
        const double maxWeldPosErrForTask = isWeldPrepare ? 0.025 : 0.025;
        if (weldPosErrNorm > maxWeldPosErrForTask)
        {
            weldPosErr *= maxWeldPosErrForTask / weldPosErrNorm;
        }
        const double preparePhase = std::clamp(weld_prepare_phase, 0.0, 1.0);
        const double handTaskScale = isWeldPrepare
                                         ? preparePhase * preparePhase * (3.0 - 2.0 * preparePhase)
                                         : 1.0;
        weldPosErr *= handTaskScale;
        kin_tasks_weld.taskLib[id].errX.block<3, 1>(0, 0) = weldPosErr;
        Eigen::MatrixXd rightHandJ = Eigen::MatrixXd::Zero(3, model_nv);
        Eigen::MatrixXd rightHanddJ = Eigen::MatrixXd::Zero(3, model_nv);
        rightHandJ.block(0, 23, 3, 5) = J_hd_r.block(0, 23, 3, 5);
        rightHanddJ.block(0, 23, 3, 5) = dJ_hd_r.block(0, 23, 3, 5);
        Eigen::Vector3d handVel = rightHandJ * dq;
        kin_tasks_weld.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kin_tasks_weld.taskLib[id].derrX.block<3, 1>(0, 0) =
            handTaskScale * weld_tcp_linear_vel_des_W - handVel.block<3, 1>(0, 0);
        kin_tasks_weld.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_weld.taskLib[id].ddxDes.block<3, 1>(0, 0) = handTaskScale * weld_tcp_linear_acc_des_W;
        kin_tasks_weld.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kin_tasks_weld.taskLib[id].dxDes.block<3, 1>(0, 0) = handTaskScale * weld_tcp_linear_vel_des_W;
        kin_tasks_weld.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * (cfg_weld_hand_kp * handTaskScale);
        kin_tasks_weld.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * (cfg_weld_hand_kd * handTaskScale);
        kin_tasks_weld.taskLib[id].J = rightHandJ;
        kin_tasks_weld.taskLib[id].dJ = rightHanddJ;
        kin_tasks_weld.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_weld.getId("RightArmRecover");
        Eigen::VectorXd target_right_arm = Eigen::VectorXd::Zero(5);
        target_right_arm << -0.3, -1.4, 0.0, 1.4, 0.0;
        if (qIniDes.size() >= 29)
        {
            target_right_arm = qIniDes.block<5, 1>(24, 0);
        }
        const double rightArmRecoverScale =
            (motionStateCur == DataBus::WeldRecover) ? std::clamp(weld_recover_phase, 0.0, 1.0) : 0.0;
        kin_tasks_weld.taskLib[id].errX = target_right_arm - q.block<5, 1>(24, 0);
        kin_tasks_weld.taskLib[id].derrX = -dq.block<5, 1>(23, 0);
        kin_tasks_weld.taskLib[id].ddxDes = Eigen::VectorXd::Zero(5);
        kin_tasks_weld.taskLib[id].dxDes = Eigen::VectorXd::Zero(5);
        kin_tasks_weld.taskLib[id].kp = Eigen::MatrixXd::Identity(5, 5) *
                                        (cfg_weld_right_arm_recover_kp * rightArmRecoverScale);
        kin_tasks_weld.taskLib[id].kd = Eigen::MatrixXd::Identity(5, 5) *
                                        (cfg_weld_right_arm_recover_kd * rightArmRecoverScale);
        kin_tasks_weld.taskLib[id].J = Eigen::MatrixXd::Zero(5, model_nv);
        if (rightArmRecoverScale > 1.0e-6)
        {
            kin_tasks_weld.taskLib[id].J.block(0, 23, 5, 5) = Eigen::MatrixXd::Identity(5, 5);
        }
        else
        {
            kin_tasks_weld.taskLib[id].errX.setZero();
            kin_tasks_weld.taskLib[id].derrX.setZero();
        }
        kin_tasks_weld.taskLib[id].dJ = Eigen::MatrixXd::Zero(5, model_nv);
        kin_tasks_weld.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

        id = kin_tasks_weld.getId("LeftArmBalance");
        Eigen::VectorXd target_left_arm = Eigen::VectorXd::Zero(5);
        target_left_arm << 0.3, 1.4, 0.0, -1.4, 0.0;
        if (qIniDes.size() >= 24)
        {
            target_left_arm = qIniDes.block<5, 1>(19, 0);
        }
        const bool isLeftArmBalanceState = (motionStateCur == DataBus::WeldPrepare ||
                                            motionStateCur == DataBus::Weld ||
                                            motionStateCur == DataBus::WeldHold ||
                                            motionStateCur == DataBus::WeldRecover);
        const double leftArmTaskScale = (motionStateCur == DataBus::WeldPrepare)
                                            ? preparePhase * preparePhase * (3.0 - 2.0 * preparePhase)
                                            : (isLeftArmBalanceState ? 1.0 : 0.0);
        Eigen::VectorXd leftArmVelCmd = Eigen::VectorXd::Zero(5);
        if (dyn_Ag.rows() >= 6 && dyn_Ag.cols() == model_nv)
        {
            const Eigen::MatrixXd AgLeft = dyn_Ag.block(3, 18, 3, 5);
            leftArmVelCmd = -cfg_weld_left_arm_momentum_gain *
                            AgLeft.completeOrthogonalDecomposition().pseudoInverse() * hAng;
            const double velNorm = leftArmVelCmd.norm();
            if (velNorm > cfg_weld_left_arm_vel_limit)
            {
                leftArmVelCmd *= cfg_weld_left_arm_vel_limit / velNorm;
            }
        }
        leftArmVelCmd *= leftArmTaskScale;
        weld_left_arm_balance_vel = leftArmVelCmd;
        kin_tasks_weld.taskLib[id].errX = target_left_arm - q.block<5, 1>(19, 0);
        kin_tasks_weld.taskLib[id].derrX = leftArmVelCmd - dq.block<5, 1>(18, 0);
        kin_tasks_weld.taskLib[id].ddxDes = Eigen::VectorXd::Zero(5);
        kin_tasks_weld.taskLib[id].dxDes = leftArmVelCmd;
        kin_tasks_weld.taskLib[id].kp = Eigen::MatrixXd::Identity(5, 5) * (cfg_weld_left_arm_kp * leftArmTaskScale);
        kin_tasks_weld.taskLib[id].kd = Eigen::MatrixXd::Identity(5, 5) * (cfg_weld_left_arm_kd * leftArmTaskScale);
        kin_tasks_weld.taskLib[id].J = Eigen::MatrixXd::Zero(5, model_nv);
        if (isLeftArmBalanceState && leftArmTaskScale > 1.0e-6 &&
            (cfg_weld_left_arm_kp > 1.0e-9 || cfg_weld_left_arm_kd > 1.0e-9 ||
             cfg_weld_left_arm_momentum_gain > 1.0e-9))
        {
            kin_tasks_weld.taskLib[id].J.block(0, 18, 5, 5) = Eigen::MatrixXd::Identity(5, 5);
        }
        else
        {
            kin_tasks_weld.taskLib[id].errX.setZero();
            kin_tasks_weld.taskLib[id].derrX.setZero();
            kin_tasks_weld.taskLib[id].dxDes.setZero();
        }
        kin_tasks_weld.taskLib[id].dJ = Eigen::MatrixXd::Zero(5, model_nv);
        kin_tasks_weld.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);
    }

    if (motionStateCur == DataBus::Walk || motionStateCur == DataBus::Walk2Stand)
    {
        kin_tasks_walk.computeAll(des_delta_q, des_dq, des_ddq, dyn_M, dyn_M_inv, dq);
        delta_q_final_kin = kin_tasks_walk.out_delta_q;
        dq_final_kin = kin_tasks_walk.out_dq;
        ddq_final_kin = kin_tasks_walk.out_ddq;
    }
    else if (motionStateCur == DataBus::Stand)
    {
        kin_tasks_stand.computeAll(des_delta_q, des_dq, des_ddq, dyn_M, dyn_M_inv, dq);
        delta_q_final_kin = kin_tasks_stand.out_delta_q;
        dq_final_kin = kin_tasks_stand.out_dq;
        ddq_final_kin = kin_tasks_stand.out_ddq;
    }
    else if (motionStateCur == DataBus::WeldPrepare || motionStateCur == DataBus::Weld ||
             motionStateCur == DataBus::WeldHold || motionStateCur == DataBus::WeldRecover)
    {
        kin_tasks_weld.computeAll(des_delta_q, des_dq, des_ddq, dyn_M, dyn_M_inv, dq);
        delta_q_final_kin = kin_tasks_weld.out_delta_q;
        dq_final_kin = kin_tasks_weld.out_dq;
        ddq_final_kin = kin_tasks_weld.out_ddq;
        const double deltaQLimit = std::max(0.0, cfg_weld_arm_delta_q_limit);
        const double dqLimit = std::max(0.0, cfg_weld_arm_dq_limit);
        const double ddqLimit = std::max(0.0, cfg_weld_arm_ddq_limit);
        for (int i = 18; i < 28 && i < model_nv; ++i)
        {
            delta_q_final_kin(i) = std::clamp(delta_q_final_kin(i), -deltaQLimit, deltaQLimit);
            dq_final_kin(i) = std::clamp(dq_final_kin(i), -dqLimit, dqLimit);
            ddq_final_kin(i) = std::clamp(ddq_final_kin(i), -ddqLimit, ddqLimit);
        }
    }
    else
    {
        delta_q_final_kin = Eigen::VectorXd::Zero(model_nv);
        dq_final_kin = Eigen::VectorXd::Zero(model_nv);
        ddq_final_kin = Eigen::VectorXd::Zero(model_nv);
    }
}

void WBC_priority_V4::copy_Eigen_to_real_t(qpOASES::real_t *target, const Eigen::MatrixXd &source, int nRows, int nCols)
{
    int count = 0;
    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            target[count++] = isinf(source(i, j)) ? qpOASES::INFTY : source(i, j);
        }
    }
}

void WBC_priority_V4::setQini(const Eigen::VectorXd &qIniDesIn, const Eigen::VectorXd &qIniCurIn)
{
    qIniDes = qIniDesIn;
    qIniCur = qIniCurIn;
}
