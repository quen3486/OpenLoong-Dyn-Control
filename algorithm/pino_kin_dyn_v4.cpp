/*
 * Pin_KinDyn implementation for speedbot_v4 robot.
 * Adapted from pino_kin_dyn.cpp for OpenLoong.
 *
 * Pinocchio joint ordering (URDF tree traversal):
 *   leg_l: 0-5, leg_r: 6-11, arm_l: 12-16, arm_r: 17-21
 *   (fixed-base model uses the same ordering, indices 0-21)
 */
#include "pino_kin_dyn_v4.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <utility>

namespace
{

Eigen::Vector3d rightWeldTcpOffsetLocal()
{
    return Eigen::Vector3d(0.0, -0.215, 0.0);
}

Eigen::Matrix3d skewMatrix(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d skew;
    skew << 0.0, -v.z(), v.y(),
            v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;
    return skew;
}

Eigen::Vector3d rightWeldTcpPosition(const pinocchio::SE3 &wristPose)
{
    return wristPose.translation() + wristPose.rotation() * rightWeldTcpOffsetLocal();
}

template <typename JacobianDerived>
void shiftSpatialJacobianToPoint(Eigen::MatrixBase<JacobianDerived> &jacobian, const Eigen::Vector3d &offset_W)
{
    if (jacobian.rows() < 6)
    {
        return;
    }
    jacobian.derived().topRows(3) -= skewMatrix(offset_W) * jacobian.derived().block(3, 0, 3, jacobian.cols());
}

template <typename DJacobianDerived, typename JacobianDerived>
void shiftSpatialJacobianTimeVariationToPoint(Eigen::MatrixBase<DJacobianDerived> &dJacobian,
                                              const Eigen::MatrixBase<JacobianDerived> &wristJacobian,
                                              const Eigen::Vector3d &offset_W,
                                              const Eigen::VectorXd &dq)
{
    if (dJacobian.rows() < 6 || wristJacobian.rows() < 6 || dq.size() != wristJacobian.cols())
    {
        return;
    }
    const Eigen::MatrixXd wristAngularJacobian = wristJacobian.derived().block(3, 0, 3, wristJacobian.cols());
    const Eigen::Vector3d wristOmega = wristAngularJacobian * dq;
    const Eigen::Vector3d offsetDot_W = wristOmega.cross(offset_W);
    dJacobian.derived().topRows(3) -=
        skewMatrix(offset_W) * dJacobian.derived().block(3, 0, 3, dJacobian.cols()) +
        skewMatrix(offsetDot_W) * wristAngularJacobian;
}

} // namespace

Pin_KinDyn_V4::Pin_KinDyn_V4(std::string urdf_pathIn)
{
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(urdf_pathIn, root_joint, model_biped);
    pinocchio::urdf::buildModel(urdf_pathIn, model_biped_fixed);
    data_biped = pinocchio::Data(model_biped);
    data_biped_fixed = pinocchio::Data(model_biped_fixed);
    model_nv = model_biped.nv;
    J_l = Eigen::MatrixXd::Zero(6, model_nv);
    J_r = Eigen::MatrixXd::Zero(6, model_nv);
    J_l_body = Eigen::MatrixXd::Zero(6, model_biped_fixed.nv);
    J_r_body = Eigen::MatrixXd::Zero(6, model_biped_fixed.nv);
    J_hd_l = Eigen::MatrixXd::Zero(6, model_nv);
    J_hd_r = Eigen::MatrixXd::Zero(6, model_nv);
    J_base = Eigen::MatrixXd::Zero(6, model_nv);
    J_hip_link = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_l = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_r = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_hd_l = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_hd_r = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_base = Eigen::MatrixXd::Zero(6, model_nv);
    q.setZero();
    dq.setZero();
    ddq.setZero();
    Rcur.setIdentity();
    dyn_M = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_M_inv = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_C = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_G = Eigen::MatrixXd::Zero(model_nv, 1);

    // Joint name lookups for speedbot_v4
    r_ankle_joint = model_biped.getJointId("right_ankle_roll_joint");
    l_ankle_joint = model_biped.getJointId("left_ankle_roll_joint");
    r_hand_joint = model_biped.getJointId("right_wrist_roll_joint");
    l_hand_joint = model_biped.getJointId("left_wrist_roll_joint");
    r_hand_joint_fixed = model_biped_fixed.getJointId("right_wrist_roll_joint");
    l_hand_joint_fixed = model_biped_fixed.getJointId("left_wrist_roll_joint");
    r_hip_joint = model_biped.getJointId("right_hip_yaw_joint");
    l_hip_joint = model_biped.getJointId("left_hip_yaw_joint");
    r_hip_roll_joint = model_biped.getJointId("right_hip_roll_joint");
    l_hip_roll_joint = model_biped.getJointId("left_hip_roll_joint");
    r_ankle_joint_fixed = model_biped_fixed.getJointId("right_ankle_roll_joint");
    l_ankle_joint_fixed = model_biped_fixed.getJointId("left_ankle_roll_joint");
    r_hip_joint_fixed = model_biped_fixed.getJointId("right_hip_yaw_joint");
    l_hip_joint_fixed = model_biped_fixed.getJointId("left_hip_yaw_joint");
    base_joint = model_biped.getJointId("root_joint");
    motorMaxTorque = Eigen::VectorXd::Zero(motorName.size());
    motorMaxPos = Eigen::VectorXd::Zero(motorName.size());
    motorMinPos = Eigen::VectorXd::Zero(motorName.size());

    Json::Reader reader;
    Json::Value root_read;
    std::ifstream in;
    const std::vector<std::string> jointConfigCandidates = {
        "joint_ctrl_config_v4.json",
        "../common/joint_ctrl_config_v4.json",
        "common/joint_ctrl_config_v4.json"};
    for (const auto &path : jointConfigCandidates)
    {
        in.open(path, std::ios::binary);
        if (in.good())
        {
            if (!reader.parse(in, root_read))
            {
                std::cerr << "[Pin_KinDyn_V4] failed to parse " << path << std::endl;
            }
            break;
        }
        in.close();
    }
    if (root_read.empty())
    {
        std::cerr << "[Pin_KinDyn_V4] failed to load joint_ctrl_config_v4.json; "
                  << "joint limits/torque limits remain zero." << std::endl;
    }
    for (int i = 0; i < (int)motorName.size(); i++)
    {
        motorMaxTorque(i) = (root_read[motorName[i]]["maxTorque"].asDouble());
        motorMaxPos(i) = (root_read[motorName[i]]["maxPos"].asDouble());
        motorMinPos(i) = (root_read[motorName[i]]["minPos"].asDouble());
    }
    motorReachLimit.assign(motorName.size(), false);
    tauJointOld = Eigen::VectorXd::Zero(motorName.size());
}

void Pin_KinDyn_V4::dataBusRead(const DataBus &robotState)
{
    q = robotState.q;
    dq = robotState.dq;
    dq.block(0, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(0, 0, 3, 1);
    dq.block(3, 0, 3, 1) = robotState.base_rot.transpose() * dq.block(3, 0, 3, 1);
    ddq = robotState.ddq;
}

void Pin_KinDyn_V4::dataBusWrite(DataBus &robotState)
{
    robotState.J_l = J_l;
    robotState.J_r = J_r;
    robotState.J_base = J_base;
    robotState.dJ_l = dJ_l;
    robotState.dJ_r = dJ_r;
    robotState.J_hd_l = J_hd_l;
    robotState.J_hd_r = J_hd_r;
    robotState.dJ_hd_l = dJ_hd_l;
    robotState.dJ_hd_r = dJ_hd_r;
    robotState.dJ_base = dJ_base;
    robotState.J_hip_link = J_hip_link;
    robotState.fe_l_pos_W = fe_l_pos;
    robotState.fe_r_pos_W = fe_r_pos;
    robotState.fe_l_pos_L = fe_l_pos_body;
    robotState.fe_r_pos_L = fe_r_pos_body;
    robotState.fe_l_rot_W = fe_l_rot;
    robotState.fe_r_rot_W = fe_r_rot;
    robotState.fe_l_rot_L = fe_l_rot_body;
    robotState.fe_r_rot_L = fe_r_rot_body;
    robotState.fe_l_vel_L = fe_l_vel_body;
    robotState.fe_r_vel_L = fe_r_vel_body;
    robotState.hip_r_pos_L = hip_r_pos_body;
    robotState.hip_l_pos_L = hip_l_pos_body;
    robotState.hip_r_pos_W = hip_r_pos;
    robotState.hip_l_pos_W = hip_l_pos;
    robotState.hd_l_pos_L = hd_l_pos_body;
    robotState.hd_l_rot_L = hd_l_rot_body;
    robotState.hd_l_pos_W = hd_l_pos;
    robotState.hd_l_rot_W = hd_l_rot;
    robotState.hd_r_pos_L = hd_r_pos_body;
    robotState.hd_r_rot_L = hd_r_rot_body;
    robotState.hd_r_pos_W = hd_r_pos;
    robotState.hd_r_rot_W = hd_r_rot;
    robotState.hip_link_pos = hip_link_pos;
    robotState.hip_link_rot = hip_link_rot;

    robotState.dyn_M = dyn_M;
    robotState.dyn_M_inv = dyn_M_inv;
    robotState.dyn_C = dyn_C;
    robotState.dyn_G = dyn_G;
    robotState.dyn_Ag = dyn_Ag;
    robotState.dyn_dAg = dyn_dAg;
    robotState.dyn_Non = dyn_Non;

    robotState.pCoM_W = CoM_pos;
    robotState.Jcom_W = Jcom;

    robotState.inertia = inertia;
}

void Pin_KinDyn_V4::computeJ_dJ()
{
    pinocchio::forwardKinematics(model_biped, data_biped, q);
    pinocchio::jacobianCenterOfMass(model_biped, data_biped, q, true);
    pinocchio::computeJointJacobiansTimeVariation(model_biped, data_biped, q, dq);
    pinocchio::updateGlobalPlacements(model_biped, data_biped);
    pinocchio::getJointJacobian(model_biped, data_biped, r_ankle_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_r);
    pinocchio::getJointJacobian(model_biped, data_biped, l_ankle_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_l);
    pinocchio::getJointJacobian(model_biped, data_biped, r_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hd_r);
    pinocchio::getJointJacobian(model_biped, data_biped, l_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hd_l);
    pinocchio::getJointJacobian(model_biped, data_biped, base_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_base);

    Eigen::Matrix<double, 6, -1> J_hip_roll_l, J_hip_roll_r;
    J_hip_roll_l = Eigen::MatrixXd::Zero(6, model_nv);
    J_hip_roll_r = Eigen::MatrixXd::Zero(6, model_nv);
    pinocchio::getJointJacobian(model_biped, data_biped, l_hip_roll_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_roll_l);
    pinocchio::getJointJacobian(model_biped, data_biped, r_hip_roll_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_roll_r);

    // For speedbot_v4: legs branch directly from base_link, hip_link = base_joint
    // Use base_joint as the "hip link" since there's no waist chain between base and legs
    pinocchio::getJointJacobian(model_biped, data_biped, base_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_link);

    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, r_ankle_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_r);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, l_ankle_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_l);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, r_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_hd_r);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, l_hand_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_hd_l);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, base_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_base);
    const Eigen::Vector3d rHandTcpOffset_W =
        data_biped.oMi[r_hand_joint].rotation() * rightWeldTcpOffsetLocal();
    const Eigen::MatrixXd J_hd_r_wrist = J_hd_r;
    shiftSpatialJacobianTimeVariationToPoint(dJ_hd_r, J_hd_r_wrist, rHandTcpOffset_W, dq);
    shiftSpatialJacobianToPoint(J_hd_r, rHandTcpOffset_W);
    fe_l_pos = data_biped.oMi[l_ankle_joint].translation();
    fe_l_rot = data_biped.oMi[l_ankle_joint].rotation();
    hip_l_pos = data_biped.oMi[l_hip_joint].translation();
    fe_r_pos = data_biped.oMi[r_ankle_joint].translation();
    fe_r_rot = data_biped.oMi[r_ankle_joint].rotation();
    hip_r_pos = data_biped.oMi[r_hip_joint].translation();
    base_pos = data_biped.oMi[base_joint].translation();
    base_rot = data_biped.oMi[base_joint].rotation();
    hd_l_pos = data_biped.oMi[l_hand_joint].translation();
    hd_l_rot = data_biped.oMi[l_hand_joint].rotation();
    hd_r_pos = rightWeldTcpPosition(data_biped.oMi[r_hand_joint]);
    hd_r_rot = data_biped.oMi[r_hand_joint].rotation();
    // For v4: hip_link uses base_joint since legs attach directly to base
    hip_link_pos = data_biped.oMi[base_joint].translation();
    hip_link_rot = data_biped.oMi[base_joint].rotation();
    Jcom = data_biped.Jcom;

    Eigen::MatrixXd Mpj;
    Mpj = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj.block(0, 0, 3, 3) = base_rot.transpose();
    Mpj.block(3, 3, 3, 3) = base_rot.transpose();
    J_l = J_l * Mpj;
    J_r = J_r * Mpj;
    J_base = J_base * Mpj;
    dJ_l = dJ_l * Mpj;
    dJ_r = dJ_r * Mpj;
    J_hd_l = J_hd_l * Mpj;
    J_hd_r = J_hd_r * Mpj;
    dJ_hd_l = dJ_hd_l * Mpj;
    dJ_hd_r = dJ_hd_r * Mpj;
    dJ_base = dJ_base * Mpj;
    J_hip_link = J_hip_link * Mpj;
    Jcom = Jcom * Mpj;

    Eigen::VectorXd q_fixed, dq_fixed;
    q_fixed = q.block(7, 0, model_biped_fixed.nv, 1);
    dq_fixed = dq.block(6, 0, model_biped_fixed.nv, 1);
    pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, q_fixed);
    pinocchio::computeJointJacobians(model_biped_fixed, data_biped_fixed, q_fixed);
    pinocchio::updateGlobalPlacements(model_biped_fixed, data_biped_fixed);
    pinocchio::getJointJacobian(model_biped_fixed, data_biped_fixed, r_ankle_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_r_body);
    pinocchio::getJointJacobian(model_biped_fixed, data_biped_fixed, l_ankle_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_l_body);
    fe_l_pos_body = data_biped_fixed.oMi[l_ankle_joint_fixed].translation();
    fe_r_pos_body = data_biped_fixed.oMi[r_ankle_joint_fixed].translation();
    fe_l_rot_body = data_biped_fixed.oMi[l_ankle_joint_fixed].rotation();
    fe_r_rot_body = data_biped_fixed.oMi[r_ankle_joint_fixed].rotation();
    hip_l_pos_body = data_biped_fixed.oMi[l_hip_joint_fixed].translation();
    hip_r_pos_body = data_biped_fixed.oMi[r_hip_joint_fixed].translation();
    hd_l_pos_body = data_biped_fixed.oMi[l_hand_joint_fixed].translation();
    hd_l_rot_body = data_biped_fixed.oMi[l_hand_joint_fixed].rotation();
    hd_r_pos_body = rightWeldTcpPosition(data_biped_fixed.oMi[r_hand_joint_fixed]);
    hd_r_rot_body = data_biped_fixed.oMi[r_hand_joint_fixed].rotation();
    fe_l_vel_body = (J_l_body * dq_fixed).block(0, 0, 3, 1);
    fe_r_vel_body = (J_r_body * dq_fixed).block(0, 0, 3, 1);
}

Eigen::Quaterniond Pin_KinDyn_V4::intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w)
{
    Eigen::Matrix3d Rcur = quat.normalized().toRotationMatrix();
    Eigen::Matrix3d Rinc = Eigen::Matrix3d::Identity();
    double theta = w.norm();
    if (theta > 1e-8)
    {
        Eigen::Vector3d w_norm;
        w_norm = w / theta;
        Eigen::Matrix3d a;
        a << 0, -w_norm(2), w_norm(1),
            w_norm(0), 0, -w_norm(0),
            -w_norm(1), w_norm(0), 0;
        Rinc = Eigen::Matrix3d::Identity() + a * sin(theta) + a * a * (1 - cos(theta));
    }
    Eigen::Matrix3d Rend = Rcur * Rinc;
    Eigen::Quaterniond quatRes;
    quatRes = Rend;
    return quatRes;
}

Eigen::VectorXd Pin_KinDyn_V4::integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI)
{
    Eigen::VectorXd qRes = Eigen::VectorXd::Zero(model_nv + 1);
    Eigen::Vector3d wDes;
    wDes << dqI(3), dqI(4), dqI(5);
    Eigen::Quaterniond quatNew, quatNow;
    quatNow.x() = qI(3);
    quatNow.y() = qI(4);
    quatNow.z() = qI(5);
    quatNow.w() = qI(6);
    quatNew = intQuat(quatNow, wDes);
    qRes = qI;
    qRes(0) += dqI(0);
    qRes(1) += dqI(1);
    qRes(2) += dqI(2);
    qRes(3) = quatNew.x();
    qRes(4) = quatNew.y();
    qRes(5) = quatNew.z();
    qRes(6) = quatNew.w();
    for (int i = 0; i < model_nv - 6; i++)
        qRes(7 + i) += dqI(6 + i);
    return qRes;
}

void Pin_KinDyn_V4::computeDyn()
{
    pinocchio::crba(model_biped, data_biped, q);
    data_biped.M.triangularView<Eigen::Lower>() = data_biped.M.transpose().triangularView<Eigen::Lower>();
    dyn_M = data_biped.M;

    pinocchio::computeMinverse(model_biped, data_biped, q);
    data_biped.Minv.triangularView<Eigen::Lower>() = data_biped.Minv.transpose().triangularView<Eigen::Lower>();
    dyn_M_inv = data_biped.Minv;

    pinocchio::computeCoriolisMatrix(model_biped, data_biped, q, dq);
    dyn_C = data_biped.C;

    pinocchio::computeGeneralizedGravity(model_biped, data_biped, q);
    dyn_G = data_biped.g;

    pinocchio::dccrba(model_biped, data_biped, q, dq);
    pinocchio::computeCentroidalMomentum(model_biped, data_biped, q, dq);
    dyn_Ag = data_biped.Ag;
    dyn_dAg = data_biped.dAg;

    dyn_Non = dyn_C * dq + dyn_G;

    pinocchio::ccrba(model_biped, data_biped, q, dq);
    inertia = data_biped.Ig.inertia().matrix();

    CoM_pos = data_biped.com[0];

    Eigen::MatrixXd Mpj, Mpj_inv;
    Mpj = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj_inv = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj.block(0, 0, 3, 3) = base_rot.transpose();
    Mpj.block(3, 3, 3, 3) = base_rot.transpose();
    Mpj_inv.block(0, 0, 3, 3) = base_rot;
    Mpj_inv.block(3, 3, 3, 3) = base_rot;
    dyn_M = Mpj_inv * dyn_M * Mpj;
    dyn_M_inv = Mpj_inv * dyn_M_inv * Mpj;
    dyn_C = Mpj_inv * dyn_C * Mpj;
    dyn_G = Mpj_inv * dyn_G;
    dyn_Non = Mpj_inv * dyn_Non;
}

// Inverse kinematics for leg posture
// Fixed-base joint order: leg_l:0-5, leg_r:6-11, arm_l:12-16, arm_r:17-21
Pin_KinDyn_V4::IkRes
Pin_KinDyn_V4::computeInK_Leg(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R,
                              const Eigen::Vector3d &Pdes_R)
{
    const pinocchio::SE3 oMdesL(Rdes_L, Pdes_L);
    const pinocchio::SE3 oMdesR(Rdes_R, Pdes_R);
    Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_biped_fixed.nv);
    // Initial guess: slight knee bend
    // left_knee = fixed index 3, right_knee = fixed index 9
    qIk[3] = -0.1;
    qIk[9] = -0.1;

    const double eps = 1e-4;
    const int IT_MAX = 100;
    const double DT = 7e-1;
    const double damp = 5e-3;
    Eigen::MatrixXd JL(6, model_biped_fixed.nv);
    Eigen::MatrixXd JR(6, model_biped_fixed.nv);
    Eigen::MatrixXd JCompact(12, model_biped_fixed.nv);
    JL.setZero();
    JR.setZero();
    JCompact.setZero();

    bool success = false;
    Eigen::Matrix<double, 6, 1> errL, errR;
    Eigen::Matrix<double, 12, 1> errCompact;
    Eigen::VectorXd v(model_biped_fixed.nv);

    pinocchio::JointIndex J_Idx_l, J_Idx_r;
    J_Idx_l = l_ankle_joint_fixed;
    J_Idx_r = r_ankle_joint_fixed;
    int itr_count{0};
    for (itr_count = 0;; itr_count++)
    {
        pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
        const pinocchio::SE3 iMdL = data_biped_fixed.oMi[J_Idx_l].actInv(oMdesL);
        const pinocchio::SE3 iMdR = data_biped_fixed.oMi[J_Idx_r].actInv(oMdesR);
        errL = pinocchio::log6(iMdL).toVector();
        errR = pinocchio::log6(iMdR).toVector();
        errCompact.block<6, 1>(0, 0) = errL;
        errCompact.block<6, 1>(6, 0) = errR;
        if (errCompact.norm() < eps)
        {
            success = true;
            break;
        }
        if (itr_count >= IT_MAX)
        {
            success = false;
            break;
        }

        pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_l, JL);
        pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_r, JR);
        Eigen::MatrixXd W;
        W = Eigen::MatrixXd::Identity(model_biped_fixed.nv, model_biped_fixed.nv);
        pinocchio::Data::Matrix6 JlogL;
        pinocchio::Data::Matrix6 JlogR;
        pinocchio::Jlog6(iMdL.inverse(), JlogL);
        pinocchio::Jlog6(iMdR.inverse(), JlogR);
        JL = -JlogL * JL;
        JR = -JlogR * JR;
        JCompact.block(0, 0, 6, model_biped_fixed.nv) = JL;
        JCompact.block(6, 0, 6, model_biped_fixed.nv) = JR;
        Eigen::Matrix<double, 12, 12> JJt;
        JJt.noalias() = JCompact * W * JCompact.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -W * JCompact.transpose() * JJt.ldlt().solve(errCompact);
        qIk = pinocchio::integrate(model_biped_fixed, qIk, v * DT);
    }

    IkRes res;
    res.err = errCompact;
    res.itr = itr_count;

    if (success)
        res.status = 0;
    else
        res.status = -1;

    res.jointPosRes = qIk;
    return res;
}

// Inverse Kinematics for hand posture
Pin_KinDyn_V4::IkRes
Pin_KinDyn_V4::computeInK_Hand(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R,
                               const Eigen::Vector3d &Pdes_R)
{
    const pinocchio::SE3 oMdesL(Rdes_L, Pdes_L);
    const pinocchio::SE3 oMdesR(Rdes_R, Pdes_R);
    Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_biped_fixed.nv);
    // Initial guess for arm joints (5 DoF per arm)
    // arm_l: fixed indices 12-16, arm_r: fixed indices 17-21
    qIk.block<5, 1>(12, 0) << 0.0, -1.0, 0.0, 0.8, 0.0;
    qIk.block<5, 1>(17, 0) << 0.0, -1.0, 0.0, 0.8, 0.0;

    const double eps = 1e-4;
    const int IT_MAX = 100;
    const double DT = 6e-1;
    const double damp = 1e-2;
    Eigen::MatrixXd JL(6, model_biped_fixed.nv);
    Eigen::MatrixXd JR(6, model_biped_fixed.nv);
    Eigen::MatrixXd JCompact(12, model_biped_fixed.nv);
    JL.setZero();
    JR.setZero();
    JCompact.setZero();

    bool success = false;
    Eigen::Matrix<double, 6, 1> errL, errR;
    Eigen::Matrix<double, 12, 1> errCompact;
    Eigen::VectorXd v(model_biped_fixed.nv);

    pinocchio::JointIndex J_Idx_l, J_Idx_r;
    J_Idx_l = l_hand_joint_fixed;
    J_Idx_r = r_hand_joint_fixed;
    int itr_count{0};
    for (itr_count = 0;; itr_count++)
    {
        pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
        const pinocchio::SE3 iMdL = data_biped_fixed.oMi[J_Idx_l].actInv(oMdesL);
        const pinocchio::SE3 iMdR = data_biped_fixed.oMi[J_Idx_r].actInv(oMdesR);
        errL = pinocchio::log6(iMdL).toVector();
        errR = pinocchio::log6(iMdR).toVector();
        errCompact.block<6, 1>(0, 0) = errL;
        errCompact.block<6, 1>(6, 0) = errR;
        if (errCompact.norm() < eps)
        {
            success = true;
            break;
        }
        if (itr_count >= IT_MAX)
        {
            success = false;
            break;
        }

        pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_l, JL);
        pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_r, JR);
        pinocchio::Data::Matrix6 JlogL;
        pinocchio::Data::Matrix6 JlogR;
        pinocchio::Jlog6(iMdL.inverse(), JlogL);
        pinocchio::Jlog6(iMdR.inverse(), JlogR);
        JL = -JlogL * JL;
        JR = -JlogR * JR;
        JCompact.block(0, 0, 6, model_biped_fixed.nv) = JL;
        JCompact.block(6, 0, 6, model_biped_fixed.nv) = JR;
        Eigen::Matrix<double, 12, 12> JJt;
        JJt.noalias() = JCompact * JCompact.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -JCompact.transpose() * JJt.ldlt().solve(errCompact);
        qIk = pinocchio::integrate(model_biped_fixed, qIk, v * DT);
    }

    IkRes res;
    res.err = errCompact;
    res.itr = itr_count;

    if (success)
        res.status = 0;
    else
        res.status = -1;

    res.jointPosRes = qIk;
    return res;
}

Pin_KinDyn_V4::IkRes
Pin_KinDyn_V4::computeRightHandPosIK(const Eigen::Vector3d &Pdes_R, const Eigen::VectorXd &qSeedFixed)
{
    Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_biped_fixed.nq);
    if (qSeedFixed.size() == model_biped_fixed.nq)
    {
        qIk = qSeedFixed;
    }
    else
    {
        qIk.block<5, 1>(17, 0) << -0.3, -1.4, 0.0, 1.4, 0.0;
    }

    for (int i = 0; i < model_biped_fixed.nq; ++i)
    {
        const double lower = model_biped_fixed.lowerPositionLimit(i);
        const double upper = model_biped_fixed.upperPositionLimit(i);
        if (std::isfinite(lower) && std::isfinite(upper) && lower < upper)
        {
            qIk(i) = std::clamp(qIk(i), lower, upper);
        }
    }

    constexpr double eps = 1e-4;
    constexpr int IT_MAX = 100;
    constexpr double DT = 0.75;
    constexpr double damp = 1e-4;
    Eigen::MatrixXd J(6, model_biped_fixed.nv);
    Eigen::MatrixXd Jpos(3, 5);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_biped_fixed.nv);
    Eigen::Vector3d err = Eigen::Vector3d::Zero();
    bool success = false;
    int itr_count = 0;

    for (itr_count = 0; itr_count <= IT_MAX; ++itr_count)
    {
        pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
        pinocchio::updateGlobalPlacements(model_biped_fixed, data_biped_fixed);
        const Eigen::Vector3d cur = rightWeldTcpPosition(data_biped_fixed.oMi[r_hand_joint_fixed]);
        err = Pdes_R - cur;
        if (err.norm() < eps)
        {
            success = true;
            break;
        }
        if (itr_count >= IT_MAX)
        {
            break;
        }

        pinocchio::computeJointJacobians(model_biped_fixed, data_biped_fixed, qIk);
        pinocchio::getJointJacobian(model_biped_fixed, data_biped_fixed,
                                    r_hand_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J);
        const Eigen::Vector3d tcpOffset_W =
            data_biped_fixed.oMi[r_hand_joint_fixed].rotation() * rightWeldTcpOffsetLocal();
        shiftSpatialJacobianToPoint(J, tcpOffset_W);
        Jpos = J.block(0, 17, 3, 5);
        Eigen::Matrix3d JJt = Jpos * Jpos.transpose();
        JJt.diagonal().array() += damp;
        Eigen::Matrix<double, 5, 1> dqArm = Jpos.transpose() * JJt.ldlt().solve(err);
        const double stepNorm = dqArm.norm();
        if (stepNorm > 0.20)
        {
            dqArm *= 0.20 / stepNorm;
        }
        v.setZero();
        v.segment<5>(17) = dqArm;
        qIk = pinocchio::integrate(model_biped_fixed, qIk, v * DT);
        for (int i = 17; i < 22; ++i)
        {
            const double lower = model_biped_fixed.lowerPositionLimit(i);
            const double upper = model_biped_fixed.upperPositionLimit(i);
            if (std::isfinite(lower) && std::isfinite(upper) && lower < upper)
            {
                qIk(i) = std::clamp(qIk(i), lower, upper);
            }
        }
    }

    pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
    pinocchio::updateGlobalPlacements(model_biped_fixed, data_biped_fixed);
    err = Pdes_R - rightWeldTcpPosition(data_biped_fixed.oMi[r_hand_joint_fixed]);

    IkRes res;
    res.status = success ? 0 : -1;
    res.itr = itr_count;
    res.err = err;
    res.jointPosRes = qIk;
    return res;
}

Eigen::Vector3d Pin_KinDyn_V4::computeRightHandPosFixed(const Eigen::VectorXd &qFixed)
{
    Eigen::VectorXd qEval = Eigen::VectorXd::Zero(model_biped_fixed.nq);
    if (qFixed.size() == model_biped_fixed.nq)
    {
        qEval = qFixed;
    }
    pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qEval);
    pinocchio::updateGlobalPlacements(model_biped_fixed, data_biped_fixed);
    return rightWeldTcpPosition(data_biped_fixed.oMi[r_hand_joint_fixed]);
}

std::array<Eigen::Vector3d, 5> Pin_KinDyn_V4::computeRightArmKeypointsFixed(const Eigen::VectorXd &qFixed)
{
    Eigen::VectorXd qEval = Eigen::VectorXd::Zero(model_biped_fixed.nq);
    if (qFixed.size() == model_biped_fixed.nq)
    {
        qEval = qFixed;
    }
    pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qEval);
    pinocchio::updateGlobalPlacements(model_biped_fixed, data_biped_fixed);

    const std::array<std::string, 4> jointNames = {
        "right_shoulder_pitch_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint"};
    std::array<Eigen::Vector3d, 5> keypoints{};
    for (int i = 0; i < static_cast<int>(jointNames.size()); ++i)
    {
        const pinocchio::JointIndex jointId = model_biped_fixed.getJointId(jointNames[i]);
        keypoints[i] = (jointId < model_biped_fixed.njoints)
                           ? data_biped_fixed.oMi[jointId].translation()
                           : Eigen::Vector3d::Zero();
    }
    keypoints[4] = rightWeldTcpPosition(data_biped_fixed.oMi[r_hand_joint_fixed]);
    return keypoints;
}

Eigen::Vector3d Pin_KinDyn_V4::computeFixedCoM(const Eigen::VectorXd &qFixed)
{
    Eigen::VectorXd qEval = Eigen::VectorXd::Zero(model_biped_fixed.nq);
    if (qFixed.size() == model_biped_fixed.nq)
    {
        qEval = qFixed;
    }
    pinocchio::centerOfMass(model_biped_fixed, data_biped_fixed, qEval);
    return data_biped_fixed.com[0];
}

void Pin_KinDyn_V4::workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT)
{
    for (int i = 0; i < (int)motorName.size(); i++)
        if (qFT(i + 7) > motorMaxPos(i))
        {
            qFT(i + 7) = motorMaxPos(i);
            motorReachLimit[i] = true;
            tauJointFT(i) = tauJointOld(i);
        }
        else if (qFT(i + 7) < motorMinPos(i))
        {
            qFT(i + 7) = motorMinPos(i);
            motorReachLimit[i] = true;
            tauJointFT(i) = tauJointOld(i);
        }
        else
            motorReachLimit[i] = false;

    tauJointOld = tauJointFT;
}
