/*
 * Fixed-base 6-DoF kinematics helper for the speedbot_v4_weld right-arm demo.
 */
#include "pino_kin_dyn_v4_weld.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
void requireFrame(const pinocchio::Model &model, const std::string &name)
{
    if (!model.existFrame(name))
    {
        throw std::runtime_error("[Pin_KinDyn_V4_Weld] required frame not found: " + name);
    }
}
} // namespace

Pin_KinDyn_V4_Weld::Pin_KinDyn_V4_Weld(std::string urdf_pathIn)
{
    pinocchio::Model full_model;
    pinocchio::urdf::buildModel(urdf_pathIn, full_model);

    std::vector<pinocchio::JointIndex> joints_to_lock;
    const std::vector<std::string> keepJoints = {
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint", "welding_gun_joint"};
    for (const auto &jointName : full_model.names)
    {
        if (jointName == "universe")
        {
            continue;
        }
        if (std::find(keepJoints.begin(), keepJoints.end(), jointName) != keepJoints.end())
        {
            continue;
        }
        const pinocchio::JointIndex jid = full_model.getJointId(jointName);
        if (jid > 0 && jid < full_model.njoints)
        {
            joints_to_lock.push_back(jid);
        }
    }
    Eigen::VectorXd ref = Eigen::VectorXd::Zero(full_model.nq);
    pinocchio::buildReducedModel(full_model, joints_to_lock, ref, model_biped_fixed);
    requireFrame(model_biped_fixed, "welding_tcp");
    welding_tcp_frame_fixed = model_biped_fixed.getFrameId("welding_tcp");
    data_biped_fixed = pinocchio::Data(model_biped_fixed);

    model_nv = static_cast<int>(model_biped_fixed.nv);
    motorMaxPos = Eigen::VectorXd::Zero(motorName.size());
    motorMinPos = Eigen::VectorXd::Zero(motorName.size());
    motorReachLimit.assign(motorName.size(), false);

    Json::Reader reader;
    Json::Value root_read;
    std::ifstream in;
    const std::vector<std::string> jointConfigCandidates = {
        "../common/joint_ctrl_config_v4_weld.json",
        "common/joint_ctrl_config_v4_weld.json",
        "joint_ctrl_config_v4_weld.json"};
    for (const auto &path : jointConfigCandidates)
    {
        in.open(path, std::ios::binary);
        if (in.good())
        {
            if (!reader.parse(in, root_read))
            {
                std::cerr << "[Pin_KinDyn_V4_Weld] failed to parse " << path << std::endl;
            }
            break;
        }
        in.close();
    }
    for (int i = 0; i < static_cast<int>(motorName.size()); ++i)
    {
        motorMaxPos(i) = root_read[motorName[i]]["maxPos"].asDouble();
        motorMinPos(i) = root_read[motorName[i]]["minPos"].asDouble();
    }
}

Eigen::VectorXd Pin_KinDyn_V4_Weld::clampRightArmQ(const Eigen::VectorXd &rightArmQ) const
{
    Eigen::VectorXd qClamp = rightArmQ;
    for (int i = 0; i < qClamp.size(); ++i)
    {
        const double lower = motorMinPos(i);
        const double upper = motorMaxPos(i);
        if (std::isfinite(lower) && std::isfinite(upper) && lower < upper)
        {
            qClamp(i) = std::clamp(qClamp(i), lower, upper);
        }
    }
    return qClamp;
}

pinocchio::SE3 Pin_KinDyn_V4_Weld::computeRightHandPoseFixed(const Eigen::VectorXd &rightArmQ)
{
    Eigen::VectorXd qFull = clampRightArmQ(rightArmQ);
    pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qFull);
    pinocchio::updateFramePlacements(model_biped_fixed, data_biped_fixed);
    return data_biped_fixed.oMf[welding_tcp_frame_fixed];
}

Eigen::Vector3d Pin_KinDyn_V4_Weld::computeRightHandPosFixed(const Eigen::VectorXd &rightArmQ)
{
    return computeRightHandPoseFixed(rightArmQ).translation();
}

Pin_KinDyn_V4_Weld::IkRes Pin_KinDyn_V4_Weld::computeRightHandPoseIK(const Eigen::Matrix3d &Rdes_R,
                                                                      const Eigen::Vector3d &Pdes_R,
                                                                      const Eigen::VectorXd &qSeedFixed)
{
    Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_biped_fixed.nq);
    if (qSeedFixed.size() == 6)
    {
        qIk = clampRightArmQ(qSeedFixed);
    }
    else
    {
        qIk << -0.2, -1.0, 0.1, 1.1, 0.0, 0.0;
    }

    constexpr double eps = 1e-4;
    constexpr int IT_MAX = 120;
    constexpr double DT = 0.5;
    constexpr double damp = 1e-3;
    Eigen::VectorXd errVec = Eigen::VectorXd::Zero(6);
    for (int itr = 0; itr <= IT_MAX; ++itr)
    {
        pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
        pinocchio::updateFramePlacements(model_biped_fixed, data_biped_fixed);
        const pinocchio::SE3 cur = data_biped_fixed.oMf[welding_tcp_frame_fixed];
        errVec.head<3>() = Pdes_R - cur.translation();
        errVec.tail<3>() = 0.5 * (cur.rotation().col(0).cross(Rdes_R.col(0)) +
                                  cur.rotation().col(1).cross(Rdes_R.col(1)) +
                                  cur.rotation().col(2).cross(Rdes_R.col(2)));
        if (errVec.norm() < eps)
        {
            return {0, itr, errVec, qIk};
        }
        pinocchio::computeJointJacobians(model_biped_fixed, data_biped_fixed, qIk);
        Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
        pinocchio::getFrameJacobian(model_biped_fixed, data_biped_fixed, welding_tcp_frame_fixed,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);
        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        JJt.diagonal().array() += damp;
        Eigen::Matrix<double, 6, 1> dqArm = J.transpose() * JJt.ldlt().solve(errVec);
        const double stepNorm = dqArm.norm();
        if (stepNorm > 0.25)
        {
            dqArm *= 0.25 / stepNorm;
        }
        qIk = pinocchio::integrate(model_biped_fixed, qIk, dqArm * DT);
        qIk = clampRightArmQ(qIk);
    }

    pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
    pinocchio::updateFramePlacements(model_biped_fixed, data_biped_fixed);
    const pinocchio::SE3 cur = data_biped_fixed.oMf[welding_tcp_frame_fixed];
    errVec.head<3>() = Pdes_R - cur.translation();
    errVec.tail<3>() = 0.5 * (cur.rotation().col(0).cross(Rdes_R.col(0)) +
                              cur.rotation().col(1).cross(Rdes_R.col(1)) +
                              cur.rotation().col(2).cross(Rdes_R.col(2)));
    return {-1, IT_MAX, errVec, qIk};
}
