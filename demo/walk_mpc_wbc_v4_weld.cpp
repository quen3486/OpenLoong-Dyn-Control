/*
 * walk_mpc_wbc_v4_weld: right-arm-only Cartesian weld demo for speedbot_v4_weld.
 */
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "MJ_interface_v4_weld.h"
#include "controller_config.h"
#include "data_logger.h"
#include "pino_kin_dyn_v4_weld.h"
#include "weld_trajectory.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>

char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/scene_v4_weld_arm_only.xml", 0, error, 1000);
mjData *mj_data = mj_makeData(mj_model);

namespace
{
constexpr int kRightArmDoF = 6;

bool readBoolEnv(const char *name, bool fallback)
{
    const char *env = std::getenv(name);
    if (env == nullptr)
    {
        return fallback;
    }
    std::string v(env);
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (v == "1" || v == "true" || v == "yes" || v == "on")
    {
        return true;
    }
    if (v == "0" || v == "false" || v == "no" || v == "off")
    {
        return false;
    }
    return fallback;
}

Eigen::VectorXd makeRightArmQ(const ControllerConfig &cfg, bool useStart)
{
    Eigen::VectorXd q = Eigen::VectorXd::Zero(kRightArmDoF);
    const auto &src = useStart ? cfg.weldRightArmStartQ : cfg.weldRightArmEndQ;
    for (int i = 0; i < kRightArmDoF; ++i)
    {
        q(i) = src[i];
    }
    return q;
}

Eigen::VectorXd vectorToEigen(const std::vector<double> &v, int expectedSize)
{
    Eigen::VectorXd out = Eigen::VectorXd::Zero(expectedSize);
    const int n = std::min<int>(expectedSize, static_cast<int>(v.size()));
    for (int i = 0; i < n; ++i)
    {
        out(i) = v[static_cast<size_t>(i)];
    }
    return out;
}

std::vector<double> eigenToStd(const Eigen::VectorXd &v)
{
    return std::vector<double>(v.data(), v.data() + v.size());
}

void writeTrajectoryCsvHeader(std::ofstream &ofs)
{
    ofs << "q_des_0,q_des_1,q_des_2,q_des_3,q_des_4,q_des_5\n";
}

void writeTrajectoryCsvRow(std::ofstream &ofs,
                           const Eigen::VectorXd &qDes)
{
    ofs << std::fixed << std::setprecision(9);
    for (int i = 0; i < qDes.size(); ++i)
    {
        if (i > 0)
        {
            ofs << ",";
        }
        ofs << qDes(i);
    }
    ofs << "\n";
}

void writeFixedQpos(mjModel *model, mjData *data, const Eigen::VectorXd &qRight)
{
    for (int i = 0; i < model->nq; ++i)
    {
        data->qpos[i] = (i < qRight.size()) ? qRight(i) : 0.0;
    }
    for (int i = 0; i < model->nv; ++i)
    {
        data->qvel[i] = 0.0;
    }
    mj_forward(model, data);
}

Eigen::Vector3d readMujocoSitePosAtQ(mjModel *model,
                                     mjData *data,
                                     int siteId,
                                     const Eigen::VectorXd &qRight)
{
    writeFixedQpos(model, data, qRight);
    return Eigen::Vector3d(data->site_xpos[3 * siteId + 0],
                           data->site_xpos[3 * siteId + 1],
                           data->site_xpos[3 * siteId + 2]);
}
} // namespace

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UIctr uiController(mj_model, mj_data);
    MJ_Interface_V4_Weld mj_interface(mj_model, mj_data);
    Pin_KinDyn_V4_Weld kinDynSolver("../models/speedbot_v4/speedbot_v4_with_right_welding_rotate_joint.urdf");
    ControllerConfig controllerConfig;
    std::string cfgErr;
    if (!loadControllerConfig("../common/controller_config_v4_weld.json", controllerConfig, &cfgErr))
    {
        std::cerr << "[ControllerConfig] fallback to defaults: " << cfgErr << std::endl;
    }

    const double simDt = mj_model->opt.timestep;
    const int mainCtrlDecimation = std::max(1, static_cast<int>(std::lround(controllerConfig.mainControlDt / simDt)));
    const double mainCtrlDt = simDt * mainCtrlDecimation;
    std::cout << "[LoopRate] sim=" << 1.0 / simDt << " Hz, main=" << 1.0 / mainCtrlDt << " Hz" << std::endl;

    const int weldingTcpSiteId = mj_name2id(mj_model, mjOBJ_SITE, "welding_tcp");
    if (weldingTcpSiteId < 0)
    {
        std::cerr << "[Init] MuJoCo site welding_tcp not found" << std::endl;
        return 1;
    }

    DataLogger logger("../record/datalog.log");
    std::ofstream trajCsv("../record/weld_arm_trajectory_1ms.csv", std::ios::out | std::ios::trunc);
    if (!trajCsv.is_open())
    {
        std::cerr << "[Record] failed to open ../record/weld_arm_trajectory_1ms.csv" << std::endl;
        return 1;
    }
    writeTrajectoryCsvHeader(trajCsv);
    trajCsv.flush();
    bool hasWeldSamples = false;

    const Eigen::VectorXd qStart = makeRightArmQ(controllerConfig, true);
    const Eigen::VectorXd qEnd = makeRightArmQ(controllerConfig, false);
    const pinocchio::SE3 tcpStart = kinDynSolver.computeRightHandPoseFixed(qStart);
    const pinocchio::SE3 tcpEnd = kinDynSolver.computeRightHandPoseFixed(qEnd);
    const Eigen::Quaterniond tcpQuatStart(tcpStart.rotation());
    const Eigen::Quaterniond tcpQuatEnd(tcpEnd.rotation());
    WeldTrajectory::Pose startPose;
    startPose.pos = tcpStart.translation();
    startPose.quat = tcpQuatStart.normalized();
    WeldTrajectory::Pose endPose;
    endPose.pos = tcpEnd.translation();
    endPose.quat = tcpQuatEnd.normalized();

    auto startIk = kinDynSolver.computeRightHandPoseIK(startPose.quat.toRotationMatrix(), startPose.pos, qStart);
    if (startIk.status != 0)
    {
        std::cerr << "[Init] start pose IK failed, err=" << startIk.err.norm() << std::endl;
        return 1;
    }
    auto endIk = kinDynSolver.computeRightHandPoseIK(endPose.quat.toRotationMatrix(), endPose.pos, qEnd);
    if (endIk.status != 0)
    {
        std::cerr << "[Init] end pose IK failed, err=" << endIk.err.norm()
                  << ". Please adjust weld_right_arm_end_q." << std::endl;
        return 1;
    }
    const Eigen::Vector3d tcpDelta = endPose.pos - startPose.pos;
    const double tcpDistance = tcpDelta.norm();
    const double tcpSpeed = std::max(controllerConfig.weldRightArmSpeed, 1e-3);

    WeldTrajectory trajectory;
    if (!trajectory.setSingleSegment(startPose, endPose, tcpSpeed, &cfgErr))
    {
        std::cerr << "[Trajectory] failed: " << cfgErr << std::endl;
        return 1;
    }
    const double duration = std::max(trajectory.totalDuration(), 1e-3);
    const Eigen::Vector3d tcpLineStartMj = readMujocoSitePosAtQ(mj_model, mj_data, weldingTcpSiteId, qStart);
    const Eigen::Vector3d tcpLineEndMj = readMujocoSitePosAtQ(mj_model, mj_data, weldingTcpSiteId, qEnd);

    std::cout << "[RightArm] start_q=" << qStart.transpose() << std::endl;
    std::cout << "[RightArm] end_q=" << qEnd.transpose() << std::endl;
    std::cout << "[RightArm] tcp_start=" << startPose.pos.transpose()
              << " tcp_end=" << endPose.pos.transpose()
              << " distance=" << tcpDistance
              << " speed=" << tcpSpeed
              << " duration=" << duration
              << " segments=" << trajectory.segmentCount() << std::endl;
    std::cout << "[Viz] mujoco_tcp_line_start=" << tcpLineStartMj.transpose()
              << " mujoco_tcp_line_end=" << tcpLineEndMj.transpose() << std::endl;

    const bool headless = readBoolEnv("HEADLESS", false);
    if (!headless)
    {
        uiController.iniGLFW();
        uiController.createWindow("speedbot_v4_weld", false);
        uiController.setTcpTrajectoryLine(tcpLineStartMj.data(), tcpLineEndMj.data());
    }

    Eigen::VectorXd startArmQ = startIk.jointPosRes;
    writeFixedQpos(mj_model, mj_data, startArmQ);
    mj_interface.updateSensorValues();

    logger.addIterm("sim_time", 1);
    logger.addIterm("tcp_start", 3);
    logger.addIterm("tcp_end", 3);
    logger.addIterm("tcp_pos_cur", 3);
    logger.addIterm("tcp_pos_des", 3);
    logger.addIterm("tcp_pos_err", 3);
    logger.addIterm("tcp_distance", 1);
    logger.addIterm("tcp_speed", 1);
    logger.addIterm("tcp_duration", 1);
    logger.addIterm("phase", 1);
    logger.addIterm("right_arm_q_cur", kRightArmDoF);
    logger.addIterm("right_arm_q_des", kRightArmDoF);
    logger.addIterm("right_arm_q_vel", kRightArmDoF);
    logger.addIterm("right_arm_track_err_max_abs", 1);
    logger.addIterm("right_arm_finished", 1);
    logger.finishItermAdding();

    bool trajectoryRunning = false;
    bool trajectoryPaused = false;
    double phase = 0.0;
    double maxTrackErr = 0.0;
    Eigen::VectorXd qCmd = startArmQ;
    Eigen::VectorXd qCur = qCmd;
    Eigen::VectorXd qSeed = startIk.jointPosRes;
    bool endSelectionChecked = false;

    std::cout << "[Key] G=start/restart, Space=pause/resume, R=return start" << std::endl;

    while (!(!headless && glfwWindowShouldClose(uiController.window)))
    {
        mj_interface.updateSensorValues();
        const UIctr::ButtonState buttonState = headless ? UIctr::ButtonState{} : uiController.getButtonState();
        if (buttonState.key_g)
        {
            trajectoryRunning = true;
            trajectoryPaused = false;
            phase = 0.0;
            maxTrackErr = 0.0;
            qSeed = startArmQ;
            endSelectionChecked = false;
            std::cout << "[Key] G: start trajectory" << std::endl;
        }
        if (buttonState.key_space)
        {
            trajectoryPaused = !trajectoryPaused;
            std::cout << "[Key] Space: " << (trajectoryPaused ? "pause" : "resume") << std::endl;
        }
        if (buttonState.key_r)
        {
            trajectoryRunning = true;
            trajectoryPaused = false;
            phase = 0.0;
            qSeed = startArmQ;
            endSelectionChecked = false;
            std::cout << "[Key] R: reset to start" << std::endl;
        }

        if (trajectoryRunning && !trajectoryPaused)
        {
            phase = std::min(phase + mainCtrlDt / duration, 1.0);
        }

        const WeldTrajectory::Sample sample = trajectory.sample(phase * duration);
        qCur = vectorToEigen(mj_interface.motor_pos, kRightArmDoF);
        const Eigen::VectorXd qVelCur = vectorToEigen(mj_interface.motor_vel, kRightArmDoF);
        auto ikRes = kinDynSolver.computeRightHandPoseIK(sample.pose.quat.toRotationMatrix(), sample.pose.pos, qSeed);
        if (ikRes.status != 0)
        {
            std::cerr << "[IK] failed at phase=" << phase << ", err=" << ikRes.err.norm() << std::endl;
            trajectoryRunning = false;
        }
        qCmd = ikRes.status == 0 ? ikRes.jointPosRes : qCmd;
        if (phase >= 1.0 && ikRes.status == 0 && !endSelectionChecked)
        {
            const double endErr = (ikRes.jointPosRes - qEnd).cwiseAbs().maxCoeff();
            if (endErr > 1e-4)
            {
                std::cerr << "[IK] end solution mismatch: max |q_ik - weld_right_arm_end_q|="
                          << endErr << ". Endpoint pose is correct, but IK selected a different joint solution."
                          << std::endl;
                trajectoryRunning = false;
            }
            endSelectionChecked = true;
        }
        if (ikRes.status == 0)
        {
            qSeed = ikRes.jointPosRes;
        }
        const double trackErr = (qCmd - qCur).cwiseAbs().maxCoeff();
        maxTrackErr = std::max(maxTrackErr, trackErr);

        std::vector<double> motorsPosDes = eigenToStd(qCmd);
        const Eigen::Vector3d tcpPosCur = kinDynSolver.computeRightHandPosFixed(qCur);
        const Eigen::Vector3d tcpPosDes = sample.pose.pos;
        const Eigen::Vector3d tcpPosErr = tcpPosDes - tcpPosCur;
        mj_interface.setMotorsPos(motorsPosDes);

        logger.startNewLine();
        logger.recItermData("sim_time", mj_data->time);
        logger.recItermData("tcp_start", startPose.pos);
        logger.recItermData("tcp_end", endPose.pos);
        logger.recItermData("tcp_pos_cur", tcpPosCur);
        logger.recItermData("tcp_pos_des", tcpPosDes);
        logger.recItermData("tcp_pos_err", tcpPosErr);
        logger.recItermData("tcp_distance", tcpDistance);
        logger.recItermData("tcp_speed", tcpSpeed);
        logger.recItermData("tcp_duration", duration);
        logger.recItermData("phase", phase);
        logger.recItermData("right_arm_q_cur", qCur);
        logger.recItermData("right_arm_q_des", qCmd);
        logger.recItermData("right_arm_q_vel", qVelCur);
        logger.recItermData("right_arm_track_err_max_abs", maxTrackErr);
        logger.recItermData("right_arm_finished", (phase >= 1.0) ? 1.0 : 0.0);
        logger.finishLine();
        if (trajectoryRunning && !trajectoryPaused)
        {
            writeTrajectoryCsvRow(trajCsv, qCmd);
            trajCsv.flush();
            hasWeldSamples = true;
        }

        if (trajectoryRunning && phase >= 1.0 && !trajectoryPaused)
        {
            trajectoryRunning = false;
            std::cout << "[Done] trajectory finished, max track err=" << maxTrackErr << std::endl;
        }
        mj_step(mj_model, mj_data);
        if (!headless)
        {
            uiController.updateScene();
        }
    }

    if (!headless)
    {
        uiController.Close();
    }
    trajCsv.flush();
    trajCsv.close();
    if (!hasWeldSamples)
    {
        std::cout << "[Record] no welding segment samples captured (press G to start welding)." << std::endl;
    }
    return 0;
}
