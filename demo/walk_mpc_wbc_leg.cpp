/*
 * walk_mpc_wbc_leg:
 * - "mujoco" backend: MPC + WBC walking demo in MuJoCo.
 * - "ros2_real" backend: MPC + WBC real-robot control via ROS2 topics.
 *
 * Direct execution runs MuJoCo. Use --ros2-real for the real-robot backend.
 */
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "MJ_interface_v4_leg.h"
#include "ROS2_interface_v4_leg.h"
#include "ROS2_state_pub_v4_leg.h"
#include "PVT_ctrl_v4_leg.h"
#include "data_logger.h"
#include "data_bus.h"
#include "pino_kin_dyn_v4_leg.h"
#include "useful_math.h"
#include "wbc_priority_v4_leg.h"
#include "mpc.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include "controller_config.h"
#include "json/json.h"
#include <string>
#include <iostream>
#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "StateEst.h"

namespace
{

const char *getEnvEither(const char *keyPrimary, const char *keyCompat = nullptr)
{
    if (keyPrimary != nullptr)
    {
        if (const char *v = std::getenv(keyPrimary); v != nullptr)
        {
            return v;
        }
    }
    if (keyCompat != nullptr)
    {
        return std::getenv(keyCompat);
    }
    return nullptr;
}

std::string toLowerCopy(std::string in)
{
    for (char &ch : in)
    {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return in;
}

bool parseBoolEnv(const char *envValue, bool &outValue)
{
    if (envValue == nullptr)
    {
        return false;
    }
    std::string v = toLowerCopy(std::string(envValue));
    if (v == "1" || v == "true" || v == "yes" || v == "on")
    {
        outValue = true;
        return true;
    }
    if (v == "0" || v == "false" || v == "no" || v == "off")
    {
        outValue = false;
        return true;
    }
    return false;
}

bool parseDoubleEnv(const char *envValue, double &outValue)
{
    if (envValue == nullptr)
    {
        return false;
    }
    char *endPtr = nullptr;
    const double v = std::strtod(envValue, &endPtr);
    if (endPtr == envValue)
    {
        return false;
    }
    outValue = v;
    return true;
}

std::string getJointCtrlConfigV4LegPath()
{
    if (const char *env = getEnvEither("OPENLOONG_JOINT_CTRL_CONFIG", "JOINT_CTRL_CONFIG"); env != nullptr && std::string(env).size() > 0)
    {
        return std::string(env);
    }
    return "../common/joint_ctrl_config_v4_leg.json";
}

bool hasArg(int argc, char **argv, const char *target)
{
    if (target == nullptr)
    {
        return false;
    }
    for (int i = 1; i < argc; i++)
    {
        if (argv[i] != nullptr && std::string(argv[i]) == target)
        {
            return true;
        }
    }
    return false;
}

struct MujocoRegressionScript
{
    bool enabled{false};
    double tCloseLoop{5.0};
    double tWalkStart{10.0};
    double tPressW{10.2};
    double tPressJ{26.0};
    double simEndTime{30.0};

    bool sentF{false};
    bool sentSpace{false};
    bool sentW{false};
    bool sentJ{false};

    void inject(double simTime, UIctr::ButtonState &button)
    {
        if (!enabled)
        {
            return;
        }

        if (!sentF && simTime >= tCloseLoop)
        {
            button.key_f = true;
            sentF = true;
            std::cout << "[AutoRegression] press F at t=" << simTime << " s" << std::endl;
        }
        if (!sentSpace && simTime >= tWalkStart)
        {
            button.key_space = true;
            sentSpace = true;
            std::cout << "[AutoRegression] press Space at t=" << simTime << " s" << std::endl;
        }
        if (!sentW && simTime >= tPressW)
        {
            button.key_w = true;
            sentW = true;
            std::cout << "[AutoRegression] press W at t=" << simTime << " s" << std::endl;
        }
        if (!sentJ && simTime >= tPressJ)
        {
            button.key_j = true;
            sentJ = true;
            std::cout << "[AutoRegression] press J at t=" << simTime << " s" << std::endl;
        }
    }
};

MujocoRegressionScript loadMujocoRegressionScriptFromEnv()
{
    MujocoRegressionScript script;
    parseBoolEnv(getEnvEither("MUJOCO_REGRESSION_SCRIPT", "OPENLOONG_MUJOCO_REGRESSION_SCRIPT"), script.enabled);

    double tmp = 0.0;
    if (parseDoubleEnv(getEnvEither("MUJOCO_REGRESSION_SIM_END", "OPENLOONG_MUJOCO_REGRESSION_SIM_END"), tmp))
    {
        script.simEndTime = std::clamp(tmp, 10.0, 300.0);
    }
    if (parseDoubleEnv(getEnvEither("MUJOCO_REGRESSION_CLOSE_LOOP_T", "OPENLOONG_MUJOCO_REGRESSION_CLOSE_LOOP_T"), tmp))
    {
        script.tCloseLoop = std::clamp(tmp, 0.5, script.simEndTime - 5.0);
    }
    if (parseDoubleEnv(getEnvEither("MUJOCO_REGRESSION_WALK_START_T", "OPENLOONG_MUJOCO_REGRESSION_WALK_START_T"), tmp))
    {
        script.tWalkStart = std::clamp(tmp, script.tCloseLoop + 0.5, script.simEndTime - 3.0);
    }
    if (parseDoubleEnv(getEnvEither("MUJOCO_REGRESSION_STOP_T", "OPENLOONG_MUJOCO_REGRESSION_STOP_T"), tmp))
    {
        script.tPressJ = std::clamp(tmp, script.tWalkStart + 6.0, script.simEndTime - 0.5);
    }

    const double walkBase = script.tWalkStart;
    script.tPressW = walkBase + 0.2;

    return script;
}

void configureCommonLegModules(const ControllerConfig &controllerConfig,
                               GaitScheduler &gaitScheduler,
                               FootPlacement &footPlacement,
                               WBC_priority_V4_Leg &WBC_solv,
                               DataBus &RobotState,
                               double stand_legLength)
{
    gaitScheduler.tSwing = controllerConfig.tSwing;
    gaitScheduler.phiSwitchMin = controllerConfig.phiSwitchMin;
    gaitScheduler.phiSwitchAutoDesign = controllerConfig.phiSwitchAutoDesign;
    gaitScheduler.phiSwitchDesignRefTSwing = controllerConfig.phiSwitchDesignRefTSwing;
    gaitScheduler.phiSwitchDesignRef = controllerConfig.phiSwitchDesignRef;
    gaitScheduler.phiSwitchDesignPower = controllerConfig.phiSwitchDesignPower;
    gaitScheduler.phiSwitchDesignMin = controllerConfig.phiSwitchDesignMin;
    gaitScheduler.phiSwitchDesignMax = controllerConfig.phiSwitchDesignMax;
    gaitScheduler.fzSwitchThreshold = controllerConfig.fzSwitchThreshold;
    gaitScheduler.fzStopThreshold = controllerConfig.fzStopThreshold;
    gaitScheduler.contactConfirmTimeSec = controllerConfig.contactConfirmTimeSec;

    RobotState.width_hips = 0.245;
    footPlacement.kp_vx = controllerConfig.kpVx;
    footPlacement.kp_vy = controllerConfig.kpVy;
    footPlacement.kp_wz = controllerConfig.kpWz;
    footPlacement.stepHeight = controllerConfig.stepHeight;
    footPlacement.legLength = stand_legLength;
    footPlacement.xOffsetL = controllerConfig.xOffsetL;
    footPlacement.yOffsetL = controllerConfig.yOffsetL;
    footPlacement.zOffsetW = controllerConfig.zOffsetW;
    footPlacement.swingTrajectoryPhase = controllerConfig.swingTrajectoryPhase;
    footPlacement.swingTrajectoryWindow = controllerConfig.swingTrajectoryWindow;
    footPlacement.zStretchStartPhi = controllerConfig.zStretchStartPhi;
    footPlacement.zStretchStep = controllerConfig.zStretchStep;
    footPlacement.zStretchMin = controllerConfig.zStretchMin;

    WBC_solv.cfg_pos_err_clamp_xy = controllerConfig.posErrClampXY;
    WBC_solv.cfg_pos_err_clamp_z = controllerConfig.posErrClampZ;
    WBC_solv.cfg_posrot_kp = controllerConfig.posRotKp;
    WBC_solv.cfg_posrot_kd = controllerConfig.posRotKd;
    WBC_solv.cfg_posrot_kp_x = controllerConfig.posRotKpX;
    WBC_solv.cfg_posrot_kp_pitch = controllerConfig.posRotKpPitch;
    WBC_solv.cfg_posrot_kd_pitch = controllerConfig.posRotKdPitch;
    WBC_solv.cfg_swing_kp = controllerConfig.swingLegKp;
    WBC_solv.cfg_swing_kd = controllerConfig.swingLegKd;
}

void configureMpcFromControllerConfig(const ControllerConfig &controllerConfig,
                                      MPC &MPC_solv,
                                      WBC_priority_V4_Leg &WBC_solv)
{
    Eigen::Matrix3d mpcInertiaCfg;
    mpcInertiaCfg << controllerConfig.mpcInertiaXx, controllerConfig.mpcInertiaXy, controllerConfig.mpcInertiaXz,
                     controllerConfig.mpcInertiaXy, controllerConfig.mpcInertiaYy, controllerConfig.mpcInertiaYz,
                     controllerConfig.mpcInertiaXz, controllerConfig.mpcInertiaYz, controllerConfig.mpcInertiaZz;
    MPC_solv.setRobotMass(controllerConfig.mpcMass);
    MPC_solv.setFrictionCoeff(controllerConfig.contactMiu);
    if (!MPC_solv.setBodyInertia(mpcInertiaCfg))
    {
        std::cerr << "[MPC] invalid inertia config, keeping previous inertia matrix." << std::endl;
    }
    MPC_solv.setUseDataBusInertia(controllerConfig.mpcUseDataBusInertia);
    MPC_solv.setFootSupportPolygon(controllerConfig.mpcDeltaFootFront, controllerConfig.mpcDeltaFootRear,
                                   controllerConfig.mpcDeltaFootLeft, controllerConfig.mpcDeltaFootRight);
    MPC_solv.setWrenchLimits(controllerConfig.mpcForceMaxXY, controllerConfig.mpcFzMaxScale,
                             controllerConfig.mpcTorqueMaxX, controllerConfig.mpcTorqueMaxY, controllerConfig.mpcTorqueMaxZ);
    MPC_solv.setHorizon(controllerConfig.mpcPredictionHorizon, controllerConfig.mpcControlHorizon);
    WBC_solv.setContactMiu(controllerConfig.contactMiu);
}

double wrapAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

struct EstimatorTruthLog
{
    double truth_base_pos_w[3]{0.0, 0.0, 0.0};
    double truth_base_vel_w[3]{0.0, 0.0, 0.0};
    double truth_rpy_w[3]{0.0, 0.0, 0.0};
    double truth_in_est_frame[3]{0.0, 0.0, 0.0};
    double est_err_pos[3]{0.0, 0.0, 0.0};
    double est_err_vel[3]{0.0, 0.0, 0.0};
    double est_err_yaw{0.0};
    double mujoco_touch_force[2]{0.0, 0.0};

    bool refReady{false};
    double p0_truth[3]{0.0, 0.0, 0.0};
    double yaw0_truth{0.0};
    double est_pos0[3]{0.0, 0.0, 0.0};

    void setReference(const double pTruthW[3], double yawTruth, const Eigen::Vector3d &estPos0)
    {
        for (int i = 0; i < 3; i++)
        {
            p0_truth[i] = pTruthW[i];
            est_pos0[i] = estPos0(i);
        }
        yaw0_truth = yawTruth;
        refReady = true;
    }

    void setMujocoTouchForces(double left, double right)
    {
        mujoco_touch_force[0] = left;
        mujoco_touch_force[1] = right;
    }

    void update(const DataBus &robotState,
                const double pTruthW[3],
                const double vTruthW[3],
                const double rpyTruthW[3])
    {
        for (int i = 0; i < 3; i++)
        {
            truth_base_pos_w[i] = pTruthW[i];
            truth_base_vel_w[i] = vTruthW[i];
            truth_rpy_w[i] = rpyTruthW[i];
        }

        if (!refReady)
        {
            return;
        }

        const double c = std::cos(-yaw0_truth);
        const double s = std::sin(-yaw0_truth);
        const double dpx = pTruthW[0] - p0_truth[0];
        const double dpy = pTruthW[1] - p0_truth[1];
        const double dpz = pTruthW[2] - p0_truth[2];
        truth_in_est_frame[0] = c * dpx - s * dpy;
        truth_in_est_frame[1] = s * dpx + c * dpy;
        truth_in_est_frame[2] = dpz;

        double vTruthInEst[3]{0.0, 0.0, 0.0};
        vTruthInEst[0] = c * vTruthW[0] - s * vTruthW[1];
        vTruthInEst[1] = s * vTruthW[0] + c * vTruthW[1];
        vTruthInEst[2] = vTruthW[2];

        for (int i = 0; i < 3; i++)
        {
            const double estPosRel = robotState.base_pos_est(i) - est_pos0[i];
            est_err_pos[i] = estPosRel - truth_in_est_frame[i];
            est_err_vel[i] = robotState.base_vel_est(i) - vTruthInEst[i];
        }

        const double truthYawInEst = wrapAngle(rpyTruthW[2] - yaw0_truth);
        est_err_yaw = wrapAngle(robotState.base_rpy(2) - truthYawInEst);
    }
};

void addCommonLoggerItems(DataLogger &logger, int robot_nv)
{
    logger.addIterm("dyn_time", 1);
    logger.addIterm("motors_pos_cur", robot_nv - 6);
    logger.addIterm("motors_pos_des", robot_nv - 6);
    logger.addIterm("motors_tor_cur", robot_nv - 6);
    logger.addIterm("motors_tor_des", robot_nv - 6);
    logger.addIterm("motors_tor_out", robot_nv - 6);
    logger.addIterm("motors_vel_des", robot_nv - 6);
    logger.addIterm("motors_vel_cur", robot_nv - 6);
    logger.addIterm("FL_est", 3);
    logger.addIterm("FR_est", 3);
    logger.addIterm("wbc_FrRes", 12);
    logger.addIterm("Fr_ff", 12);
    logger.addIterm("base_pos_des", 3);
    logger.addIterm("base_pos", 3);
    logger.addIterm("base_pos_est", 3);
    logger.addIterm("baseLinVel", 3);
    logger.addIterm("base_vel_est", 3);
    logger.addIterm("base_rpy", 3);
    logger.addIterm("base_omega_W", 3);
    logger.addIterm("eul_est", 3);
    logger.addIterm("truth_base_pos_w", 3);
    logger.addIterm("truth_base_vel_w", 3);
    logger.addIterm("truth_rpy_w", 3);
    logger.addIterm("mujoco_touch_force", 2);
    logger.addIterm("truth_in_est_frame", 3);
    logger.addIterm("est_err_pos", 3);
    logger.addIterm("est_err_vel", 3);
    logger.addIterm("est_err_yaw", 1);
    logger.addIterm("Ufe", 13);
    logger.addIterm("phi", 1);
    logger.addIterm("phiSwitchMinRuntime", 1);
    logger.addIterm("tSwing", 1);
    logger.addIterm("mainControlDt", 1);
    logger.addIterm("mpcControlDt", 1);
    logger.addIterm("mpcPredictionHorizon", 1);
    logger.addIterm("mpcControlHorizon", 1);
    logger.addIterm("js_vel_des", 3);
    logger.addIterm("js_omega_des", 3);
    logger.addIterm("swingDesPosCur_W", 3);
    logger.addIterm("swingDesPosFinal_W", 3);
    logger.addIterm("fe_l_pos_W", 3);
    logger.addIterm("fe_r_pos_W", 3);
    logger.addIterm("hip_l_pos_W", 3);
    logger.addIterm("hip_r_pos_W", 3);
    logger.addIterm("swingStartPos_W", 3);
    logger.addIterm("stanceDesPos_W", 3);
    logger.addIterm("qpStatus_MPC", 1);
    logger.addIterm("legState", 1);
    logger.addIterm("motionState", 1);
    logger.finishItermAdding();
}

void recordCommonLogger(DataLogger &logger,
                        const DataBus &RobotState,
                        double dynTime,
                        double mainCtrlDt,
                        double mpcCtrlDt,
                        const EstimatorTruthLog &truthLog)
{
    double baseLinVelLog[3] = {
        RobotState.baseLinVel[0],
        RobotState.baseLinVel[1],
        RobotState.baseLinVel[2]};
    double truthBasePosLog[3] = {
        truthLog.truth_base_pos_w[0],
        truthLog.truth_base_pos_w[1],
        truthLog.truth_base_pos_w[2]};
    double truthBaseVelLog[3] = {
        truthLog.truth_base_vel_w[0],
        truthLog.truth_base_vel_w[1],
        truthLog.truth_base_vel_w[2]};
    double truthRpyLog[3] = {
        truthLog.truth_rpy_w[0],
        truthLog.truth_rpy_w[1],
        truthLog.truth_rpy_w[2]};
    double truthInEstLog[3] = {
        truthLog.truth_in_est_frame[0],
        truthLog.truth_in_est_frame[1],
        truthLog.truth_in_est_frame[2]};
    double estErrPosLog[3] = {
        truthLog.est_err_pos[0],
        truthLog.est_err_pos[1],
        truthLog.est_err_pos[2]};
    double estErrVelLog[3] = {
        truthLog.est_err_vel[0],
        truthLog.est_err_vel[1],
        truthLog.est_err_vel[2]};
    double mujocoTouchLog[2] = {
        truthLog.mujoco_touch_force[0],
        truthLog.mujoco_touch_force[1]};

    logger.startNewLine();
    logger.recItermData("dyn_time", dynTime);
    logger.recItermData("motors_pos_cur", RobotState.motors_pos_cur);
    logger.recItermData("motors_pos_des", RobotState.motors_pos_des);
    logger.recItermData("motors_tor_cur", RobotState.motors_tor_cur);
    logger.recItermData("motors_tor_des", RobotState.motors_tor_des);
    logger.recItermData("motors_tor_out", RobotState.motors_tor_out);
    logger.recItermData("motors_vel_cur", RobotState.motors_vel_cur);
    logger.recItermData("motors_vel_des", RobotState.motors_vel_des);
    logger.recItermData("FL_est", RobotState.FL_est);
    logger.recItermData("FR_est", RobotState.FR_est);
    logger.recItermData("wbc_FrRes", RobotState.wbc_FrRes);
    logger.recItermData("Fr_ff", RobotState.Fr_ff);
    logger.recItermData("base_pos_des", RobotState.base_pos_des);
    logger.recItermData("base_pos", RobotState.base_pos);
    logger.recItermData("base_pos_est", RobotState.base_pos_est);
    logger.recItermData("baseLinVel", baseLinVelLog);
    logger.recItermData("base_vel_est", RobotState.base_vel_est);
    logger.recItermData("base_rpy", RobotState.base_rpy);
    logger.recItermData("base_omega_W", RobotState.base_omega_W);
    logger.recItermData("eul_est", RobotState.eul_est);
    logger.recItermData("truth_base_pos_w", truthBasePosLog);
    logger.recItermData("truth_base_vel_w", truthBaseVelLog);
    logger.recItermData("truth_rpy_w", truthRpyLog);
    logger.recItermData("mujoco_touch_force", mujocoTouchLog);
    logger.recItermData("truth_in_est_frame", truthInEstLog);
    logger.recItermData("est_err_pos", estErrPosLog);
    logger.recItermData("est_err_vel", estErrVelLog);
    logger.recItermData("est_err_yaw", truthLog.est_err_yaw);
    logger.recItermData("Ufe", RobotState.fe_react_tau_cmd);
    logger.recItermData("phi", RobotState.phi);
    logger.recItermData("phiSwitchMinRuntime", RobotState.phiSwitchMinRuntime);
    logger.recItermData("tSwing", RobotState.tSwing);
    logger.recItermData("mainControlDt", mainCtrlDt);
    logger.recItermData("mpcControlDt", mpcCtrlDt);
    logger.recItermData("mpcPredictionHorizon", static_cast<double>(RobotState.mpcPredictionHorizon));
    logger.recItermData("mpcControlHorizon", static_cast<double>(RobotState.mpcControlHorizon));
    logger.recItermData("js_vel_des", RobotState.js_vel_des);
    logger.recItermData("js_omega_des", RobotState.js_omega_des);
    logger.recItermData("swingDesPosCur_W", RobotState.swingDesPosCur_W);
    logger.recItermData("swingDesPosFinal_W", RobotState.swingDesPosFinal_W);
    logger.recItermData("fe_l_pos_W", RobotState.fe_l_pos_W);
    logger.recItermData("fe_r_pos_W", RobotState.fe_r_pos_W);
    logger.recItermData("hip_l_pos_W", RobotState.hip_l_pos_W);
    logger.recItermData("hip_r_pos_W", RobotState.hip_r_pos_W);
    logger.recItermData("swingStartPos_W", RobotState.swingStartPos_W);
    logger.recItermData("stanceDesPos_W", RobotState.stanceDesPos_W);
    logger.recItermData("qpStatus_MPC", static_cast<double>(RobotState.qpStatus_MPC));
    logger.recItermData("legState", RobotState.legState);
    logger.recItermData("motionState", RobotState.motionState);
    logger.finishLine();
}

class PhaseTransitionCommandBlender
{
public:
    PhaseTransitionCommandBlender(double blendTimeSec, double contactForceBlendTimeSec)
        : blendTimeSec_(std::clamp(blendTimeSec, 0.0, 0.2)),
          contactForceBlendTimeSec_(std::clamp(contactForceBlendTimeSec, 0.0, 0.2))
    {
    }

    void reset()
    {
        active_ = false;
        forceBlendActive_ = false;
        elapsed_ = 0.0;
        forceBlendElapsed_ = 0.0;
        hasLastCommand_ = false;
        lastLegState_ = DataBus::DSt;
        forceBlendLegState_ = DataBus::DSt;
        lastPosCommand_.clear();
        lastTorCommand_.clear();
        startPosCommand_.clear();
        startTorCommand_.clear();
        startForceTorCommand_.clear();
    }

    void apply(DataBus &state, double dt, bool enabled)
    {
        const bool validCommand = state.motors_pos_des.size() == state.motors_tor_des.size() &&
                                  !state.motors_pos_des.empty();
        const bool canBlend = enabled && state.motionState == DataBus::Walk && validCommand;

        if (!canBlend)
        {
            active_ = false;
            forceBlendActive_ = false;
            rememberCommand(state);
            return;
        }

        const std::vector<double> targetPos = state.motors_pos_des;
        const std::vector<double> targetTor = state.motors_tor_des;

        const bool legSwitched = hasLastCommand_ &&
                                 lastLegState_ != DataBus::DSt &&
                                 state.legState != DataBus::DSt &&
                                 state.legState != lastLegState_;
        if (legSwitched)
        {
            startPosCommand_ = lastPosCommand_;
            startTorCommand_ = lastTorCommand_;
            elapsed_ = 0.0;
            active_ = blendTimeSec_ > 0.0 &&
                      startPosCommand_.size() == state.motors_pos_des.size() &&
                      startTorCommand_.size() == state.motors_tor_des.size();

            startForceTorCommand_ = lastTorCommand_;
            forceBlendElapsed_ = 0.0;
            forceBlendLegState_ = state.legState;
            forceBlendActive_ = contactForceBlendTimeSec_ > 0.0 &&
                                startForceTorCommand_.size() == state.motors_tor_des.size();
        }

        if (active_)
        {
            const double ratio = std::clamp(elapsed_ / blendTimeSec_, 0.0, 1.0);
            const double alpha = ratio * ratio * (3.0 - 2.0 * ratio);
            for (size_t i = 0; i < targetPos.size(); ++i)
            {
                state.motors_pos_des[i] = (1.0 - alpha) * startPosCommand_[i] + alpha * targetPos[i];
                state.motors_tor_des[i] = (1.0 - alpha) * startTorCommand_[i] + alpha * targetTor[i];
            }

            elapsed_ += std::max(dt, 0.0);
            if (elapsed_ >= blendTimeSec_)
            {
                active_ = false;
            }
        }

        if (forceBlendActive_)
        {
            const double ratio = std::clamp(forceBlendElapsed_ / contactForceBlendTimeSec_, 0.0, 1.0);
            const double beta = ratio * ratio * (3.0 - 2.0 * ratio);
            const int begin = (forceBlendLegState_ == DataBus::LSt) ? 0 : 6;
            const int end = begin + 6;
            for (int i = begin; i < end && i < static_cast<int>(targetTor.size()); ++i)
            {
                state.motors_tor_des[static_cast<size_t>(i)] =
                    (1.0 - beta) * startForceTorCommand_[static_cast<size_t>(i)] +
                    beta * targetTor[static_cast<size_t>(i)];
            }

            forceBlendElapsed_ += std::max(dt, 0.0);
            if (forceBlendElapsed_ >= contactForceBlendTimeSec_)
            {
                forceBlendActive_ = false;
            }
        }

        rememberCommand(state);
    }

private:
    void rememberCommand(const DataBus &state)
    {
        if (state.motors_pos_des.size() == state.motors_tor_des.size() &&
            !state.motors_pos_des.empty())
        {
            lastPosCommand_ = state.motors_pos_des;
            lastTorCommand_ = state.motors_tor_des;
            hasLastCommand_ = true;
        }
        else
        {
            hasLastCommand_ = false;
        }
        lastLegState_ = state.legState;
    }

    double blendTimeSec_{0.0};
    double contactForceBlendTimeSec_{0.0};
    double elapsed_{0.0};
    double forceBlendElapsed_{0.0};
    bool active_{false};
    bool forceBlendActive_{false};
    bool hasLastCommand_{false};
    DataBus::LegState lastLegState_{DataBus::DSt};
    DataBus::LegState forceBlendLegState_{DataBus::DSt};
    std::vector<double> lastPosCommand_;
    std::vector<double> lastTorCommand_;
    std::vector<double> startPosCommand_;
    std::vector<double> startTorCommand_;
    std::vector<double> startForceTorCommand_;
};

class TerminalKeyReader
{
public:
    ~TerminalKeyReader()
    {
        shutdown();
    }

    bool initialize(std::string *errMsg = nullptr)
    {
        if (initialized_)
        {
            return true;
        }

        if (!isatty(STDIN_FILENO))
        {
            if (errMsg != nullptr)
            {
                *errMsg = "stdin is not a tty";
            }
            return false;
        }

        if (tcgetattr(STDIN_FILENO, &oldTermios_) != 0)
        {
            if (errMsg != nullptr)
            {
                *errMsg = std::string("tcgetattr failed: ") + std::strerror(errno);
            }
            return false;
        }

        termios raw = oldTermios_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
        {
            if (errMsg != nullptr)
            {
                *errMsg = std::string("tcsetattr failed: ") + std::strerror(errno);
            }
            return false;
        }

        oldFlags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (oldFlags_ < 0)
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldTermios_);
            if (errMsg != nullptr)
            {
                *errMsg = std::string("fcntl(F_GETFL) failed: ") + std::strerror(errno);
            }
            return false;
        }

        if (fcntl(STDIN_FILENO, F_SETFL, oldFlags_ | O_NONBLOCK) != 0)
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldTermios_);
            if (errMsg != nullptr)
            {
                *errMsg = std::string("fcntl(F_SETFL) failed: ") + std::strerror(errno);
            }
            return false;
        }

        initialized_ = true;
        return true;
    }

    void shutdown()
    {
        if (!initialized_)
        {
            return;
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldTermios_);
        if (oldFlags_ >= 0)
        {
            fcntl(STDIN_FILENO, F_SETFL, oldFlags_);
        }
        initialized_ = false;
    }

    UIctr::ButtonState poll()
    {
        UIctr::ButtonState buttonState;
        if (!initialized_)
        {
            return buttonState;
        }

        while (true)
        {
            char ch = 0;
            const ssize_t n = ::read(STDIN_FILENO, &ch, 1);
            if (n == 1)
            {
                if (ch == ' ')
                {
                    buttonState.key_space = true;
                    continue;
                }

                const char k = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
                if (k == 'w')
                    buttonState.key_w = true;
                else if (k == 's')
                    buttonState.key_s = true;
                else if (k == 'a')
                    buttonState.key_a = true;
                else if (k == 'd')
                    buttonState.key_d = true;
                else if (k == 'h')
                    buttonState.key_h = true;
                else if (k == 'j')
                    buttonState.key_j = true;
                else if (k == 'q')
                    buttonState.key_q = true;
                else if (k == 'e')
                    buttonState.key_e = true;
                else if (k == 'f')
                    buttonState.key_f = true;
                else if (k == 'g')
                    buttonState.key_g = true;
                continue;
            }

            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            {
                break;
            }
            break;
        }
        return buttonState;
    }

private:
    bool initialized_{false};
    int oldFlags_{-1};
    termios oldTermios_{};
};

struct RealJointSafetyParam
{
    std::string name;
    double kp{0.0};
    double kd{0.0};
    double minPos{-3.14};
    double maxPos{3.14};
    double maxSpeed{0.0};
    double maxTorque{0.0};
};

const std::array<std::string, 12> kRealLegJointNames = {
    "left_hip_roll_joint", "left_hip_yaw_joint", "left_hip_pitch_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};

bool loadRealJointSafetyParams(std::vector<RealJointSafetyParam> &outParams,
                               std::string &loadedPath,
                               std::string &errMsg)
{
    const std::array<std::string, 3> candidates = {
        getJointCtrlConfigV4LegPath(),
        "joint_ctrl_config_v4_leg.json",
        "common/joint_ctrl_config_v4_leg.json"};

    Json::Value root;
    std::string parseErr;
    for (const std::string &path : candidates)
    {
        std::ifstream in(path, std::ios::binary);
        if (!in.is_open())
            continue;

        Json::CharReaderBuilder builder;
        builder["allowComments"] = true;
        builder["collectComments"] = false;
        if (!Json::parseFromStream(builder, in, &root, &parseErr))
        {
            errMsg = "failed to parse " + path + ": " + parseErr;
            return false;
        }
        loadedPath = path;
        break;
    }

    if (loadedPath.empty())
    {
        errMsg = "failed to open joint_ctrl_config_v4_leg.json";
        return false;
    }

    outParams.clear();
    outParams.reserve(kRealLegJointNames.size());
    for (const std::string &name : kRealLegJointNames)
    {
        if (!root.isMember(name))
        {
            errMsg = "joint config missing " + name;
            return false;
        }
        const Json::Value &joint = root[name];
        const std::array<const char *, 6> required = {"kp", "kd", "minPos", "maxPos", "maxSpeed", "maxTorque"};
        for (const char *key : required)
        {
            if (!joint.isMember(key) || !joint[key].isNumeric())
            {
                errMsg = "joint config " + name + " missing numeric " + key;
                return false;
            }
        }

        RealJointSafetyParam param;
        param.name = name;
        param.kp = joint.isMember("closedLoopKp") && joint["closedLoopKp"].isNumeric()
                       ? joint["closedLoopKp"].asDouble()
                       : joint["kp"].asDouble();
        param.kd = joint.isMember("closedLoopKd") && joint["closedLoopKd"].isNumeric()
                       ? joint["closedLoopKd"].asDouble()
                       : joint["kd"].asDouble();
        param.minPos = joint["minPos"].asDouble();
        param.maxPos = joint["maxPos"].asDouble();
        param.maxSpeed = joint["maxSpeed"].asDouble();
        param.maxTorque = joint["maxTorque"].asDouble();
        outParams.push_back(param);
    }

    errMsg.clear();
    return true;
}

class RealSafetyMonitor
{
public:
    RealSafetyMonitor(const ControllerConfig &config,
                      const std::vector<RealJointSafetyParam> &jointParams)
        : jointParams_(jointParams),
          pvtTorqueLimitScale_(config.realPvtTorqueLimitScale)
    {
        parseBoolEnv(getEnvEither("OPENLOONG_REAL_SAFETY_ENABLE", "REAL_SAFETY_ENABLE"), enabled_);
        double tmp = 0.0;
        if (parseDoubleEnv(getEnvEither("OPENLOONG_REAL_SAFETY_ROLL_PITCH_LIMIT_DEG", "REAL_SAFETY_ROLL_PITCH_LIMIT_DEG"), tmp))
            rollPitchLimitRad_ = std::max(1.0, tmp) * kDeg2Rad;
        if (parseDoubleEnv(getEnvEither("OPENLOONG_REAL_SAFETY_ANGVEL_LIMIT_RAD_S", "REAL_SAFETY_ANGVEL_LIMIT_RAD_S"), tmp))
            angVelLimitRadS_ = std::max(0.1, tmp);
        if (parseDoubleEnv(getEnvEither("OPENLOONG_REAL_SAFETY_CMD_JUMP_LIMIT_RAD", "REAL_SAFETY_CMD_JUMP_LIMIT_RAD"), tmp))
            cmdJumpLimitRad_ = std::max(0.001, tmp);
    }

    void printConfig() const
    {
        std::cout << "[Safety-Real] " << (enabled_ ? "enabled" : "disabled")
                  << ", roll_pitch_limit=" << rollPitchLimitRad_ / kDeg2Rad << " deg"
                  << ", angvel_limit=" << angVelLimitRadS_ << " rad/s"
                  << ", cmd_jump_limit=" << cmdJumpLimitRad_ << " rad"
                  << ", pvt_torque_limit_scale=" << pvtTorqueLimitScale_ << std::endl;
        if (jointParams_.size() == expectedJointCount_)
        {
            std::cout << "[Safety-Real] PVT torque limits:";
            for (const RealJointSafetyParam &param : jointParams_)
            {
                std::cout << " " << param.name << "=" << param.maxTorque * pvtTorqueLimitScale_;
            }
            std::cout << " N*m" << std::endl;
        }
    }

    void resetCommandHistory()
    {
        hasLastCommand_ = false;
        lastCommand_.clear();
    }

    bool validateSensor(const DataBus &state, std::string &reason) const
    {
        if (!enabled_)
            return true;

        for (int i = 0; i < 3; i++)
        {
            if (!std::isfinite(state.rpy[i]) || !std::isfinite(state.baseAngVel[i]) || !std::isfinite(state.baseAcc[i]))
            {
                reason = "sensor contains NaN/Inf";
                return false;
            }
        }
        if (std::fabs(state.rpy[0]) > rollPitchLimitRad_ || std::fabs(state.rpy[1]) > rollPitchLimitRad_)
        {
            std::ostringstream oss;
            oss << "raw IMU roll/pitch over limit: roll=" << state.rpy[0] << ", pitch=" << state.rpy[1];
            reason = oss.str();
            return false;
        }
        const double omegaNorm = std::sqrt(state.baseAngVel[0] * state.baseAngVel[0] +
                                           state.baseAngVel[1] * state.baseAngVel[1] +
                                           state.baseAngVel[2] * state.baseAngVel[2]);
        if (omegaNorm > angVelLimitRadS_)
        {
            std::ostringstream oss;
            oss << "base angular velocity over limit: norm=" << omegaNorm;
            reason = oss.str();
            return false;
        }
        if (!isFiniteVector(state.motors_pos_cur) ||
            !isFiniteVector(state.motors_vel_cur) ||
            !isFiniteVector(state.motors_tor_cur))
        {
            reason = "joint feedback contains NaN/Inf";
            return false;
        }
        if (jointParams_.size() != expectedJointCount_)
        {
            reason = "joint safety parameters invalid";
            return false;
        }
        for (size_t i = 0; i < expectedJointCount_; i++)
        {
            const RealJointSafetyParam &param = jointParams_[i];
            const double qCur = state.motors_pos_cur[i];
            const double dqCur = state.motors_vel_cur[i];
            if (qCur < param.minPos || qCur > param.maxPos)
            {
                std::ostringstream oss;
                oss << "joint feedback position over limit: " << param.name
                    << ", q_cur=" << qCur
                    << ", range=[" << param.minPos << ", " << param.maxPos << "]";
                reason = oss.str();
                return false;
            }
            if (std::fabs(dqCur) > param.maxSpeed)
            {
                std::ostringstream oss;
                oss << "joint feedback velocity over limit: " << param.name
                    << ", dq_cur=" << dqCur
                    << ", limit=" << param.maxSpeed;
                reason = oss.str();
                return false;
            }
        }
        return true;
    }

    bool validateEstimatedState(const DataBus &state, std::string &reason) const
    {
        if (!enabled_)
            return true;
        for (int i = 0; i < 3; i++)
        {
            if (!std::isfinite(state.base_rpy(i)) || !std::isfinite(state.base_omega_W(i)))
            {
                reason = "estimated base state contains NaN/Inf";
                return false;
            }
        }
        if (std::fabs(state.base_rpy(0)) > rollPitchLimitRad_ || std::fabs(state.base_rpy(1)) > rollPitchLimitRad_)
        {
            std::ostringstream oss;
            oss << "estimated roll/pitch over limit: roll=" << state.base_rpy(0)
                << ", pitch=" << state.base_rpy(1);
            reason = oss.str();
            return false;
        }
        const double omegaNorm = state.base_omega_W.norm();
        if (omegaNorm > angVelLimitRadS_)
        {
            std::ostringstream oss;
            oss << "estimated angular velocity over limit: norm=" << omegaNorm;
            reason = oss.str();
            return false;
        }
        return true;
    }

    bool validateCommandAndRemember(const DataBus &state,
                                    const std::vector<double> &cmd,
                                    const std::vector<double> &tauFf,
                                    std::string &reason)
    {
        if (!enabled_)
            return true;
        if (!isValidCommandVector(cmd))
        {
            reason = "command vector invalid: size/nonfinite/range";
            return false;
        }
        if (tauFf.size() < expectedJointCount_ || !isFiniteVector(tauFf))
        {
            reason = "torque feedforward vector invalid: size/nonfinite";
            return false;
        }
        if (jointParams_.size() != expectedJointCount_ ||
            state.motors_pos_cur.size() < expectedJointCount_ ||
            state.motors_vel_cur.size() < expectedJointCount_)
        {
            reason = "joint safety input size invalid";
            return false;
        }

        for (size_t i = 0; i < expectedJointCount_; i++)
        {
            const RealJointSafetyParam &param = jointParams_[i];
            const double qCur = state.motors_pos_cur[i];
            const double dqCur = state.motors_vel_cur[i];
            const double qDes = cmd[i];
            const double tauFfVal = tauFf[i];
            if (qDes < param.minPos || qDes > param.maxPos)
            {
                std::ostringstream oss;
                oss << "joint command position over limit: " << param.name
                    << ", q_des=" << qDes
                    << ", range=[" << param.minPos << ", " << param.maxPos << "]";
                reason = oss.str();
                return false;
            }

            const double tauPd = param.kp * (qDes - qCur) + param.kd * (0.0 - dqCur);
            const double tauEst = tauPd + tauFfVal;
            const double tauLimit = param.maxTorque * pvtTorqueLimitScale_;
            if (std::fabs(tauEst) > tauLimit)
            {
                std::ostringstream oss;
                oss << "estimated PVT torque over limit: " << param.name
                    << ", q_cur=" << qCur
                    << ", q_des=" << qDes
                    << ", dq_cur=" << dqCur
                    << ", tau_pd=" << tauPd
                    << ", tau_ff=" << tauFfVal
                    << ", tau_est=" << tauEst
                    << ", limit=" << tauLimit;
                reason = oss.str();
                return false;
            }
        }
        if (hasLastCommand_)
        {
            double maxJump = 0.0;
            for (size_t i = 0; i < expectedJointCount_; i++)
                maxJump = std::max(maxJump, std::fabs(cmd[i] - lastCommand_[i]));
            if (maxJump > cmdJumpLimitRad_)
            {
                std::ostringstream oss;
                oss << "command jump over limit: max_jump=" << maxJump;
                reason = oss.str();
                return false;
            }
        }
        lastCommand_.assign(cmd.begin(), cmd.begin() + expectedJointCount_);
        hasLastCommand_ = true;
        return true;
    }

private:
    static constexpr size_t expectedJointCount_{12};
    static constexpr double kDeg2Rad{3.14159265358979323846 / 180.0};

    bool enabled_{true};
    bool hasLastCommand_{false};
    double rollPitchLimitRad_{12.0 * kDeg2Rad};
    double angVelLimitRadS_{5.0};
    double cmdJumpLimitRad_{0.25};
    double pvtTorqueLimitScale_{0.5};
    std::vector<RealJointSafetyParam> jointParams_;
    std::vector<double> lastCommand_;

    static bool isFiniteVector(const std::vector<double> &values)
    {
        for (double v : values)
        {
            if (!std::isfinite(v))
                return false;
        }
        return true;
    }

    bool isValidCommandVector(const std::vector<double> &cmd) const
    {
        return cmd.size() >= expectedJointCount_ &&
               isFiniteVector(cmd);
    }

};

void applyLegControlStateMachine(const ControllerConfig &controllerConfig,
                                 const UIctr::ButtonState &buttonState,
                                 DataBus &RobotState,
                                 JoyStickInterpreter &jsInterp,
                                 GaitScheduler &gaitScheduler,
                                 bool &openLoopPhaseActive,
                                 bool autoWalkEnabled,
                                 bool &autoWalkStarted,
                                 double autoWalkSpeed,
                                 double &xv_des,
                                 double xv_step,
                                 double xv_max,
                                 double xv_min,
                                 double turnRateCmd,
                                 double ctrlTime)
{
    if (buttonState.key_f && openLoopPhaseActive)
    {
        openLoopPhaseActive = false;
        RobotState.motionState = DataBus::Stand;
        jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
        jsInterp.setVxDesLPara(0.0, controllerConfig.vxStopRampTime);
        jsInterp.setVyDesLPara(0.0, controllerConfig.vxStopRampTime);
        jsInterp.setWzDesLPara(0.0, controllerConfig.wzStopRampTime);
        std::cout << "[OpenLoop] closed-loop enabled at t=" << ctrlTime << " s" << std::endl;
    }

    if (!openLoopPhaseActive)
    {
        if (autoWalkEnabled && !autoWalkStarted && RobotState.motionState == DataBus::Stand)
        {
            gaitScheduler.start();
            jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
            jsInterp.setVxDesLPara(autoWalkSpeed, controllerConfig.autoStartRampTime);
            RobotState.motionState = DataBus::Walk;
            autoWalkStarted = true;
            std::cout << "[AutoWalk] started at t=" << ctrlTime << " s" << std::endl;
        }

        if (buttonState.key_space && RobotState.motionState == DataBus::Stand)
        {
            gaitScheduler.start();
            jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
            RobotState.motionState = DataBus::Walk;
        }
        else if (buttonState.key_space && RobotState.motionState == DataBus::Walk && std::fabs(jsInterp.vxLGen.y) < 0.01)
        {
            RobotState.motionState = DataBus::Walk2Stand;
            jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
        }

        if (buttonState.key_a && RobotState.motionState != DataBus::Stand)
        {
            if (jsInterp.wzLGen.yDes < 0)
                jsInterp.setWzDesLPara(0, controllerConfig.wzStopRampTime);
            else
                jsInterp.setWzDesLPara(turnRateCmd, controllerConfig.wzRampTime);
        }
        if (buttonState.key_d && RobotState.motionState != DataBus::Stand)
        {
            if (jsInterp.wzLGen.yDes > 0)
                jsInterp.setWzDesLPara(0, controllerConfig.wzStopRampTime);
            else
                jsInterp.setWzDesLPara(-turnRateCmd, controllerConfig.wzRampTime);
        }

        if (buttonState.key_w && RobotState.motionState != DataBus::Stand)
            jsInterp.setVxDesLPara(xv_des, controllerConfig.vxRampTime);

        if (buttonState.key_s && RobotState.motionState != DataBus::Stand)
            jsInterp.setVxDesLPara(-std::fabs(xv_des), controllerConfig.vxRampTime);

        if (buttonState.key_j && RobotState.motionState != DataBus::Stand)
        {
            jsInterp.setVxDesLPara(0, controllerConfig.vxStopRampTime);
            jsInterp.setWzDesLPara(0, controllerConfig.wzStopRampTime);
            if (RobotState.motionState == DataBus::Walk)
            {
                RobotState.motionState = DataBus::Walk2Stand;
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
            }
        }

        if (buttonState.key_e)
        {
            xv_des = std::min(std::round((xv_des + xv_step) * 10.0) / 10.0, xv_max);
            if (RobotState.motionState != DataBus::Stand && std::fabs(jsInterp.vxLGen.yDes) > 1e-3)
            {
                const double dir = (jsInterp.vxLGen.yDes >= 0.0) ? 1.0 : -1.0;
                jsInterp.setVxDesLPara(dir * xv_des, controllerConfig.speedUpdateRampTime);
            }
        }

        if (buttonState.key_q)
        {
            xv_des = std::max(std::round((xv_des - xv_step) * 10.0) / 10.0, xv_min);
            if (RobotState.motionState != DataBus::Stand && std::fabs(jsInterp.vxLGen.yDes) > 1e-3)
            {
                const double dir = (jsInterp.vxLGen.yDes >= 0.0) ? 1.0 : -1.0;
                jsInterp.setVxDesLPara(dir * xv_des, controllerConfig.speedUpdateRampTime);
            }
        }

        if (buttonState.key_h)
        {
            jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
            jsInterp.setWzDesLPara(0, controllerConfig.headingResetRampTime);
        }
    }

    if (RobotState.motionState == DataBus::Walk2Stand || openLoopPhaseActive)
        jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));

    if (openLoopPhaseActive)
    {
        RobotState.motionState = DataBus::Stand;
        jsInterp.setVxDesLPara(0.0, controllerConfig.autoStartRampTime);
        jsInterp.setVyDesLPara(0.0, controllerConfig.autoStartRampTime);
        jsInterp.setWzDesLPara(0.0, controllerConfig.autoStartRampTime);
    }
}

bool runEstimationAndDynamics(DataBus &RobotState,
                              Pin_KinDyn_V4_Leg &kinDynSolver,
                              StateEst &StateModule,
                              double dynTime)
{
    bool estInitThisStep = false;
    if (dynTime > 1 && StateModule.flag_init)
    {
        std::cout << "init state module" << std::endl;
        StateModule.init(RobotState);
        estInitThisStep = true;
    }

    StateModule.set(RobotState);
    StateModule.update();
    StateModule.get(RobotState);

    kinDynSolver.dataBusRead(RobotState);
    kinDynSolver.computeJ_dJ();
    kinDynSolver.computeDyn();
    kinDynSolver.dataBusWrite(RobotState);

    StateModule.setF(RobotState);
    StateModule.updateF();
    StateModule.getF(RobotState);
    return estInitThisStep;
}

void runControlPipeline(const ControllerConfig &controllerConfig,
                        DataBus &RobotState,
                        Pin_KinDyn_V4_Leg &kinDynSolver,
                        JoyStickInterpreter &jsInterp,
                        GaitScheduler &gaitScheduler,
                        FootPlacement &footPlacement,
                        MPC &MPC_solv,
                        WBC_priority_V4_Leg &WBC_solv,
                        const Eigen::VectorXd &qIniDes,
                        const Eigen::VectorXd &qIniLeg,
                        double stand_legLength,
                        double foot_height,
                        int &mpcCtrlCount,
                        int mpcCtrlDecimation,
                        bool setIniWhenWalk2Stand,
                        bool holdFixedStandPose)
{

    if (setIniWhenWalk2Stand && RobotState.motionState == DataBus::Walk2Stand)
    {
        jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
    }

    if (RobotState.motionState == DataBus::Walk || RobotState.motionState == DataBus::Walk2Stand)
    {
        jsInterp.step();
        RobotState.js_pos_des(2) = stand_legLength + foot_height;
        jsInterp.dataBusWrite(RobotState);

        MPC_solv.enable();

        gaitScheduler.dataBusRead(RobotState);
        gaitScheduler.step();
        gaitScheduler.dataBusWrite(RobotState);

        footPlacement.dataBusRead(RobotState);
        footPlacement.getSwingPos();
        footPlacement.dataBusWrite(RobotState);
    }
    else
    {
        MPC_solv.disable();
    }

    if (RobotState.motionState == DataBus::Walk2Stand)
    {
        WBC_solv.setQini(qIniDes, RobotState.q);
        WBC_solv.fe_l_pos_des_W = RobotState.fe_l_pos_W;
        WBC_solv.fe_r_pos_des_W = RobotState.fe_r_pos_W;
        WBC_solv.fe_l_rot_des_W = RobotState.fe_l_rot_W;
        WBC_solv.fe_r_rot_des_W = RobotState.fe_r_rot_W;
        WBC_solv.pCoMDes = RobotState.pCoM_W;
    }

    mpcCtrlCount = mpcCtrlCount + 1;
    if (mpcCtrlCount >= mpcCtrlDecimation)
    {
        MPC_solv.dataBusRead(RobotState);
        MPC_solv.cal();
        mpcCtrlCount = 0;
    }

    if (RobotState.motionState == DataBus::Walk || RobotState.motionState == DataBus::Walk2Stand)
    {
        MPC_solv.dataBusWrite(RobotState);
    }
    else
    {
        RobotState.Fr_ff = Eigen::VectorXd::Zero(12);
        RobotState.des_ddq = Eigen::VectorXd::Zero(RobotState.model_nv);
        RobotState.des_dq = Eigen::VectorXd::Zero(RobotState.model_nv);
        RobotState.des_delta_q = Eigen::VectorXd::Zero(RobotState.model_nv);
        RobotState.base_rpy_des << 0.0, 0.0, jsInterp.thetaZ;
        RobotState.base_pos_des = RobotState.js_pos_des;
        RobotState.base_pos_des(2) = stand_legLength + foot_height;
        RobotState.Fr_ff << 0, 0, 290, 0, 0, 0,
            0, 0, 290, 0, 0, 0;
    }

    WBC_solv.dataBusRead(RobotState);
    WBC_solv.computeDdq(kinDynSolver);
    WBC_solv.computeTau();
    WBC_solv.dataBusWrite(RobotState);

    if (holdFixedStandPose && RobotState.motionState == DataBus::Stand)
    {
        RobotState.motors_pos_des = eigen2std(qIniLeg);
        RobotState.motors_vel_des.assign(RobotState.model_nv - 6, 0.0);
        RobotState.motors_tor_des.assign(RobotState.model_nv - 6, 0.0);
    }
    else
    {
        Eigen::Matrix<double, 1, nx> L_diag;
        Eigen::Matrix<double, 1, nu> K_diag;
        L_diag << 1.0, 1.0, 1.0,
            1e-3, 100.0, 1.0,
            1e-3, 1e-3, 1e-3,
            1, 100.0, 1.0;
        K_diag << 1.0, 1.0, 1.0,
            1.0, 1, 1.0,
            1.0, 1.0, 1.0,
            1.0, 1, 1.0,
            1.0;
        MPC_solv.set_weight(1e-6, L_diag, K_diag);

        Eigen::VectorXd pos_des = kinDynSolver.integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
        RobotState.motors_pos_des = eigen2std(pos_des.block(7, 0, RobotState.model_nv - 6, 1));
        RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
        RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);
    }
}

int runMujoco(const ControllerConfig &controllerConfig)
{
    char error[1000] = "Could not load binary model";
    mjModel *mj_model = mj_loadXML("../models/scene_v4_leg.xml", 0, error, 1000);
    if (mj_model == nullptr)
    {
        std::cerr << "[MuJoCo] failed to load model: " << error << std::endl;
        return 1;
    }
    mjData *mj_data = mj_makeData(mj_model);

    UIctr uiController(mj_model, mj_data);
    MJ_Interface_V4_Leg mj_interface(mj_model, mj_data);
    Pin_KinDyn_V4_Leg kinDynSolver("../models/speedbot_v4/speedbot_v4_leg.urdf");

    const double simDt = mj_model->opt.timestep;
    const int mainCtrlDecimation = std::max(1, static_cast<int>(std::lround(controllerConfig.mainControlDt / simDt)));
    const double mainCtrlDt = simDt * mainCtrlDecimation;
    const int mpcCtrlDecimation = std::max(1, static_cast<int>(std::lround(controllerConfig.mpcControlDt / mainCtrlDt)));
    const double mpcCtrlDt = mainCtrlDt * mpcCtrlDecimation;
    std::cout << "[Backend] mujoco" << std::endl;
    std::cout << "[LoopRate] sim=" << 1.0 / simDt << " Hz, main=" << 1.0 / mainCtrlDt
              << " Hz, mpc=" << 1.0 / mpcCtrlDt << " Hz" << std::endl;

    DataBus RobotState(kinDynSolver.model_nv);
    WBC_priority_V4_Leg WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, mainCtrlDt);
    MPC MPC_solv(mpcCtrlDt);
    GaitScheduler gaitScheduler(0.4, mainCtrlDt);
    const std::string jointCtrlConfigPath = getJointCtrlConfigV4LegPath();
    PVT_Ctr_V4_Leg pvtCtr(mainCtrlDt, jointCtrlConfigPath.c_str());
    FootPlacement footPlacement;
    JoyStickInterpreter jsInterp(mainCtrlDt);
    PhaseTransitionCommandBlender commandBlender(controllerConfig.phaseTransitionBlendTimeSec,
                                                controllerConfig.contactForceBlendTimeSec);
    DataLogger logger("../record/datalog.log");
    StateEst StateModule(mainCtrlDt);
    ROS2_StatePub_V4_Leg simRos2StatePub;

    configureMpcFromControllerConfig(controllerConfig, MPC_solv, WBC_solv);

    bool simRos2StatePubEnabled = controllerConfig.simEnableRos2StatePub;
    int simRos2PubCount = 0;
    int simRos2PubDecimation = std::max(1, static_cast<int>(std::lround(controllerConfig.simRosPublishDt / simDt)));
    if (simRos2StatePubEnabled)
    {
        std::string simRosErr;
        if (!simRos2StatePub.initialize(controllerConfig, &simRosErr))
        {
            std::cerr << "[ROS2-Sim] initialization failed: " << simRosErr << std::endl;
            simRos2StatePubEnabled = false;
        }
        else
        {
            std::cout << "[ROS2-Sim] enabled, publish_dt=" << simRos2PubDecimation * simDt
                      << " s, imu_topic=" << controllerConfig.rosTopicImu
                      << ", joint_topic=" << controllerConfig.rosTopicJointStates << std::endl;
        }
    }

    const bool headlessMode = std::getenv("OPENLOONG_HEADLESS") != nullptr;
    if (!headlessMode)
    {
        uiController.iniGLFW();
        uiController.enableTracking();
        uiController.createWindow("Demo_V4_Leg", false);
    }
    UIctr::ButtonState buttonState;
    std::cout << "[OpenLoop] press F to enable closed-loop walk control." << std::endl;

    double stand_legLength = 0.95;
    double foot_height = 0.053;
    double xv_des = controllerConfig.forwardSpeedDefault;
    double xv_step = controllerConfig.speedStep;
    double xv_max = controllerConfig.speedMax;
    double xv_min = controllerConfig.speedMin;
    const double turnRateCmd = controllerConfig.turnRateCmd;
    bool autoWalkEnabled = false;
    bool autoWalkStarted = false;
    double autoWalkSpeed = std::clamp(controllerConfig.forwardSpeedDefault, 0.0, 0.6);
    parseBoolEnv(std::getenv("AUTOWALK"), autoWalkEnabled);
    if (autoWalkEnabled)
    {
        std::cout << "[AutoWalk] enabled, speed=" << autoWalkSpeed << " m/s" << std::endl;
    }
    MujocoRegressionScript regressionScript = loadMujocoRegressionScriptFromEnv();
    if (regressionScript.enabled)
    {
        std::cout << "[AutoRegression] enabled"
                  << " (straight walking: F -> Space -> W -> J)"
                  << ", close_loop_t=" << regressionScript.tCloseLoop
                  << ", walk_start_t=" << regressionScript.tWalkStart
                  << ", stop_t=" << regressionScript.tPressJ
                  << ", sim_end_t=" << regressionScript.simEndTime << std::endl;
    }

    configureCommonLegModules(controllerConfig, gaitScheduler, footPlacement, WBC_solv, RobotState, stand_legLength);

    const int robot_nq = kinDynSolver.model_nv + 1;
    const int robot_nv = robot_nq - 1;

    Eigen::Vector3d fe_l_pos_L_des = {0, 0.1225, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des = {0, -0.1225, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des = {0, 0, 0};
    Eigen::Vector3d fe_r_eul_L_des = {0, 0, 0};
    Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
    // Note: empirically measured closed-loop balance pose was tested but unstable in open-loop.
    // Keeping original IK result for open-loop stability.
    Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(mj_model->nq, 1);
    qIniDes.block(7, 0, mj_model->nq - 7, 1) = resLeg.jointPosRes;
    WBC_solv.setQini(qIniDes, RobotState.q);

    addCommonLoggerItems(logger, robot_nv);

    int mainCtrlCount = mainCtrlDecimation - 1;
    int mpcCtrlCount = mpcCtrlDecimation - 1;
    EstimatorTruthLog truthLog;
    double truthBasePosW[3]{0.0, 0.0, 0.0};
    double truthBaseVelW[3]{0.0, 0.0, 0.0};
    double truthRpyW[3]{0.0, 0.0, 0.0};
    std::vector<double> truthJointTor;

    bool openLoopPhaseActive = true;
    double simEndTime = regressionScript.enabled ? regressionScript.simEndTime : 200.0;

    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;

    while (headlessMode ? (simTime < simEndTime) : (!glfwWindowShouldClose(uiController.window)))
    {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && (headlessMode || uiController.runSim))
        {
            mj_step(mj_model, mj_data);
            simTime = mj_data->time;
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);
            if (simRos2StatePubEnabled)
            {
                simRos2PubCount++;
                if (simRos2PubCount >= simRos2PubDecimation)
                {
                    simRos2StatePub.publishState(RobotState);
                    simRos2StatePub.spinSome();
                    simRos2PubCount = 0;
                }
            }

            mainCtrlCount++;
            if (mainCtrlCount < mainCtrlDecimation)
            {
                continue;
            }
            mainCtrlCount = 0;

            mj_interface.getTruthSnapshot(truthBasePosW, truthBaseVelW, truthRpyW, truthJointTor);
            const bool estInitThisStep = runEstimationAndDynamics(RobotState, kinDynSolver, StateModule, simTime);
            if (estInitThisStep)
            {
                truthLog.setReference(truthBasePosW, truthRpyW[2], RobotState.base_pos_est);
            }

            buttonState = uiController.getButtonState();
            regressionScript.inject(simTime, buttonState);
            applyLegControlStateMachine(controllerConfig, buttonState, RobotState, jsInterp, gaitScheduler,
                                        openLoopPhaseActive, autoWalkEnabled, autoWalkStarted, autoWalkSpeed,
                                        xv_des, xv_step, xv_max, xv_min, turnRateCmd, simTime);

            runControlPipeline(controllerConfig, RobotState, kinDynSolver,
                               jsInterp, gaitScheduler, footPlacement, MPC_solv, WBC_solv,
                               qIniDes, resLeg.jointPosRes, stand_legLength, foot_height,
                               mpcCtrlCount, mpcCtrlDecimation,
                               true, openLoopPhaseActive);
            commandBlender.apply(RobotState, mainCtrlDt, !openLoopPhaseActive);
            truthLog.update(RobotState, truthBasePosW, truthBaseVelW, truthRpyW);
            truthLog.setMujocoTouchForces(mj_interface.f3d[2][0], mj_interface.f3d[2][1]);

            pvtCtr.dataBusRead(RobotState);
            if (openLoopPhaseActive)
            {
                pvtCtr.calMotorsPVT(110.0 / 1000.0 / 180.0 * 3.1415);
            }
            else
            {
                pvtCtr.applyClosedLoopPD();
                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);

            mj_interface.setMotorsTorque(RobotState.motors_tor_out);

            recordCommonLogger(logger, RobotState, simTime, mainCtrlDt, mpcCtrlDt, truthLog);
        }

        if (mj_data->time >= simEndTime)
            break;

        if (!headlessMode)
            uiController.updateScene();
    }

    if (!headlessMode)
        uiController.Close();
    return 0;
}

int runRos2Real(const ControllerConfig &controllerConfig)
{
#if OPENLOONG_HAS_ROS2
    const double mainCtrlDt = std::clamp(controllerConfig.mainControlDt, 1e-4, 0.05);
    const double mpcCtrlDt = std::clamp(controllerConfig.mpcControlDt, mainCtrlDt, 0.5);
    const int mpcCtrlDecimation = std::max(1, static_cast<int>(std::lround(mpcCtrlDt / mainCtrlDt)));
    std::cout << "[Backend] ros2_real" << std::endl;
    std::cout << "[LoopRate] main=" << 1.0 / mainCtrlDt << " Hz, mpc=" << 1.0 / (mainCtrlDt * mpcCtrlDecimation) << " Hz" << std::endl;

    Pin_KinDyn_V4_Leg kinDynSolver("../models/speedbot_v4/speedbot_v4_leg.urdf");
    DataBus RobotState(kinDynSolver.model_nv);
    WBC_priority_V4_Leg WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, mainCtrlDt);
    MPC MPC_solv(mainCtrlDt * mpcCtrlDecimation);
    GaitScheduler gaitScheduler(0.4, mainCtrlDt);
    FootPlacement footPlacement;
    JoyStickInterpreter jsInterp(mainCtrlDt);
    DataLogger logger("../record/datalog.log");
    StateEst StateModule(mainCtrlDt);
    ROS2_Interface_V4_Leg ros2Interface;

    std::string ros2Err;
    if (!ros2Interface.initialize(controllerConfig, &ros2Err))
    {
        std::cerr << "[ROS2] initialization failed: " << ros2Err << std::endl;
        return 1;
    }

    configureMpcFromControllerConfig(controllerConfig, MPC_solv, WBC_solv);

    double stand_legLength = 0.95;
    double foot_height = 0.053;
    configureCommonLegModules(controllerConfig, gaitScheduler, footPlacement, WBC_solv, RobotState, stand_legLength);

    const int robot_nq = kinDynSolver.model_nv + 1;
    const int robot_nv = robot_nq - 1;

    Eigen::Vector3d fe_l_pos_L_des = {0, 0.1225, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des = {0, -0.1225, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des = {0, 0, 0};
    Eigen::Vector3d fe_r_eul_L_des = {0, 0, 0};
    Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
    // Note: empirically measured closed-loop balance pose was tested but unstable in open-loop.
    // Keeping original IK result for open-loop stability.
    Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(robot_nq, 1);
    qIniDes.block(7, 0, robot_nq - 7, 1) = resLeg.jointPosRes;
    WBC_solv.setQini(qIniDes, RobotState.q);

    std::vector<RealJointSafetyParam> jointSafetyParams;
    std::string jointSafetyPath;
    std::string jointSafetyErr;
    if (!loadRealJointSafetyParams(jointSafetyParams, jointSafetyPath, jointSafetyErr))
    {
        std::cerr << "[Safety-Real] failed to load joint safety config: " << jointSafetyErr << std::endl;
        return 1;
    }
    std::cout << "[Safety-Real] joint limits/PVT gains loaded: " << jointSafetyPath << std::endl;

    addCommonLoggerItems(logger, robot_nv);

    int mpcCtrlCount = mpcCtrlDecimation - 1;
    EstimatorTruthLog truthLog;
    PhaseTransitionCommandBlender commandBlender(controllerConfig.phaseTransitionBlendTimeSec,
                                                controllerConfig.contactForceBlendTimeSec);
    double ctrlTime = 0.0;
    bool openLoopPhaseActive = true;
    bool publishEnabled = false;
    bool safetyStopped = false;
    UIctr::ButtonState buttonState;
    bool autoWalkEnabled = false;
    bool autoWalkStarted = false;
    double autoWalkSpeed = std::clamp(controllerConfig.forwardSpeedDefault, 0.0, 0.6);
    double xv_des = controllerConfig.forwardSpeedDefault;
    const double xv_step = controllerConfig.speedStep;
    const double xv_max = controllerConfig.speedMax;
    const double xv_min = controllerConfig.speedMin;
    const double turnRateCmd = controllerConfig.turnRateCmd;
    std::cout << "[PublishGate-Real] startup is subscribe-only. Press G to start/stop control publishing." << std::endl;
    std::cout << "[OpenLoop-Real] after G starts publishing, press F to enable closed-loop stand control." << std::endl;
    std::cout << "[Key-Real] G(publish on/off) F(closed-loop) Space(stand/walk) W/S/A/D(move) Q/E(speed) J(stop) H(reset yaw)" << std::endl;

    TerminalKeyReader terminalKeyReader;
    std::string terminalErr;
    const bool terminalKeyEnabled = terminalKeyReader.initialize(&terminalErr);
    if (!terminalKeyEnabled)
    {
        std::cerr << "[Key-Real] terminal keyboard disabled: " << terminalErr << std::endl;
    }
    RealSafetyMonitor safetyMonitor(controllerConfig, jointSafetyParams);
    safetyMonitor.printConfig();

    auto forceSubscribeOnlyStand = [&]()
    {
        publishEnabled = false;
        openLoopPhaseActive = true;
        RobotState.motionState = DataBus::Stand;
        autoWalkStarted = false;
        jsInterp.reset();
        jsInterp.setVxDesLPara(0.0, controllerConfig.vxStopRampTime);
        jsInterp.setVyDesLPara(0.0, controllerConfig.vxStopRampTime);
        jsInterp.setWzDesLPara(0.0, controllerConfig.wzStopRampTime);
        safetyMonitor.resetCommandHistory();
        commandBlender.reset();
    };

    auto stopPublishingForSafety = [&](const std::string &reason)
    {
        if (!safetyStopped)
        {
            std::cerr << "[Safety-Real] stopped publishing control commands: " << reason << std::endl;
            std::cerr << "[Safety-Real] restart the controller manually after checking the robot." << std::endl;
        }
        safetyStopped = true;
        forceSubscribeOnlyStand();
    };

    auto nextTick = std::chrono::steady_clock::now();
    bool staleWarned = false;

    while (rclcpp::ok())
    {
        nextTick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(mainCtrlDt));

        ros2Interface.spinSome();
        buttonState = terminalKeyReader.poll();
        if (buttonState.key_g)
        {
            if (safetyStopped)
            {
                std::cerr << "[PublishGate-Real] G ignored after safety stop. Please restart manually." << std::endl;
            }
            else if (publishEnabled)
            {
                forceSubscribeOnlyStand();
                std::cout << "[PublishGate-Real] control publishing stopped by G." << std::endl;
            }
            else
            {
                publishEnabled = true;
                openLoopPhaseActive = true;
                RobotState.motionState = DataBus::Stand;
                autoWalkStarted = false;
                jsInterp.reset();
                safetyMonitor.resetCommandHistory();
                commandBlender.reset();
                std::cout << "[PublishGate-Real] control publishing enabled by G. Open-loop stand command active." << std::endl;
            }
        }

        if (!ros2Interface.isReady())
        {
            std::this_thread::sleep_until(nextTick);
            continue;
        }

        if (!ros2Interface.hasFreshData(controllerConfig.rosDataTimeoutSec))
        {
            if (!staleWarned)
            {
                staleWarned = true;
                std::cerr << "[ROS2] sensor stream stale." << std::endl;
            }
            if (publishEnabled)
                stopPublishingForSafety("sensor stream stale");
            std::this_thread::sleep_until(nextTick);
            continue;
        }
        staleWarned = false;

        ctrlTime += mainCtrlDt;
        ros2Interface.dataBusWrite(RobotState);

        std::string safetyReason;
        if (!safetyMonitor.validateSensor(RobotState, safetyReason))
        {
            stopPublishingForSafety(safetyReason);
            recordCommonLogger(logger, RobotState, ctrlTime, mainCtrlDt, mainCtrlDt * mpcCtrlDecimation, truthLog);
            std::this_thread::sleep_until(nextTick);
            continue;
        }

        runEstimationAndDynamics(RobotState, kinDynSolver, StateModule, ctrlTime);
        if (!safetyMonitor.validateEstimatedState(RobotState, safetyReason))
        {
            stopPublishingForSafety(safetyReason);
            recordCommonLogger(logger, RobotState, ctrlTime, mainCtrlDt, mainCtrlDt * mpcCtrlDecimation, truthLog);
            std::this_thread::sleep_until(nextTick);
            continue;
        }

        if (!publishEnabled)
        {
            openLoopPhaseActive = true;
            RobotState.motionState = DataBus::Stand;
            jsInterp.reset();
            recordCommonLogger(logger, RobotState, ctrlTime, mainCtrlDt, mainCtrlDt * mpcCtrlDecimation, truthLog);
            std::this_thread::sleep_until(nextTick);
            continue;
        }

        applyLegControlStateMachine(controllerConfig, buttonState, RobotState, jsInterp, gaitScheduler,
                                    openLoopPhaseActive, autoWalkEnabled, autoWalkStarted, autoWalkSpeed,
                                    xv_des, xv_step, xv_max, xv_min, turnRateCmd, ctrlTime);

        runControlPipeline(controllerConfig, RobotState, kinDynSolver,
                           jsInterp, gaitScheduler, footPlacement, MPC_solv, WBC_solv,
                           qIniDes, resLeg.jointPosRes, stand_legLength, foot_height,
                           mpcCtrlCount, mpcCtrlDecimation,
                           true, openLoopPhaseActive);
        commandBlender.apply(RobotState, mainCtrlDt, !openLoopPhaseActive);

        if (!safetyMonitor.validateCommandAndRemember(RobotState,
                                                      RobotState.motors_pos_des,
                                                      RobotState.motors_tor_des,
                                                      safetyReason))
        {
            stopPublishingForSafety(safetyReason);
            recordCommonLogger(logger, RobotState, ctrlTime, mainCtrlDt, mainCtrlDt * mpcCtrlDecimation, truthLog);
            std::this_thread::sleep_until(nextTick);
            continue;
        }

        ros2Interface.setMotorsCommand(RobotState.motors_pos_des, RobotState.motors_tor_des);
        recordCommonLogger(logger, RobotState, ctrlTime, mainCtrlDt, mainCtrlDt * mpcCtrlDecimation, truthLog);

        std::this_thread::sleep_until(nextTick);
    }

    return 0;
#else
    (void)controllerConfig;
    std::cerr << "[ROS2] ros2_real backend requested, but OPENLOONG_HAS_ROS2=0 at build time." << std::endl;
    return 1;
#endif
}

} // namespace

int main(int argc, char **argv)
{
    ControllerConfig controllerConfig;
    controllerConfig.forwardSpeedDefault = 0.4;
    controllerConfig.speedStep = 0.1;
    controllerConfig.speedMax = 1.2;
    controllerConfig.speedMin = 0.0;
    controllerConfig.turnRateCmd = 0.2;

    const char *cfgEnv = std::getenv("OPENLOONG_CONTROLLER_CONFIG");
    const std::string cfgPath = (cfgEnv != nullptr && std::string(cfgEnv).size() > 0)
                                    ? std::string(cfgEnv)
                                    : std::string("../common/controller_config_v4_leg.json");
    std::cout << "[ControllerConfig] loading: " << cfgPath << std::endl;

    std::string controllerConfigErr;
    if (!loadControllerConfig(cfgPath, controllerConfig, &controllerConfigErr))
    {
        std::cerr << "[ControllerConfig] fallback to built-in defaults: " << controllerConfigErr << std::endl;
    }

    const bool useRos2Real = hasArg(argc, argv, "--ros2-real");
    if (useRos2Real)
    {
        controllerConfig.simEnableRos2StatePub = false;
        return runRos2Real(controllerConfig);
    }
    return runMujoco(controllerConfig);
}
