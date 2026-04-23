/*
 * walk_mpc_wbc_leg:
 * - "mujoco" backend: MPC + WBC walking demo in MuJoCo.
 * - "ros2_real" backend: MPC + WBC real-robot control via ROS2 topics.
 *
 * Backend/mode is selected by environment variable CONTROL_MODE.
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
#include <string>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <thread>
#include <cstdlib>
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

struct MujocoRegressionScript
{
    bool enabled{false};
    double tCloseLoop{5.0};
    double tWalkStart{10.0};
    double tPressW{10.2};
    double tPressA{13.0};
    double tPressD{15.0};
    double tPressQ{17.0};
    double tPressE{19.0};
    double tPressH{21.0};
    double tPressJ{26.0};
    double simEndTime{30.0};

    bool sentF{false};
    bool sentSpace{false};
    bool sentW{false};
    bool sentA{false};
    bool sentD{false};
    bool sentQ{false};
    bool sentE{false};
    bool sentH{false};
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
        if (!sentA && simTime >= tPressA)
        {
            button.key_a = true;
            sentA = true;
            std::cout << "[AutoRegression] press A at t=" << simTime << " s" << std::endl;
        }
        if (!sentD && simTime >= tPressD)
        {
            button.key_d = true;
            sentD = true;
            std::cout << "[AutoRegression] press D at t=" << simTime << " s" << std::endl;
        }
        if (!sentQ && simTime >= tPressQ)
        {
            button.key_q = true;
            sentQ = true;
            std::cout << "[AutoRegression] press Q at t=" << simTime << " s" << std::endl;
        }
        if (!sentE && simTime >= tPressE)
        {
            button.key_e = true;
            sentE = true;
            std::cout << "[AutoRegression] press E at t=" << simTime << " s" << std::endl;
        }
        if (!sentH && simTime >= tPressH)
        {
            button.key_h = true;
            sentH = true;
            std::cout << "[AutoRegression] press H at t=" << simTime << " s" << std::endl;
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
        script.simEndTime = std::clamp(tmp, 20.0, 300.0);
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
    script.tPressA = walkBase + 3.0;
    script.tPressD = walkBase + 5.0;
    script.tPressQ = walkBase + 7.0;
    script.tPressE = walkBase + 9.0;
    script.tPressH = walkBase + 11.0;

    return script;
}

void applyControllerEnvOverrides(ControllerConfig &cfg)
{
    if (const char *modeEnv = getEnvEither("CONTROL_MODE", "OPENLOONG_CONTROL_MODE"); modeEnv != nullptr)
    {
        const std::string mode = toLowerCopy(std::string(modeEnv));
        if (mode == "mujoco")
        {
            cfg.controlBackend = "mujoco";
            cfg.simEnableRos2StatePub = false;
        }
        else if (mode == "mujoco_ros2")
        {
            cfg.controlBackend = "mujoco";
            cfg.simEnableRos2StatePub = true;
        }
        else if (mode == "ros2_real")
        {
            cfg.controlBackend = "ros2_real";
            cfg.simEnableRos2StatePub = false;
        }
    }

    bool boolTmp = false;
    if (parseBoolEnv(getEnvEither("SIM_ENABLE_ROS2_STATE_PUB", "OPENLOONG_SIM_ENABLE_ROS2_STATE_PUB"), boolTmp))
    {
        cfg.simEnableRos2StatePub = boolTmp;
    }

    if (const char *simPubDtEnv = getEnvEither("SIM_ROS_PUBLISH_DT", "OPENLOONG_SIM_ROS_PUBLISH_DT"); simPubDtEnv != nullptr)
    {
        char *endPtr = nullptr;
        const double v = std::strtod(simPubDtEnv, &endPtr);
        if (endPtr != simPubDtEnv)
        {
            cfg.simRosPublishDt = std::clamp(v, 1e-3, 0.1);
        }
    }
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
    logger.addIterm("base_pos_des", 3);
    logger.addIterm("base_pos", 3);
    logger.addIterm("base_pos_est", 3);
    logger.addIterm("baseLinVel", 3);
    logger.addIterm("base_vel_est", 3);
    logger.addIterm("base_rpy", 3);
    logger.addIterm("eul_est", 3);
    logger.addIterm("truth_base_pos_w", 3);
    logger.addIterm("truth_base_vel_w", 3);
    logger.addIterm("truth_rpy_w", 3);
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
    logger.recItermData("base_pos_des", RobotState.base_pos_des);
    logger.recItermData("base_pos", RobotState.base_pos);
    logger.recItermData("base_pos_est", RobotState.base_pos_est);
    logger.recItermData("baseLinVel", baseLinVelLog);
    logger.recItermData("base_vel_est", RobotState.base_vel_est);
    logger.recItermData("base_rpy", RobotState.base_rpy);
    logger.recItermData("eul_est", RobotState.eul_est);
    logger.recItermData("truth_base_pos_w", truthBasePosLog);
    logger.recItermData("truth_base_vel_w", truthBaseVelLog);
    logger.recItermData("truth_rpy_w", truthRpyLog);
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
    logger.recItermData("qpStatus_MPC", static_cast<double>(RobotState.qpStatus_MPC));
    logger.recItermData("legState", RobotState.legState);
    logger.recItermData("motionState", RobotState.motionState);
    logger.finishLine();
}

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
    PVT_Ctr_V4_Leg pvtCtr(mainCtrlDt, "../common/joint_ctrl_config_v4_leg.json");
    FootPlacement footPlacement;
    JoyStickInterpreter jsInterp(mainCtrlDt);
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

    uiController.iniGLFW();
    uiController.enableTracking();
    uiController.createWindow("Demo_V4_Leg", false);
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

    while (!glfwWindowShouldClose(uiController.window))
    {
        simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0 && uiController.runSim)
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
            truthLog.update(RobotState, truthBasePosW, truthBaseVelW, truthRpyW);

            pvtCtr.dataBusRead(RobotState);
            if (openLoopPhaseActive)
            {
                pvtCtr.calMotorsPVT(110.0 / 1000.0 / 180.0 * 3.1415);
            }
            else
            {
                double kp = 1.;
                double kd = 1.;

                pvtCtr.setJointPD(400 * kp, 15 * kd, "left_hip_roll_joint");
                pvtCtr.setJointPD(200 * kp, 10 * kd, "left_hip_yaw_joint");
                pvtCtr.setJointPD(300 * kp, 10 * kd, "left_hip_pitch_joint");
                pvtCtr.setJointPD(300 * kp, 14 * kd, "left_knee_joint");
                pvtCtr.setJointPD(300 * kp, 18 * kd, "left_ankle_pitch_joint");
                pvtCtr.setJointPD(300 * kp, 16 * kd, "left_ankle_roll_joint");

                pvtCtr.setJointPD(400 * kp, 15 * kd, "right_hip_roll_joint");
                pvtCtr.setJointPD(200 * kp, 10 * kd, "right_hip_yaw_joint");
                pvtCtr.setJointPD(300 * kp, 10 * kd, "right_hip_pitch_joint");
                pvtCtr.setJointPD(300 * kp, 14 * kd, "right_knee_joint");
                pvtCtr.setJointPD(300 * kp, 18 * kd, "right_ankle_pitch_joint");
                pvtCtr.setJointPD(300 * kp, 16 * kd, "right_ankle_roll_joint");
                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);

            mj_interface.setMotorsTorque(RobotState.motors_tor_out);

            recordCommonLogger(logger, RobotState, simTime, mainCtrlDt, mpcCtrlDt, truthLog);
        }

        if (mj_data->time >= simEndTime)
            break;

        uiController.updateScene();
    }

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
    Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(robot_nq, 1);
    qIniDes.block(7, 0, robot_nq - 7, 1) = resLeg.jointPosRes;
    WBC_solv.setQini(qIniDes, RobotState.q);

    addCommonLoggerItems(logger, robot_nv);

    int mpcCtrlCount = mpcCtrlDecimation - 1;
    EstimatorTruthLog truthLog;
    double ctrlTime = 0.0;
    bool openLoopPhaseActive = true;
    UIctr::ButtonState buttonState;
    bool autoWalkEnabled = false;
    bool autoWalkStarted = false;
    double autoWalkSpeed = std::clamp(controllerConfig.forwardSpeedDefault, 0.0, 0.6);
    double xv_des = controllerConfig.forwardSpeedDefault;
    const double xv_step = controllerConfig.speedStep;
    const double xv_max = controllerConfig.speedMax;
    const double xv_min = controllerConfig.speedMin;
    const double turnRateCmd = controllerConfig.turnRateCmd;
    parseBoolEnv(std::getenv("AUTOWALK"), autoWalkEnabled);
    if (autoWalkEnabled)
    {
        std::cout << "[AutoWalk-Real] enabled, speed=" << autoWalkSpeed << " m/s" << std::endl;
    }
    std::cout << "[OpenLoop-Real] press F to enable closed-loop walk control." << std::endl;
    std::cout << "[Key-Real] Space(stand/walk) W/S/A/D(move) Q/E(speed) J(stop) H(reset yaw)" << std::endl;

    TerminalKeyReader terminalKeyReader;
    std::string terminalErr;
    const bool terminalKeyEnabled = terminalKeyReader.initialize(&terminalErr);
    if (!terminalKeyEnabled)
    {
        std::cerr << "[Key-Real] terminal keyboard disabled: " << terminalErr << std::endl;
    }

    auto nextTick = std::chrono::steady_clock::now();
    bool staleWarned = false;

    while (rclcpp::ok())
    {
        nextTick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(mainCtrlDt));

        ros2Interface.spinSome();
        if (!ros2Interface.isReady())
        {
            ros2Interface.setMotorsPosition(eigen2std(resLeg.jointPosRes));
            std::this_thread::sleep_until(nextTick);
            continue;
        }

        if (!ros2Interface.hasFreshData(controllerConfig.rosDataTimeoutSec))
        {
            if (!staleWarned)
            {
                staleWarned = true;
                std::cerr << "[ROS2] sensor stream stale, holding stand command." << std::endl;
            }
            ros2Interface.setMotorsPosition(eigen2std(resLeg.jointPosRes));
            std::this_thread::sleep_until(nextTick);
            continue;
        }
        staleWarned = false;

        ctrlTime += mainCtrlDt;
        ros2Interface.dataBusWrite(RobotState);
        runEstimationAndDynamics(RobotState, kinDynSolver, StateModule, ctrlTime);
        buttonState = terminalKeyReader.poll();

        applyLegControlStateMachine(controllerConfig, buttonState, RobotState, jsInterp, gaitScheduler,
                                    openLoopPhaseActive, autoWalkEnabled, autoWalkStarted, autoWalkSpeed,
                                    xv_des, xv_step, xv_max, xv_min, turnRateCmd, ctrlTime);

        runControlPipeline(controllerConfig, RobotState, kinDynSolver,
                           jsInterp, gaitScheduler, footPlacement, MPC_solv, WBC_solv,
                           qIniDes, resLeg.jointPosRes, stand_legLength, foot_height,
                           mpcCtrlCount, mpcCtrlDecimation,
                           true, openLoopPhaseActive);

        ros2Interface.setMotorsPosition(RobotState.motors_pos_des);
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

    applyControllerEnvOverrides(controllerConfig);

    if (controllerConfig.controlBackend == "ros2_real")
    {
        return runRos2Real(controllerConfig);
    }
    return runMujoco(controllerConfig);
}
