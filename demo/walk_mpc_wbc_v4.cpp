/*
 * walk_mpc_wbc_v4: MPC + WBC walking demo for speedbot_v4 robot.
 */
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "GLFW_callbacks.h"
#include "MJ_interface_v4.h"
#include "ROS2_state_pub_v4.h"
#include "PVT_ctrl_v4.h"
#include "data_logger.h"
#include "data_bus.h"
#include "pino_kin_dyn_v4.h"
#include "useful_math.h"
#include "wbc_priority_v4.h"
#include "mpc.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"
#include "controller_config.h"
#include <string>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include "StateEst.h"

char error[1000] = "Could not load binary model";
mjModel *mj_model = mj_loadXML("../models/scene_v4.xml", 0, error, 1000);
mjData *mj_data = mj_makeData(mj_model);

int main(int argc, char **argv)
{
    // initialize classes
    UIctr uiController(mj_model, mj_data);
    MJ_Interface_V4 mj_interface(mj_model, mj_data);
    Pin_KinDyn_V4 kinDynSolver("../models/speedbot_v4/speedbot_v4.urdf");

    ControllerConfig controllerConfig;
    controllerConfig.forwardSpeedDefault = 0.4;
    controllerConfig.speedStep = 0.1;
    controllerConfig.speedMax = 1.2;
    controllerConfig.speedMin = 0.0;
    controllerConfig.turnRateCmd = 0.2;
    const char *cfgEnv = std::getenv("OPENLOONG_CONTROLLER_CONFIG");
    const std::string cfgPath = (cfgEnv != nullptr && std::string(cfgEnv).size() > 0)
                                    ? std::string(cfgEnv)
                                    : std::string("../common/controller_config_v4.json");
    std::cout << "[ControllerConfig] loading: " << cfgPath << std::endl;
    std::string controllerConfigErr;
    if (!loadControllerConfig(cfgPath, controllerConfig, &controllerConfigErr))
    {
        std::cerr << "[ControllerConfig] fallback to built-in defaults: " << controllerConfigErr << std::endl;
    }
    const double simDt = mj_model->opt.timestep;
    const int mainCtrlDecimation = std::max(1, static_cast<int>(std::lround(controllerConfig.mainControlDt / simDt)));
    const double mainCtrlDt = simDt * mainCtrlDecimation;
    const int mpcCtrlDecimation = std::max(1, static_cast<int>(std::lround(controllerConfig.mpcControlDt / mainCtrlDt)));
    const double mpcCtrlDt = mainCtrlDt * mpcCtrlDecimation;
    std::cout << "[LoopRate] sim=" << 1.0 / simDt << " Hz, main=" << 1.0 / mainCtrlDt
              << " Hz, mpc=" << 1.0 / mpcCtrlDt << " Hz" << std::endl;

    DataBus RobotState(kinDynSolver.model_nv);
    WBC_priority_V4 WBC_solv(kinDynSolver.model_nv, 18, 22, 0.7, mainCtrlDt);
    MPC MPC_solv(mpcCtrlDt);
    GaitScheduler gaitScheduler(0.4, mainCtrlDt);
    PVT_Ctr_V4 pvtCtr(mainCtrlDt, "../common/joint_ctrl_config_v4.json");
    FootPlacement footPlacement;
    JoyStickInterpreter jsInterp(mainCtrlDt);
    DataLogger logger("../record/datalog.log");
    StateEst StateModule(mainCtrlDt);
    ROS2_StatePub_V4 simRos2StatePub;
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

    // initialize UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking();
    uiController.createWindow("Demo_V4", false);
    UIctr::ButtonState buttonState;
    std::cout << "[OpenLoop] press F to enable closed-loop walk control." << std::endl;

    // initialize variables
    // speedbot_v4: leg length ~0.983m, use 0.95 for slight bend; foot height ~0.053m     
    double stand_legLength = 0.95;  // desired baselink height
    double foot_height = 0.053;     // distance between the foot ankel joint and the bottom
    double xv_des = controllerConfig.forwardSpeedDefault; // desired velocity in x direction
    double xv_step = controllerConfig.speedStep;          // speed increment per key press
    double xv_max = controllerConfig.speedMax;            // speed magnitude upper bound
    double xv_min = controllerConfig.speedMin;            // speed magnitude lower bound
    const double turnRateCmd = controllerConfig.turnRateCmd;

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

    const int robot_nq = kinDynSolver.model_nv + 1;
    const int robot_nv = robot_nq - 1;

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

    std::vector<double> motors_pos_des(robot_nv - 6, 0);
    std::vector<double> motors_pos_cur(robot_nv - 6, 0);
    std::vector<double> motors_vel_des(robot_nv - 6, 0);
    std::vector<double> motors_vel_cur(robot_nv - 6, 0);
    std::vector<double> motors_tau_des(robot_nv - 6, 0);
    std::vector<double> motors_tau_cur(robot_nv - 6, 0);

    // ini position for foot-end and hand
    // speedbot_v4: hip y offset ≈ ±0.1225, x offset ≈ 0
    Eigen::Vector3d fe_l_pos_L_des = {0, 0.1225, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des = {0, -0.1225, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des = {0, 0, 0};
    Eigen::Vector3d fe_r_eul_L_des = {0, 0, 0};
    Eigen::Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));

    // arm initial pose: 5 DoF per arm (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
    Eigen::VectorXd hd_l_des, hd_r_des;
    hd_l_des.resize(5);
    hd_r_des.resize(5);
    hd_l_des << 0.3, 1.4, 0, -1.4, 0;
    hd_r_des << -0.3, -1.4, 0, 1.4, 0;

    auto resLeg = kinDynSolver.computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);
    Eigen::VectorXd qIniDes = Eigen::VectorXd::Zero(mj_model->nq, 1);
    qIniDes.block(7, 0, mj_model->nq - 7, 1) = resLeg.jointPosRes;
    // Overwrite arm joints in q-space:
    // arm_l: fixed indices 12-16 → q[19..23]
    // arm_r: fixed indices 17-21 → q[24..28]
    qIniDes.block(19, 0, 5, 1) = hd_l_des;
    qIniDes.block(24, 0, 5, 1) = hd_r_des;
    WBC_solv.setQini(qIniDes, RobotState.q);

    // register data logger items
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

    //// main loop
    int mainCtrlCount = mainCtrlDecimation - 1;
    int mpcCtrlCount = mpcCtrlDecimation - 1;

    bool openLoopPhaseActive = true;
    double simEndTime = 200;
    const char *autoWalkEnv = std::getenv("AUTOWALK");
    const bool autoWalk = (autoWalkEnv != nullptr) && (std::string(autoWalkEnv) == "1");
    bool autoWalkStarted = false;
    bool autoStopTriggered = false;
    bool stopToStandPending = false;
    const double stopTransitionVxThresh = 0.05;
    const double stopTransitionWzThresh = 0.08;
    double autoStopTime = -1.0;
    const char *autoStopEnv = std::getenv("OPENLOONG_AUTOSTOP_TIME");
    if (autoStopEnv != nullptr)
    {
        autoStopTime = std::atof(autoStopEnv);
    }
    const char *simEndEnv = std::getenv("OPENLOONG_SIM_END");
    if (simEndEnv != nullptr)
    {
        simEndTime = std::max(1.0, std::atof(simEndEnv));
    }

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

            if (simTime > 1 && StateModule.flag_init)
            {
                std::cout << "init state module" << std::endl;
                StateModule.init(RobotState);
            }

            buttonState = uiController.getButtonState();
            if (buttonState.key_f && openLoopPhaseActive)
            {
                openLoopPhaseActive = false;
                stopToStandPending = false;
                RobotState.motionState = DataBus::Stand;
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                jsInterp.setVxDesLPara(0.0, controllerConfig.vxStopRampTime);
                jsInterp.setVyDesLPara(0.0, controllerConfig.vxStopRampTime);
                jsInterp.setWzDesLPara(0.0, controllerConfig.wzStopRampTime);
                std::cout << "[OpenLoop] closed-loop enabled at t=" << simTime << " s" << std::endl;
            }
            if (!openLoopPhaseActive)
            {
                if (buttonState.key_space && RobotState.motionState == DataBus::Stand)
                {
                    gaitScheduler.start();
                    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                    RobotState.motionState = DataBus::Walk;
                }
                else if (buttonState.key_space && RobotState.motionState == DataBus::Walk)
                {
                    // Graceful stop: first ramp speed/yaw-rate to zero, then switch to Walk2Stand.
                    jsInterp.setVxDesLPara(0.0, controllerConfig.vxStopRampTime);
                    jsInterp.setWzDesLPara(0.0, controllerConfig.wzStopRampTime);
                    stopToStandPending = true;
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

                // W: forward at current xv_des
                if (buttonState.key_w && RobotState.motionState != DataBus::Stand)
                    jsInterp.setVxDesLPara(xv_des, controllerConfig.vxRampTime);

                // S: backward at current xv_des (negative)
                if (buttonState.key_s && RobotState.motionState != DataBus::Stand)
                    jsInterp.setVxDesLPara(-fabs(xv_des), controllerConfig.vxRampTime);

                // J: emergency stop to stand
                if (buttonState.key_j && RobotState.motionState != DataBus::Stand)
                {
                    jsInterp.setVxDesLPara(0, controllerConfig.vxStopRampTime);
                    jsInterp.setWzDesLPara(0, controllerConfig.wzStopRampTime);
                    stopToStandPending = true;
                    std::cout << "[Joystick] J: stop and stand" << std::endl;
                }

                // E: increase speed
                if (buttonState.key_e)
                {
                    xv_des = std::min(std::round((xv_des + xv_step) * 10.0) / 10.0, xv_max);
                    if (RobotState.motionState != DataBus::Stand && std::fabs(jsInterp.vxLGen.yDes) > 1e-3)
                    {
                        const double dir = (jsInterp.vxLGen.yDes >= 0.0) ? 1.0 : -1.0;
                        jsInterp.setVxDesLPara(dir * xv_des, controllerConfig.speedUpdateRampTime);
                    }
                    std::cout << "[Speed] xv_des=" << xv_des << " m/s" << std::endl;
                }

                // Q: decrease speed
                if (buttonState.key_q)
                {
                    xv_des = std::max(std::round((xv_des - xv_step) * 10.0) / 10.0, xv_min);
                    if (RobotState.motionState != DataBus::Stand && std::fabs(jsInterp.vxLGen.yDes) > 1e-3)
                    {
                        const double dir = (jsInterp.vxLGen.yDes >= 0.0) ? 1.0 : -1.0;
                        jsInterp.setVxDesLPara(dir * xv_des, controllerConfig.speedUpdateRampTime);
                    }
                    std::cout << "[Speed] xv_des=" << xv_des << " m/s" << std::endl;
                }

                if (buttonState.key_h)
                {
                    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                    jsInterp.setWzDesLPara(0, controllerConfig.headingResetRampTime);
                    std::cout << "[Joystick] H: reset heading reference" << std::endl;
                }

                if (autoWalk && !autoWalkStarted && RobotState.motionState == DataBus::Stand)
                {
                    gaitScheduler.start();
                    jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                    RobotState.motionState = DataBus::Walk;
                    jsInterp.setVxDesLPara(xv_des, controllerConfig.autoStartRampTime);
                    autoWalkStarted = true;
                    std::cout << "[AutoWalk] started with vx_des=" << xv_des << " m/s" << std::endl;
                }
                if (autoWalk && autoWalkStarted && !autoStopTriggered && autoStopTime > 0.0 &&
                    simTime >= autoStopTime && RobotState.motionState == DataBus::Walk)
                {
                    jsInterp.setVxDesLPara(0.0, controllerConfig.vxStopRampTime);
                    jsInterp.setWzDesLPara(0.0, controllerConfig.wzStopRampTime);
                    stopToStandPending = true;
                    autoStopTriggered = true;
                    std::cout << "[AutoWalk] auto stop triggered at t=" << simTime << " s" << std::endl;
                }

                if (stopToStandPending && RobotState.motionState == DataBus::Walk)
                {
                    if (std::fabs(jsInterp.vxLGen.y) < stopTransitionVxThresh &&
                        std::fabs(jsInterp.wzLGen.y) < stopTransitionWzThresh)
                    {
                        RobotState.motionState = DataBus::Walk2Stand;
                        jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));
                        stopToStandPending = false;
                        std::cout << "[Stop2Stand] switch to Walk2Stand at t=" << simTime
                                  << " s, vxGen=" << jsInterp.vxLGen.y
                                  << ", wzGen=" << jsInterp.wzLGen.y << std::endl;
                    }
                }
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

            if (openLoopPhaseActive)
            {
                RobotState.motionState = DataBus::Stand;
                stopToStandPending = false;
                jsInterp.setVxDesLPara(0.0, controllerConfig.vxStopRampTime);
                jsInterp.setVyDesLPara(0.0, controllerConfig.vxStopRampTime);
                jsInterp.setWzDesLPara(0.0, controllerConfig.wzStopRampTime);
            }

            if (RobotState.motionState == DataBus::Walk2Stand || openLoopPhaseActive)
                jsInterp.setIniPos(RobotState.q(0), RobotState.q(1), RobotState.base_rpy(2));

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
            else{
                MPC_solv.disable();
            }

            if (openLoopPhaseActive || RobotState.motionState == DataBus::Walk2Stand)
            {
                WBC_solv.setQini(qIniDes, RobotState.q);
                WBC_solv.fe_l_pos_des_W = RobotState.fe_l_pos_W;
                WBC_solv.fe_r_pos_des_W = RobotState.fe_r_pos_W;
                WBC_solv.fe_l_rot_des_W = RobotState.fe_l_rot_W;
                WBC_solv.fe_r_rot_des_W = RobotState.fe_r_rot_W;
                WBC_solv.pCoMDes = RobotState.pCoM_W;
            }

            // MPC
            mpcCtrlCount = mpcCtrlCount + 1;
            if (mpcCtrlCount >= mpcCtrlDecimation) {
                MPC_solv.dataBusRead(RobotState);
                MPC_solv.cal();
                mpcCtrlCount = 0;
            }

            if (RobotState.motionState==DataBus::Walk || RobotState.motionState==DataBus::Walk2Stand) {
                MPC_solv.dataBusWrite(RobotState);
            }
            else {
                RobotState.Fr_ff = Eigen::VectorXd::Zero(12);
                RobotState.des_ddq = Eigen::VectorXd::Zero(RobotState.model_nv);
                RobotState.des_dq = Eigen::VectorXd::Zero(RobotState.model_nv);
                RobotState.des_delta_q = Eigen::VectorXd::Zero(RobotState.model_nv);
                RobotState.base_rpy_des << 0.0, 0.0, jsInterp.thetaZ;
                RobotState.base_pos_des= RobotState.js_pos_des;
                RobotState.base_pos_des(2) = stand_legLength+foot_height;
                // ~59 kg robot, ~290 N per foot
                RobotState.Fr_ff<<0,0,290,0,0,0,
                        0,0,290,0,0,0;
            }

            // WBC
            WBC_solv.dataBusRead(RobotState);
            WBC_solv.computeDdq(kinDynSolver);
            WBC_solv.computeTau();
            WBC_solv.dataBusWrite(RobotState);

            // joint command
            if (openLoopPhaseActive)
            {
                Eigen::VectorXd temp = resLeg.jointPosRes;
                // arm_l at fixed indices 12-16, arm_r at fixed indices 17-21
                temp.block(12, 0, 5, 1) = hd_l_des;
                temp.block(17, 0, 5, 1) = hd_r_des;
                RobotState.motors_pos_des = eigen2std(temp);
                RobotState.motors_vel_des = motors_vel_des;
                RobotState.motors_tor_des = motors_tau_des;
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
                RobotState.motors_pos_des = eigen2std(pos_des.block(7, 0, robot_nv - 6, 1));
                RobotState.motors_vel_des = eigen2std(RobotState.wbc_dq_final);
                RobotState.motors_tor_des = eigen2std(RobotState.wbc_tauJointRes);
            }

            // joint PVT controller
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

            logger.startNewLine();
            logger.recItermData("dyn_time", simTime);
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
            logger.recItermData("baseLinVel", RobotState.baseLinVel);
            logger.recItermData("base_vel_est", RobotState.base_vel_est);
            logger.recItermData("base_rpy", RobotState.base_rpy);
            logger.recItermData("eul_est", RobotState.eul_est);
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

        if (mj_data->time >= simEndTime)
            break;

        uiController.updateScene();
    };
    uiController.Close();

    return 0;
}
