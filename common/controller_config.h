/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include <string>

struct ControllerConfig
{
    double simRosPublishDt{0.01};

    // ROS2 topic names for real backend
    std::string rosTopicImu{"/imu/data"};
    std::string rosTopicJointStates{"/joint_states"};
    std::string rosTopicActionCmd{"/rl_motion_control_command"};
    double rosDataTimeoutSec{0.2};
    double realPvtTorqueLimitScale{0.5};

    // Control loop timing
    double mainControlDt{0.001};
    double mpcControlDt{0.005};
    double phaseTransitionBlendTimeSec{0.02};
    double contactForceBlendTimeSec{0.03};

    // Gait scheduler / phase switching
    double tSwing{0.4};
    double phiSwitchMin{0.6};
    bool phiSwitchAutoDesign{false};
    double phiSwitchDesignRefTSwing{0.4};
    double phiSwitchDesignRef{0.6};
    double phiSwitchDesignPower{1.0};
    double phiSwitchDesignMin{0.15};
    double phiSwitchDesignMax{0.9};
    double fzSwitchThreshold{280.0};
    double fzStopThreshold{200.0};
    double contactConfirmTimeSec{0.02};

    // Foot placement
    double kpVx{0.03};
    double kpVy{0.03};
    double kpWz{0.03};
    double stepHeight{0.12};
    double xOffsetL{-0.07};
    double yOffsetL{0.04};
    double zOffsetW{-0.035};
    double swingTrajectoryPhase{0.2};
    double swingTrajectoryWindow{1.4};
    double zStretchStartPhi{0.98};
    double zStretchStep{-0.002};
    double zStretchMin{-0.05};

    // Joystick command shaping
    double vxRampTime{2.0};
    double vxStopRampTime{0.5};
    double wzRampTime{1.0};
    double wzStopRampTime{0.5};
    double speedUpdateRampTime{0.6};
    double headingResetRampTime{0.3};
    double autoStartRampTime{1.0};
    double turnRateCmd{0.2};

    // Speed knobs
    double forwardSpeedDefault{0.4};
    double speedStep{0.1};
    double speedMax{1.2};
    double speedMin{0.0};

    // WBC walk-task tuning
    double posErrClampXY{0.02};
    double posErrClampZ{0.005};
    double posRotKp{500.0};
    double posRotKd{10.0};
    double posRotKpX{100.0};
    double posRotKpPitch{800.0};
    double posRotKdPitch{10.0};
    double swingLegKp{500.0};
    double swingLegKd{20.0};

    // Contact / friction params (shared source for MPC & WBC)
    double contactMiu{0.7};

    // MPC model / constraint params
    double mpcMass{77.35};
    double mpcDeltaFootFront{0.073};
    double mpcDeltaFootRear{0.125};
    double mpcDeltaFootLeft{0.025};
    double mpcDeltaFootRight{0.025};
    double mpcForceMaxXY{1000.0};
    double mpcFzMaxScale{3.0};
    double mpcTorqueMaxX{20.0};
    double mpcTorqueMaxY{80.0};
    double mpcTorqueMaxZ{100.0};
    int mpcPredictionHorizon{10};
    int mpcControlHorizon{3};
    bool mpcUseDataBusInertia{true};
    double mpcInertiaXx{12.61};
    double mpcInertiaXy{0.0};
    double mpcInertiaXz{0.37};
    double mpcInertiaYy{11.15};
    double mpcInertiaYz{0.01};
    double mpcInertiaZz{2.15};
};

bool loadControllerConfig(const std::string &jsonPath, ControllerConfig &outProfile, std::string *errMsg = nullptr);
