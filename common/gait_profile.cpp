/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "gait_profile.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include "json/json.h"

namespace
{
void readOptionalDouble(const Json::Value &root, const char *key, double &target)
{
    if (root.isMember(key) && root[key].isNumeric())
    {
        target = root[key].asDouble();
    }
}

void readOptionalBool(const Json::Value &root, const char *key, bool &target)
{
    if (root.isMember(key) && root[key].isBool())
    {
        target = root[key].asBool();
    }
}
} // namespace

bool loadGaitProfile(const std::string &jsonPath, GaitProfile &outProfile, std::string *errMsg)
{
    std::ifstream in(jsonPath, std::ios::binary);
    if (!in.is_open())
    {
        if (errMsg != nullptr)
        {
            *errMsg = "failed to open file: " + jsonPath;
        }
        return false;
    }

    Json::CharReaderBuilder builder;
    // 显式允许 JSON 注释（// 或 /* */），便于配置文件内写调参说明
    builder["allowComments"] = true;
    builder["collectComments"] = false;
    Json::Value root;
    std::string parseErr;
    if (!Json::parseFromStream(builder, in, &root, &parseErr))
    {
        if (errMsg != nullptr)
        {
            *errMsg = "failed to parse json: " + parseErr;
        }
        return false;
    }

    readOptionalDouble(root, "t_swing", outProfile.tSwing);
    readOptionalDouble(root, "phi_switch_min", outProfile.phiSwitchMin);
    readOptionalDouble(root, "fz_switch_threshold", outProfile.fzSwitchThreshold);
    readOptionalDouble(root, "fz_stop_threshold", outProfile.fzStopThreshold);

    readOptionalDouble(root, "foot_kp_vx", outProfile.kpVx);
    readOptionalDouble(root, "foot_kp_vy", outProfile.kpVy);
    readOptionalDouble(root, "foot_kp_wz", outProfile.kpWz);
    readOptionalDouble(root, "step_height", outProfile.stepHeight);
    readOptionalDouble(root, "x_offset_l", outProfile.xOffsetL);
    readOptionalDouble(root, "y_offset_l", outProfile.yOffsetL);
    readOptionalDouble(root, "z_offset_w", outProfile.zOffsetW);
    readOptionalDouble(root, "swing_traj_phase", outProfile.swingTrajectoryPhase);
    readOptionalDouble(root, "swing_traj_window", outProfile.swingTrajectoryWindow);
    readOptionalDouble(root, "z_stretch_start_phi", outProfile.zStretchStartPhi);
    readOptionalDouble(root, "z_stretch_step", outProfile.zStretchStep);
    readOptionalDouble(root, "z_stretch_min", outProfile.zStretchMin);

    readOptionalDouble(root, "vx_ramp_time", outProfile.vxRampTime);
    readOptionalDouble(root, "vx_stop_ramp_time", outProfile.vxStopRampTime);
    readOptionalDouble(root, "wz_ramp_time", outProfile.wzRampTime);
    readOptionalDouble(root, "wz_stop_ramp_time", outProfile.wzStopRampTime);
    readOptionalDouble(root, "speed_update_ramp_time", outProfile.speedUpdateRampTime);
    readOptionalDouble(root, "heading_reset_ramp_time", outProfile.headingResetRampTime);
    readOptionalDouble(root, "auto_start_ramp_time", outProfile.autoStartRampTime);
    readOptionalDouble(root, "turn_rate_cmd", outProfile.turnRateCmd);
    readOptionalDouble(root, "forward_speed_default", outProfile.forwardSpeedDefault);
    readOptionalDouble(root, "speed_step", outProfile.speedStep);
    readOptionalDouble(root, "speed_max", outProfile.speedMax);
    readOptionalDouble(root, "speed_min", outProfile.speedMin);

    readOptionalDouble(root, "wbc_pos_err_clamp_xy", outProfile.posErrClampXY);
    readOptionalDouble(root, "wbc_pos_err_clamp_z", outProfile.posErrClampZ);
    readOptionalDouble(root, "wbc_posrot_kp", outProfile.posRotKp);
    readOptionalDouble(root, "wbc_posrot_kd", outProfile.posRotKd);
    readOptionalDouble(root, "wbc_posrot_kp_x", outProfile.posRotKpX);
    readOptionalDouble(root, "wbc_posrot_kp_pitch", outProfile.posRotKpPitch);
    readOptionalDouble(root, "wbc_posrot_kd_pitch", outProfile.posRotKdPitch);
    readOptionalDouble(root, "wbc_swing_kp", outProfile.swingLegKp);
    readOptionalDouble(root, "wbc_swing_kd", outProfile.swingLegKd);
    readOptionalDouble(root, "contact_miu", outProfile.contactMiu);
    readOptionalDouble(root, "mpc_mass", outProfile.mpcMass);
    readOptionalDouble(root, "mpc_delta_foot_front", outProfile.mpcDeltaFootFront);
    readOptionalDouble(root, "mpc_delta_foot_rear", outProfile.mpcDeltaFootRear);
    readOptionalDouble(root, "mpc_delta_foot_left", outProfile.mpcDeltaFootLeft);
    readOptionalDouble(root, "mpc_delta_foot_right", outProfile.mpcDeltaFootRight);
    readOptionalDouble(root, "mpc_force_max_xy", outProfile.mpcForceMaxXY);
    readOptionalDouble(root, "mpc_fz_max_scale", outProfile.mpcFzMaxScale);
    readOptionalDouble(root, "mpc_torque_max_x", outProfile.mpcTorqueMaxX);
    readOptionalDouble(root, "mpc_torque_max_y", outProfile.mpcTorqueMaxY);
    readOptionalDouble(root, "mpc_torque_max_z", outProfile.mpcTorqueMaxZ);
    readOptionalBool(root, "mpc_use_databus_inertia", outProfile.mpcUseDataBusInertia);
    readOptionalDouble(root, "mpc_inertia_xx", outProfile.mpcInertiaXx);
    readOptionalDouble(root, "mpc_inertia_xy", outProfile.mpcInertiaXy);
    readOptionalDouble(root, "mpc_inertia_xz", outProfile.mpcInertiaXz);
    readOptionalDouble(root, "mpc_inertia_yy", outProfile.mpcInertiaYy);
    readOptionalDouble(root, "mpc_inertia_yz", outProfile.mpcInertiaYz);
    readOptionalDouble(root, "mpc_inertia_zz", outProfile.mpcInertiaZz);

    // Basic safety clamping
    outProfile.tSwing = std::max(outProfile.tSwing, 0.05);
    outProfile.phiSwitchMin = std::clamp(outProfile.phiSwitchMin, 0.0, 0.99);
    outProfile.fzSwitchThreshold = std::max(outProfile.fzSwitchThreshold, 0.0);
    outProfile.fzStopThreshold = std::max(outProfile.fzStopThreshold, 0.0);
    outProfile.stepHeight = std::max(outProfile.stepHeight, 0.0);
    outProfile.swingTrajectoryPhase = std::clamp(outProfile.swingTrajectoryPhase, 1e-4, 0.99);
    outProfile.swingTrajectoryWindow = std::max(outProfile.swingTrajectoryWindow, outProfile.swingTrajectoryPhase + 1e-3);
    outProfile.zStretchStartPhi = std::clamp(outProfile.zStretchStartPhi, 0.0, 1.2);
    outProfile.speedStep = std::max(outProfile.speedStep, 0.0);
    outProfile.speedMax = std::max(outProfile.speedMax, 0.0);
    outProfile.speedMin = std::max(outProfile.speedMin, 0.0);
    outProfile.vxRampTime = std::max(outProfile.vxRampTime, 1e-3);
    outProfile.vxStopRampTime = std::max(outProfile.vxStopRampTime, 1e-3);
    outProfile.wzRampTime = std::max(outProfile.wzRampTime, 1e-3);
    outProfile.wzStopRampTime = std::max(outProfile.wzStopRampTime, 1e-3);
    outProfile.speedUpdateRampTime = std::max(outProfile.speedUpdateRampTime, 1e-3);
    outProfile.headingResetRampTime = std::max(outProfile.headingResetRampTime, 1e-3);
    outProfile.autoStartRampTime = std::max(outProfile.autoStartRampTime, 1e-3);
    outProfile.turnRateCmd = std::max(-3.0, std::min(3.0, outProfile.turnRateCmd));
    outProfile.contactMiu = std::clamp(outProfile.contactMiu, 0.01, 2.0);
    outProfile.mpcMass = std::max(outProfile.mpcMass, 1.0);
    outProfile.mpcDeltaFootFront = std::max(outProfile.mpcDeltaFootFront, 1e-4);
    outProfile.mpcDeltaFootRear = std::max(outProfile.mpcDeltaFootRear, 1e-4);
    outProfile.mpcDeltaFootLeft = std::max(outProfile.mpcDeltaFootLeft, 1e-4);
    outProfile.mpcDeltaFootRight = std::max(outProfile.mpcDeltaFootRight, 1e-4);
    outProfile.mpcForceMaxXY = std::max(outProfile.mpcForceMaxXY, 1.0);
    outProfile.mpcFzMaxScale = std::max(outProfile.mpcFzMaxScale, 0.1);
    outProfile.mpcTorqueMaxX = std::max(outProfile.mpcTorqueMaxX, 0.1);
    outProfile.mpcTorqueMaxY = std::max(outProfile.mpcTorqueMaxY, 0.1);
    outProfile.mpcTorqueMaxZ = std::max(outProfile.mpcTorqueMaxZ, 0.1);
    outProfile.mpcInertiaXx = std::max(outProfile.mpcInertiaXx, 1e-6);
    outProfile.mpcInertiaYy = std::max(outProfile.mpcInertiaYy, 1e-6);
    outProfile.mpcInertiaZz = std::max(outProfile.mpcInertiaZz, 1e-6);

    if (errMsg != nullptr)
    {
        errMsg->clear();
    }
    return true;
}
