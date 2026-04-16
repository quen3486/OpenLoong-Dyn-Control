/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/

#include "gait_scheduler.h"
#include <algorithm>
#include <cmath>

GaitScheduler::GaitScheduler(double tSwingIn, double dtIn)
{
    tSwing = tSwingIn;
    dt = dtIn;
    phi = 0.0;
    isIni = false;
    firstleg = DataBus::LSt;
    phiSwitchMinRuntime = phiSwitchMin;
    legState = DataBus::DSt;
    legStateNext = firstleg;
    motionState = DataBus::Stand;
    enableNextStep = false;
    touchDown = false;
}

double GaitScheduler::designPhiSwitchMin() const
{
    if (!phiSwitchAutoDesign)
    {
        return std::clamp(phiSwitchMin, 0.0, 0.99);
    }
    const double tRef = std::max(0.05, phiSwitchDesignRefTSwing);
    const double tNow = std::max(0.05, tSwing);
    const double ratio = tRef / tNow;
    const double designed = phiSwitchDesignRef * std::pow(ratio, phiSwitchDesignPower);
    const double lo = std::min(phiSwitchDesignMin, phiSwitchDesignMax);
    const double hi = std::max(phiSwitchDesignMin, phiSwitchDesignMax);
    return std::clamp(designed, lo, hi);
}

void GaitScheduler::dataBusRead(const DataBus &robotState)
{
    if (motionState != DataBus::Stand && stepNumCur == 0)
        legState = firstleg;

    model_nv = robotState.model_nv;
    torJoint = Eigen::VectorXd::Zero(model_nv - 6);
    for (int i = 0; i < model_nv - 6; i++)
    {
        torJoint[i] = robotState.motors_tor_cur[i];
    }
    dyn_M = robotState.dyn_M;
    dyn_Non = robotState.dyn_Non;
    J_l = robotState.J_l;
    dJ_l = robotState.dJ_l;
    J_r = robotState.J_r;
    dJ_r = robotState.dJ_r;
    Fz_L_m = robotState.fL[2];
    Fz_R_m = robotState.fR[2];
    hip_l_pos_W = robotState.hip_l_pos_W;
    hip_r_pos_W = robotState.hip_r_pos_W;
    fe_r_pos_W = robotState.fe_r_pos_W;
    fe_l_pos_W = robotState.fe_l_pos_W;
    fe_l_rot_W = robotState.fe_l_rot_W;
    fe_r_rot_W = robotState.fe_r_rot_W;
    dq = robotState.dq;
    motionState = robotState.motionState;
}

void GaitScheduler::dataBusWrite(DataBus &robotState)
{
    robotState.tSwing = tSwing;
    robotState.swingStartPos_W = swingStartPos_W;
    robotState.stanceDesPos_W = stanceStartPos_W;
    robotState.posHip_W = posHip_W;
    robotState.posST_W = posST_W;
    robotState.theta0 = theta0;
    robotState.legState = legState;
    robotState.legStateNext = legStateNext;
    robotState.phi = phi;
    robotState.phiSwitchMinRuntime = phiSwitchMinRuntime;
    robotState.FL_est = FLest;
    robotState.FR_est = FRest;
    if (legState == DataBus::LSt)
    {
        robotState.stance_fe_pos_cur_W = fe_l_pos_W;
        robotState.stance_fe_rot_cur_W = fe_l_rot_W;
    }
    else if (legState == DataBus::RSt)
    {
        robotState.stance_fe_pos_cur_W = fe_r_pos_W;
        robotState.stance_fe_rot_cur_W = fe_r_rot_W;
    }
    else
    {
        robotState.stance_fe_pos_cur_W = 0.5 * (fe_l_pos_W + fe_r_pos_W);
        robotState.stance_fe_rot_cur_W = fe_l_rot_W;
    }
    robotState.motionState = motionState;
}

void GaitScheduler::step()
{
    Eigen::VectorXd tauAll;
    tauAll = Eigen::VectorXd::Zero(model_nv);
    tauAll.block(6, 0, model_nv - 6, 1) = torJoint;
    FLest = -pseudoInv_SVD(J_l * dyn_M.inverse() * J_l.transpose()) * (J_l * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_l * dq);
    FRest = -pseudoInv_SVD(J_r * dyn_M.inverse() * J_r.transpose()) * (J_r * dyn_M.inverse() * (tauAll - dyn_Non) + dJ_r * dq);

    phiSwitchMinRuntime = designPhiSwitchMin();
    double dPhi{0.0};

    if (motionState == DataBus::Walk2Stand)
    {
        enableNextStep = false;
        start_walk = false;
        if (touchDown)
            motionState = DataBus::Stand;
    }

    if (motionState == DataBus::Stand)
    {
        dPhi = 0.0;
        phi = 0.0;
        isIni = false;
        enableNextStep = false;
        stepNumCur = 0;
        legState = DataBus::DSt;
        legStateNext = firstleg;
    }
    else if (motionState == DataBus::Walk)
    {
        enableNextStep = true;
        dPhi = 1.0 / std::max(0.05, tSwing) * dt;
    }
    else if (motionState == DataBus::Walk2Stand)
    {
        if (legState != DataBus::DSt)
        {
            dPhi = 1.0 / std::max(0.05, tSwing) * dt;
        }
    }

    phi += dPhi;
    if (enableNextStep && legState != DataBus::DSt)
        touchDown = false;

    if (!isIni && start_walk)
    {
        isIni = true;
        legState = firstleg;
        if (legState == DataBus::LSt)
        {
            swingStartPos_W = fe_r_pos_W;
            stanceStartPos_W = fe_l_pos_W;
        }
        else
        {
            swingStartPos_W = fe_l_pos_W;
            stanceStartPos_W = fe_r_pos_W;
        }
    }

    if (enableNextStep)
    {
        if (legState == DataBus::LSt && FRest[2] >= fzSwitchThreshold && phi >= phiSwitchMinRuntime)
        {
            legState = DataBus::RSt;
            swingStartPos_W = fe_l_pos_W;
            stanceStartPos_W = fe_r_pos_W;
            phi = 0.0;
            stepNumCur++;
        }
        else if (legState == DataBus::RSt && FLest[2] >= fzSwitchThreshold && phi >= phiSwitchMinRuntime)
        {
            legState = DataBus::LSt;
            swingStartPos_W = fe_r_pos_W;
            stanceStartPos_W = fe_l_pos_W;
            phi = 0.0;
            stepNumCur++;
        }
    }

    if (!enableNextStep)
    {
        if (legState == DataBus::LSt && FRest[2] >= fzStopThreshold && phi >= phiSwitchMinRuntime)
        {
            touchDown = true;
            stepNumCur++;
            legState = DataBus::DSt;
            phi = 0.0;
        }
        if (legState == DataBus::RSt && FLest[2] >= fzStopThreshold && phi >= phiSwitchMinRuntime)
        {
            touchDown = true;
            stepNumCur++;
            legState = DataBus::DSt;
            phi = 0.0;
        }
        if (legState == DataBus::DSt)
        {
            touchDown = true;
            phi = 0.0;
        }
    }

    if (phi >= 1.0)
    {
        phi = 1.0;
    }
    if (legState == DataBus::DSt)
    {
        phi = 0.0;
    }

    if (legState == DataBus::LSt)
    {
        posHip_W = hip_r_pos_W;
        posST_W = fe_l_pos_W;
        theta0 = -3.1415 * 0.5;
        if (motionState == DataBus::Walk)
            legStateNext = DataBus::RSt;
        else if (motionState == DataBus::Walk2Stand)
            legStateNext = DataBus::DSt;
    }
    else if (legState == DataBus::RSt)
    {
        posHip_W = hip_l_pos_W;
        posST_W = fe_r_pos_W;
        theta0 = 3.1415 * 0.5;
        if (motionState == DataBus::Walk)
            legStateNext = DataBus::LSt;
        else if (motionState == DataBus::Walk2Stand)
            legStateNext = DataBus::DSt;
    }
    else
    {
        if (firstleg == DataBus::LSt)
        {
            posHip_W = hip_r_pos_W;
            posST_W = fe_l_pos_W;
            theta0 = -3.1415 * 0.5;
            legStateNext = DataBus::LSt;
        }
        else
        {
            posHip_W = hip_l_pos_W;
            posST_W = fe_r_pos_W;
            theta0 = 3.1415 * 0.5;
            legStateNext = DataBus::RSt;
        }
        if (!enableNextStep)
        {
            legStateNext = DataBus::DSt;
        }
    }
}

void GaitScheduler::start()
{
    start_walk = true;
}
