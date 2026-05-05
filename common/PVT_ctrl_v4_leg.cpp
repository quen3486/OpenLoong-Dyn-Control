/*
 * PVT_Ctr_V4_Leg implementation for speedbot_v4 leg-only robot.
 * Adapted from PVT_ctrl_v4.cpp for OpenLoong.
 */
#include "PVT_ctrl_v4_leg.h"

PVT_Ctr_V4_Leg::PVT_Ctr_V4_Leg(double timeStepIn, const char *jsonPath) {
    jointNum=motorName.size();

    tau_out_lpf.assign(jointNum,LPF_Fst());
    motor_vel.assign(jointNum,0);
    motor_pos_cur.assign(jointNum,0);
    motor_pos_des_old.assign(jointNum,0);
    motor_tor_out_link.assign(jointNum,0);
    motor_tor_out_motor.assign(jointNum,0);
    pvt_Kp.assign(jointNum,0);
    pvt_Kd.assign(jointNum,0);
    closedLoopKp.assign(jointNum,0);
    closedLoopKd.assign(jointNum,0);
    maxTor.assign(jointNum,400);
    maxVel.assign(jointNum,50);
    maxPos.assign(jointNum,3.14);
    minPos.assign(jointNum,-3.14);
    PV_enable.assign(jointNum,1);
    gear.assign(jointNum,1.0);

    Json::Reader reader;
    Json::Value root_read;
    std::ifstream in(jsonPath,std::ios::binary);

    reader.parse(in,root_read);
    for (int i=0;i<jointNum;i++){
        const Json::Value &joint = root_read[motorName[i]];
        pvt_Kp[i]=joint["kp"].asDouble();
        pvt_Kd[i]=joint["kd"].asDouble();
        closedLoopKp[i]=joint.isMember("closedLoopKp") && joint["closedLoopKp"].isNumeric()
                            ? joint["closedLoopKp"].asDouble()
                            : pvt_Kp[i];
        closedLoopKd[i]=joint.isMember("closedLoopKd") && joint["closedLoopKd"].isNumeric()
                            ? joint["closedLoopKd"].asDouble()
                            : pvt_Kd[i];
        maxTor[i]=joint["maxTorque"].asDouble();
        maxVel[i]=joint["maxSpeed"].asDouble();
        maxPos[i]=joint["maxPos"].asDouble();
        minPos[i]=joint["minPos"].asDouble();
        double fc=joint["PVT_LPF_Fc"].asDouble();
        gear[i] = joint["gear"].asDouble();
        tau_out_lpf[i].setPara(fc, timeStepIn);
        tau_out_lpf[i].ftOut(0);
    }
}

void PVT_Ctr_V4_Leg::dataBusRead(DataBus &busIn) {
    for (int i=0;i<jointNum;i++)
    {
        motor_pos_cur[i]=busIn.motors_pos_cur[i];
        motor_vel[i]=busIn.motors_vel_cur[i];
    }
    motor_pos_des=busIn.motors_pos_des;
    motor_vel_des=busIn.motors_vel_des;
    motor_tor_des=busIn.motors_tor_des;
}

void PVT_Ctr_V4_Leg::dataBusWrite(DataBus &busIn) {
    busIn.motors_tor_out=motor_tor_out_motor;
}

void PVT_Ctr_V4_Leg::setJointPD(double kp, double kd, const char *jointName) {
    auto it = std::find(motorName.begin(), motorName.end(), jointName);

    int id=-1;
    if (it != motorName.end()) {
        id = std::distance(motorName.begin(), it);
    } else {
        std::cout << jointName << " NOT found!" << std::endl;
    }
    pvt_Kp[id]=kp;
    pvt_Kd[id]=kd;
}

void PVT_Ctr_V4_Leg::applyClosedLoopPD() {
    for (int i=0;i<jointNum;i++)
    {
        pvt_Kp[i]=closedLoopKp[i];
        pvt_Kd[i]=closedLoopKd[i];
    }
}

void PVT_Ctr_V4_Leg::calMotorsPVT() {
    for (int i=0;i<jointNum;i++)
    {
        double tauDes{0};
        tauDes=PV_enable[i]*pvt_Kp[i]*(motor_pos_des[i]-motor_pos_cur[i])+PV_enable[i]*pvt_Kd[i]*(motor_vel_des[i]-motor_vel[i]);
        tauDes=tau_out_lpf[i].ftOut(tauDes)+motor_tor_des[i];
        if (fabs(tauDes)>=fabs(maxTor[i]))
            tauDes= sign(tauDes)*maxTor[i];
        motor_tor_out_motor[i]=tauDes/gear[i];
        motor_tor_out_link[i]=tauDes;
        motor_pos_des_old[i]=motor_pos_des[i];
    }
}

void PVT_Ctr_V4_Leg::calMotorsPVT(double deltaP_Lim) {
    for (int i=0;i<jointNum;i++)
    {
        double tauDes{0};
        double delta=motor_pos_des[i]-motor_pos_des_old[i];
        if (fabs(delta)>= fabs(deltaP_Lim))
            delta=deltaP_Lim * sign(delta);
        double pDes=delta+motor_pos_des_old[i];
        tauDes=PV_enable[i]*pvt_Kp[i]*(pDes-motor_pos_cur[i])+PV_enable[i]*pvt_Kd[i]*(motor_vel_des[i]-motor_vel[i]);
        tauDes=tau_out_lpf[i].ftOut(tauDes)+motor_tor_des[i];
        if (fabs(tauDes)>=fabs(maxTor[i]))
            tauDes= sign(tauDes)*maxTor[i];
        motor_tor_out_motor[i]=tauDes/gear[i];
        motor_tor_out_link[i]=tauDes;
        motor_pos_des_old[i]=pDes;
    }
}

double PVT_Ctr_V4_Leg::sign(double in) {
    if (in>=0)
        return 1.0;
    else
        return -1.0;
}

void PVT_Ctr_V4_Leg::enablePV() {
    PV_enable.assign(jointNum,1);
}

void PVT_Ctr_V4_Leg::enablePV(int jtId) {
    PV_enable[jtId]=1;
}

void PVT_Ctr_V4_Leg::disablePV() {
    PV_enable.assign(jointNum,0);
}

void PVT_Ctr_V4_Leg::disablePV(int jtId) {
    PV_enable[jtId]=0;
}
