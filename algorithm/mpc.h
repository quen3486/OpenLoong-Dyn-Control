/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://gitee.com/panda_23/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "data_bus.h"
#include "qpOASES.hpp"

const uint16_t  mpc_N_max = 20;
const uint16_t  ch_max = 10;
const uint16_t  mpc_N_default = 10;
const uint16_t  ch_default = 3;
const uint16_t  nx = 12;
const uint16_t  nu = 13;

const uint16_t  ncfr_single = 4;
const uint16_t  ncfr = ncfr_single*2;

const uint16_t  ncstxya = 1;
const uint16_t  ncstxy_single = ncstxya*4;
const uint16_t  ncstxy = ncstxy_single*2;

const uint16_t  ncstza = 2;
const uint16_t  ncstz_single = ncstza*4;
const uint16_t  ncstz = ncstz_single*2;
const uint16_t  nc = ncfr + ncstxy + ncstz;

class MPC{
public:
    MPC(double dtIn);

    void    set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag);
    void    cal();
    void    dataBusRead(DataBus &Data);
    void    dataBusWrite(DataBus &Data);
    void    setRobotMass(double massIn);
    void    setFrictionCoeff(double muIn);
    bool    setBodyInertia(const Eigen::Matrix3d &inertiaIn);
    void    setUseDataBusInertia(bool enable);
    void    setFootSupportPolygon(double xFrontIn, double xRearIn, double yLeftIn, double yRightIn);
    void    setWrenchLimits(double forceXYMaxIn, double fzMaxScaleIn,
                            double torqueXMaxIn, double torqueYMaxIn, double torqueZMaxIn);
    void    setHorizon(int predHorizonIn, int ctrlHorizonIn);
    int     getPredictionHorizon() const;
    int     getControlHorizon() const;
    double  getRobotMass() const;

    void    enable();
    void    disable();
    bool    get_ENA();

private:
    void    copy_Eigen_to_real_t(qpOASES::real_t* target, Eigen::MatrixXd source, int nRows, int nCols);

    bool    EN = false;

    //single rigid body model
    Eigen::Matrix<double,nx,nx>   Ac[mpc_N_max], A[mpc_N_max];
    Eigen::Matrix<double,nx,nu>   Bc[mpc_N_max], B[mpc_N_max];
    Eigen::Matrix<double,nx,1>    Cc, C;

    Eigen::MatrixXd   Aqp;
    Eigen::MatrixXd   Aqp1;
    Eigen::MatrixXd   Bqp1;
    Eigen::MatrixXd   Bqp;
    Eigen::VectorXd   Cqp1;
    Eigen::VectorXd   Cqp;

    Eigen::VectorXd           Ufe;
    Eigen::Matrix<double,nu,1>              Ufe_pre;
    Eigen::VectorXd        Xd;
    Eigen::Matrix<double,nx,1>              X_cur;
    Eigen::Matrix<double,nx,1>              X_cal;
    Eigen::Matrix<double,nx,1>              X_cal_pre;
    Eigen::Matrix<double,nx,1>              dX_cal;

    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic>    L;
    Eigen::MatrixXd            K, M;
    double alpha;
    Eigen::MatrixXd          H;
    Eigen::VectorXd              c;

    Eigen::VectorXd               u_low, u_up;
    Eigen::MatrixXd          As;
    Eigen::VectorXd               bs;
    double      max[6], min[6];

    double m, g, miu, delta_foot[4];
    double fzMaxScale;
    bool useDataBusInertia;
    Eigen::Matrix<double,3,1>   pCoM;
    Eigen::Matrix<double,6,1>   pf2com, pf2comd, pe;
    Eigen::Matrix<double,6,1>   pf2comi[mpc_N_max];
    Eigen::Matrix<double,3,3>   Ic;
    Eigen::Matrix<double,3,3>   R_curz[mpc_N_max];
    Eigen::Matrix<double,3,3>   R_cur;
    Eigen::Matrix<double,3,3>   R_w2f, R_f2w;

    int legStateCur;
    int legStateNext;
    int legState[mpc_N_max];
    int predHorizon;
    int ctrlHorizon;
    double  dt;

    //qpOASES
    qpOASES::QProblem QP;
    qpOASES::real_t qp_H[nu*ch_max * nu*ch_max];
    qpOASES::real_t qp_As[nc*ch_max * nu*ch_max];
    qpOASES::real_t qp_c[nu*ch_max];
    qpOASES::real_t qp_lbA[nc*ch_max];
    qpOASES::real_t qp_ubA[nc*ch_max];
    qpOASES::real_t qp_lu[nu*ch_max];
    qpOASES::real_t qp_uu[nu*ch_max];
    qpOASES::int_t nWSR=100;
    qpOASES::real_t cpu_time=0.1;
    qpOASES::real_t xOpt_iniGuess[nu*ch_max];

	double			qp_cpuTime;
    int 			qp_Status, qp_nWSR;
};
