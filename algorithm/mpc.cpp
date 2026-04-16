/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://gitee.com/panda_23/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "mpc.h"
#include "useful_math.h"
#include <algorithm>
#include <vector>
#include <Eigen/Eigenvalues>

namespace
{
double clampMin(double v, double lo)
{
    return (v < lo) ? lo : v;
}

bool isValidInertia(const Eigen::Matrix3d &inertiaIn)
{
    if (!inertiaIn.allFinite())
    {
        return false;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(inertiaIn);
    if (solver.info() != Eigen::Success)
    {
        return false;
    }
    return solver.eigenvalues().minCoeff() > 1e-6;
}
}

MPC::MPC(double dtIn) : QP(nu * ch_max, nc * ch_max)
{
    // Runtime values are expected to be configured by demo (gait profile) or DataBus.
    m = 1.0;
    g = -9.8;
    miu = 0.7;
    fzMaxScale = 3.0;
    useDataBusInertia = true;
    Ic.setIdentity();
    predHorizon = mpc_N_default;
    ctrlHorizon = ch_default;

    delta_foot[0] = 0.073;
    delta_foot[1] = 0.125;
    delta_foot[2] = 0.025;
    delta_foot[3] = 0.025;

    max[0] = 1000.0;
    max[1] = 1000.0;
    max[2] = -fzMaxScale * m * g;
    max[3] = 20.0;
    max[4] = 80.0;
    max[5] = 100.0;

    min[0] = -1000.0;
    min[1] = -1000.0;
    min[2] = 0.0;
    min[3] = -20.0;
    min[4] = -80.0;
    min[5] = -100.0;

    // single rigid body model
    for (int i = 0; i < (mpc_N_max); i++)
    {
        Ac[i].setZero();
        Bc[i].setZero();
        A[i].setZero();
        B[i].setZero();
        R_curz[i].setIdentity();
        legState[i] = DataBus::DSt;
    }
    Cc.setZero();
    C.setZero();

    Aqp = Eigen::MatrixXd::Zero(nx * mpc_N_max, nx);
    Aqp1 = Eigen::MatrixXd::Zero(nx * mpc_N_max, nx * mpc_N_max);
    Bqp1 = Eigen::MatrixXd::Zero(nx * mpc_N_max, nu * mpc_N_max);
    Bqp = Eigen::MatrixXd::Zero(nx * mpc_N_max, nu * ch_max);
    Cqp1 = Eigen::VectorXd::Zero(nx * mpc_N_max);
    Cqp = Eigen::VectorXd::Zero(nx * mpc_N_max);

    Ufe = Eigen::VectorXd::Zero(nu * ch_max);
    Ufe_pre.setZero();

    Xd = Eigen::VectorXd::Zero(nx * mpc_N_max);
    X_cur.setZero();
    X_cal.setZero();
    dX_cal.setZero();

    L = Eigen::MatrixXd::Zero(nx * mpc_N_max, nx * mpc_N_max);
    K = Eigen::MatrixXd::Zero(nu * ch_max, nu * ch_max);
    M = Eigen::MatrixXd::Zero(nu * ch_max, nu * ch_max);
    alpha = 0.0;
    H = Eigen::MatrixXd::Zero(nu * ch_max, nu * ch_max);
    c = Eigen::VectorXd::Zero(nu * ch_max);

    u_low = Eigen::VectorXd::Zero(nu * ch_max);
    u_up = Eigen::VectorXd::Zero(nu * ch_max);
    As = Eigen::MatrixXd::Zero(nc * ch_max, nu * ch_max);
    bs = Eigen::VectorXd::Zero(nc * ch_max);

    pCoM.setZero();
    pf2com.setZero();
    pe.setZero();
    R_cur.setIdentity();
    R_w2f.setIdentity();
    R_f2w.setIdentity();

    qpOASES::Options option;
    option.printLevel = qpOASES::PL_LOW;
    QP.setOptions(option);

    dt = dtIn;
}

void MPC::setRobotMass(double massIn)
{
    m = clampMin(massIn, 1.0);
    max[2] = -fzMaxScale * m * g;
}

void MPC::setFrictionCoeff(double muIn)
{
    miu = std::clamp(muIn, 0.01, 2.0);
}

bool MPC::setBodyInertia(const Eigen::Matrix3d &inertiaIn)
{
    Eigen::Matrix3d inertiaSym = 0.5 * (inertiaIn + inertiaIn.transpose());
    if (!isValidInertia(inertiaSym))
    {
        return false;
    }
    Ic = inertiaSym;
    return true;
}

void MPC::setUseDataBusInertia(bool enable)
{
    useDataBusInertia = enable;
}

void MPC::setFootSupportPolygon(double xFrontIn, double xRearIn, double yLeftIn, double yRightIn)
{
    delta_foot[0] = clampMin(xFrontIn, 1e-4);
    delta_foot[1] = clampMin(xRearIn, 1e-4);
    delta_foot[2] = clampMin(yLeftIn, 1e-4);
    delta_foot[3] = clampMin(yRightIn, 1e-4);
}

void MPC::setWrenchLimits(double forceXYMaxIn, double fzMaxScaleIn,
                          double torqueXMaxIn, double torqueYMaxIn, double torqueZMaxIn)
{
    const double forceXYMax = clampMin(forceXYMaxIn, 1.0);
    const double torqueXMax = clampMin(torqueXMaxIn, 0.1);
    const double torqueYMax = clampMin(torqueYMaxIn, 0.1);
    const double torqueZMax = clampMin(torqueZMaxIn, 0.1);

    fzMaxScale = clampMin(fzMaxScaleIn, 0.1);

    max[0] = forceXYMax;
    max[1] = forceXYMax;
    max[2] = -fzMaxScale * m * g;
    max[3] = torqueXMax;
    max[4] = torqueYMax;
    max[5] = torqueZMax;

    min[0] = -forceXYMax;
    min[1] = -forceXYMax;
    min[2] = 0.0;
    min[3] = -torqueXMax;
    min[4] = -torqueYMax;
    min[5] = -torqueZMax;
}

void MPC::setHorizon(int predHorizonIn, int ctrlHorizonIn)
{
    predHorizon = std::clamp(predHorizonIn, 1, static_cast<int>(mpc_N_max));
    ctrlHorizon = std::clamp(ctrlHorizonIn, 1, static_cast<int>(ch_max));
    if (ctrlHorizon > predHorizon)
    {
        ctrlHorizon = predHorizon;
    }
}

int MPC::getPredictionHorizon() const
{
    return predHorizon;
}

int MPC::getControlHorizon() const
{
    return ctrlHorizon;
}

double MPC::getRobotMass() const
{
    return m;
}

void MPC::set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag)
{
    const int N = predHorizon;
    const int C = ctrlHorizon;
    const int nxN = nx * N;
    const int nuC = nu * C;

    L.setZero();
    K.setZero();
    alpha = u_weight;

    Eigen::VectorXd L_diag_N(nxN);
    Eigen::VectorXd K_diag_N(nuC);
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < nx; j++)
        {
            L_diag_N(i * nx + j) = L_diag(0, j);
        }
    }
    for (int i = 0; i < C; i++)
    {
        for (int j = 0; j < nu; j++)
        {
            K_diag_N(i * nu + j) = K_diag(0, j);
        }
    }

    for (int i = 0; i < nxN; i++)
    {
        L(i, i) = L_diag_N(i);
    }
    for (int i = 0; i < nuC; i++)
    {
        K(i, i) = K_diag_N(i);
    }

    for (int i = 0; i < N; i++)
    {
        L.block(i * nx + 3, i * nx + 3, 3, 3) = R_curz[i] * L.block(i * nx + 3, i * nx + 3, 3, 3) * R_curz[i].transpose();
        L.block(i * nx + 6, i * nx + 6, 3, 3) = R_curz[i] * L.block(i * nx + 6, i * nx + 6, 3, 3) * R_curz[i].transpose();
        L.block(i * nx + 9, i * nx + 9, 3, 3) = R_curz[i] * L.block(i * nx + 9, i * nx + 9, 3, 3) * R_curz[i].transpose();
    }

    for (int i = 0; i < C; i++)
    {
        K.block(i * nu, i * nu, 3, 3) = R_curz[i] * K.block(i * nu, i * nu, 3, 3) * R_curz[i].transpose();
        K.block(i * nu + 3, i * nu + 3, 3, 3) = R_curz[i] * K.block(i * nu + 3, i * nu + 3, 3, 3) * R_curz[i].transpose();
        K.block(i * nu + 6, i * nu + 6, 3, 3) = R_curz[i] * K.block(i * nu + 6, i * nu + 6, 3, 3) * R_curz[i].transpose();
        K.block(i * nu + 9, i * nu + 9, 3, 3) = R_curz[i] * K.block(i * nu + 9, i * nu + 9, 3, 3) * R_curz[i].transpose();
    }
}

void MPC::dataBusRead(DataBus &Data)
{
    // set value
    X_cur.block<3, 1>(0, 0) = Data.base_rpy;
    X_cur.block<3, 1>(3, 0) = Data.q.block<3, 1>(0, 0);
    X_cur.block<3, 1>(6, 0) = Data.dq.block<3, 1>(3, 0);
    X_cur.block<3, 1>(9, 0) = Data.dq.block<3, 1>(0, 0);

    const int N = predHorizon;
    if (EN)
    {
        for (int i = 0; i < (N - 1); i++)
            Xd.block<nx, 1>(nx * i, 0) = Xd.block<nx, 1>(nx * (i + 1), 0);
        for (int j = 0; j < 3; j++)
            Xd(nx * (N - 1) + j) = Data.js_eul_des(j);
        for (int j = 0; j < 3; j++)
            Xd(nx * (N - 1) + 3 + j) = Data.js_pos_des(j);
        for (int j = 0; j < 3; j++)
            Xd(nx * (N - 1) + 6 + j) = Data.js_omega_des(j);
        for (int j = 0; j < 3; j++)
            Xd(nx * (N - 1) + 9 + j) = Data.js_vel_des(j);
    }
    else
    {
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < 3; j++)
                Xd(nx * i + j) = X_cur(j);
            for (int j = 0; j < 3; j++)
                Xd(nx * i + 3 + j) = X_cur(3 + j);
            for (int j = 0; j < 3; j++)
                Xd(nx * i + 6 + j) = X_cur(6 + j);
            for (int j = 0; j < 3; j++)
                Xd(nx * i + 9 + j) = X_cur(9 + j);
        }
    }

    R_cur = eul2Rot(X_cur(0), X_cur(1), X_cur(2));
    for (int i = 0; i < mpc_N_max; i++)
    {
        R_curz[i] = Rz3(X_cur(2));
    }
    pCoM = X_cur.block<3, 1>(3, 0);
    pe.block<3, 1>(0, 0) = Data.fe_l_pos_W;
    pe.block<3, 1>(3, 0) = Data.fe_r_pos_W;

    pf2com.block<3, 1>(0, 0) = pe.block<3, 1>(0, 0) - pCoM;
    pf2com.block<3, 1>(3, 0) = pe.block<3, 1>(3, 0) - pCoM;
    pf2comd.block<3, 1>(0, 0) = pe.block<3, 1>(0, 0) - Xd.block<3, 1>(3, 0);
    pf2comd.block<3, 1>(3, 0) = pe.block<3, 1>(3, 0) - Xd.block<3, 1>(3, 0);

    if (useDataBusInertia)
    {
        Eigen::Matrix3d inertiaSym = 0.5 * (Data.inertia + Data.inertia.transpose());
        if (isValidInertia(inertiaSym))
        {
            Ic = inertiaSym;
        }
    }

    legStateCur = Data.legState;
    legStateNext = Data.legStateNext;

    const double tSwingRef = std::max(0.05, Data.tSwing);
    for (int i = 0; i < N; i++)
    {
        const double aa = i * dt / tSwingRef;
        const double phip = Data.phi + aa;

        if (phip > 1.0)
            legState[i] = legStateNext;
        else
            legState[i] = legStateCur;
    }
    for (int i = N; i < mpc_N_max; i++)
    {
        legState[i] = legState[N - 1];
    }

    Eigen::Matrix<double, 3, 3> R_slop;
    R_slop = eul2Rot(Data.slop(0), Data.slop(1), Data.slop(2));
    if (legStateCur == DataBus::RSt)
        R_f2w = Data.fe_r_rot_W;
    else if (legStateCur == DataBus::LSt)
        R_f2w = Data.fe_l_rot_W;
    else
        R_f2w = R_slop;
    R_w2f = R_f2w.transpose();
}

void MPC::cal()
{
    const int N = predHorizon;
    const int C = ctrlHorizon;
    const int nxN = nx * N;
    const int nuN = nu * N;
    const int nuC = nu * C;
    const int ncC = nc * C;

    if (EN)
    {
        Aqp.setZero();
        Aqp1.setZero();
        Bqp1.setZero();
        Bqp.setZero();
        As.setZero();

        // qp pre
        for (int i = 0; i < N; i++)
        {
            Ac[i].setZero();
            Bc[i].setZero();
            Ac[i].block<3, 3>(0, 6) = R_curz[i].transpose();
            Ac[i].block<3, 3>(3, 9) = Eigen::MatrixXd::Identity(3, 3);
            A[i] = Eigen::MatrixXd::Identity(nx, nx) + dt * Ac[i];
        }
        for (int i = 0; i < N; i++)
        {
            pf2comi[i] = pf2com;
            Eigen::Matrix3d Ic_W_inv;
            Ic_W_inv = (R_curz[i] * Ic * R_curz[i].transpose()).inverse();
            Bc[i].block<3, 3>(6, 0) = Ic_W_inv * CrossProduct_A(pf2comi[i].block<3, 1>(0, 0));
            Bc[i].block<3, 3>(6, 3) = Ic_W_inv;
            Bc[i].block<3, 3>(6, 6) = Ic_W_inv * CrossProduct_A(pf2comi[i].block<3, 1>(3, 0));
            Bc[i].block<3, 3>(6, 9) = Ic_W_inv;
            Bc[i].block<3, 3>(9, 0) = Eigen::MatrixXd::Identity(3, 3) / m;
            Bc[i].block<3, 3>(9, 6) = Eigen::MatrixXd::Identity(3, 3) / m;
            Bc[i]((nx - 1), (nu - 1)) = 1.0 / m;
            B[i] = dt * Bc[i];
        }
        for (int i = 0; i < N; i++)
            Aqp.block(i * nx, 0, nx, nx) = Eigen::MatrixXd::Identity(nx, nx);
        for (int i = 0; i < N; i++)
            for (int j = 0; j < i + 1; j++)
                Aqp.block(i * nx, 0, nx, nx) = A[j] * Aqp.block(i * nx, 0, nx, nx);

        for (int i = 0; i < N; i++)
            for (int j = 0; j < i + 1; j++)
                Aqp1.block(i * nx, j * nx, nx, nx) = Eigen::MatrixXd::Identity(nx, nx);
        for (int i = 1; i < N; i++)
            for (int j = 0; j < i; j++)
                for (int k = j + 1; k < (i + 1); k++)
                    Aqp1.block(i * nx, j * nx, nx, nx) = A[k] * Aqp1.block(i * nx, j * nx, nx, nx);

        for (int i = 0; i < N; i++)
            Bqp1.block(i * nx, i * nu, nx, nu) = B[i];

        Eigen::MatrixXd Bqp11 = Eigen::MatrixXd::Zero(nuN, nuC);
        Bqp11.block(0, 0, nuC, nuC) = Eigen::MatrixXd::Identity(nuC, nuC);
        for (int i = 0; i < (N - C); i++)
            Bqp11.block(nuC + i * nu, nu * (C - 1), nu, nu) = Eigen::MatrixXd::Identity(nu, nu);

        Eigen::MatrixXd B_tmp = Eigen::MatrixXd::Zero(nxN, nuC);
        B_tmp = Bqp1.block(0, 0, nxN, nuN) * Bqp11;
        Bqp.block(0, 0, nxN, nuC) = Aqp1.block(0, 0, nxN, nxN) * B_tmp;

        Eigen::VectorXd delta_U = Eigen::VectorXd::Zero(nuC);
        for (int i = 0; i < C; i++)
        {
            if (legState[i] == DataBus::LSt)
                delta_U(nu * i + 2) = m * g;
            else if (legState[i] == DataBus::RSt)
                delta_U(nu * i + 8) = m * g;
            else
            {
                delta_U(nu * i + 2) = 0.5 * m * g;
                delta_U(nu * i + 8) = 0.5 * m * g;
            }
        }

        const Eigen::MatrixXd L_act = L.block(0, 0, nxN, nxN);
        const Eigen::MatrixXd K_act = K.block(0, 0, nuC, nuC);
        const Eigen::MatrixXd Aqp_act = Aqp.block(0, 0, nxN, nx);
        const Eigen::MatrixXd Bqp_act = Bqp.block(0, 0, nxN, nuC);
        const Eigen::VectorXd Xd_act = Xd.block(0, 0, nxN, 1);

        Eigen::MatrixXd H_act = 2.0 * (Bqp_act.transpose() * L_act * Bqp_act + alpha * K_act) +
                                1e-10 * Eigen::MatrixXd::Identity(nuC, nuC);
        Eigen::VectorXd c_act = 2.0 * Bqp_act.transpose() * L_act * (Aqp_act * X_cur - Xd_act) +
                                2.0 * alpha * K_act * delta_U;

        // friction constraint
        Eigen::Matrix<double, ncfr_single, 3> Asfr111, Asfr11;
        Eigen::Matrix<double, ncfr, nu> Asfr1;
        Eigen::MatrixXd Asfr = Eigen::MatrixXd::Zero(ncfr * C, nuC);
        Asfr111.setZero();
        Asfr1.setZero();
        Asfr111 << -1.0, 0.0, -1.0 / sqrt(2.0) * miu,
            1.0, 0.0, -1.0 / sqrt(2.0) * miu,
            0.0, -1.0, -1.0 / sqrt(2.0) * miu,
            0.0, 1.0, -1.0 / sqrt(2.0) * miu;
        Asfr11 = Asfr111 * R_w2f;
        Asfr1.block<ncfr_single, 3>(0, 0) = Asfr11;
        Asfr1.block<ncfr_single, 3>(ncfr_single, 6) = Asfr11;

        for (int i = 0; i < C; i++)
            Asfr.block(ncfr * i, i * nu, ncfr, nu) = Asfr1;

        // moment constraint x y
        double sign_xy[4]{1.0, -1.0, -1.0, 1.0};
        Eigen::Matrix<double, 3, 1> gxyz[4];
        gxyz[0] << 0.0, 1.0, 0.0;
        gxyz[1] << 0.0, 1.0, 0.0;
        gxyz[2] << 1.0, 0.0, 0.0;
        gxyz[3] << 1.0, 0.0, 0.0;
        Eigen::Matrix<double, 3, 1> r[4];
        Eigen::Matrix<double, 3, 1> p[4];
        Eigen::Matrix<double, ncstxya, 6> Astxy_r[4];
        Eigen::Matrix<double, ncstxy_single, 6> Astxy11;
        Eigen::Matrix<double, ncstxy, nu> Astxy1;
        Eigen::MatrixXd Astxy = Eigen::MatrixXd::Zero(ncstxy * C, nuC);
        Astxy_r[0].setZero();
        Astxy_r[1].setZero();
        Astxy_r[2].setZero();
        Astxy_r[3].setZero();
        Astxy11.setZero();
        Astxy1.setZero();

        r[0] << 0.0, 1.0, 0.0;
        r[1] << 0.0, 1.0, 0.0;
        r[2] << 1.0, 0.0, 0.0;
        r[3] << 1.0, 0.0, 0.0;

        p[0] << delta_foot[0], 0.0, 0.0;
        p[1] << -delta_foot[1], 0.0, 0.0;
        p[2] << 0.0, delta_foot[2], 0.0;
        p[3] << 0.0, -delta_foot[3], 0.0;

        for (int i = 0; i < 4; i++)
        {
            Astxy_r[i].block<1, 3>(0, 0) =
                sign_xy[i] * gxyz[i].transpose() * R_w2f * R_f2w * r[i] * (R_f2w * r[i]).transpose() *
                CrossProduct_A(R_f2w * p[i]);
            Astxy_r[i].block<1, 3>(0, 3) = sign_xy[i] * gxyz[i].transpose() * R_w2f;
            Astxy11.block<ncstxya, 6>(i * ncstxya, 0) = Astxy_r[i];
        }
        Astxy1.block<ncstxy_single, 6>(0, 0) = Astxy11;
        Astxy1.block<ncstxy_single, 6>(ncstxy_single, 6) = Astxy11;
        for (int i = 0; i < C; i++)
            Astxy.block(ncstxy * i, nu * i, ncstxy, nu) = Astxy1;

        // moment constraint z
        Eigen::Matrix<double, ncstza, 6> Astz_r[4];
        Eigen::Matrix<double, ncstz_single, 6> Astz11;
        Eigen::Matrix<double, ncstz, nu> Astz1;
        Eigen::MatrixXd Astz = Eigen::MatrixXd::Zero(ncstz * C, nuC);
        Astz_r[0].setZero();
        Astz_r[1].setZero();
        Astz_r[2].setZero();
        Astz_r[3].setZero();
        Astz11.setZero();
        Astz1.setZero();

        for (int i = 0; i < 4; i++)
        {
            Astz_r[i].block<1, 3>(0, 0) = -sqrt(p[i](0) * p[i](0) + p[i](1) * p[i](1) + p[i](2) * p[i](2)) * miu *
                                          Eigen::Matrix<double, 1, 3>(0.0, 0.0, 1.0) * R_w2f;
            Astz_r[i].block<1, 3>(0, 3) = Eigen::Matrix<double, 1, 3>(0.0, 0.0, 1.0) * R_w2f;
            Astz_r[i].block<1, 3>(1, 0) = Astz_r[i].block<1, 3>(0, 0);
            Astz_r[i].block<1, 3>(1, 3) = -1 * Astz_r[i].block<1, 3>(0, 3);
            Astz11.block<ncstza, 6>(i * ncstza, 0) = Astz_r[i];
        }
        Astz1.block<ncstz_single, 6>(0, 0) = Astz11;
        Astz1.block<ncstz_single, 6>(ncstz_single, 6) = Astz11;
        for (int i = 0; i < C; i++)
            Astz.block(ncstz * i, nu * i, ncstz, nu) = Astz1;

        Eigen::MatrixXd As_act = Eigen::MatrixXd::Zero(ncC, nuC);
        As_act.block(0, 0, ncfr * C, nuC) = Asfr;
        As_act.block(ncfr * C, 0, ncstxy * C, nuC) = Astxy;
        As_act.block(ncfr * C + ncstxy * C, 0, ncstz * C, nuC) = Astz;

        Eigen::VectorXd Guess_value = Eigen::VectorXd::Zero(nuC);
        Eigen::VectorXd u_low_act = Eigen::VectorXd::Zero(nuC);
        Eigen::VectorXd u_up_act = Eigen::VectorXd::Zero(nuC);

        for (int i = 0; i < C; i++)
        {
            if (legState[i] == DataBus::DSt)
            {
                Guess_value(i * nu + 2) = -0.5 * m * g;
                Guess_value(i * nu + 8) = -0.5 * m * g;
                Guess_value(i * nu + 12) = m * g;
                for (int j = 0; j < 6; j++)
                {
                    u_low_act(i * nu + j) = min[j];
                    u_low_act(i * nu + j + 6) = min[j];
                    u_up_act(i * nu + j) = max[j];
                    u_up_act(i * nu + j + 6) = max[j];
                }
                u_low_act(i * nu + 12) = m * g;
                u_up_act(i * nu + 12) = m * g;
            }
            else if (legState[i] == DataBus::LSt)
            {
                Guess_value(i * nu + 2) = -m * g;
                Guess_value(i * nu + 8) = 0.0;
                Guess_value(i * nu + 12) = m * g;
                for (int j = 0; j < 6; j++)
                {
                    u_low_act(i * nu + j) = min[j];
                    u_low_act(i * nu + j + nu / 2) = 0.0;
                    u_up_act(i * nu + j) = max[j];
                    u_up_act(i * nu + j + nu / 2) = 0.0;
                }

                u_low_act(i * nu + 12) = m * g;
                u_up_act(i * nu + 12) = m * g;
            }
            else if (legState[i] == DataBus::RSt)
            {
                Guess_value(i * nu + 2) = 0.0;
                Guess_value(i * nu + 8) = -m * g;
                Guess_value(i * nu + 12) = m * g;
                for (int j = 0; j < 6; j++)
                {
                    u_low_act(i * nu + j) = 0.0;
                    u_low_act(i * nu + j + nu / 2) = min[j];
                    u_up_act(i * nu + j) = 0.0;
                    u_up_act(i * nu + j + nu / 2) = max[j];
                }
                u_low_act(i * nu + 12) = m * g;
                u_up_act(i * nu + 12) = m * g;
            }
        }

        Eigen::VectorXd lbA_act = Eigen::VectorXd::Constant(ncC, -1e7);
        Eigen::VectorXd ubA_act = Eigen::VectorXd::Constant(ncC, 1e7);

        for (int i = 0; i < C; i++)
        {
            if (legState[i] == DataBus::DSt)
            {
                ubA_act.block(ncfr * i, 0, ncfr, 1).setZero();
                ubA_act.block(ncfr * C + ncstxy * i, 0, ncstxy, 1).setZero();
                ubA_act.block(ncfr * C + ncstxy * C + ncstz * i, 0, ncstz, 1).setZero();
            }
            else if (legState[i] == DataBus::LSt)
            {
                ubA_act.block(ncfr * i, 0, ncfr_single, 1).setZero();
                ubA_act.block(ncfr * C + ncstxy * i, 0, ncstxy_single, 1).setZero();
                ubA_act.block(ncfr * C + ncstxy * C + ncstz * i, 0, ncstz_single, 1).setZero();
            }
            else if (legState[i] == DataBus::RSt)
            {
                ubA_act.block(ncfr * i + ncfr_single, 0, ncfr_single, 1).setZero();
                ubA_act.block(ncfr * C + ncstxy * i + ncstxy_single, 0, ncstxy_single, 1).setZero();
                ubA_act.block(ncfr * C + ncstxy * C + ncstz * i + ncstz_single, 0, ncstz_single, 1).setZero();
            }
        }

        // Solve active-size QP directly to avoid padded inactive dimensions.
        qpOASES::QProblem qpAct(nuC, ncC);
        qpOASES::Options option;
        option.printLevel = qpOASES::PL_LOW;
        qpAct.setOptions(option);

        Eigen::MatrixXd H_sym = 0.5 * (H_act + H_act.transpose());
        std::vector<qpOASES::real_t> qpH(static_cast<size_t>(nuC * nuC), 0.0);
        std::vector<qpOASES::real_t> qpAs(static_cast<size_t>(ncC * nuC), 0.0);
        std::vector<qpOASES::real_t> qpc(static_cast<size_t>(nuC), 0.0);
        std::vector<qpOASES::real_t> qplbA(static_cast<size_t>(ncC), 0.0);
        std::vector<qpOASES::real_t> qpubA(static_cast<size_t>(ncC), 0.0);
        std::vector<qpOASES::real_t> qplu(static_cast<size_t>(nuC), 0.0);
        std::vector<qpOASES::real_t> qpuu(static_cast<size_t>(nuC), 0.0);
        std::vector<qpOASES::real_t> xGuess(static_cast<size_t>(nuC), 0.0);

        copy_Eigen_to_real_t(qpH.data(), H_sym, nuC, nuC);
        copy_Eigen_to_real_t(qpc.data(), c_act, nuC, 1);
        copy_Eigen_to_real_t(qpAs.data(), As_act, ncC, nuC);
        copy_Eigen_to_real_t(qplbA.data(), lbA_act, ncC, 1);
        copy_Eigen_to_real_t(qpubA.data(), ubA_act, ncC, 1);
        copy_Eigen_to_real_t(qplu.data(), u_low_act, nuC, 1);
        copy_Eigen_to_real_t(qpuu.data(), u_up_act, nuC, 1);
        copy_Eigen_to_real_t(xGuess.data(), Guess_value, nuC, 1);

        qpOASES::returnValue res;
        nWSR = 1000000;
        cpu_time = dt;
        res = qpAct.init(qpH.data(), qpc.data(), qpAs.data(), qplu.data(), qpuu.data(),
                         qplbA.data(), qpubA.data(), nWSR, &cpu_time, xGuess.data());

        qp_Status = qpOASES::getSimpleStatus(res);
        qp_nWSR = nWSR;
        qp_cpuTime = cpu_time;

        if (res != qpOASES::SUCCESSFUL_RETURN)
        {
            printf("failed!!!!!!!!!!!!!\n");
        }

        std::vector<qpOASES::real_t> xOpt(static_cast<size_t>(nuC), 0.0);
        qpAct.getPrimalSolution(xOpt.data());
        Ufe.setZero();
        if (qp_Status == 0)
        {
            for (int i = 0; i < nuC; i++)
                Ufe(i) = xOpt[static_cast<size_t>(i)];
        }

        dX_cal = Ac[0] * X_cur + Bc[0] * Ufe.block<nu, 1>(0, 0);
        Eigen::Matrix<double, nx, 1> delta_X;
        delta_X.setZero();
        for (int i = 0; i < 3; i++)
        {
            delta_X(i) = 0.5 * dX_cal(i + 6) * dt * dt;
            delta_X(i + 3) = 0.5 * dX_cal(i + 9) * dt * dt;
            delta_X(i + 6) = dX_cal(i + 6) * dt;
            delta_X(i + 9) = dX_cal(i + 9) * dt;
        }

        X_cal = (Aqp_act * X_cur + Bqp_act * Ufe.block(0, 0, nuC, 1)).block<nx, 1>(0, 0) + delta_X;

        Ufe_pre = Ufe.block<nu, 1>(0, 0);
    }
    else
    {
        Ufe.setZero();
        Ufe(2) = -0.5 * m * g;
        Ufe(8) = -0.5 * m * g;
        Ufe(12) = m * g;
        Ufe_pre.setZero();
    }
}

void MPC::dataBusWrite(DataBus &Data)
{
    Data.Xd = Xd.block(0, 0, nx * predHorizon, 1);
    Data.X_cur = X_cur;
    Data.fe_react_tau_cmd = Ufe.block(0, 0, nu, 1);
    Data.X_cal = X_cal;
    Data.dX_cal = dX_cal;

    Data.qp_nWSR_MPC = nWSR;
    Data.qp_cpuTime_MPC = cpu_time;
    Data.qpStatus_MPC = qp_Status;

    Data.Fr_ff = Ufe.block<12, 1>(0, 0);
    Data.mpcPredictionHorizon = predHorizon;
    Data.mpcControlHorizon = ctrlHorizon;

    double k = 5;
    Data.des_ddq.block<2, 1>(0, 0) << dX_cal(9), dX_cal(10);

    Data.des_ddq(5) = k * (Xd(6 + 2) - Data.dq(5));

    Data.des_dq.block<3, 1>(0, 0) << Xd(9 + 0), Xd(9 + 1), Xd(9 + 2);
    Data.des_dq.block<2, 1>(3, 0) << 0.0, 0.0;
    Data.des_dq(5) = Xd(6 + 2);

    Data.des_delta_q.block<2, 1>(0, 0) = Data.des_dq.block<2, 1>(0, 0) * dt;
    Data.des_delta_q(5) = Data.des_dq(5) * dt;

    Data.base_rpy_des << 0.005, 0.00, Xd(2);
    Data.base_pos_des << Xd(3 + 0), Xd(3 + 1), Xd(3 + 2);
}

void MPC::enable()
{
    EN = true;
}
void MPC::disable()
{
    EN = false;
}

bool MPC::get_ENA()
{
    return EN;
}

void MPC::copy_Eigen_to_real_t(qpOASES::real_t *target, Eigen::MatrixXd source, int nRows, int nCols)
{
    int count = 0;

    // Strange Behavior: Eigen matrix matrix(count) is stored by columns (not rows)
    // real_t is stored by rows, same to C array
    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            target[count] = source(i, j);
            count++;
        }
    }
}
