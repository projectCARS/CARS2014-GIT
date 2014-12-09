#include "headers.h"
#include "EKF.h"
#include "MotionModel.h"
#include "STModel.h"
#include <Eigen/Dense>
#include <random>

using Eigen::MatrixXd;
using Eigen::VectorXd;

STModel::STModel()
{
    m_numStates = 6;
    m_numMeasurements = 3;
    m_numControlSignals = 2;

    f = MatrixXd::Zero(m_numStates, 1);
    F = MatrixXd::Zero(m_numStates, m_numStates);
    H = MatrixXd::Zero(m_numMeasurements, m_numStates);

    R = MatrixXd::Zero(m_numMeasurements, m_numMeasurements);
    Q = MatrixXd::Zero(m_numStates, m_numStates);

    matrixSetup();
}


STModel::~STModel()
{
}

void STModel::matrixSetup()
{
    F = MatrixXd::Identity(m_numStates, m_numStates);

    // CHANGE H
    H << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 0, 1, 0;

    //Design parameters
    R(0, 0) = 0.005;
    R(1, 1) = 0.005;
    R(2, 2) = 0.001;

    Q(0, 0) = 1;
    Q(1, 1) = 1;
    Q(2, 2) = 100;
    Q(3, 3) = 1;
    Q(4, 4) = 60;
    Q(5, 5) = 80;

    m = 0.0406;
    Cm1 = 0.6148;
    Cm2 = 0.0477;
    Cm3 = 0.0357;
    Cf = 0.4835;
    lf = 0.003;
    Iz = 0.000016344;
    kSteer = 0.4549;
    mSteer = -0.8623;
    kThrottle = -2.259;
    mThrottle = 3.217;

}

void STModel::updateModel(VectorXd xhat, double T)
{
    double X = xhat(0);
    double Y = xhat(1);
    double Vx = xhat(2);
    double h = xhat(3);
    double w = xhat(4);
    double Vy = xhat(5);

    double dutyCycles = calcDutycycles();
    double thetaF = calcThetaF();

    double alphaF = calcAlphaF(Vx, Vy, h, thetaF);
    double FLat = calcLatForce(alphaF);

    // System Dynamics
    f(0) = X + (cos(h)*Vx + sin(h)*Vy)*T;
    f(1) = Y + (sin(h)*Vx + cos(h)*Vy)*T;
    f(2) = (dutyCycles * (Cm1 - Cm2*Vx) + thetaF * (FLat - Cm3 * FLat) - Cm3 * Vy * w) / m - Vy*w;
    f(3) = h + w*T;
    f(4) = w + T*FLat*lf/Iz;
    f(5) = FLat/m + Vx*w;


    double dFdVx = Cf*((w + Vy)/((w + Vy)*(w + Vy) + Vx*Vx));
    double dFdVy = -Cf*((Vx)/((w + Vy)*(w + Vy) + Vx*Vx));
    double dFdw = dFdVy;


    // System jacobian
    F(0, 0) = 1;
    F(0, 2) = cos(h)*T;
    F(0, 3) = T*(cos(h)*Vy -sin(h)*Vx);
    F(0, 5) = sin(h)*T;

    F(1, 1) = 1;
    F(1, 2) = sin(h)*T;
    F(1, 3) = T*(cos(h)*Vx -sin(h)*Vy);
    F(1, 5) = cos(h)*T;

    F(2, 2) = (-dutyCycles*Cm2 + thetaF*(dFdVx - Cm3*dFdVx))/m;
    F(2, 4) = (thetaF*(dFdw - Cm3*dFdw) -Cm3*Vy)/m - Vy;
    F(2, 5) = (thetaF*(dFdVy - Cm3*dFdVy) -Cm3*w)/m - w;

    F(3, 3) = 1;
    F(3, 4) = T;

    F(4, 2) = T*lf/Iz * dFdVx;
    F(4, 4) = T*lf/Iz * dFdw + 1;
    F(4, 5) = T*lf/Iz * dFdVy;

    F(5, 2) = w + dFdVx/m;
    F(5, 4) = Vx + dFdw/m;
    F(5, 5) = dFdVy/m;

    //TODO: Add dynamics for G and H...
}

void STModel::addInput(float u_gas, float u_turn)
{
    m_gas = u_gas;
    m_turn = u_turn;
}

float STModel::calcDutycycles()
{
    return kThrottle*m_gas + mThrottle;
}

float STModel::calcThetaF()
{
    return kSteer*m_turn + mSteer;
}

float STModel::calcAlphaF(float Vx, float Vy, float omegaZ, float thetaF)
{
    return atan((Vy + lf*omegaZ)/Vx) - thetaF;
}

float STModel::calcLatForce(float alphaF)
{
    return -Cf*alphaF;
}
