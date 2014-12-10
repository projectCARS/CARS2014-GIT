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


    H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0;

    //Design parameters
    R(0, 0) = 0.005;
    R(1, 1) = 0.005;
    R(2, 2) = 0.001;

    // Tuna dessa
    Q(0, 0) = 0.001;
    Q(1, 1) = 0.001;
    Q(2, 2) = 0.001;
    Q(3, 3) = 0.001;
    Q(4, 4) = 0.001;
    Q(5, 5) = 0.001;

    m = 0.0406f;
    Cm1 = 0.6148f;
    Cm2 = 0.0477f;
    Cm3 = 0.0357f;
    Cf = 0.4835f;
    lf = 0.003f;
    Iz = 0.000016344f;
    kSteer = 0.4549f;
    mSteer = -0.8623f;
    kThrottle = -2.259f;
    mThrottle = 3.217f;
    //m_gas = 0;
    //m_turn = 0;

}

void STModel::updateModel(VectorXd xhat, double T)
{
    double X = xhat(0);
    double Y = xhat(1);
    double Vx = xhat(2);
    double h = xhat(3);
    double w = xhat(4);
    double Vy = xhat(5);

    if(T > 0.5)
        T = 0.0067;

    qDebug("x: %f\nY: %f\nVx: %f\nh: %f\nw: %f\nVy: %f\ngas: %f\nturn: %f\nT: %f\n",X,Y,Vx,h,w,Vy,m_gas, m_turn,T);

    double dutyCycles = calcDutycycles();
    double thetaF = calcThetaF();

    double alphaF = calcAlphaF(Vx, Vy, h, thetaF);
    double FLat = calcLatForce(alphaF);


    // System Dynamics
    f(0) = X + (cos(h)*Vx + sin(h)*Vy)*T;
    f(1) = Y + (sin(h)*Vx + cos(h)*Vy)*T;
    f(2) = Vx + ((dutyCycles * (Cm1 - Cm2*Vx) + thetaF * (FLat - Cm3 * FLat) - Cm3 * Vy * w) / m - Vy*w)*T;
    f(3) = h + w*T;
    f(4) = w + T*FLat*lf/Iz;
    f(5) = Vy + (FLat/m + Vx*w)*T;

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

    F(2, 2) = 1 + T*(-dutyCycles*Cm2 + thetaF*(dFdVx - Cm3*dFdVx))/m;
    F(2, 4) = T*(thetaF*(dFdw - Cm3*dFdw) -Cm3*Vy)/m - Vy;
    F(2, 5) = T*(thetaF*(dFdVy - Cm3*dFdVy) -Cm3*w)/m - w;

    F(3, 3) = 1;
    F(3, 4) = T;

    F(4, 2) = T*(dFdVx*lf/Iz);
    F(4, 4) = 1 + T*(dFdw*lf/Iz);
    F(4, 5) = T*(dFdVy*lf/Iz);

    F(5, 2) = T*(w + T*dFdVx/m);
    F(5, 4) = T*(Vx + dFdw/m);
    F(5, 5) = 1 + T*(dFdVy/m);

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
    if(Vx == 0)
    {
        Vx = 0.01;
    }

    return atan((Vy + lf*omegaZ)/Vx) - thetaF;
}

float STModel::calcLatForce(float alphaF)
{
    return -Cf*alphaF;
}
