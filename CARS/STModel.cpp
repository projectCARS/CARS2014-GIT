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
    R(0, 0) = 1;
    R(1, 1) = 1;
    R(2, 2) = 0.001;

    // Tuna dessa
    Q(0, 0) = 1;
    Q(1, 1) = 1;
    Q(2, 2) = 0.001;
    Q(3, 3) = 1;
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
    m_gas = 0;
    m_turn = 0;

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

    qDebug("x: %f\nY: %f\nVx: %f\nh: %f\nw: %f\nVy: %f\ngas: %f\nturn: %f\nT: %f\n", X, Y, Vx, h, w, Vy, m_gas, m_turn,T);

    double dutyCycles = calcDutycycles();
    double thetaF = calcThetaF();

    double alphaF = calcAlphaF(Vx, Vy, h, thetaF);
    double FLat = calcLatForce(alphaF);

    if(Vx < 0.3 || Vy < 0.3 || w < 0.3)
        FLat = 0;


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

    if(Vx < 0.3 || Vy < 0.3 || w < 0.3)
    {
        dFdVx = 0;
        dFdVy = 0;
        dFdw = 0;
    }


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
    float64 signal[2];
    signal[0] = u_gas;
    signal[1] = u_turn;
    decimalToVoltage(signal);
    m_gas = signal[0];
    m_turn = signal[1];
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

void STModel::decimalToVoltage(float64 *decimal)
{

    m_linearizationBreak = 0.35;
    m_voltGasMin = 1.4710;
    m_voltGasIntervall = 1.4693;

    m_voltReverseThreshold = 1.68;


    m_minGasVolt = 0.91; // 1.14;
    m_maxGasVolt = 2.21; // 2.31;
    m_minTurnVolt = 0.11;
    m_maxTurnVolt = 2.97; //3.242;

    m_gasNeutral = 1.61;
    m_turnNeutral = 1.53;

    // Transform gas signal
    // If gas is 0

    if (decimal[0] == 0)
    {
        decimal[0] = m_gasNeutral;
    }
    // If gas is 0 - m_linearizationBreak
    else if (decimal[0] > 0 && decimal[0] <= m_linearizationBreak)
    {
        decimal[0] = m_voltGasMin - decimal[0] / m_linearizationBreak * (m_voltGasMin - m_voltGasIntervall);
    }
    // If gas is m_linearizationBreak - 100%
    else if (decimal[0] > m_linearizationBreak && decimal[0] <= 1)//m_linearizationBreak && decimal[0] <= 1)
    {
        //decimal[0] = m_voltGasIntervall - (decimal[0] - m_linearizationBreak) / (1 - m_linearizationBreak)* (m_voltGasIntervall - m_minGasVolt);
        decimal[0] = -0.33 + m_voltGasIntervall - (decimal[0] - m_linearizationBreak) / (1 - m_linearizationBreak)* (m_voltGasIntervall - m_minGasVolt);
        //decimal[0] = -0.513*decimal[0] + 1.423;
    }
    // If break/reverse
    else if (decimal[0] < 0 && decimal[0] >= -1)
    {
        //decimal[0] = m_voltReverseThreshold + -decimal[0] * (m_maxGasVolt - m_voltReverseThreshold);
        decimal[0] = -0.2 + m_voltReverseThreshold + -decimal[0] * (m_maxGasVolt - m_voltReverseThreshold)*4;
    }
    // If out of bounds
    else
    {
        decimal[0] = m_gasNeutral;
        std::cout << "Gas signal is out of bounds: " << decimal[0] << std::endl;
    }

    qDebug() << "gas again: " << decimal[0];

    // Transform turn signal
    if (decimal[1] == 0)
    {
        decimal[1] = m_turnNeutral;
    }
    else if (decimal[1] > 0 && decimal[1] <= 1)
    {
        decimal[1] = m_turnNeutral + decimal[1] * (m_maxTurnVolt - m_turnNeutral);
        if (decimal[1] > m_maxTurnVolt)
        {
            std::cout << "Turn signal is too high: " << decimal[1] << std::endl;
        }
    }
    else if (decimal[1] < 0 && decimal[1] >= -1)
    {
        decimal[1] = m_turnNeutral - -decimal[1] * (m_turnNeutral - m_minTurnVolt);
        if (decimal[1] < m_minTurnVolt)
        {
            std::cout << "Turn signal is too low: " << decimal[1] << std::endl;
        }
    }
    else
    {
        std::cout << "Turn signal is out of bounds: " << decimal[1] << std::endl;
    }
}
