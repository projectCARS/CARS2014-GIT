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
    Q(0, 0) = 1;
    Q(1, 1) = 1;
    Q(2, 2) = 100;
    Q(3, 3) = 1;
    Q(4, 4) = 100;
    Q(5, 5) = 20;

    m = 0.0406f;
    Cm1 = 0.6148f;
    Cm2 = 0.0477f;
    Cm3 = 0.0357f;
    Cf = 0.4835f;
    lf = 0.003f;
    Iz = 0.000016344f;
    kTurn = 0.10f;
    mTurn = -0.1f*(1.43f); // 1.53 in this case is turnNeutral from IOControl
    mThrottle = -0.0437f;
    m_gas = 0;
    m_turn = 0;
    p1 = 1.0989f;
    p2 = -7.7212f;
    p3 = 20.2859f;
    p4 = -23.6540f;
    p5 = 10.3348f;
    q1 = -6.9779f;
    q2 = 17.8926f;
    q3 = -20.1442f;
    q4 = 8.5883f;
    turngain = 0.1f;

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
        T = 1/150;

    qDebug("Before:\n x: %f\nY: %f\nVx: %f\nh: %f\nw: %f\nVy: %f\ngas: %f\nturn: %f\nT: %f\n", X, Y, Vx, h, w, Vy, m_gas, m_turn, T);

    double dutyCycles = calcDutycycles();
    double thetaF = calcThetaF();
    double FyFront = calcFyFront(thetaF, Vx);
    double FxRear = calcFxRear(dutyCycles, Vx);

    //qDebug("Turn signal: %f, thetaF: %f, FyFront: %f", m_turn, thetaF, FyFront);

    // System Dynamics
    f(0) = X + (cos(h)*Vx + sin(h)*Vy)*T;
    f(1) = Y + (sin(h)*Vx + cos(h)*Vy)*T;
    f(2) = Vx + (FxRear/m)*T;
    f(3) = h + w*T;
    f(4) = w + (15*FyFront*lf/Iz)*T;
    f(5) = Vy + (FyFront/m)*T;

    qDebug("After:\n x: %f\nY: %f\nVx: %f\nh: %f\nw: %f\nVy: %f\n", f(0), f(1), f(2), f(3), f(4), f(5));
    qDebug() << "Duty Cycles: " << dutyCycles << "\nFxRear" << FxRear;

    /* This section is for calculating terms used in the jacobian.
     * Currently (2015-01-08), a simpler version of the STModel is used where these are not needed
    double dAlphaF[3];
    double dFxFront[3];
    double dFyFront[3];
    double dFxRear[3];

    dAlphaF[0] = (Vy + lf*w)/((Vy + lf*w)*(Vy + lf*w) + Vx*Vx);
    dAlphaF[1] = -(lf*Vx)/((Vy + lf*w)*(Vy + lf*w) + Vx*Vx);
    dAlphaF[2] = -Vx/((Vy + lf*w)*(Vy + lf*w) + Vx*Vx);

    dFxFront[0] = -3*sin(thetaF)*k_alphaF*dAlphaF[0];
    dFxFront[1] = -3*sin(thetaF)*k_alphaF*dAlphaF[1];
    dFxFront[2] = -3*sin(thetaF)*k_alphaF*dAlphaF[2];

    dFyFront[0] = sin(thetaF)*k_alphaF*dAlphaF[0]; //- (thetaF*0.035)/(Vx*Vx);
    dFyFront[1] =  sin(thetaF)*k_alphaF*dAlphaF[1];
    dFyFront[2] = sin(thetaF)*k_alphaF*dAlphaF[2];

    dFxRear[0] = -Cm2*dutyCycles - Cm3*dFxFront[0];
    dFxRear[1] = -Cm3*dFxFront[1];
    dFxRear[2] = -Cm3*dFxFront[2];

    // The model is not accurate for low speed
    if(Vx == 0 || (Vy < 0.2 && w < 0.2))
    {
        for(int i = 0; i < 3; i++)
        {
            dAlphaF[i] = 0;
            dFxFront[i] = 0;
            dFyFront[i] = 0;
            dFxRear[i] = 0;
        }
    }
    */

    // System Jacobian
    F(0, 0) = 1;
    F(0, 2) = cos(h)*T;
    F(0, 3) = T*(cos(h)*Vy -sin(h)*Vx);
    F(0, 5) = sin(h)*T;

    F(1, 1) = 1;
    F(1, 2) = sin(h)*T;
    F(1, 3) = T*(cos(h)*Vx -sin(h)*Vy);
    F(1, 5) = cos(h)*T;

    F(2, 2) = 1 - Cm2*dutyCycles*T/m; // 1 + (dFxRear[0] + dFxFront[0])*T/m;
    F(2, 4) = 0; // (dFxRear[1] + dFxFront[1])*T/m;
    F(2, 5) = 0; // (dFxRear[2] + dFxFront[2])*T/m;

    F(3, 3) = 1;
    F(3, 4) = T;

    F(4, 2) = 0; // T*(dFyFront[0]*lf/Iz);
    F(4, 4) = 1; // 1 + T*(dFyFront[1]*lf/Iz);
    F(4, 5) = 0; // T*(dFyFront[2]*lf/Iz);

    F(5, 2) = 0; // (dFyFront[0])*T/m;
    F(5, 4) = 0; // (dFyFront[1])*T/m;
    F(5, 5) = 1; // 1 + (dFyFront[2])*T/m;

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
    if(m_gas > 1.45 && m_gas < 1.66)
        return 0;
    else
        return  mThrottle + ((p1*pow(m_gas,4) + p2*pow(m_gas,3) + p3*pow(m_gas,2) + p4*m_gas + p5) / (pow(m_gas,4) + q1*pow(m_gas,3) + q2*pow(m_gas,2) + q3*m_gas + q4));
}

float STModel::calcThetaF()
{
    return (kTurn*m_turn + mTurn);
}

// Currently not used, was used in a more detailed (not working) version of STModel
float STModel::calcAlphaF(float Vx, float Vy, float w, float thetaF)
{
    if(Vx == 0)
    {
        Vx = 0.001f;
    }
    return thetaF - atan((Vy+lf*w)/Vx);
}

float STModel::calcFyFront(float thetaF, float Vx)
{
    //return (sin(thetaF)*k_alphaF*alphaF)+ k* thetaF*0.035*(1/Vx);
    return turngain * thetaF * Vx*Vx;
}

// Currently not used, was used in a more detailed (not working) version of STModel
float STModel::calcFxFront(float thetaF, float alphaF)
{
    return -3*sin(thetaF)*k_alphaF*alphaF;
}

// Calculate the Force in the cars x-direction from the rear wheels
float STModel::calcFxRear(float D, float Vx)
{
    return Cm1*D - Cm2*D*Vx;
}

// Map interval [-1,1] to [m_minVal,m_maxVal].
void STModel::decimalToVoltage(float64 *decimal)
{

    float m_voltGasIntervall = 1.4693f;

    float m_voltGasMin = 1.4710f;
    float m_voltGasMax = 0.95f;
    float m_voltGasSlope = m_voltGasMax - m_voltGasMin;

    float m_voltBrakeThreshold = 1.472f;

    float m_reverseGasSlope = 0.51f;

    float m_minTurnVolt = 0.11f;
    float m_maxTurnVolt = 2.97f; //3.242;

    float m_gasNeutral = 1.61f;
    float m_turnNeutral = 1.53f;

    // Transform gas signal if gas is 0
    if (decimal[0] == 0)
    {
        decimal[0] = m_gasNeutral;
    }
    else if (decimal[0] > 0 && decimal[0] <= 1) // If positive gas signal
    {
        decimal[0] = m_voltGasIntervall + decimal[0] * m_voltGasSlope;
    }
    // If break/reverse
    else if (decimal[0] < 0 && decimal[0] >= -1)
    {
        decimal[0] = m_voltBrakeThreshold  - decimal[0] * m_reverseGasSlope;
    }
    // If out of bounds
    else
    {
        decimal[0] = m_gasNeutral;
    }

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
        std::cout << "Turn signal is out of bounds2: " << decimal[1] << std::endl;
    }
}
