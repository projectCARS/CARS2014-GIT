#include "headers.h"
#include "EKF.h"
#include "MotionModel.h"
#include "CTModel.h"
#include <Eigen/Dense>
#include <random>

using Eigen::MatrixXd;
using Eigen::VectorXd;

CTModel::CTModel()
{
	m_numStates = 5;
	m_numMeasurements = 3;
	m_numControlSignals = 2;

    f = MatrixXd::Zero(m_numStates, 1);
    F = MatrixXd::Zero(m_numStates, m_numStates);
    H = MatrixXd::Zero(m_numMeasurements, m_numStates);

    R = MatrixXd::Zero(m_numMeasurements, m_numMeasurements);
    Q = MatrixXd::Zero(m_numStates, m_numStates);

	matrixSetup();
}


CTModel::~CTModel()
{
}

void CTModel::matrixSetup()
{
    F = MatrixXd::Identity(m_numStates, m_numStates);

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
}

void CTModel::updateModel(VectorXd xhat, double T)
{
	double X = xhat(0);
	double Y = xhat(1);
	double v = xhat(2);
	double h = xhat(3);
	double w = xhat(4);

	// System dynamics
	f(0) = X + 2 * v / w * sin(w*T / 2) * cos(h + w*T / 2);
	f(1) = Y + 2 * v / w * sin(w*T / 2) * sin(h + w*T / 2);
	f(2) = v;
	f(3) = h + w*T;
	f(4) = w;

	// System jacobian
    F(0,0) = 1;
	F(0, 2) = 2 / w * sin(w*T / 2) * cos(h + (w*T) / 2);
	F(0, 3) = -2 * v / w * sin(w*T / 2) * sin(h + (w*T) / 2);
    F(0, 4) = v / (w*w) * (w*T*cos(h + w*T) - sin(h + w*T) + sin(h));

    F(1,1) = 1;
    F(2,2) = 1;
    F(3,3) = 1;
    F(4,4) = 1;

	F(1, 2) = 2 / w * sin(w*T / 2) * sin(h + (w*T) / 2);
	F(1, 3) = 2 * v / w * sin(w*T / 2) * cos(h + (w*T) / 2);
    F(1, 4) = v / (w*w) * (w*T*cos(h + w*T) - sin(h + w*T) + sin(h));

    F(3, 4) = T;

	//TODO: Add dynamics for G and H...
}

void CTModel::addInput(float u_gas, float u_turn){}
