#include "headers.h"
#include "definitions.h"
#include "EKF.h"
#include "Filter.h"
#include "CTModel.h"
#include "STModel.h"
#include <Eigen/Dense>
#include <math.h>

#include <QDebug>

EKF::EKF(MotionModelType::Enum motionModelType)
{
    // Initialize motion model.
    switch (motionModelType)
    {
        case MotionModelType::CTModel:
            M = new CTModel();
            break;
        case MotionModelType::STModel:
            M = new STModel();
            break;
        default:
            std::cout << "Error: Motion model type not implemented, in EKF::EKF(), EKF.cpp" << std::endl;
    }
    // Set vector and matrix sizes
    xhat = VectorXd::Zero(M->getNumStates());
    // Avoid division by zero by assigning a small value.
    xhat(M->getNumStates() - 1) = 1.e-6;
    u = VectorXd::Zero(M->getNumControlSignals());
    z = VectorXd::Zero(M->getNumMeasurements());
    I = MatrixXd::Identity(M->getNumStates(), M->getNumStates());
    P = I;
    // Get covariance matrices from model.
    R = M->getR();
    Q = M->getQ();
    // Initialize kalman filter.
    firstState();
    // Start timing for dynamic time step length.
    T = 1 / 150;
    time = clock();
    // Set measurement boolean to false.
    m_newMeasurement = false;
}

EKF::~EKF()
{
    //delete M;
}

void EKF::firstState()
{
    // Relinearization.
	updateModel();
    // Covariance.
    MatrixXd S = H * P * H.transpose() + R;
    // Kalman gain.
    MatrixXd K = P * H.transpose() * (S.inverse());
    // Innovation.
	MatrixXd innovation = z - H * xhat;
    // Estimate.
	xhat = xhat + K*innovation;
    // Estimate covariance.
	P = (I - K * H) * P;
}

void EKF::updateFilter(void)
{
    T = ((double)(clock() - time)) / CLOCKS_PER_SEC;
    time = clock();

    // Relinearization.
	updateModel();

    // Predict states.
	VectorXd xhatpred = f;
    MatrixXd Ppred = F*P*F.transpose() + Q;

    // If new measurement is found.
    if (m_newMeasurement)
	{
        angleDiff = xhatpred(3) - z(2);
        if (abs(angleDiff) > M_PI){
            if (angleDiff > 0)
                z(2) = z(2) + 2*M_PI;
            else
                z(2) = z(2) - 2*M_PI;
        }

        //Covariance.
		MatrixXd S = H*Ppred*H.transpose() + R;

        // Kalman gain.
		MatrixXd K = Ppred*H.transpose()*S.inverse();

        // Innovation.
		MatrixXd innovation = z - H*xhatpred;

        // Estimate:
		xhat = xhatpred + K*innovation;
        // Estimate covariance.
		P = (I - K*H)*Ppred;
	}
	else
	{
        // Return model predictions.
		xhat = xhatpred;
		P = Ppred;
	}
    if (xhat(3) < -M_PI || xhat(3) > M_PI)
        xhat(3) = fmod(xhat(3)+ 3*M_PI, 2*M_PI) - M_PI;

    // Set measurement boolean to false (we have now used our measurements and turned them into old ones).
	m_newMeasurement = false;
}

void EKF::addMeasurement(float x, float y, float theta)
{
	/* Add measurements to the filter. We assume here that all motion models
	measure x, y and theta. */
    z(0) = x;
    z(1) = y;
    z(2) = theta;
    // Set measurement boolean to true (we just recieved new measurements).
	m_newMeasurement = true;
}

void EKF::addInputSignals(float gas, float turn)
{
    //Add control signals to filter.
    u(0) = gas;
    u(1) = turn;
    M->addInput(u(0), u(1));
}

std::vector<float> EKF::getState(void)
{
	std::vector<float> state(xhat.size());
	for (int i=0; i < xhat.size(); i++)
	{
        state[i] = xhat[i];
	}
	return state;
}

void EKF::updateModel()
{
	M->updateModel(xhat, T);
	f = M->getf();
	F = M->getF();
	H = M->getH();
}
