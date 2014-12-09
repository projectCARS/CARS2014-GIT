#pragma once
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class MotionModel
{
public:
    ~MotionModel(){}

    // Get state space model matrices.
	virtual VectorXd getf() = 0;
    virtual MatrixXd getF() = 0;
	virtual MatrixXd getH() = 0;
	virtual MatrixXd getR() = 0;
	virtual MatrixXd getQ() = 0;

    // For determaining filter matrix sizes.
	virtual int getNumStates() = 0;
	virtual int getNumMeasurements() = 0;
	virtual int getNumControlSignals() = 0;

    // Relinearize model matrices around current states.
	virtual void updateModel(VectorXd xhat, double T) = 0;
    virtual void addInput(float u_gas, float u_turn) = 0;
};
