#pragma once
#include "Filter.h"
#include "CTModel.h"
#include <Eigen/Dense>
#include "definitions.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @brief Extended Kalman filter
 * @details The extended kalman filter is used to estimate the state of the car from a mathematical model, see MotionModel,
 *  and measured values, see CarMeasurement.
 * It is a subclass of Filter.
 */
class EKF : public Filter
{
private:
    /**
     * @brief The model used by the filter, see MotionModel.
     */
    MotionModel *M;

    /** @name Extended Kalman matrices and vectors
     * @brief All the matrices and vectors used for extended kalman filter calculations.
     */
    ///@{
    /**
     * @brief Contains the latest states of an object.
     * @details The states of the model at a certain time.
     */
    VectorXd xhat;
    /**
     * @brief Contains the latest control signal.
     * @details The input signals of the model at a certain time.
     */
    VectorXd u;
    /**
     * @brief Contains the latest raw states from the camera
     * @details The measurement data in the model at a certain time.
     */
    VectorXd z;
    /**
     * @brief System equation vector.
     * @details Consists of equations that estimates the next state depending on current states.
     */
    VectorXd f;
    // State space model matrices
    /**
     *
     */
    MatrixXd F, H;
    /**
     * @brief System noise covariance matrix
     */
    MatrixXd R;
    /**
     * @brief Measurement noise covaraince matrix
     */
    MatrixXd Q;
    /**
     * @brief Covariance matrix
     */
    MatrixXd P;
    // Identity matrix
    MatrixXd I;
    // Time variables for dynamic time step length
    double T;
    int time;
    // True if a new measurement is avaliable, otherwise false.
    bool m_newMeasurement;
    float angleDiff;

public:
	EKF(MotionModelType::Enum motionModelType);
	~EKF();

	// Add a new measurement to the filter.
	virtual void addMeasurement(float x, float y, float theta);
    // Add a new set of inputsignals to the filter
    virtual void addInputSignals(float gas, float turn);
    // Create new state estimates.
	virtual void updateFilter(void);
    // Get current state estimate.
	virtual std::vector<float> getState(void);
    // Returns true if the filter has recieved new measurements, otherwise false.
    virtual bool hasNewMeasurement(void){return m_newMeasurement;}

private:
    // Initiate Kalman filter
	void firstState();

    // Relinearizes the model matrices around current states
	virtual void updateModel();
};
