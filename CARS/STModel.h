#pragma once
#include "MotionModel.h"
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @brief Coordinated turn model
 * @details The model takes the basic assumptions from a coordinated turn model and does not use the information about the input signals.
 * It is a subclass of MotionModel.
 */
class STModel : public MotionModel
{
private:
    /** @name Matrix dimensions
     * @brief Contains the matrix dimensions. They are set in the constructor, CTModel::CTModel
     */

    void decimalToVoltage(float64 *decimal);
    ///@{
    /**
     * @brief Number of states the model derives.
     */
    int m_numStates;
    /**
     * @brief Number of states measured by the camera used in the model.
     */
    int m_numMeasurements;
    /**
     * @brief Number of input signals used in the model.
     */
    int m_numControlSignals;

    /**
     * @brief The vectors used in the filter calculations.
     */
    VectorXd f;
    /**
     * @brief The matrices used in the filter calculations.
     */
    MatrixXd F, H, R, Q;

    /**
     * @brief The parameters used in the STmodel
     */
    float m, Cm1, Cm2, Cm3, Cf, lf, Iz,kTurn, mTurn, kThrottle, mThrottle, k_alphaF;
    float m_gas, m_turn;
    float p1, p2, p3, p4, p5, q1, q2, q3, q4;

    // At what percentage the linearization will change.
    float64 m_linearizationBreak;
    // Highest voltage for gas.
    float64 m_voltGasMin;
    // When linearization algorithm switch.
    float64 m_voltGasIntervall;
    // Lowest voltage to reverse the car
    float64 m_voltReverseThreshold;
    float64 m_gasNeutral;
    float64 m_turnNeutral;
    float64 m_minGasVolt;
    float64 m_maxGasVolt;
    float64 m_minTurnVolt;
    float64 m_maxTurnVolt;

public:
    /**
     * @brief The constructor initializes all the matrices and vectors.
     */
    STModel();

    /**
     * @brief Destructor
     */
    ~STModel();

    /** @name Matrix get methods
     * @brief A way to get each vector or matrix the model contains.
     * @details Is required by MotionModel and is called by EKF::updateFilter.
     * @return Each vector or matrix.
     */
    ///@{
    virtual VectorXd getf(){ return f; }
    virtual MatrixXd getF(){ return F; }
    virtual MatrixXd getH(){ return H; }
    virtual MatrixXd getR(){ return R; }
    virtual MatrixXd getQ(){ return Q; }
    ///@}

    /** @name Model dimension get methods
     * @brief Ways to get the dimensions for the matrices used by the model.
     * @details Is required by MotionModel and is called by EKF::EKF.
     * @return The dimensions for all matrices used by the model.
     */
    ///@{
    virtual int getNumStates(){ return m_numStates; }
    virtual int getNumMeasurements(){ return m_numMeasurements; }
    virtual int getNumControlSignals(){ return m_numControlSignals; }
    ///@}

    /**
     * @brief Linearises the model around the current state.
     * @details Is required by MotionModel. Is called by EKF::firstState and EKF::updateFilter.
     *
     * @param xhat Current estimated state
     * @param T Time since the last state was calculated, approximately.
     */
    virtual void updateModel(VectorXd xhat, double T);
    virtual void addInput(float u_gas, float u_turn);

private:
    /**
     * @brief Gives initial values to F, G, H, Q and R
     */
    void matrixSetup();

    // returns Duty cycles as a function of gas control signal
    float calcDutycycles();
    // returns thetaF as a function of turn control signal
    float calcThetaF();
    // Returns AlphaF parameter used when updating the model
    float calcAlphaF(float Vx, float Vy, float omegaZ, float thetaF);
    // returns lateral force
    float calcLatForce(float alphaF);

    // returns the force in the car's y-direction for the front wheels
    float calcFyFront(float Vx, float thetaF, float alphaF);
    // returns the force in the car's x-direction for the front wheels
    float calcFxFront(float thetaF, float alphaF);

    float calcFxRear(float D, float Vx, float Fxf);

};
