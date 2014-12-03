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

private:
    /**
     * @brief Gives initial values to F, G, H, Q and R
     */
    void matrixSetup();

};
