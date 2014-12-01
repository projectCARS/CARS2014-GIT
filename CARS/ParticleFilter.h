#pragma once

#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "Filter.h"
#include "CTModel.h"
#include "VirtualSensor.h"
#include <Eigen/Dense>
#include "functions.h"
#include "definitions.h"
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include "omp.h"

using namespace tbb;

class ParticleFilter : public Filter
{
private:
    float posX[NUMBER_OF_PARTICLES];
    float posY[NUMBER_OF_PARTICLES];
    float yaw[NUMBER_OF_PARTICLES];
    float vel[NUMBER_OF_PARTICLES];
    float angvel[NUMBER_OF_PARTICLES];
    float posXPoints[NUMBER_OF_PARTICLES];
    float posYPoints[NUMBER_OF_PARTICLES];
    float yawPoints[NUMBER_OF_PARTICLES];
    float velPoints[NUMBER_OF_PARTICLES];
    float angvelPoints[NUMBER_OF_PARTICLES];
    float weights[NUMBER_OF_PARTICLES];

    float noncumulativeWeights[NUMBER_OF_PARTICLES];
    float cumulativeWeights[NUMBER_OF_PARTICLES];

    float sumStates[5];

    Eigen::MatrixXf carPattern;
    float expectedSpeed;
    int limit;
    int noNegatives = 0;
    bool noCar;

    cv::Mat m_img;

    // Used to convert to world coordinates
    cv::Mat distCoeffs;
    cv::Mat cameraMatrix;
    cv::Mat cameraToWorldMatrix;
    Eigen::Matrix3f cameraToWorldMatrix_Eigen;
    cv::Mat worldToCameraMatrix;
    Eigen::Matrix3f worldToCameraMatrix_Eigen;

    float m_x = 0, m_y = 0, m_theta = 0;
    float m_gas, m_turn;

    bool imageMode = true;

    double T = 1/140;

    VirtualSensor vs;

    MotionModel *M;

    VectorXd xhat;

    bool m_newMeasurement;

    float gaussianNoise(void);
    int findFirst(const float value);

public:
    ParticleFilter(Eigen::MatrixXf ID, float speed, int lim, MotionModelType::Enum motionModelType);
    ~ParticleFilter();

    void setState(float state[3]);
    void extensiveSearch(cv::Mat img);
    void propagate(void);
    void propagateWorldCoordinates(void);
    void propagateCT(void);
    void propagateST(void);
    void update(const cv::Mat img);
    void parallelUpdate(const cv::Mat img);
    void noImgUpdate(float x, float y, float theta);

    void nonLinearUndistort(float input[2], float output[2]);
    void cameraToWorldCoordinates(float cameraPoint[2], float worldPoint[2]);

    void resample(void);
    void systematicResample(void);

    void logStates(std::ofstream *logFile);
    bool carGone(void){ return noCar; }

    void markCar(const float state[3], cv::Mat img);

    float *getPosX(void){ return posX; }
    float *getPosY(void){ return posY; }
    float *getYaw(void){ return yaw; };
    float *getPosXPoints(void){ return posXPoints; };
    float *getPosYPoints(void){ return posYPoints; };
    float *getYawPoints(void){ return yawPoints; };
    float *getSumStates(void){ return sumStates; };

    virtual void addMeasurement(float x, float y, float theta);
    // Add a new set of inputsignals to the filter
    virtual void addInputSignals(float gas, float turn);
    // Create new state estimates.
    virtual void updateFilter(void);
    // Get current state estimate.
    virtual std::vector<float> getState(void);
    // Returns true if the filter has recieved new measurements, otherwise false.
    virtual bool hasNewMeasurement(void);

    virtual void addImageMeasurement(cv::Mat img);

};



#endif // PARTICLE_FILTER_H
