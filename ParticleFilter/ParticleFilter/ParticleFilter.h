#pragma once
#include "Filter.h"
#include "CTModel.h"
#include <Eigen/Dense>
#include "definitions.h"
#include "ParticleFilter.cpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ParticleFilter : public Filter
{
private:
    float posX[NUMBER_OF_PARTICLES];
    float posY[NUMBER_OF_PARTICLES];
    float yaw[NUMBER_OF_PARTICLES];
    float posXPoints[NUMBER_OF_PARTICLES];
    float posYPoints[NUMBER_OF_PARTICLES];
    float yawPoints[NUMBER_OF_PARTICLES];
    float weights[NUMBER_OF_PARTICLES];
    float cumulativeWeights[NUMBER_OF_PARTICLES];
    float sumStates[3];

    Eigen::MatrixXf carPattern;
    float expectedSpeed;
    int limit;
    bool noCar;
    bool m_newMeasurement;

    float gaussianNoise(void);
    int findFirst(const float value);

public:
    ParticleFilter(Eigen::MatrixXf ID, float speed, int lim, MotionModelType::Enum motionModelType);

    ~ParticleFilter();

    void setState(float state[3]);
    void extensiveSearch(cv::Mat img);
    void propagate(void);
    void update(const cv::Mat img);
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

    void addMeasurement2(const cv::Mat img);
    virtual void addMeasurement(float x, float y, float theta);
    // Add a new set of inputsignals to the filter
    virtual void addInputSignals(float gas, float turn);
    // Create new state estimates.
    virtual void updateFilter(void);
    // Get current state estimate.
    virtual std::vector<float> getState(void);
    // Returns true if the filter has recieved new measurements, otherwise false.
    virtual bool hasNewMeasurement(void){return m_newMeasurement;}

};
