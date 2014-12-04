
#include "definitions.h"
#include "headers.h"
#include "ParticleFilter.h"
#include "Filter.h"
#include "CTModel.h"
#include <Eigen/Dense>
#include <math.h>
#include <cmath>
#include <QDebug>
#include <omp.h>
#include <fstream>
#include <iostream>


ParticleFilter::ParticleFilter(Eigen::MatrixXf ID, float speed, int lim, MotionModelType::Enum motionModelType)
{
    switch (motionModelType)
    {
        case MotionModelType::CTModel:
        {
            mType = motionModelType;
            M = new CTModel();
        }
            break;
        case MotionModelType::STModel:
        {
            mType = motionModelType;
            M = new STModel();
        }
            break;
        default:
            std::cout << "Error: Motion model type not implemented, in ParticleFilter::ParticleFilter(), ParticleFilter.cpp" << std::endl;
    }

    imageMode = true;
    LoadTrack();

    time = omp_get_wtime();
    carPattern = ID;
    expectedSpeed = speed;
    limit = lim;
    noCar = true;

    T = 1/100;
    noCarCounter = 1;
    //xhat = Eigen::MatrixXf::Zero(M->getNumStates(),NUMBER_OF_PARTICLES);
    //xhatpred = Eigen::MatrixXf::Zero(M->getNumStates(), NUMBER_OF_PARTICLES);
    //sumState = Eigen::VectorXf::Zero(M->getNumStates());

    sumStates[0] = 0;
    sumStates[1] = 0;
    sumStates[2] = 0;
    sumStates[3] = 0;
    sumStates[4] = 0;
    sumStates[5] = 0;
    sumStates[6] = 0;
    sumStates[7] = 0;
    qDebug("0");
    m_img = cv::imread("indata/ImageWithCar.jpg", CV_LOAD_IMAGE_GRAYSCALE);

    if (fileExists("indata/calibrationData.yml"))
    {
        cv::FileStorage storage("indata/calibrationData.yml", cv::FileStorage::READ);
        storage["distCoeffs"] >> distCoeffs;
        storage["cameraMatrix"] >> cameraMatrix;
        storage["cameraToWorldMatrix"] >> cameraToWorldMatrix;
        storage.release();
        cv::cv2eigen(cameraToWorldMatrix, cameraToWorldMatrix_Eigen);
    }
    else
    {
        std::cout << "Class particleFilter:	Cannot load calibrationData.yml" << std::endl;
    }
}

ParticleFilter::~ParticleFilter()
{
}

void ParticleFilter::setState(float state[5])
{
    for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
    {
        posX[i] = state[0];
        posY[i] = state[1];
        vel[i] = state[2];
        yaw[i] = state[3];
        angvel[i] = state[4];
    }
}

// Propagate according to random walk model using camera coordinates
void ParticleFilter::propagate(void)
{
    omp_set_num_threads(3);
#pragma omp parallel for
    for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
    {
        // update points with random walk model
        velPoints[i]  = expectedSpeed + 25 * gaussianNoise();
        yawPoints[i] = yaw[i] + M_PI / 3 * gaussianNoise(); // angle
        if (yawPoints[i] < -M_PI || yawPoints[i] > M_PI)
            yawPoints[i] = fmod(yawPoints[i]+ 3*M_PI, 2*M_PI) - M_PI;
        posXPoints[i] = posX[i] + velPoints[i]*sin(yawPoints[i]);
        posYPoints[i] = posY[i] - velPoints[i]*cos(yawPoints[i]);
    }
}

// Propagate according to random walk model using world coordinates
void ParticleFilter::propagateWorldCoordinates(void)
{
    omp_set_num_threads(3);
#pragma omp parallel for
    for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
    {
        // update points with random walk model
        velPoints[i]  = (expectedSpeed + 25 * gaussianNoise())/PIXELS_PER_METER;
        yawPoints[i] = yaw[i] + M_PI / 5 * gaussianNoise(); // angle
        if (yawPoints[i] < -M_PI || yawPoints[i] > M_PI)
            yawPoints[i] = fmod(yawPoints[i]+ 3*M_PI, 2*M_PI) - M_PI;
        posXPoints[i] = posX[i] + velPoints[i]*sin(yawPoints[i]);
        posYPoints[i] = posY[i] - velPoints[i]*cos(yawPoints[i]);
    }
}

// Propagate according to coordinated turn model using camera coordinates
void ParticleFilter::propagateCT(void)
{
    // Update the time step
    T = ((double)(omp_get_wtime() - time));
    time = omp_get_wtime();

    omp_set_num_threads(3);
#pragma omp parallel for
    for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
    {
        /*
        xhatpred(2,i) = xhat(2,i) + gaussianNoise()*30; //Velocity
        xhatpred(4,i) = xhat(4,i) + M_PI / 7 * gaussianNoise(); // Angular velocity
        xhatpred(3,i) = xhat(3,i) + xhat(4,i)*T; //Yaw
        if (xhatpred(3,i) < -M_PI || xhatpred(3,i) > M_PI)
            xhatpred(3,i) = fmod(xhatpred(3,i) + 3*M_PI, 2*M_PI) - M_PI;
        xhatpred(0,i) = xhat(0,i) + 2 * xhatpred(2,i) / xhatpred(4,i) * sin(xhatpred(4,i)*T / 2) * cos(xhatpred(3,i) + xhatpred(4,i)*T / 2);
        xhatpred(1,i) = xhat(1,i) + 2 * xhatpred(2,i) / xhatpred(4,i) * sin(xhatpred(4,i)*T / 2) * sin(xhatpred(3,i) + xhatpred(4,i)*T / 2);
        */
        // update points with CT model
        velPoints[i] = vel[i] + gaussianNoise()*30;
        angvelPoints[i] = angvel[i] + M_PI / 7 * gaussianNoise();
        yawPoints[i] = yaw[i] + angvel[i]*T;
        if (yawPoints[i] < -M_PI || yawPoints[i] > M_PI)
            yawPoints[i] = fmod(yawPoints[i] + 3*M_PI, 2*M_PI) - M_PI;
        posXPoints[i] = posX[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * cos(yaw[i] + angvelPoints[i]*T / 2);
        posYPoints[i] = posY[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * sin(yaw[i] + angvelPoints[i]*T / 2);

    }
}

// Propagate according to coordinated turn model using world coordinates
void ParticleFilter::propagateCTWorldCoordinates(void)
{
    T = ((double)(omp_get_wtime() - time));
    time = omp_get_wtime();

    omp_set_num_threads(3);
#pragma omp parallel for
    for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
    {
        // update points with CT model
        velPoints[i] = vel[i] + gaussianNoise()*0.05;
        angvelPoints[i] = angvel[i] + M_PI / 7 * gaussianNoise();
        yawPoints[i] = yaw[i] + angvel[i]*T;
        if (yawPoints[i] < -M_PI || yawPoints[i] > M_PI)
            yawPoints[i] = fmod(yawPoints[i]+ 3*M_PI, 2*M_PI) - M_PI;
        posXPoints[i] = posX[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * cos(yaw[i] + angvelPoints[i]*T / 2);
        posYPoints[i] = posY[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * sin(yaw[i] + angvelPoints[i]*T / 2);
    }
}

void ParticleFilter::propagateST(void)
{
    // Update the time step
    T = ((double)(omp_get_wtime() - time));
    time = omp_get_wtime();

    dutyCycles = calcDutycycles();
    thetaF = calcThetaF();

    float alphaF, FLat;

    for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
    {
        //alphaF = calcAlphaF(VxPoints[i], VyPoints[i], yawPoints[i], thetaF);
        FLat = calcLatForce(alphaF);

        //angvelPoints[i] = angvel[i] +
    }
}

float ParticleFilter::gaussianNoise(void)
{
    static float rand1, rand2;

    rand1 = rand() / ((float)RAND_MAX);
    // Something not to small, since it's stored as a float
    if (rand1 < 1e-20)
    {
        rand1 = 1e-20f;
    }
    rand1 = -2 * log(rand1);
    rand2 = (rand() / ((float)RAND_MAX)) * 2 * M_PI;

    return sqrt(rand1) * cos(rand2);
}

void ParticleFilter::update(const cv::Mat img)
{
    double tStart = 0;
    double tEnd = 0;
    // Add border values around each point controlled
    cv::Rect ROI = cv::Rect(0, 0, limit, limit);
    cv::Mat imgROI(limit, limit, CV_8UC1, cv::Scalar(0, 0, 0));

    int points = carPattern.rows();
    Eigen::Matrix2f rotate, translate;
    Eigen::MatrixXf pos;
    Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(2, points);
    float val, sum;
    float totSum = 0.;

    tStart = omp_get_wtime();

    for (int n = 0; n < NUMBER_OF_PARTICLES; n++)
    {
        // Rotate and translate pattern according to hypothesis
        rotate << cos(yawPoints[n]), -sin(yawPoints[n]),
            sin(yawPoints[n]), cos(yawPoints[n]);
        translate << posXPoints[n], 0,
            posYPoints[n], 0;
        pos = rotate*carPattern.transpose() + translate*ones;

        val = 0.;
        sum = 0.;
        for (int p = 0; p < points; p++)
        {
            // Update ROI
            ROI.x = pos(0, p);
            ROI.y = pos(1, p);
            //std::cout << "checking: [" << ROI.x << ", " << ROI.y << std::endl;
            if (ROI.x - limit > 0 && ROI.x + limit < img.cols && ROI.y - limit > 0 && ROI.y + limit < img.rows)
            {
                imgROI = img(ROI);
                cv::Scalar mean = cv::mean(imgROI);

                // Throws the hypothesis if no point found.This gives some speedup.
                if (mean.val[0] == 0)
                {
                    //totSum = 0;
                    sum = 0;
                    break;
                }
                else
                {
                    sum += mean.val[0];
                }
            }
        }
        //if(sum != 0)
            //std::cout << sum << std::endl;
        totSum += sum;
        cumulativeWeights[n] = totSum; // save weights cumulative sum for later normalization
    }

    tEnd = omp_get_wtime();
    //std::cout << "For time: " << tEnd - tStart << std::endl;
}

// Parallelized version of update
void ParticleFilter::parallelUpdate(const cv::Mat img)
{
    // Add border values around each point controlled

    int points = carPattern.rows();
    Eigen::Matrix2f rotate, translate;
    Eigen::MatrixXf pos;
    Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(2, points);
    float sum = 0;
    //tStart = omp_get_wtime();

omp_set_num_threads(3);
#pragma omp parallel for private(rotate, translate, pos, sum)
    for (int n = 0; n < NUMBER_OF_PARTICLES; n++)
    {
        // Add border values around each point controlled
        cv::Rect ROI = cv::Rect(0, 0, limit, limit);
        cv::Mat imgROI(limit, limit, CV_8UC1, cv::Scalar(0, 0, 0));

        /*
        // Rotate and translate pattern according to hypothesis
        rotate << cos((float)xhatpred(3,n)), -sin((float)xhatpred(3,n)),sin((float)xhatpred(3,n)), cos((float)xhatpred(3,n));
        translate << (float)xhatpred(0,n), 0, (float)xhatpred(1,n), 0;
        */

        // Rotate and translate pattern according to hypothesis
        rotate << cos(yawPoints[n]), -sin(yawPoints[n]),
            sin(yawPoints[n]), cos(yawPoints[n]);
        translate << posXPoints[n], 0,
            posYPoints[n], 0;


        pos = rotate*carPattern.transpose() + translate*ones;
        sum = 0.;
        for (int p = 0; p < points; p++)
        {

            // Update ROI
            ROI.x = pos(0, p);
            ROI.y = pos(1, p);

            if (ROI.x - limit > 0 && ROI.x + limit < img.cols && ROI.y - limit > 0 && ROI.y + limit < img.rows)
            {
                {
                    imgROI = img(ROI);
                }

                cv::Scalar mean = cv::mean(imgROI);

                // Throws the hypothesis if no point found. This gives some speedup.
                if (mean.val[0] == 0)
                {
                    sum = 0;
                    break;
                }
                else
                {
                    sum += mean.val[0];
                }
            }
        }
        noncumulativeWeights[n] = sum; // save the weights in non-cumulative form in the parallelized loop
    }

    // Form a cumulative sum of the weights
    cumulativeWeights[0] = noncumulativeWeights[0];
    for (int i = 1; i < NUMBER_OF_PARTICLES; i++)
    {
        cumulativeWeights[i] = cumulativeWeights[i - 1] + noncumulativeWeights[i];
    }

    //tEnd = omp_get_wtime();
   //std::cout << "Parallel for time: " << tEnd - tStart << std::endl;

}

// Update the weights based on measurements from the find cars algorithm
void ParticleFilter::noImgUpdate(float x, float y, float theta)
{
    float val, sum, angle, measAngle, weight;
    measAngle = theta;
    double base = 0;
    numPartInCritReg = 0;

    // Convert the measured angle to a value between 0 and 1
    if(measAngle < 0)
    {
        measAngle = (2.0*M_PI + measAngle)/(2.0*M_PI);
    }
    else
    {
        measAngle = measAngle/(2.0*M_PI);
    }

    //omp_set_num_threads(3);
    //#pragma omp parallel for private(angle, sum, val, theta, oldPos , pos)
    for(int n = 0; n < NUMBER_OF_PARTICLES; n++)
    {
        // Update number of particles in the region between -pi/2 and pi/2
        if(yawPoints[n] > -(M_PI*0.4) && yawPoints[n] < M_PI*0.4)
        {
            numPartInCritReg++;
        }

        float xWorld = (PIXELS_PER_METER*posXPoints[n]);
        float yWorld = (PIXELS_PER_METER*posYPoints[n]);

        // If the hypothesis is outside of the world then discard it
        if(posXPoints[n] < 0 || posXPoints[n] > 2.5 || posYPoints[n] < 0 || posYPoints[n] > 2.0)
        {
            sum = 0;
            noncumulativeWeights[n] = 0;
        }
        // If the hypothesis is outside the track boundaries then also discard it
        else if(trackConstraints[(int)yWorld][(int)xWorld] == 0)
        {
            sum = 0;
            noncumulativeWeights[n] = 0;
        }
        else
        {
            val = 0.;
            sum = 0.;
            angle = yawPoints[n];

            sum += sqrt(pow((posXPoints[n] - x), 2.0) + pow((posYPoints[n] - y), 2.0));
            sum = 1.0/sum;

            // convert angle to value between -pi and pi
            while(abs(angle) > 2.0*M_PI)
            {
                if(angle > 0)
                    angle -= 2.0*M_PI;
                else
                    angle += 2.0*M_PI;
            }
            if(angle > M_PI)
                angle -= 2.0*M_PI;
            else if(angle < (-M_PI))
                angle += 2.0*M_PI;

            //std::cout << "yawPoints value: " << angle << std::endl;

            // Normalize angles to value between 0 and 1
            if(angle < 0)
            {
                angle = (2*M_PI + angle)/(2.0*M_PI);
            }
            else
            {
                angle = angle/(2.0*M_PI);
            }

            //std::cout << "after: " << angle << std::endl << std::endl;

            // Multiply the sum weight with the difference between the angle state and measurement to the power of 20
            if(abs(angle - measAngle) > 0.5 && angle > 0.5)
            {
                base = 1 - abs((angle - 1) - measAngle);
                weight = pow(base, 20);
            }
            else if(abs(angle - measAngle) > 0.5 && measAngle > 0.5)
            {
                base = 1 - abs(angle - (measAngle - 1));
                weight = pow(base, 20);
            }
            else
            {
                base = 1 - abs(angle - measAngle);
                weight = pow(base, 20);
            }

            sum = sum*weight;

            // In the parallel loop it's not possible to make a cumulative sum, do that outside the loop
            noncumulativeWeights[n] = sum;
        }
    }

    // Form a cumulative formed array of weights
    cumulativeWeights[0] = noncumulativeWeights[0];
    for (int i = 1; i < NUMBER_OF_PARTICLES; i++)
    {
        cumulativeWeights[i] = cumulativeWeights[i - 1] + noncumulativeWeights[i];
    }
}

// Resample the particles using multinomial resampling
// Not updated to work with non-image mode as of (2014-12-02)
void ParticleFilter::resample(void)
{
    // randomly, uniformally, sample from the cummulative distribution of the probability distribution generated by the
    // weighted vector 'weight'.
    float randNo;
    int index = 0;
    sumStates[0] = 0;
    sumStates[1] = 0;
    sumStates[2] = 0;
    sumStates[3] = 0;
    sumStates[4] = 0;
    if (cumulativeWeights[NUMBER_OF_PARTICLES - 1] != 0)
    {
        // Normalize weights to form a cumulative summed probability distribution
        for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
        {
            cumulativeWeights[i] /= cumulativeWeights[NUMBER_OF_PARTICLES - 1];
        }

        for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
        {
            randNo = rand() / ((float)RAND_MAX);
            index = findFirst(randNo);
            posX[i] = posXPoints[index];
            sumStates[0] += posXPoints[index];

            posY[i] = posYPoints[index];
            sumStates[1] += posYPoints[index];

            vel[i] = velPoints[index];
            sumStates[2] += velPoints[index];

            yaw[i] = yawPoints[index];

            if(!imageMode){
                // Add 2*PI to the angle if the majority of the particles are close to the turning point around -pi and pi
                if(yawPoints[index] < 0 && numPartInCritReg < NUMBER_OF_PARTICLES*0.1f)
                {
                    sumStates[3] += (yawPoints[index] + 2*M_PI);
                    //std::cout << "1 " << yawPoints[k] + 2*M_PI << std::endl;
                }
                else{
                    sumStates[3] += yawPoints[index];
                    //std::cout << "2 " << yawPoints[k] << std::endl;
                }
            }
            else
                sumStates[3] += yawPoints[index];

            angvel[i] = angvelPoints[index];
            sumStates[4] += angvelPoints[index];
        }
    }
    else
    {
        noCar = true;

        // try to save the situation with model
        for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
        {
            posX[i] = posXPoints[i];
            sumStates[0] += posXPoints[i];

            posY[i] = posYPoints[i];
            sumStates[1] += posYPoints[i];

            vel[i] = velPoints[i];
            sumStates[2] += velPoints[i];

            yaw[i] = yawPoints[i];
            sumStates[3] += yawPoints[i];

            angvel[i] = angvelPoints[i];
            sumStates[4] += angvelPoints[i];
        }

    }
}

// Resample using systematic resampling which is faster than multinomial sampling
void ParticleFilter::systematicResample(void)
{
    // randomly, uniformally, sample from the cummulative distribution of the probability distribution generated by the
    // weighted vector 'weight'.
    float ordNum[NUMBER_OF_PARTICLES];
    sumStates[0] = 0;
    sumStates[1] = 0;
    sumStates[2] = 0;
    sumStates[3] = 0;
    sumStates[4] = 0;
    int k = 0;

    if (cumulativeWeights[NUMBER_OF_PARTICLES - 1] != 0)
    {
        ordNum[0] = (rand() / ((float)RAND_MAX))/NUMBER_OF_PARTICLES;

        // Normalize weights to form a cumulative summed probability distribution
        // TODO Parallelize this and see if it gives speedup
        for (int i = 0; i < NUMBER_OF_PARTICLES - 1; ++i)
        {
            cumulativeWeights[i] /= cumulativeWeights[NUMBER_OF_PARTICLES - 1];
            ordNum[i + 1] = ((i + 1) + (rand() / ((float)RAND_MAX))) / NUMBER_OF_PARTICLES;
        }
        cumulativeWeights[NUMBER_OF_PARTICLES - 1] /= cumulativeWeights[NUMBER_OF_PARTICLES - 1];

        k = 0;
        for(int j = 0; j < NUMBER_OF_PARTICLES - 1; j++)
        {

            while(cumulativeWeights[k] < ordNum[j])
                k++;

            posX[j] = posXPoints[k];
            sumStates[0] += posXPoints[k];
            posY[j] = posYPoints[k];
            sumStates[1] += posYPoints[k];
            vel[j] = velPoints[k];
            sumStates[2] += velPoints[k];

            yaw[j] = yawPoints[k];
            if(!imageMode)
            {
                // Add 2*PI to the angle if the majority of the particles are close to the turning point around -pi and pi
                if(yawPoints[k] < 0 && numPartInCritReg < NUMBER_OF_PARTICLES*0.1f)
                {
                    sumStates[3] += (yawPoints[k] + 2*M_PI);
                    //std::cout << "1 " << yawPoints[k] + 2*M_PI << std::endl;
                }
                else{
                    sumStates[3] += yawPoints[k];
                    //std::cout << "2 " << yawPoints[k] << std::endl;
                }
            }
            else
                sumStates[3] += yawPoints[k];

            angvel[j] = angvelPoints[k];
            sumStates[4] += angvelPoints[k];

            /*
            xhat(0,j) = xhatpred(0,k);
            sumStates[0] += xhatpred(0,k);
            xhat(1,j) = xhatpred(1,k);
            sumStates[1] += xhatpred(1,k);
            xhat(2,j) = xhatpred(2,k);
            sumStates[2] += xhatpred(2,k);

            xhat(3,j) = xhatpred(3,k);
            if(!imageMode){
                // Add 2*PI to the angle if the majority of the particles are close to the turning point around -pi and pi
                if(xhatpred(3,k) < 0 && numPartInCritReg < NUMBER_OF_PARTICLES*0.1f)
                {
                    sumStates[3] += (xhatpred(3,k) + 2*M_PI);
                    //std::cout << "1 " << yawPoints[k] + 2*M_PI << std::endl;
                }
                else{
                    sumStates[3] += xhatpred(3,k);
                    //std::cout << "2 " << yawPoints[k] << std::endl;
                }
            }
            else
                sumStates[3] += xhatpred(3,k);

            xhat(4,j) = xhatpred(4,k);
            sumStates[4] += xhatpred(4,k);
            */
        }
    }
    else
    {
        noCar = true;
        for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
        {
            posX[i] = posXPoints[i];
            sumStates[0] += posX[i];
            posY[i] = posYPoints[i];
            sumStates[1] += posY[i];
            vel[i] = velPoints[i];
            sumStates[2] += velPoints[i];
            yaw[i] = yawPoints[i];

            if(!imageMode)
            {
                // Add 2*PI to the angle if the majority of the particles are close to the turning point around -pi and pi
                if(yawPoints[i] < 0 && numPartInCritReg < NUMBER_OF_PARTICLES*0.1f)
                {
                    sumStates[3] += (yawPoints[i] + 2*M_PI);
                }
                else{
                    sumStates[3] += yawPoints[i];
                    //std::cout << "2 " << yawPoints[k] << std::endl;
                }
            }
            else
            {
                sumStates[3] += yawPoints[i];
            }
            angvel[i] = angvelPoints[i];
            sumStates[4] += angvelPoints[i];

            /*
            xhat(0,i) = xhatpred(0,i);
            sumStates[0] += xhatpred(0,i);
            xhat(1,i) = xhatpred(1,i);
            sumStates[1] += xhatpred(1,i);
            xhat(2,i) = xhatpred(2,i);
            sumStates[2] += xhatpred(2,i);

            xhat(3,i) = xhatpred(3,i);
            if(!imageMode){
                // Add 2*PI to the angle if the majority of the particles are close to the turning point around -pi and pi
                if(xhatpred(3,i) < 0 && numPartInCritReg < NUMBER_OF_PARTICLES*0.1f)
                {
                    sumStates[3] += (xhatpred(3,i) + 2*M_PI);
                    //std::cout << "1 " << yawPoints[k] + 2*M_PI << std::endl;
                }
                else{
                    sumStates[3] += xhatpred(3,i);
                    //std::cout << "2 " << yawPoints[k] << std::endl;
                }
            }
            else
                sumStates[3] += xhatpred(3,i);

            xhat(4,i) = xhatpred(4,i);
            sumStates[4] += xhatpred(4,i);
            */
        }
    }
}

void ParticleFilter::extensiveSearch(cv::Mat img)
{
    // Detect contrasting image features for seeding states
    cv::vector<cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;
    findContours(img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Merge connected components
    int i, j;
    std::vector<cv::Point> blobs;
    if (!contours.empty())
    {
        for (i = 0; i < contours.size(); i++)
        {
            int x = 0;
            int y = 0;
            int len = contours.at(i).size();
            // naive approach by averaging contour points
            for (j = 0; j < len; j++)
            {
                //std::cout << "in component " << i << ", [" << contours.at(i).at(j).x << ", " << contours.at(i).at(j).y << "]" << std::endl;
                x += contours.at(i).at(j).x;
                y += contours.at(i).at(j).y;
            }
            cv::Point point;
            point.x = x / len;
            point.y = y / len;
            blobs.push_back(point);
        }
    }
    else
    {
        std::cout << "Class ParticleFilter:	Cannot find any image features in extensive search" << std::endl;
    }

    int index;
    float yawGuess;
    if (!contours.empty()) // Guess position based on image features
    {
        for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
        {
            yawGuess = (rand() / ((float)RAND_MAX))*M_PI * 2; // pick any angle

            // Feed image feature points + guessed yaw to state vectors
            yaw[i] = yawGuess;
            index = rand() % blobs.size(); // pick one of the probable points;
            posX[i] = blobs[index].x;
            posY[i] = blobs[index].y;
            vel[i] = 0.3f + gaussianNoise()*0.1;
            angvel[i] = 0;
        }
    }
    else // Look over entire image
    {
        for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
        {
            posX[i] = (rand() / ((float)RAND_MAX))*img.cols; // pick any posX
            posY[i] = (rand() / ((float)RAND_MAX))*img.rows; // pick any posY
            yaw[i] = (rand() / ((float)RAND_MAX))*M_PI * 2; // pick any angle
            vel[i] = 0.3f + gaussianNoise()*0.1;
            angvel[i] = gaussianNoise()*0.1;
        }
    }
    noCar = false; // To break extensive search. This will be tried on next image search.
}

void ParticleFilter::updateFilter()
{
    // Update the filter using an image as measurement
    if(imageMode)
    {
        /*if (carGone())
        {
            qDebug("noCarCounter: %i ",noCarCounter);
            if(1)//noCarCounter > 0)
            {
                time = omp_get_wtime();
                std::cout << "Class ParticleFilter:	Could not find any matching hypotheses. Attempting extensive search" << std::endl;
                extensiveSearch(m_img);
                noCarCounter = 2;
            }
            else
            {
                qDebug("hej");
                noCarCounter++;
                T = ((double)(omp_get_wtime() - time));
                time = omp_get_wtime();
                //omp_set_num_threads(3);
                //#pragma omp parallel for
                for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
                {
                    /*
                    xhatpred(2,i) = xhat(2,i);
                    xhatpred(4,i) = xhat(4,i);
                    xhatpred(3,i) = xhat(3,i) + xhatpred(4,i)*T;
                    if (xhatpred(3,i) < -M_PI || xhatpred(3,i) > M_PI)
                        xhatpred(3,i) = fmod(xhatpred(3,i) + 3*M_PI, 2*M_PI) - M_PI;
                    xhatpred(0,i) = xhat(0,i) + 2 * xhatpred(2,i) / xhatpred(4,i) * sin(xhatpred(4,i)*T / 2) * cos(xhat(3,i) + xhatpred(4,i)*T / 2);
                    xhatpred(1,i) = xhat(1,i) + 2 * xhatpred(2,i) / xhatpred(4,i) * sin(xhatpred(4,i)*T / 2) * sin(xhat(3,i) + xhatpred(4,i)*T / 2);

                    // update points with CT model
                    velPoints[i] = vel[i];
                    angvelPoints[i] = angvel[i];
                    yawPoints[i] = yaw[i] + angvelPoints[i]*T;
                    if (yawPoints[i] < -M_PI || yawPoints[i] > M_PI)
                        yawPoints[i] = fmod(yawPoints[i]+ 3*M_PI, 2*M_PI) - M_PI;
                    posXPoints[i] = posX[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * cos(yaw[i] + angvelPoints[i]*T / 2);
                    posYPoints[i] = posY[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * sin(yaw[i] + angvelPoints[i]*T / 2);

                }
                parallelUpdate(m_img);
                systematicResample();
            }*/
        if (carGone())
        {
            std::cout << "Class ParticleFilter:	Could not find any matching hypotheses. Attempting extensive search" << std::endl;
            extensiveSearch(m_img);
        }
        else
        {
            propagate();
            update(m_img);
            resample();
        }

        /*else
        {
            //Propagate using camera coordinates
            propagate();
            parallelUpdate(m_img);
            systematicResample();

        }*/
        //drawThreadData.sumStates = getSumStates();
    }
    else // Update the filter using x, y, theta values as measurements
    {
        if (carGone())
        {
            qDebug("gar Gone");
            for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
            {
                float yawGuess = ((rand() / ((float)RAND_MAX))*M_PI * 2) - M_PI; // pick any angle
                yaw[i] = yawGuess;
                posX[i] = m_x + 0.1*gaussianNoise();
                posY[i] = m_y + 0.1*gaussianNoise();
                vel[i] = 0.3f + 0.1*gaussianNoise();
                angvel[i] = 0.1*gaussianNoise();
            }
            noCar = false;
        }
        if(m_newMeasurement)
        {
            switch(mType)
            {
                case MotionModelType::Enum::CTModel:
                {
                    propagateCTWorldCoordinates();
                }
                    break;
                case MotionModelType::Enum::STModel:
                {
                    //std::cout << "st";
                }
                    break;
                default:
                    propagateCTWorldCoordinates();
            }

            noImgUpdate(m_x, m_y, m_theta);
            systematicResample();
        }
        else // If a new measaurement was not found
        {
            switch(mType)
            {
                case MotionModelType::Enum::CTModel:
                {
                    //Update according to model prediction
                    for(int i = 0; i < NUMBER_OF_PARTICLES; i++)
                    {
                        vel[i] = velPoints[i];
                        angvel[i] = angvelPoints[i];
                        yaw[i] = yawPoints[i] + angvel[i]*T;
                        if (yawPoints[i] < -M_PI || yawPoints[i] > M_PI)
                            yawPoints[i] = fmod(yawPoints[i]+ 3*M_PI, 2*M_PI) - M_PI;
                        posX[i] = posXPoints[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * cos(yaw[i] + angvelPoints[i]*T / 2);
                        posY[i] = posYPoints[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * sin(yaw[i] + angvelPoints[i]*T / 2);
                    }
                    noImgUpdate(m_x, m_y, m_theta);
                    systematicResample();
                }
                    break;
                case MotionModelType::Enum::STModel:
                {
                    // Propagate according to STModel
                }
                    break;
                default:
                std::cout << "default";
            }

        }
    // SHOULD BE REMOVED drawThreadData.sumStates = getSumStates();
    }
    m_newMeasurement = false;
}

int ParticleFilter::findFirst(const float value)
{
    for (int i = 0; i < NUMBER_OF_PARTICLES; ++i)
    {
        if (cumulativeWeights[i] >= value)
        {
            return i;
        }
    }
    std::cout << "Class ParticleFilter:	Error in findFirst()" << std::endl;
    //std::cout << "Class ParticleFilter:	Trying to find > " << value << "in array with max " << cumulativeWeights[NUMBER_OF_PARTICLES - 1] << std::endl;
    return -1;
}

void ParticleFilter::logStates(std::ofstream *logFile)
{
    *logFile << sumStates[0] / NUMBER_OF_PARTICLES << " " << sumStates[1] / NUMBER_OF_PARTICLES << " " << sumStates[3] / NUMBER_OF_PARTICLES << std::endl;
}

std::vector<float> ParticleFilter::getState(void)
{
    std::vector<float> state(M->getNumStates());
    for (int i=0; i < M->getNumStates(); i++)
    {
        state[i] = sumStates[i]/NUMBER_OF_PARTICLES;
        std::cout << "i: " << i << " " << state[i] << std::endl;
    }
    if(sumStates[3]/NUMBER_OF_PARTICLES > M_PI || sumStates[3]/NUMBER_OF_PARTICLES < -M_PI)
    {
        sumStates[3] = fmod(sumStates[3]+ 3*M_PI, 2*M_PI) - M_PI;
    }
    return state;
}

void ParticleFilter::addInputSignals(float gas, float turn)
{
    m_gas = gas;
    m_turn = turn;
}

void ParticleFilter::addMeasurement(float x, float y, float theta)
{
    m_x = x;
    m_y = y;
    m_theta = theta;
    m_newMeasurement = true;
}

void ParticleFilter::addImageMeasurement(cv::Mat img)
{
    img.copyTo(m_img);
    m_newMeasurement = true;
}

bool ParticleFilter::hasNewMeasurement(void)
{
    return m_newMeasurement;
}

void ParticleFilter::nonLinearUndistort(float input[2], float output[2])
{
    double k1, k2, p1, p2, k3, fx, cx, fy, cy, z, x, y, r2, dx, dy, scale, xBis, yBis;
    // Following a somewhat standardized notation for distortion coefficients, the matrix elements are as follows:
    k1 = distCoeffs.at<double>(0, 0);
    k2 = distCoeffs.at<double>(0, 1);
    p1 = distCoeffs.at<double>(0, 2);
    p2 = distCoeffs.at<double>(0, 3);
    k3 = distCoeffs.at<double>(0, 4);
    fx = cameraMatrix.at<double>(0, 0);
    cx = cameraMatrix.at<double>(0, 2);
    fy = cameraMatrix.at<double>(1, 1);
    cy = cameraMatrix.at<double>(1, 2);
    z = 1.;

    x = (input[0] - cx) / fx;
    y = (input[1] - cy) / fy;
    r2 = x*x + y*y;

    dx = 2 * p1*x*y + p2*(r2 + 2 * x*x);
    dy = p1*(r2 + 2 * y*y) + 2 * p2*x*y;
    scale = (1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);

    xBis = x*scale + dx;
    yBis = y*scale + dy;

    output[0] = xBis*fx + cx;
    output[1] = yBis*fy + cy;
}

void ParticleFilter::cameraToWorldCoordinates(float cameraPoint[2], float worldPoint[2])
{
    float nonLinearUndistorted[2];
    Eigen::Vector3f worldPoint_Eigen;
    Eigen::Vector3f undistCameraPoint;

    nonLinearUndistort(cameraPoint, nonLinearUndistorted);

    undistCameraPoint << nonLinearUndistorted[0], nonLinearUndistorted[1], 1;

    worldPoint_Eigen = cameraToWorldMatrix_Eigen*undistCameraPoint;
    worldPoint_Eigen = worldPoint_Eigen / worldPoint_Eigen[2];
    worldPoint[0] = worldPoint_Eigen[0]/PIXELS_PER_METER;
    worldPoint[1] = worldPoint_Eigen[1]/PIXELS_PER_METER;
}

void ParticleFilter::LoadTrack() {
  int x, y;
  std::ifstream in("track.txt");

  if (!in) {
    std::cout << "Cannot open file.\n";
    return;
  }

  for (y = 0; y < 1200; y++) {
    for (x = 0; x < 1500; x++) {
      in >> trackConstraints[y][x];
    }
  }
  in.close();
}

float ParticleFilter::calcDutycycles()
{
    return kThrottle*m_gas + mThrottle;
}

float ParticleFilter::calcThetaF()
{
    return kSteer*m_turn + mSteer;
}

float ParticleFilter::calcAlphaF(float Vx, float Vy, float omegaZ, float thetaF)
{
    return atan((Vy + lf*omegaZ)/Vx) - thetaF;
}

float ParticleFilter::calcLatForce(float alphaF)
{
    return -Cf*alphaF;
}
