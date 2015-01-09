
#include "definitions.h"
#include "headers.h"
#include "ParticleFilter.h"
#include "Filter.h"
#include "CTModel.h"
#include "STModel.h"
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

            // Set the Parameters needed for the ST-model
            m = 0.0406f;
            Cm1 = 0.6148f;
            Cm2 = 0.0477f;
            Cm3 = 0.0357f;
            Cf = 0.4835f;
            lf = 0.003f;
            Iz = 0.000016344f;
            kTurn = 0.10f;
            mTurn = -1.355f;
            kThrottle = -2.259f;
            mThrottle = -0.0437f;
            k_alphaF = 0.4835f;
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
            break;
        default:
            std::cout << "Error: Motion model type not implemented, in ParticleFilter::ParticleFilter(), ParticleFilter.cpp" << std::endl;
    }

    // Set this to true to use raw images as measurements instead of x,y,theta measurements as inputs.
    imageMode = false;

    // Load the track constraints from file
    LoadTrack();

    // Initialize the timer
    time = omp_get_wtime();
    carPattern = ID;
    expectedSpeed = speed;
    limit = lim;
    noCar = true;

    T = 1/120;
    noCarCounter = 31;
    //xhat = Eigen::MatrixXf::Zero(M->getNumStates(),NUMBER_OF_PARTICLES);
    //xhatpred = Eigen::MatrixXf::Zero(M->getNumStates(), NUMBER_OF_PARTICLES);
    //sumState = Eigen::VectorXf::Zero(M->getNumStates());

    sumStates[0] = 0;
    sumStates[1] = 0;
    sumStates[2] = 0;
    sumStates[3] = 0;
    sumStates[4] = 0;
    sumStates[5] = 0;

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

// Destructor
ParticleFilter::~ParticleFilter()
{
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
        yawPoints[i] = yaw[i] + M_PI / 5 * gaussianNoise(); // angle
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
        /* TODO...  Add this Code to be used when changing the state vector to a single matrix.
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
        if(yawPoints[i] > M_PI)
            yawPoints[i] = yawPoints[i] - 2*M_PI;
        if(yawPoints[i] < -M_PI)
            yawPoints[i] = yawPoints[i] + 2*M_PI;
        posXPoints[i] = posX[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * cos(yaw[i] + angvelPoints[i]*T / 2);
        posYPoints[i] = posY[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * sin(yaw[i] + angvelPoints[i]*T / 2);

    }
}

// Propagate according to coordinated turn model using world coordinates. Used in noImage mode
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
        if(yawPoints[i] > M_PI)
            yawPoints[i] = yawPoints[i] - 2*M_PI;
        if(yawPoints[i] < -M_PI)
            yawPoints[i] = yawPoints[i] + 2*M_PI;
        posXPoints[i] = posX[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * cos(yaw[i] + angvelPoints[i]*T / 2);
        posYPoints[i] = posY[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * sin(yaw[i] + angvelPoints[i]*T / 2);
    }
}

// Propagate according to Single Track model. Not Updated to work with the updated ST model as of 2015-01-08.
// TODO... Add the correct dynamics from STmodel.cpp, This is also needed in the resampling step
void ParticleFilter::propagateST(void)
{
    // Update the time step
    T = ((double)(omp_get_wtime() - time));
    time = omp_get_wtime();

    dutyCycles = calcDutycycles();
    thetaF = calcThetaF();

    float FLat = 0;

    omp_set_num_threads(3);
#pragma omp parallel for private(FLat)
    for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
    {

        velPoints[i] = (dutyCycles * (Cm1 - Cm2*vel[i]) + thetaF * (FLat - Cm3 * FLat) - Cm3 * latvel[i] * angvel[i]) / m - latvel[i]*angvel[i] + gaussianNoise()*0.05;
        angvelPoints[i] = angvel[i] + T*FLat*lf/Iz + M_PI / 7 * gaussianNoise();
        latvelPoints[i] = FLat/m + vel[i]*angvel[i] + gaussianNoise()*0.05;
        yawPoints[i] = yaw[i] + angvelPoints[i]*T;
        posXPoints[i] = posX[i] + (cos(yawPoints[i])*velPoints[i] + sin(yawPoints[i])*latvelPoints[i])*T;
        posYPoints[i] = posY[i] + (sin(yawPoints[i])*velPoints[i] + cos(yawPoints[i])*latvelPoints[i])*T;
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

// Old update function used to add a weight to each particle. Use parallelUpdate or noImageUpdate instead
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
        totSum += sum;
        cumulativeWeights[n] = totSum; // save weights cumulative sum for later normalization
    }

    tEnd = omp_get_wtime();
    //std::cout << "For time: " << tEnd - tStart << std::endl;
}

// Parallelized version of update. Every particle is paired with a weight used to form a cumulative distribution
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
                imgROI = img(ROI);
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
}

// Update the weights based on measurements from the find cars algorithm
void ParticleFilter::noImgUpdate(float x, float y, float theta)
{
    float sum, angle, measAngle;
    measAngle = theta;
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

 omp_set_num_threads(3);
    #pragma omp parallel for private(angle, sum)
    for(int n = 0; n < NUMBER_OF_PARTICLES; n++)
    {
        // Update number of particles in the region between -pi/2 and pi/2
        if(yawPoints[n] > -(M_PI*0.4) && yawPoints[n] < M_PI*0.4)
        {
            numPartInCritReg++;
        }
        double base = 0;
        float weight = 0;
        float xWorld = (PIXELS_PER_METER*posXPoints[n]);
        float yWorld = (PIXELS_PER_METER*posYPoints[n]);

        // If the hypothesis is outside of the world then discard it
        if(posXPoints[n] < 0 || posXPoints[n] > 2.5 || posYPoints[n] < 0 || posYPoints[n] > 2.0)
        {
            sum = 0;
            noncumulativeWeights[n] = 0;
        }
        // If the hypothesis is outside the track boundaries then discard it
        else if(trackConstraints[(int)yWorld][(int)xWorld] == 0)
        {
            sum = 0;
            noncumulativeWeights[n] = 0;
        }
        else
        {
            sum = 0.;
            angle = yawPoints[n];

            // Add the inverse of the difference between the measured position and particle position to the weight
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

            // Normalize angles to value between 0 and 1
            if(angle < 0)
            {
                angle = (2*M_PI + angle)/(2.0*M_PI);
            }
            else
            {
                angle = angle/(2.0*M_PI);
            }

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

// Resample the particles using multinomial resampling,
// Not updated to work with non-image mode as of (2014-12-02), use systematic resampling instead
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
    // randomly, uniformally, sample from the cumulative distribution of the probability distribution generated by the
    // weighted vector 'weight'.
    float ordNum[NUMBER_OF_PARTICLES];
    float randNr;
    sumStates[0] = 0;
    sumStates[1] = 0;
    sumStates[2] = 0;
    sumStates[3] = 0;
    sumStates[4] = 0;
    int k = 0;

    if ((imageMode && cumulativeWeights[NUMBER_OF_PARTICLES - 1] != 0) || (!imageMode && cumulativeWeights[NUMBER_OF_PARTICLES - 1] > NUMBER_OF_PARTICLES))
    {
        randNr = (rand() / ((float)RAND_MAX)); // Generate a single uniform random number
        ordNum[0] = randNr/NUMBER_OF_PARTICLES;

        // Normalize weights to form a cumulative summed probability distribution
        for (int i = 0; i < NUMBER_OF_PARTICLES - 1; ++i)
        {
            cumulativeWeights[i] /= cumulativeWeights[NUMBER_OF_PARTICLES - 1];
            ordNum[i + 1] = ((i + 1) + randNr) / NUMBER_OF_PARTICLES;
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
                }
                else
                {
                    sumStates[3] += yawPoints[k];
                }
            }
            else
            {
                sumStates[3] += yawPoints[k];
            }

            angvel[j] = angvelPoints[k];
            sumStates[4] += angvelPoints[k];

            if(mType == MotionModelType::STModel) // The ST model has an additional state over the CT model
            {
                latvel[j] = latvelPoints[k];
                sumStates[5] += latvelPoints[k];
            }

            /* TODO... implement a generalized state vector instead of the 5/6 different vectors.
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
                }
                else
                {
                    sumStates[3] += xhatpred(3,k);
                }
            }
            else
                sumStates[3] += xhatpred(3,k);

            xhat(4,j) = xhatpred(4,k);
            sumStates[4] += xhatpred(4,k);
            */
        }
        noCar = false;
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
            if(mType == MotionModelType::STModel)
            {
                latvel[i] = latvelPoints[i];
                sumStates[5] += latvelPoints[i];
            }

            /* TODO... implement a generalized state vector instead of the 5/6 different vectors.
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

// Function used in ImageMode. When the filter can't find the car. Use an image to match a car to the best matching pattern
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

            posXPoints[i] = posX[i];
            posYPoints[i] = posY[i];
            velPoints[i] = vel[i];
            yawPoints[i] = yaw[i];
            angvelPoints[i] = angvel[i];
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

            posXPoints[i] = posX[i];
            posYPoints[i] = posY[i];
            velPoints[i] = vel[i];
            yawPoints[i] = yaw[i];
            angvelPoints[i] = angvel[i];
        }
    }
    noCar = false; // To break extensive search. This will be tried on next image search.
}

// This function is called from the processingThread. Calls all the functions to update the filter.
void ParticleFilter::updateFilter()
{
    // Update the filter using an image as measurement
    if(imageMode)
    {
        if (carGone())
        {
            if(noCarCounter > 50)
            {
                time = omp_get_wtime();
                std::cout << "Class ParticleFilter:	Could not find any matching hypotheses. Attempting extensive search" << std::endl;
                extensiveSearch(m_img);
                noCarCounter = 0;
            }
            else
            {
                noCarCounter++;
                T = ((double)(omp_get_wtime() - time));
                time = omp_get_wtime();
                //omp_set_num_threads(3);
                //#pragma omp parallel for
                for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
                {
                    /* TODO... implement a generalized state vector instead of the 5/6 different vectors.
                    xhatpred(2,i) = xhat(2,i);
                    xhatpred(4,i) = xhat(4,i);
                    xhatpred(3,i) = xhat(3,i) + xhatpred(4,i)*T;
                    if (xhatpred(3,i) < -M_PI || xhatpred(3,i) > M_PI)
                        xhatpred(3,i) = fmod(xhatpred(3,i) + 3*M_PI, 2*M_PI) - M_PI;
                    xhatpred(0,i) = xhat(0,i) + 2 * xhatpred(2,i) / xhatpred(4,i) * sin(xhatpred(4,i)*T / 2) * cos(xhat(3,i) + xhatpred(4,i)*T / 2);
                    xhatpred(1,i) = xhat(1,i) + 2 * xhatpred(2,i) / xhatpred(4,i) * sin(xhatpred(4,i)*T / 2) * sin(xhat(3,i) + xhatpred(4,i)*T / 2);
                    */
                    // update points with CT model
                    velPoints[i] = vel[i];
                    angvelPoints[i] = angvel[i];
                    yawPoints[i] = yaw[i] + angvelPoints[i]*T;
                    if(yawPoints[i] > M_PI)
                        yawPoints[i] = yawPoints[i] - 2*M_PI;
                    if(yawPoints[i] < -M_PI)
                        yawPoints[i] = yawPoints[i] + 2*M_PI;
                    posXPoints[i] = posX[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * cos(yaw[i] + angvelPoints[i]*T / 2);
                    posYPoints[i] = posY[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * sin(yaw[i] + angvelPoints[i]*T / 2);

                }
            }
            parallelUpdate(m_img);
            systematicResample();
        }

        else
        {
            //Propagate using camera coordinates
            propagateCT();
            parallelUpdate(m_img);
            systematicResample();

        }
    }
    else // Update the filter using x, y, theta values as measurements
    {
        if(m_newMeasurement)
        {
            if (carGone()) //Initialize the particles around the measurements.
            {
                for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
                {
                    float yawGuess = ((rand() / ((float)RAND_MAX))*M_PI * 2) - M_PI; // pick any angle
                    yaw[i] = yawGuess;
                    posX[i] = m_x + 0.1*gaussianNoise();
                    posY[i] = m_y + 0.1*gaussianNoise();
                    vel[i] = 0.3f + 0.1*gaussianNoise();
                    angvel[i] = 0.1*gaussianNoise();
                    if(mType == MotionModelType::STModel)
                    {
                        latvel[i] = 0.1*gaussianNoise();
                    }
                }
                noCar = false;
            }
            switch(mType)
            {
                case MotionModelType::Enum::CTModel:
                {
                    propagateCTWorldCoordinates();
                }
                    break;
                case MotionModelType::Enum::STModel:
                {
                    propagateST();
                }
                    break;
                default:
                    propagateCTWorldCoordinates();
            }

            noImgUpdate(m_x, m_y, m_theta);
            systematicResample();
        }
        else // If a new measurement was not found, update the states according to the Model
        {
            switch(mType)
            {
                case MotionModelType::Enum::CTModel:
                {
                    T = ((double)(omp_get_wtime() - time));
                    time = omp_get_wtime();
                        //Update according to CT model prediction
                    float oldPosX = sumStates[0]/NUMBER_OF_PARTICLES;
                    float oldPosY = sumStates[1]/NUMBER_OF_PARTICLES;
                    float oldVel = sumStates[2]/NUMBER_OF_PARTICLES;
                    float oldYaw = sumStates[3]/NUMBER_OF_PARTICLES;
                    float oldAngVel = sumStates[4]/NUMBER_OF_PARTICLES;

                    float newVel = oldVel;
                    float newAngVel = oldAngVel;
                    float newYaw = oldYaw + oldAngVel*T;
                    if(newYaw > M_PI)
                        newYaw = newYaw - 2*M_PI;
                    if(newYaw < -M_PI)
                        newYaw = newYaw + 2*M_PI;
                    float newPosX = oldPosX + 2 * oldVel / oldAngVel * sin(oldAngVel*T / 2) * cos(oldYaw + oldAngVel*T / 2);
                    float newPosY = oldPosY + 2 * oldVel / oldAngVel * sin(oldAngVel*T / 2) * sin(oldYaw + oldAngVel*T / 2);

                    if(newYaw < -(M_PI/2))
                    {
                        newYaw = (newYaw + 2*M_PI);
                    }

                    sumStates[0] = 0;
                    sumStates[1] = 0;
                    sumStates[2] = 0;
                    sumStates[3] = 0;
                    sumStates[4] = 0;

                    for(int i = 0; i < NUMBER_OF_PARTICLES; i++)
                    {
                        sumStates[0] += newPosX;
                        sumStates[1] += newPosY;
                        sumStates[2] += newVel;
                        sumStates[3] += newYaw;
                        sumStates[4] += newAngVel;
                    }                    
                }
                    break;
                case MotionModelType::Enum::STModel:
                {
                    // Propagate according to STModel
                    T = ((double)(omp_get_wtime() - time));
                    time = omp_get_wtime();
                        //Update according to CT model prediction
                    float oldPosX = sumStates[0]/NUMBER_OF_PARTICLES;
                    float oldPosY = sumStates[1]/NUMBER_OF_PARTICLES;
                    float oldVel = sumStates[2]/NUMBER_OF_PARTICLES;
                    float oldYaw = sumStates[3]/NUMBER_OF_PARTICLES;
                    float oldAngVel = sumStates[4]/NUMBER_OF_PARTICLES;
                    float oldLatVel = sumStates[5]/NUMBER_OF_PARTICLES;

                    float newVel = oldVel;
                    float newAngVel = oldAngVel;
                    float newLatVel = oldLatVel;
                    float newYaw = oldYaw + oldAngVel*T;

                    if(newYaw > M_PI)
                        newYaw = newYaw - 2*M_PI;
                    if(newYaw < -M_PI)
                        newYaw = newYaw + 2*M_PI;

                    // Update this to ST
                    float newPosX = oldPosX + 2 * oldVel / oldAngVel * sin(oldAngVel*T / 2) * cos(oldYaw + oldAngVel*T / 2);
                    float newPosY = oldPosY + 2 * oldVel / oldAngVel * sin(oldAngVel*T / 2) * sin(oldYaw + oldAngVel*T / 2);

                    sumStates[0] = 0;
                    sumStates[1] = 0;
                    sumStates[2] = 0;
                    sumStates[3] = 0;
                    sumStates[4] = 0;
                    sumStates[5] = 0;

                    for(int i = 0; i < NUMBER_OF_PARTICLES; i++)
                    {
                        sumStates[0] += newPosX;
                        sumStates[1] += newPosY;
                        sumStates[2] += newVel;
                        sumStates[3] += newYaw;
                        sumStates[4] += newAngVel;
                        sumStates[5] += newLatVel;
                    }
                }
                    break;
                default:
                std::cout << "default";
            }
        }
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

// Returns the relevant states
std::vector<float> ParticleFilter::getState(void)
{
    std::vector<float> state(M->getNumStates());
    for (int i=0; i < M->getNumStates(); i++)
    {
        state[i] = sumStates[i]/NUMBER_OF_PARTICLES;
    }
    if(state[3] > M_PI)
        state[3] = state[3] - 2*M_PI;
    if(state[3] < -M_PI)
        state[3] = state[3] + 2*M_PI;

    return state;
}

// Used when using the ST-model
void ParticleFilter::addInputSignals(float gas, float turn)
{
    float64 signal[2];
    signal[0] = gas;
    signal[1] = turn;
    decimalToVoltage(signal);
    m_gas = signal[0];
    m_turn = signal[1];
}

// Add a new set of measurements to the filter
void ParticleFilter::addMeasurement(float x, float y, float theta)
{
    m_x = x;
    m_y = y;
    m_theta = theta;
    m_newMeasurement = true;
}

// Add a new image measurement to the filter
void ParticleFilter::addImageMeasurement(cv::Mat img)
{
    img.copyTo(m_img);
    m_newMeasurement = true;
}

// Loads the track constraints from file
void ParticleFilter::LoadTrack()
{
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

// Used in ST model
float ParticleFilter::calcDutycycles()
{
    if(m_gas > 1.45 || m_gas < 1.66)
        return 0;
    else
        return  mThrottle + ((p1*pow(m_gas,4) + p2*pow(m_gas,3) + p3*pow(m_gas,2) + p4*m_gas + p5) / (pow(m_gas,4) + q1*pow(m_gas,3) + q2*pow(m_gas,2) + q3*m_gas + q4));
}

// Used in ST model
float ParticleFilter::calcThetaF()
{
    return kTurn*m_turn + mTurn;
}
// Used in ST model
float ParticleFilter::calcFyFront(float thetaF)
{
    return turngain * thetaF;
}

// Calculate the Force in the cars x-direction from the rear wheels
float ParticleFilter::calcFxRear(float D, float Vx)
{
    return Cm1*D - Cm2*D*Vx;
}

// Map interval [-1,1] to [m_minVal,m_maxVal].
void ParticleFilter::decimalToVoltage(float64 *decimal)
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
