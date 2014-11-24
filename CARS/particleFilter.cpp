
#include "definitions.h"
#include "headers.h"
#include "ParticleFilter.h"
#include "Filter.h"
#include "CTModel.h"
#include <Eigen/Dense>
#include <math.h>
#include <QDebug>
#include <omp.h>





ParticleFilter::ParticleFilter(Eigen::MatrixXf ID, float speed, int lim, MotionModelType::Enum motionModelType)
{
    switch (motionModelType)
    {
        case MotionModelType::CTModel:
            M = new CTModel();
            break;
        default:
            std::cout << "Error: Motion model type not implemented, in ParticleFilter::ParticleFilter(), ParticleFilter.cpp" << std::endl;
    }

    carPattern = ID;
    expectedSpeed = speed;
    limit = lim;
    noCar = true;
    xhat = VectorXd::Zero(M->getNumStates());
    sumStates[0] = 0;
    sumStates[1] = 0;
    sumStates[2] = 0;
    m_img = cv::imread("indata/ImageWithCar.jpg", CV_LOAD_IMAGE_GRAYSCALE);
}

ParticleFilter::~ParticleFilter()
{
}

void ParticleFilter::setState(float state[3])
{
    for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
    {
        posX[i] = state[0];
        posY[i] = state[1];
        yaw[i] = state[3];
    }
}

void ParticleFilter::propagate(void)
{

    omp_set_num_threads(3);
#pragma omp parallel for
    for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
    {
        // update points with random walk model
        velPoints[i]  = expectedSpeed + 25 * gaussianNoise();
       // velPoints[i]  = vel[i] + 1 * gaussianNoise();
        //if(velPoints[i] < -0.2)
       //     velPoints[i] = 0;
        yawPoints[i] = yaw[i] + M_PI / 5 * gaussianNoise() * 2; // angle
        posXPoints[i] = posX[i] + velPoints[i]*sin(yawPoints[i]);
        posYPoints[i] = posY[i] - velPoints[i]*cos(yawPoints[i]);

    }
}

void ParticleFilter::propagateCT(void)
{
    for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
    {
        // update points with CT model
        velPoints[i] = vel[i] + gaussianNoise()*0.1;
        if(velPoints[i] < 0)
            velPoints[i] = 0;
        yawPoints[i] = yaw[i] + angvel[i]*T;
        angvelPoints[i] = angvel[i] + gaussianNoise()*0.1;
        posXPoints[i] = posX[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * cos(yaw[i] + angvelPoints[i]*T / 2);
        posYPoints[i] = posY[i] + 2 * velPoints[i] / angvelPoints[i] * sin(angvelPoints[i]*T / 2) * sin(yaw[i] + angvelPoints[i]*T / 2);


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
    std::cout << "For time: " << tEnd - tStart << std::endl;
}

// Parallelized version of update
void ParticleFilter::parallelUpdate(const cv::Mat img)
{
    // Add border values around each point controlled

    //cv::Rect ROI = cv::Rect(0, 0, limit, limit);
    //cv::Mat imgROI(limit, limit, CV_8UC1, cv::Scalar(0, 0, 0));
    double tStart = 0;
    double tEnd = 0;

    int points = carPattern.rows();
    Eigen::Matrix2f rotate, translate;
    Eigen::MatrixXf pos;
    Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(2, points);
    float val = 0;
    float sum = 0;
    //tStart = omp_get_wtime();

omp_set_num_threads(3);
#pragma omp parallel for private(rotate, translate, pos, sum, val)
    for (int n = 0; n < NUMBER_OF_PARTICLES; n++)
    {
        // Add border values around each point controlled
        cv::Rect ROI = cv::Rect(0, 0, limit, limit);
        cv::Mat imgROI(limit, limit, CV_8UC1, cv::Scalar(0, 0, 0));

        // Rotate and translate pattern according to hypothesis
        rotate << cos(yawPoints[n]), -sin(yawPoints[n]),
            sin(yawPoints[n]), cos(yawPoints[n]);
        translate << posXPoints[n], 0,
            posYPoints[n], 0;
        pos = rotate*carPattern.transpose() + translate*ones;

        //std::cout << (int)pos(0,0) << " " << (int)pos(1,0) << std::endl;

        val = 0.;
        sum = 0.;
        for (int p = 0; p < points; p++)
        {
            //std::cout << img.cols << std::endl;
            // Update ROI
            ROI.x = pos(0, p);
            ROI.y = pos(1, p);
            //std::cout << "checking: [" << ROI.x << ", " << ROI.y << std::endl;
            if (ROI.x - limit > 0 && ROI.x + limit < img.cols && ROI.y - limit > 0 && ROI.y + limit < img.rows)
            {
                {
                    imgROI = img(ROI);
                }
                //std::cout << img.cols << std::endl;
                cv::Scalar mean = cv::mean(imgROI);

                // Throws the hypothesis if no point found. This gives some speedup.
                if (mean.val[0] == 0)
                {
                    //std::cout << img.rows;
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
        for (int i = 0; i < NUMBER_OF_PARTICLES - 1; ++i)
        {
            cumulativeWeights[i] /= cumulativeWeights[NUMBER_OF_PARTICLES - 1];
            ordNum[i + 1] = ((i + 1) + (rand() / ((float)RAND_MAX))) / NUMBER_OF_PARTICLES;
        }
        cumulativeWeights[NUMBER_OF_PARTICLES - 1] /= cumulativeWeights[NUMBER_OF_PARTICLES - 1];

        k = 0;
        for(int j = 0; j < NUMBER_OF_PARTICLES - 1; j++){

            while(cumulativeWeights[k] < ordNum[j])
                k++;

            posX[j] = posXPoints[k];
            sumStates[0] += posXPoints[k];
            posY[j] = posYPoints[k];
            sumStates[1] += posYPoints[k];
            vel[j] = velPoints[k];
            sumStates[2] += velPoints[k];
            yaw[j] = yawPoints[k];
            sumStates[3] += yawPoints[k];
            angvel[j] = angvelPoints[k];
            sumStates[4] += angvelPoints[k];
        }
    }
    else
    {
        noCar = true;
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
                vel[i] = 0.3f;
                angvel[i] = vel[i];
            }
        }
        else // Look over entire image
        {
            for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
            {
                posX[i] = (rand() / ((float)RAND_MAX))*img.cols; // pick any posX
                posY[i] = (rand() / ((float)RAND_MAX))*img.rows; // pick any posY
                yaw[i] = (rand() / ((float)RAND_MAX))*M_PI * 2; // pick any angle
                vel[i] = 0.3;
                angvel[i] = vel[i];
            }
        }
        noCar = false; // To break extensive search. This will be tried on next image search.
    }

void ParticleFilter::addInputSignals(float gas, float turn)
{
    //Dummy
}

std::vector<float> ParticleFilter::getState(void)
{
    std::vector<float> state(xhat.size());
    for (int i=0; i < xhat.size(); i++)
    {
        state[i] = sumStates[i]/NUMBER_OF_PARTICLES;
    }

    return state;
}

void ParticleFilter::updateFilter()
{
    //double time1 = 0, time2 = 0, time3 = 0, time4 = 0;
    if (carGone())
    {
        std::cout << "Class ParticleFilter:	Could not find any matching hypotheses. Attempting extensive search" << std::endl;
        extensiveSearch(m_img);
    }
    else
    {
        //time1 = omp_get_wtime();
        propagate();
        //time2 = omp_get_wtime();
        parallelUpdate(m_img);
        //time3 = omp_get_wtime();
        //update(m_img);
        systematicResample();
        //time4 = omp_get_wtime();
    }
    //std::cout << time2 - time1 << " " << time3 - time2 << " " << time4 - time3 << std::endl;
    drawThreadData.sumStates = getSumStates();
}

void ParticleFilter::addMeasurement(float x, float y, float theta)
{
    //Dummy
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
