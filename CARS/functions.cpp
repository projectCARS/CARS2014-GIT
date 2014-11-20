// Function declarations.
#include "headers.h"
#include "classes.h"
#include "definitions.h"
#include "functions.h"

/*#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/features2d.hpp"*/



void updateLapData(CarData &cardata)
{
    float carX, carY, dist;
    carX = cardata.state[0];
    carY = cardata.state[1];

    //Dont do anything until car has passed start line for the first time.
    if (!cardata.lapData.firstLapStarted)
    {
        if (carY<.5 && carX > 1.3 && carX < 1.7) //only proced if car is near the finish line
        {
            //coordinates for point(Q) on finish line closest to car. Qx=1.5 and Qy=carY
            float Qx = 1.5;
            dist = abs(carX-Qx);
            if (dist<0.02)
            {
                cardata.lapData.lapTimer.restart();
                cardata.lapData.firstLapStarted = true;
                cardata.lapData.bestTime = 1000;
                qDebug() << "start first lap";
            }
        }

    }
    else
    {
        cardata.lapData.lapTime = cardata.lapData.lapTimer.elapsed();
        cardata.lapData.lapTime /= 1000;
        lapData.lapTime = lapData.lapTimer.elapsed()/1000.0;
        //lapData.lapTime /= 1000;
        if (carY<.5 && carX > 1.3 && carX < 1.7) //is car near finishing line?
        {
            //find distance to finnishing line
            float Qx = 1.5;
            dist = abs(carX-Qx);
            //is dist very small(and timer big) save lap time and reset timer
            if (dist < 0.02 && cardata.lapData.lapTime > 3)
            {
                cardata.lapData.firstLapDone = true;
                cardata.lapData.lastLapTime = cardata.lapData.lapTime;
                cardata.lapData.lapTimer.restart();
                qDebug() << "lapTime: " << cardata.lapData.lapTime;
                if (cardata.lapData.lapTime < cardata.lapData.bestTime)
                {
                    cardata.lapData.bestTime = cardata.lapData.lapTime;
                    std::cout << "bestTime: " << cardata.lapData.bestTime << std::endl;
                }
            }
            //update LapData with information


        }
    }
}

// Used in class Calibrator
bool fileExists(const std::string& file) {
    struct stat buf;
    return (stat(file.c_str(), &buf) == 0);
}

void logData(float sysTime, std::vector<CarData> carData, std::vector<CarMeasurement> carMeasurements, std::vector<Signal> signal, std::ofstream *logFile)
{
    int precision = 3; //Floating point precision in logging

    for (int i = 0; i < carData.size(); i++)
    {
        if (i == 0) // For now, log only car '0'
        {
            (*logFile) << std::setprecision(5) << sysTime << " " << carData[i].id << " ";

            for (int j = 0; j < carData[i].state.size(); j++)
            {
                (*logFile) << std::setprecision(precision) << carData[i].state[j] << " ";
            }

            for (int j = 0; j < carMeasurements.size(); j++)
            {
                if (carMeasurements[j].id == 0) // For now, log only car '0'
                {
                    (*logFile) << std::setprecision(precision) << carMeasurements[j].x << " " << carMeasurements[j].y << " " << carMeasurements[j].theta << " ";
                }
            }
            if (carMeasurements.size() == 0)
            {
                (*logFile) << "NaN NaN NaN ";
            }

            (*logFile) << std::setprecision(precision) << signal[i].gas << " " << signal[i].turn << " ";

            (*logFile) << "\n";
        }
        //std::cout << "[" << carData[i].state.size() << ", " << carMeasurements.size() << ", " << signals.size() << "]" << std::endl;
    }
}

void drawStatesToImg(Eigen::MatrixXf carPattern, float *posX, float *posY, float *yaw, cv::Mat img, int selection)
{
    int points = carPattern.rows();
    Eigen::Matrix2f rotate, translate;
    Eigen::MatrixXf pos;
    Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(2, points);
    switch (selection)
    {
    case 1:
    {
        for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
        {
            rotate << cos(yaw[i]), -sin(yaw[i]),
                    sin(yaw[i]), cos(yaw[i]);
            translate << posX[i], 0,
                    posY[i], 0;
            pos = rotate*carPattern.transpose() + translate*ones;

            for (int p = 0; p < points; p++)
            {
                if (pos(0, p) > 0 && pos(0, p) < img.cols && pos(1, p) > 0 && pos(1, p) < img.rows)
                {
                    cv::circle(img, cv::Point(pos(0, p), pos(1, p)), 1.5, cv::Scalar(255, 150, 150), 1, 8, 0);
                }
            }
        }
    }
        break;
    case 2:
    {
        for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
        {
            rotate << cos(yaw[i]), -sin(yaw[i]),
                    sin(yaw[i]), cos(yaw[i]);
            translate << posX[i], 0,
                    posY[i], 0;
            pos = rotate*carPattern.transpose() + translate*ones;

            for (int p = 0; p < points; p++)
            {
                if (pos(0, p) > 0 && pos(0, p) < img.cols && pos(1, p) > 0 && pos(1, p) < img.rows)
                {
                    cv::circle(img, cv::Point(pos(0, p), pos(1, p)), 4, cv::Scalar(0, 0, 255), -1, 8, 0);
                }
            }
        }
    }
        break;
    }
}

void drawCar(Eigen::MatrixXf carPattern, float x, float y, float yaw, cv::Mat img)
{
    int points = carPattern.rows();
    Eigen::Matrix2f rotate, translate;
    Eigen::MatrixXf pos;
    Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(2, points);


    rotate << cos(yaw), -sin(yaw),
            sin(yaw), cos(yaw);
    translate << x, 0,
            y, 0;
    pos = rotate*carPattern.transpose() + translate*ones;

    for (int p = 0; p < points; p++)
    {
        if (pos(0, p) > 0 && pos(0, p) < img.cols && pos(1, p) > 0 && pos(1, p) < img.rows)
        {
            cv::circle(img, cv::Point(pos(0, p), pos(1, p)), 4, cv::Scalar(255, 0, 0), -1, 8, 0);
        }
    }



}
