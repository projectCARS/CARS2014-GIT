#pragma once

#include "definitions.h"

void updateLapData(CarData &cardata);

void logData(float sysTime, std::vector<CarData> carData, std::vector<CarMeasurement> carMeasurements, std::vector<Signal> signal, std::ofstream *logFile);

// Used in class Calibrator to check for car existence.
bool fileExists(const std::string &file);

void drawStatesToImg(Eigen::MatrixXf carPattern, float *posX, float *posY, float *yaw, cv::Mat img, int selection);

void drawCar(Eigen::MatrixXf carPattern, float x, float y, float yaw, cv::Mat img);
