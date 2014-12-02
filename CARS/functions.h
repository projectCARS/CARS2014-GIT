#pragma once

#include "definitions.h"

bool updateRaceData(CarData &cardata, RaceSettings &raceSettings);
void updateLapData(CarData &cardata);


void logData(float sysTime, std::vector<CarData> carData, std::vector<CarMeasurement> carMeasurements, std::vector<Signal> signal, std::ofstream *logFile);

// Used in class Calibrator to check for car existence.
bool fileExists(const std::string &file);

void drawStatesToImg(Eigen::MatrixXf carPattern, float *posX, float *posY, float *yaw, cv::Mat img, int selection);

void drawCar(Eigen::MatrixXf carPattern, float x, float y, float yaw, cv::Mat img);

//used in PIDadaptiveGain to do spline interpolation. code from: http://www.pcs.cnu.edu/~bbradie/cinterpolation.html
void cubic_nak ( int n, double *x, double *f, double *b, double *c, double *d );
double spline_eval ( int n, double *x, double *f, double *b, double *c, double *d, double t );
void tridiagonal ( int n, double *c, double *a, double *b, double *r ); //called by cubic_nak
