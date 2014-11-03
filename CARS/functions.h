#pragma once

void updateLapData(std::vector<float> oldRawStates, std::vector<float> rawStates);

void logData(float sysTime, std::vector<CarData> carData, std::vector<CarMeasurement> carMeasurements, std::vector<Signal> signal, std::ofstream *logFile);

// Used in class Calibrator to check for car existence.
bool fileExists(const std::string &file);


