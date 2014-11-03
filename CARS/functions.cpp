// Function declarations.
#include "headers.h"
#include "classes.h"
#include "definitions.h"
#include "functions.h"

/*#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/features2d.hpp"*/



void updateLapData(std::vector<float> oldRawStates, std::vector<float> rawStates)
{
	// To be implemented.
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
