#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

#include <QElapsedTimer>
#include <QVector>


#include <windows.h>


// This file contains definitions, macros, etc.
#pragma once

using Eigen::MatrixXd;
using Eigen::VectorXd;

// ---------- Macros ----------
// Unit convesion; 1m = PIXELS_PER_METER
#define PIXELS_PER_METER 600

// For debug purposes.
#define CAMERA_IS_AVALIABLE
#define NIDAQ_IS_AVALIABLE

// Voltage corresponding to neutral angle and gas.
#define TURN_VOLT_NEUTRAL 1.55
#define GAS_VOLT_NEUTRAL 1.79

// Track width in meters.
#define TRACK_WIDTH 0.445

// Number of particles to use in the particle filter
#define NUMBER_OF_PARTICLES     10000

// Controls which parameters are used in PIDController.
#define safeMode
// #define fullForce

// ---------- Globals ----------
// Radius of circle (squared) surrounding the car. Used in controller.
const double gCarRadius = (TRACK_WIDTH / 2 + 0.05)*(TRACK_WIDTH / 2 + 0.05)*0.8; //default 0.8;

// ---------- Enums ----------
namespace CarMode
{
    enum Enum
    {
        Auto,
        Assisted,
        Manual,
        NotConnected,
    };
}

namespace Racemode
{
    enum Enum
    {
        On,
        Off,
    };
}

namespace FilterType
{
    enum Enum
    {
        EKF,
        ParticleFilter,
        NoFilter,
    };
}

namespace MotionModelType
{
    enum Enum
    {
        CTModel,
        STModel,
    };
}

namespace ControllerType
{
    enum Enum
    {
        PIDController,
        PIDControllerSR,
        PIDadaptiveGain,
    };
}

namespace HandController
{
    enum Enum
    {
        HandControl_1,
        HandControl_2,
    };
}

namespace GeneralDrawSettings
{
    enum Enum
    {
        Reference               = 1 << 0,
        Animations              = 1 << 1,
    };
}

namespace CarSpecificDrawSettings
{
    enum Enum
    {
        Path                    = 1 << 0,
        Circle                  = 1 << 1,
        Rectangle               = 1 << 2,
        CarTracks               = 1 << 3,
        Timer                   = 1 << 4,
    };
}




// ---------- Structs ----------
// Struct passed from fildCars();
struct CarMeasurement
{
    int id;
    float x;
    float y;
    float theta;
};

struct Signal
{
    float gas;
    float turn;
};

// Struct to store race data.
struct RaceSettings
{
    bool doRace = false;
    bool raceStarted = false;
    bool raceDone = false;
    int winnerID;
    int numberOfLaps = 10;
    std::vector<int> carID;
};

// Struct to store lap data.
struct LapData
{
    //int checkPoint = 730;
    //int check1, check2;
    //clock_t startTime, t2;
    //float currTime;             // used to get time for log file.

    QElapsedTimer lapTimer;
    float lapTime, bestTime, lastLapTime;
    bool firstLapStarted = false;
    bool firstLapDone = false;
    int lapNumber = 0;
    //std::vector<float> bestTimes;
};

// Struct used by processing thread to send data to other threads.
struct CarData
{
    int id;
    bool active;
    // True if the car is active but is not detected, otherwise false.
    bool lost;
    CarMode::Enum mode;
    FilterType::Enum filter;
    std::vector<float> state;
    LapData lapData;
    HandController::Enum handController;
};

//Struct to pass plot data from controller to mainWindow
struct PlotData
{
    bool makePlot = false;
    bool newData2plot1 = false;
    bool newData2plot2 = false;
    bool newData2plot3 = false;
    bool newData2plot4 = false;
    QVector<double> Xvalues,Xvalues2,Yvalues1,Yvalues2,Yvalues3,Yvalues4,axisRange;

};

// Perhaps: struct IOControllerData ?

// RawData struct - passed from PGRCamera.
struct RawData
{
    unsigned char *pData;
    unsigned int dataSize;
    unsigned int rows;
    unsigned int cols;
};

// The main thread uses this struct to communicate with the draw thread.
struct DrawThreadData
{
    std::vector<cv::Mat> background;
    std::vector<int> carTimerID;

    cv::Mat image;
    cv::Mat processedImage;
    //cv::Mat evaluatedImage;
    std::vector<CarData> carData;
    RaceSettings raceData;
    float *sumStates;
    Eigen::MatrixXf pattern;
};

// The main thread uses this struct to communicate with the regulator thread.
struct ControllerThreadData
{
    std::vector<CarData> carData;
    std::vector<Signal> signal; // gas, turn.

};

// ---------- External declarations ----------
extern struct DrawThreadData drawThreadData;
extern struct ControllerThreadData controllerThreadData;
extern struct PlotData AdaptiveRefPlotData, AdaptiveTimePlotData;
//extern struct RaceSettings raceSettings;
// Critical section that is used during communication between main thread and draw thread.
extern CRITICAL_SECTION csDrawThreadData, csControllerThreadData, csPlotData;
extern HANDLE hDrawThreadEvent;
extern HANDLE hControllerThreadEvent1, hControllerThreadEvent_signalsWritten, hControllerThreadEvent_signalsRead;
// Reference signals. Should be placed in regulatorThreadData?
extern std::vector<float> gRef;
extern std::vector<float> vRef;
extern std::vector<float> aRef;
// Length of reference signal (number of coordinate pairs).
extern int gRefLen;

#endif // DEFINITIONS_H
