#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#define NOMINMAX

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

#include <QElapsedTimer>
#include <QVector>

#include <stdlib.h>
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
        PIDdefault,
        PIDuser,
        PIDadaptiveGain,
        PIDadaptiveSection,
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
// Struct passed from findCars();
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
    MotionModelType::Enum modelType;
    FilterType::Enum filter;
    std::vector<float> state;
    LapData lapData;
    HandController::Enum handController;
};

//Struct to pass plot data to mainWindow
/*
 *  Every plot window has its own global struct.
 *
 *  makePlot - indicates wheter or not to makes plots.  if false no data will be saved
 *             to the struct and no data will be read. On program start up settingsfile
 *             is read, and makePlot is updated accordingly.
 *  newDeataReady - vector with booleans indicating if the data in Y[i] is updated
 *  X - vector containg all x-vectors
 *  Y - vector containg all y-vectors
 *  axisRange - vector with max-min values. [Xmin Xmax Ymin Ymax]
 *  numOfGraphs - the number of graphs currently active in plotWindow
 * */
struct PlotData
{
    std::string name;
    bool makePlot = false;
    std::vector<bool> newDataReady;
    QVector<QVector<double>> X,Y;
    QVector<double> axisRange;
    int numOfGraphs = 1;

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
extern struct PlotData AdaptiveRefPlotData, AdaptiveTimePlotData, AdaptiveGainPlotData, UserSpeedPlotData;
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
