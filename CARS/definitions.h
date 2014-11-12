#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// This file contains definitions, macros, etc.
#pragma once

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

// Controls which parameters are used in PIDController.
#define safeMode
// #define fullForce

// ---------- Globals ----------
// Radius of circle (squared) surrounding the car. Used in controller.
const float gCarRadius = (TRACK_WIDTH / 2 + 0.05)*(TRACK_WIDTH / 2 + 0.05)*0.8;

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

namespace FilterType
{
    enum Enum
    {
        EKF,
        NoFilter,
    };
}

namespace MotionModelType
{
    enum Enum
    {
        CTModel,
    };
}

namespace ControllerType
{
    enum Enum
    {
        PIDController,
        PIDControllerSR,
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

// Struct used by processing thread to send data to other threads.
struct CarData
{
    int id;
    bool active;
    // True if the car is active but is not detected, otherwise false.
    bool lost;
    CarMode::Enum mode;
    std::vector<float> state;
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

    cv::Mat image;
    cv::Mat processedImage;
    //cv::Mat evaluatedImage;
    std::vector<CarData> carData;
};

// Struct to store lap data.
struct LapData
{
    int checkPoint = 730;
    int check1, check2;
    //clock_t startTime, t2;
    float currTime, lapTime, bestTime;
    std::vector<float> bestTimes;
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
extern struct LapData lapData;
// Critical section that is used during communication between main thread and draw thread.
extern CRITICAL_SECTION csDrawThreadData, csControllerThreadData;
extern HANDLE hDrawThreadEvent;
extern HANDLE hControllerThreadEvent1, hControllerThreadEvent_signalsWritten, hControllerThreadEvent_signalsRead;
// Reference signals. Should be placed in regulatorThreadData?
extern std::vector<float> gRef;
extern std::vector<float> vRef;
// Length of reference signal (number of coordinate pairs).
extern int gRefLen;

#endif // DEFINITIONS_H
