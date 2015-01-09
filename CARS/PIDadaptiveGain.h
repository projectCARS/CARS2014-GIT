#ifndef PIDadaptiveGain_H
#define PIDadaptiveGain_H

#pragma once
#include "Controller.h"
#include <chrono>
#include <fstream>
#include <QElapsedTimer>
#include "qtheaders.h"
#include <stdlib.h>
#include <time.h>



class PIDadaptiveGain : public Controller
{
private:

    int m_ID;
    int m_startInd;
    int m_refIndCircle;
    int m_refIndClosest;
    float m_dist_lateral;
    float Kp_forward;
    float Kp_brake;
    float Ki;
    float Kd;
    std::vector<float> m_turnPID;

    float m_prevI = 0;
    float m_prevIturn = 0;
    float m_prevD;
    float m_prevAngError;
    float m_refSpeed;


    std::vector<float> m_vRef;
    std::vector<float> m_aRef;
    double m_gain, m_bGain;
    double m_offset;
    std::vector<float> m_LapError;
    bool m_checkPoint, m_firstLapDone;
    QElapsedTimer timer;
    float m_lLap, m_bLap;

    std::chrono::time_point<std::chrono::system_clock> start, end;

public:
    PIDadaptiveGain(int ID);
    ~PIDadaptiveGain();

    // Calculates and returns a vector with gas and turn signal.
    virtual void calcSignals(std::vector<float> &state, float &gas, float &turn);
    // Calculates and returns a turn signal.
    virtual void calcTurnSignal(std::vector<float> &state, float &turn);
    //floacalcGasSignaled(std::vector<float> &state);
    virtual bool isBacking(void){ return false; }

private:
    // Finds closest point on the reference path to the car
    void findClosestReferencePoint(std::vector<float> &state);
    // Finds the intersection between the reference curve and the circle around the car with radius gCarRadius.
    int findIntersection(std::vector<float> &state, int m_startInd);
    // Calculates turn signal based on point on reference curve.
    float calcTurnSignal(std::vector<float> &state, int refInd);
    // Calculates gas signal based on speed profile.
    float calcGasSignal(std::vector<float> &state, float m_refSpeed);
    // Calcultes reference speed for regulation
    float calcRefSpeed(int refInd, double gain);
    // Update speed reference gain(m_gain and possibly m_offset)
    void updateSpeedReferenceGain();
    // is the current lap Done? Meaning "is the car on the finishing line?"
    bool lapDone(std::vector<float> &state);

};
#endif // PIDadaptiveGain_H
