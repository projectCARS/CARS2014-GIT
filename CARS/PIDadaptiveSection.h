#ifndef PIDADAPTIVESECTION_H
#define PIDADAPTIVESECTION_H

#pragma once
#include "Controller.h"
#include <chrono>
#include <fstream>
#include <QElapsedTimer>
#include "qtheaders.h"
#include <stdlib.h>
#include <time.h>



class PIDadaptiveSection : public Controller
{
private:

    int m_ID;
    int m_startInd;
    int m_refIndCircle;
    int m_refIndClosest;
    float m_dist_lateral;
    float m_turnGain;
    float Kp_forward;
    float Kp_brake;
    float Ki;
    float Kd;
    std::vector<float> m_turnPID;

    float m_prevI = 0;
    float m_prevIturn = 0;
    float m_prevD;
    float m_prevAngError;
    float m_prevSpeed;
    float m_refSpeed;
    float m_refGas;

    bool m_onPath;
    float m_oldRefInd;
    std::vector<float> m_vRef;
    std::vector<float> m_aRef;
    double m_gain, m_bGain;
    double m_offset;
    std::vector<float> m_LapError;
    bool m_checkPoint, m_firstLapDone;
    QElapsedTimer timer;
    float m_lLap, m_bLap;

    int m_IndexSection;
    int m_IndexSectionOld;

    float oldIncrease = 0;
    float newIncrease = 0;

    //for section
    float m_numOfInterval;
    std::vector<int> m_noIncreaseCounter;
    std::vector<int> m_noDecreaseCounter;
    float m_intervalLength;
    int m_length2mid;
    std::vector<int> m_intervalStartIndexes;
    std::vector<int> m_intervalMidIndexes;
    std::vector<float> m_refSpeedShort;
    std::vector<float> m_refSpeedShortBest;
    std::vector<float> m_times;
    std::vector<float> m_timerTimes;
    std::vector<float> m_timesBest;
    QElapsedTimer timerSection;
    bool m_firstLapStarted;  //section also uses m_firstLapDone declared above
    std::vector<bool> m_sectionVisited;

    //end for section


    std::chrono::time_point<std::chrono::system_clock> start, end;

    QElapsedTimer timeSRgain;
    std::ofstream logFileAdaptive;


public:
    PIDadaptiveSection(int ID);
    ~PIDadaptiveSection();

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
    float calcRefSpeed(int refInd);
    // Update speed reference gain(m_gain and possibly m_offset)
    void updateSpeedReferenceGain();
    //
    void updateSectionTimers(int IndexSection);
    //
    void updateSpeedReference();
    //
    int findClosestSection(int index);
    //
    bool newSectionEntered(int IndexCurrent);

    bool visitedAllSections();

    void newLapStarted();

};
#endif // PIDADAPTIVESECTION_H
