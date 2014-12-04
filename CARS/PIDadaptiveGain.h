#ifndef PIDADAPTIVEGAIN_H
#define PIDADAPTIVEGAIN_H

#pragma once
#include "Controller.h"
#include <chrono>
#include <fstream>
#include <QElapsedTimer>
#include "qtheaders.h"
#include <stdlib.h>
#include <time.h>
//#include "plotDialog.h"


class PIDadaptiveGain : public Controller
{
private:

    int m_ID;
    int m_startInd;
    int m_refIndCircle;
    int m_refIndClosest;
    float m_dist_lateral;
    float m_turnGain;
    std::vector<float> m_speedPID;
    std::vector<float> m_turnPID;

    float m_prevI;
    float m_prevD;
    float m_prevAngError;
    float m_prevSpeed;
    float m_refSpeed;
    float m_refGas;

    bool m_onPath;
    float m_oldRefInd;
    std::vector<float> m_vRef;
    double m_gain, m_bGain, m_gainBest;
    double m_offset;
    std::vector<float> m_LapError;
    bool m_checkPoint, m_firstLapDone;
    QElapsedTimer timer;
    float m_lLap, m_bLap;

    int m_IndexSection;
    int m_IndexSectionOld;


    //for section
    float m_numOfInterval;
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

    //making plots
   /* bool makePlots;
    plotDialog pD;
    //end plots
    */
    std::chrono::time_point<std::chrono::system_clock> start, end;

    QElapsedTimer timeSRgain;
    std::ofstream logFileAdaptive;


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
    float calcRefSpeed(std::vector<float> &state, int refInd, double gain, double offset);
    // Update speed reference gain(m_gain and possibly m_offset)
    void updateSpeedReferenceGain();
    //
    void updateSectionTimers(int IndexSection);
    //
    void updateSpeedReference();
    //
    bool lapDone(std::vector<float> &state);
    //
    int findClosestSection(std::vector<float> &state);
    //
    bool newSectionEntered(std::vector<float> &state, int IndexCurrent);

};
#endif // PIDADAPTIVEGAIN_H
