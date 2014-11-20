#pragma once
#include "Controller.h"
#include <chrono>
#include <fstream>

#include <QElapsedTimer>
//#include <QTime> //for timeSR

#include "qtheaders.h"
#include <QElapsedTimer>


class PIDControllerSR : public Controller
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

    std::chrono::time_point<std::chrono::system_clock> start, end;

    QElapsedTimer timeSR;
    std::ofstream logFileSR;


public:
    PIDControllerSR(int ID);
    ~PIDControllerSR();

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
    float calcRefSpeed(std::vector<float> &state, int refInd);
    // Updates the refernce speed vector
    void updateSpeedRef(std::vector<float> &state, int refInd, int lateralError);
};
