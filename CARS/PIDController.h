#pragma once
#include "Controller.h"
#include <chrono>

class PIDController : public Controller
{
private:
    //int m_gain = 1;

    int m_ID;
    int m_startInd;
    int m_refInd;
    float m_turnGain;
    std::vector<float> m_speedPID;
    std::vector<float> m_turnPID;

    float m_prevI;
    float m_prevIturn;
    float m_prevD;
    float m_prevAngError;
    float m_prevSpeed;
    float m_refSpeed;
    float m_refGas;

    std::chrono::time_point<std::chrono::system_clock> start, end, startTurn, endTurn;

    cv::Mat speedProfile;

public:
	PIDController(int ID);
	~PIDController();

	// Calculates and returns a vector with gas and turn signal.
    virtual void calcSignals(std::vector<float> &state, float &gas, float &turn);
    // Calculates and returns a turn signal.
    virtual void calcTurnSignal(std::vector<float> &state, float &turn);
    float findSpeed(std::vector<float> &state);
    virtual bool isBacking(void){ return false; }

private:
	// Finds the intersection between the reference curve and the circle around the car with radius gCarRadius.
	int findIntersection(std::vector<float> &state, int m_startInd);
	// Calculates turn signal based on point on reference curve. 
	float calcTurnSignal(std::vector<float> &state, int refInd);
    // Calculates gas signal based on speed profile.
    float calcGasSignal(std::vector<float> &state, int refGas);
    // Calculates gas signal based on diffAngle.
    float calcGasSignalAlt(std::vector<float> &state, float m_refSpeed);
    float calcRefSpeed(std::vector<float> &state, int refInd);
};
