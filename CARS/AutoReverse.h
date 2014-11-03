#pragma once
#include "Controller.h"
#include "definitions.h"
#include "headers.h"
#include "PIDController.h"

class AutoReverse : public Controller
{
private:
	int m_backingCounter;
	float m_prevSignal;
	float m_startPosX;
	float m_startPosY;

    float backingSequence(std::vector<float> &state);

public:
	AutoReverse();
	~AutoReverse();

	bool m_isBacking;

	virtual bool isBacking(){ return m_isBacking; }
	// Calculates and returns a vector with gas and turn signal.
	virtual void calcSignals(std::vector<float> &state, float &gas, float &turn);
	// Calculates and returns a turn signal.
	virtual void calcTurnSignal(std::vector<float> &state, float &turn);
};

