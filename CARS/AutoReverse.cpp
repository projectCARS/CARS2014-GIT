#include "headers.h"
#include "definitions.h"
#include "AutoReverse.h"
#include "functions.h"
#include "PIDController.h"
#include <chrono>

AutoReverse::AutoReverse()
{
	m_backingCounter = 0;
}

AutoReverse::~AutoReverse()
{
}

// Calculates and returns a vector with gas and turn signal.
void AutoReverse::calcSignals(std::vector<float> &state, float &gas, float &turn)
{
    gas = backingSequence(state);
	turn = 0;
}

// Calculates and returns a turn signal.
void AutoReverse::calcTurnSignal(std::vector<float> &state, float &turn)
{
    // Dummie
}

float AutoReverse::backingSequence(std::vector<float> &state)
{

	float signal = 0;
	m_isBacking = true;
	
	if (m_backingCounter <= 20)
	{
		m_backingCounter++;
		signal = -1;						//full negative throttle
		m_prevSignal = signal;
	}
	else if (m_backingCounter <= 150)
	{
		m_backingCounter++;
        signal = m_prevSignal/1.5 - 0.005;			//Slow transition to Neutral voltage
		m_prevSignal = signal;
		m_startPosX = state[0];
		m_startPosY = state[1];
	}
	else if (m_backingCounter <= 350)
	{
		m_backingCounter++;
        signal = -0.1f;						//Slow back
	}
	else
	{
		m_backingCounter = 0;
        float backinDistance = sqrt(pow((m_startPosX - state[0]), 2) + pow(m_startPosY - state[1], 2));
        if (backinDistance < 0.05)
        {
			m_isBacking = true;
            //std::cout << "Error: Backing sequence failed, please hold on for another try" << std::endl;
		}
        else
        {
            m_isBacking = false;
		}
	}
	return signal;
}
