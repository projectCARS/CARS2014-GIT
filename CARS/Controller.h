#pragma once

class Controller
{
public:
	/* This class does not need a constructor since it has no member 
	variables that needs to be initialized.*/

	// Virtual destructor. Must exist so that the destructor of a derived class can be called.
    virtual ~Controller(){}

	// Calculate input signals to controller. Result is written to the vector signals.
	//virtual void calcSignals(std::vector<float>::iterator start, std::vector<float>::iterator end) = 0;

    virtual bool isBacking(void) = 0;
	// Calculates and returns a vector with gas and turn signal.
    virtual void calcSignals(std::vector<float> &state, float &gas, float &turn) = 0;
	// Calculates and returns a turn signal.
    virtual void calcTurnSignal(std::vector<float> &state, float &turn) = 0;
};
