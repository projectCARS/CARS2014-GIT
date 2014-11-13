#pragma once

// Abstract class.

class Filter
{
public:
    ~Filter() {}

	// Pure virtual methods.
    // Add a new measurement to the filter.
    virtual void addMeasurement(float x, float y, float theta) = 0;
    // Add a new set of inputsignals to the filter
    virtual void addInputSignals(float gas, float turn) = 0;
    // Create new state estimates.
    virtual void updateFilter(void) = 0;
    // Get current state estimets.
	virtual std::vector<float> getState(void) = 0;
    // Informs you if the filter has recieved new measurements.
	virtual bool hasNewMeasurement(void) = 0;
};
