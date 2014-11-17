#pragma once
#include "Filter.h"
#include <Eigen/Dense>
#include "definitions.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Returns unfiltered measurement data

// Sub class, inherit Filter
class NoFilter : public Filter
{
private:
	VectorXd xhat, z;
	double T;
	int time;

	// True if a new measurement is avaliable, otherwise false.
	bool m_newMeasurement;

public:
	NoFilter();
	~NoFilter();

	// Add a new measurement to the filter.
    virtual void addMeasurement(float x, float y, float theta);
    // Add a new image measurement to the filter.
    virtual void addImageMeasurement(cv::Mat img){}
	// Add a new set of inputsignals to the filter
	virtual void addInputSignals(float gas, float turn);
	// Create new state estimates.
	virtual void updateFilter(void);
	// Get current state estimets.
	virtual std::vector<float> getState(void);
	// Informs you if the filter has recieved new measurements.
    virtual bool hasNewMeasurement(void) { return m_newMeasurement; }


private:
	
	// Relinearizes the model matrixes around current states
	virtual void updateModel();
};
