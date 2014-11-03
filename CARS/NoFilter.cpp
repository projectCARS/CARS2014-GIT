#include "headers.h"
#include "definitions.h"
#include "NoFilter.h"
#include "Filter.h"
#include <Eigen/Dense>
#include <math.h>

NoFilter::NoFilter()
{
	// No need for model, we want to return raw states.

	// Set vector sizes
	xhat.resize(5);
	z.resize(3);
	// Start timing for dynamic time step length
	T = 1 / 150;
	time = clock();
	// Set measurement boolean to false
	m_newMeasurement = false;
}

NoFilter::~NoFilter()
{
}

void NoFilter::updateFilter(void)
{
	// Calculate time step length
	T = ((double)(clock() - time)) / CLOCKS_PER_SEC;
	time = clock();
	//Estimate
	xhat << z(0),
			z(1),
			sqrt(pow(xhat(0) - z(0), 2) + pow(xhat(1) - z(1), 2)) / T,
			z(2),
			(xhat(3) - z(2)) / T;
	// Set measurement boolean to false (we have now used our measurements and turned them into old ones)
	m_newMeasurement = false;
}

void NoFilter::addMeasurement(float x, float y, float theta)
{
	/* Add measurements to the filter. We assume here that all motion models
	measure x, y and theta. */
	z(0) = x;
	z(1) = y;
	z(2) = theta;
	// Set measurement boolean to true (we just recieved new measurements)
	m_newMeasurement = true;
}

// This should be placed in the abstract class as a default implementation.
void NoFilter::addInputSignals(float gas, float turn)
{	//Dummie
}

std::vector<float> NoFilter::getState(void)
{
	// Convert state vector, xhat, to a std::vector with the same content
	std::vector<float> state(xhat.size());
	for (int i = 0; i < xhat.size(); i++)
	{
		state[i] = xhat[i];
	}
	return state;
}

void NoFilter::updateModel()
{   //Dummie
}
