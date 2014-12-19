#pragma once

#include "EKF.h"
#include "ParticleFilter.h"


class Car
{
private:
	// The filter used to estimate the cars state vector.
	Filter *m_filter;

	// The cars ID number.
	int m_id;
	// True if the car is active, otherwise false.
	bool m_active;
	// True if the car is active but is not detected, otherwise false.
	bool m_lost;
	// The current mode of the car. 
	CarMode::Enum m_mode;
    // The handController of the car
    HandController::Enum m_handController;


    FilterType::Enum m_filtertype;

	// Counts the number of times an active car is not detected.
	int m_numberOfLosses = 0;
	// Counts the number of times an inactive car is detected.
	int m_numberOfFalseDetections = 0;
	// Counter used to determine when an inactive car should become active.
	int m_activeTimer = 0;
	// Counter used to determine when an active car should become inactive.
	int m_inactiveTimer = 0;
	// Number of samples before an active car becomes inactive.
    const int m_inactiveTimerMax = 500000;
	// Number of samples before an inactive car becomes active.
	const int m_activeTimerMax = 20; 

public:

    Car(int id, CarMode::Enum mode, FilterType::Enum filterType, MotionModelType::Enum motionModelType, HandController::Enum handController);
    // Default constructor. Required by std::vector::resize.
    Car() : Car(-1, CarMode::Auto, FilterType::EKF, MotionModelType::CTModel, HandController::HandControl_1) {}
    ~Car();

	// Returns true if the car is active, otherwise false.
    bool isActive() { return m_active; }
	// Returns true if the car is lost, otherwise false.
    bool isLost() { return m_lost; }
	// Returns the cars id.
    int getId(void) { return m_id; }
	// Returns the state vector of the car.
    std::vector<float> getState(void) { return m_filter->getState(); }
	// Returns the mode of the car.
    CarMode::Enum getMode() { return m_mode; }
    // returns the filter type of the car
    FilterType::Enum getFiltertype(){ return m_filtertype;}
	// Returns true if the filter has received a new measurement.
    bool hasNewMeasurement() { return m_filter->hasNewMeasurement(); }

	// Adds measurements to the filter.
    void addMeasurement(float x, float y, float theta);

    void addImageMeasurement(cv::Mat img);
    // Adds input signal to the filter.
    void addInputSignals(float gas, float turn);
	// Calls the filters updateFilter() function.
	void updateFilter();

private:
	// This function is called in each time step for all cars that are active but not detected. 
	void tickTimer();
};


