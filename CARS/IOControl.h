#pragma once

#include "NIDAQmx.h"
#include "definitions.h"
#include "headers.h"
#include <fstream>



class IOControl
{
private:
	int m_ID;

	TaskHandle m_hTaskPower = 0;
	TaskHandle m_hTaskInput = 0;
	TaskHandle m_hTaskOutput = 0;

	float64 m_minValChannel;
	float64 m_maxValChannel;

	// At what percentage the linearization will change.
	float64 m_linearizationBreak; 
	// Highest voltage for gas.
	float64 m_voltGasThreshold; 
	// When linearization algorithm switch.
	float64 m_voltGasIntervall; 
    // Lowest voltage to reverse the car
    float64 m_voltReverseThreshold;

	float64 m_minGasVolt;
	float64 m_maxGasVolt;
	float64 m_minTurnVolt;
	float64 m_maxTurnVolt;

	float64 m_gasNeutral;
	float64 m_turnNeutral;

	float64 m_tempArray[2];
    float64 m_prevSignals[2];

    const char *analog_output;
    const char *analog_input;
    const char *digital_output;


   //for VoltageLog-file
   //QElapsedTimer timeVoltageLog;
   std::ofstream logFileVoltageLog;
   bool writeVoltageLog;

public:
	IOControl(int ID);
    ~IOControl();

	// Initialize the controller.
	void init(void);
	// Starts the IO controller.
	void startController(void);
	// Stops the IO controller.
	void stopController(void);

    // Get received control signal from input pins (-1 to 1).
    void receiveSignals(float &gas, float &turn);
    //void receiveSignals(std::vector<float> &signals);
	// Takes values (-1 to 1) and send it to output pins in corresponding voltage.
    void sendSignals(float gas, float turn, CarData &carData);
	// Takes input signals from the hand controller and sends it as output to the car.
	void manualControl(void);
	// Sends the gas signal from the hand controller and the turn signal turnSignal to the car.
    void assistedControl(float turnSignal);
	
private:
	// Transforms voltage to a interval between -1 and 1.
	void voltageToDecimal(float64 *values);
	// Transforms an interval between -1 and 1 to corresponding voltage.
	void decimalToVoltage(float64 *values);
	// Recieve the voltage from the input pins.
    void receiveSignalsVolt(float64 *signal);
	// Send the voltage to the output pins.
    void sendSignalsVolt(float64 *signal);
	// Turns the IO controller on.
	void controllerOn(void);
	// Turns the IO controller off.
	void controllerOff(void);
};
