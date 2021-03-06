#pragma once

#include "NIDAQmx.h"
#include "definitions.h"
#include "headers.h"
#include <fstream>
#include <stdlib.h>


class IOControl
{
private:
	int m_ID;

    TaskHandle m_hTaskInput = 0;
	TaskHandle m_hTaskPower = 0;
	TaskHandle m_hTaskOutput = 0;
    //for controller 2
    TaskHandle m_c2TaskInput = 0;
    TaskHandle m_c2TaskOutput = 0;
    TaskHandle m_c2TaskPower = 0;

    float64 m_minValChannel = 0;
    float64 m_maxValChannel = 3;

	// At what percentage the linearization will change.
	float64 m_linearizationBreak; 
    // Highest voltage for gas -> lowest gas
    float64 m_voltGasMin;
    // Lowest voltage for gas -> highest gas
    float64 m_voltGasMax;
    // Slope of the gas curve
    float64 m_voltGasSlope;
    // Slope of the reverse curve
    float64 m_reverseGasSlope;
	// When linearization algorithm switch.
	float64 m_voltGasIntervall; 
    // Lowest voltage to reverse the car
    float64 m_voltReverseThreshold;
    // Lowest voltage when braking
    float64 m_voltBrakeThreshold;

	float64 m_minGasVolt;
	float64 m_maxGasVolt;
	float64 m_minTurnVolt;
	float64 m_maxTurnVolt;

	float64 m_gasNeutral;
	float64 m_turnNeutral;
    float64 m_gasFull;

	float64 m_tempArray[2];
    float64 m_prevSignals[2];

    const char *analog_output;
    const char *analog_input;// = "Dev1/ai1:4";
            //analog_input = "Dev1/ai1:4";                    // pin [33 65 30 28]
    const char *digital_output_power;
    const char *digital_output_PWM;


    float64 m_freq[2];
    float64 m_dutyCycle[2];
    float64 m_factorVol2Duty;

    HandController::Enum m_handController;


   //for VoltageLog-file
   QElapsedTimer timeVoltageLog;
   //std::ofstream logFileVoltageLog;
   //std::stringstream str;

   /*activates logfiles for IOcontrol
    * Works only for handcontrol1 and only for one car. If several cars exist, even if they are marked as "not connected", problems may occur.
    * Default this should be set to false. Anly set to true even this logData is of interest.
    */
   bool writeVoltageLogAuto = false;
   bool writeVoltageLogMan = false;
   bool writeVoltageLogAss = false;

public:

    IOControl(int ID, HandController::Enum handController);
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
    void sendSignals(float gas, float turn, CarData &carData, bool isBacking);
	// Takes input signals from the hand controller and sends it as output to the car.
    void manualControl(CarData &carData);
	// Sends the gas signal from the hand controller and the turn signal turnSignal to the car.
    void assistedControl(float turnSignal, CarData &carData);
	
private:
	// Transforms voltage to a interval between -1 and 1.
	void voltageToDecimal(float64 *values);
	// Transforms an interval between -1 and 1 to corresponding voltage.
    void decimalToVoltage(float64 *values, bool isBacking);
	// Recieve the voltage from the input pins.
    void receiveSignalsVolt(float64 *signal);
	// Send the voltage to the output pins.
    void sendSignalsVolt(float64 *signal);
	// Turns the IO controller on.
	void controllerOn(void);
	// Turns the IO controller off.
    void controllerOff(void);
    // Transform a voltage to corresponding duty cycle
    void voltageToDutyCycle(float64 *values);

};
