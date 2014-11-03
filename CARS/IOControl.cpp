#include "headers.h"
#include "definitions.h"
#include "IOControl.h"

IOControl::IOControl(int ID)
{
    m_ID = ID;
#ifdef NIDAQ_IS_AVALIABLE
    DAQmxCreateTask("", &m_hTaskInput);
    DAQmxCreateTask("", &m_hTaskOutput);
    DAQmxCreateTask("", &m_hTaskPower);

    m_prevSignals[0] = 0;
    m_prevSignals[1] = 0;


#endif
    switch (m_ID)
    {
        case 0:
            m_minValChannel = 0;
            m_maxValChannel = 3;

            m_linearizationBreak = 0.35;
            m_voltGasThreshold = 1.4710;
            m_voltGasIntervall = 1.4693;

            m_voltReverseThreshold = 1.68;


            m_minGasVolt = 0.91; // 1.14;
            m_maxGasVolt = 2.21; // 2.31;
            m_minTurnVolt = 0.11;
            m_maxTurnVolt = 2.97; //3.242;

            m_gasNeutral = 1.61;
            m_turnNeutral = 1.53;


            analog_output = "Dev1/ao0:1";
            analog_input = "Dev1/ai1:2";
            digital_output = "Dev1/port0/line7";
            break;
        case 1:
            m_minValChannel = 0;
            m_maxValChannel = 3;

            m_linearizationBreak = 0.38;
            m_voltGasThreshold = 1.4706;
            m_voltGasIntervall = 1.4693;

            m_minGasVolt = 0.91; // 1.14;
            m_maxGasVolt = 2.10; // 2.31;
            m_minTurnVolt = 0.17;
            m_maxTurnVolt = 2.9; //3.242;

            m_gasNeutral = 1.58;
            m_turnNeutral = 1.53;

            analog_output = "Dev1/ao0:1";
            analog_input = "Dev1/ai1:2";
            digital_output = "Dev1/port0/line7";
            break;
        case 2:
            m_minValChannel = 0;
            m_maxValChannel = 3;

            m_linearizationBreak = 0.38;
            m_voltGasThreshold = 1.4706;
            m_voltGasIntervall = 1.4693;

            m_minGasVolt = 0.91; // 1.14;
            m_maxGasVolt = 2.10; // 2.31;
            m_minTurnVolt = 0.17;
            m_maxTurnVolt = 2.9; //3.242;

            m_gasNeutral = 1.58;
            m_turnNeutral = 1.53;

            analog_output = "Dev1/ao0:1";
            analog_input = "Dev1/ai1:2";
            digital_output = "Dev1/port0/line7";
            break;
        default:
            m_minValChannel = 0;
            m_maxValChannel = 3;

            m_linearizationBreak = 0.38;
            m_voltGasThreshold = 1.4706;
            m_voltGasIntervall = 1.4693;

            m_minGasVolt = 0.3;
            m_maxGasVolt = 2.7;
            m_minTurnVolt = 0.17;
            m_maxTurnVolt = 2.9;

            m_gasNeutral = 1.58;
            m_turnNeutral = 1.53;

            analog_output = "";
            analog_input = "";
            digital_output = "";
            break;
    }
}

IOControl::~IOControl()
{
}

void IOControl::init(void)
{
#ifdef NIDAQ_IS_AVALIABLE
	DAQmxCreateAIVoltageChan(m_hTaskInput, analog_input, "", DAQmx_Val_RSE, m_minValChannel, m_maxValChannel, DAQmx_Val_Volts, NULL);
	DAQmxCreateAOVoltageChan(m_hTaskOutput, analog_output, "", m_minValChannel, m_maxValChannel, DAQmx_Val_Volts, "");
	DAQmxCreateDOChan(m_hTaskPower, digital_output, "", DAQmx_Val_ChanForAllLines);
#endif
}

void IOControl::startController(void)
{
#ifdef NIDAQ_IS_AVALIABLE
	DAQmxStartTask(m_hTaskInput);
	DAQmxStartTask(m_hTaskOutput);
	DAQmxStartTask(m_hTaskPower);
	controllerOn();
#endif
}

void IOControl::stopController(void)
{
#ifdef NIDAQ_IS_AVALIABLE
	controllerOff();
		
	DAQmxStopTask(m_hTaskInput);
	DAQmxStopTask(m_hTaskOutput);
	DAQmxStopTask(m_hTaskPower);

	DAQmxClearTask(m_hTaskInput);
	DAQmxClearTask(m_hTaskOutput);
	DAQmxClearTask(m_hTaskPower);
#endif
}

void IOControl::receiveSignals(float &gas, float &turn)
{
#ifdef NIDAQ_IS_AVALIABLE
    receiveSignalsVolt(m_tempArray);
    voltageToDecimal(m_tempArray);
    gas = m_tempArray[0];
    turn = m_tempArray[1];
#endif
}

void IOControl::sendSignals(float gas, float turn)
{
#ifdef NIDAQ_IS_AVALIABLE
    m_tempArray[0] = gas;
    m_tempArray[1] = turn;
    decimalToVoltage(m_tempArray);
    sendSignalsVolt(m_tempArray);
#endif
}

void IOControl::controllerOn(void)
{
#ifdef NIDAQ_IS_AVALIABLE
    float64 signal[2];
    // Start the controller with full gas. This ensures that the car starts in racing mode.
    signal[0] = 0.9;
    signal[1] = m_turnNeutral;
    DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL);

    Sleep(20);
    uInt8 value[1];
    value[0] = 1;
    // Turn on controller.
    DAQmxWriteDigitalLines(m_hTaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL);

    // Sleep to ensure that the signals have been written.
    Sleep(200);
    signal[0] = m_gasNeutral; //0.9;
    signal[1] = m_turnNeutral;
    // Send neutral signals.
    DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL);
#endif
}

void IOControl::controllerOff(void)
{
#ifdef NIDAQ_IS_AVALIABLE
    float64 signal[2];
    signal[0] = m_gasNeutral;
    signal[1] = m_turnNeutral;
    DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL);
	uInt8 value[1];
	value[0] = 0;
	DAQmxWriteDigitalLines(m_hTaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL);
    signal[0] = 0;
    signal[1] = 0;
    DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL);
#endif
}

void IOControl::manualControl(void)
{
#ifdef NIDAQ_IS_AVALIABLE
    // Read input signals and save to m_tempArray
    receiveSignalsVolt(m_tempArray);

    // Send signals (m_tempControlSignals) back to the controller
    sendSignalsVolt(m_tempArray);
#endif
}

// Sends the gas signal from the hand controller and the turn signal turnSignal to the car.
void IOControl::assistedControl(float turnSignal)
{
#ifdef NIDAQ_IS_AVALIABLE
    // Read input signals from hand controller.
    receiveSignalsVolt(m_tempArray);

    // Convert turn signal to volt.
    float64 voltArray[2];
    voltArray[1] = turnSignal;
    // Dummy value to avoid warnings
    voltArray[0] = 0;
    decimalToVoltage(voltArray);

    // Write gas signal from hand controller.
    voltArray[0] = m_tempArray[0];

    // Send signals to the car.
    sendSignalsVolt(voltArray);
#endif
}

void IOControl::voltageToDecimal(float64 *voltage)
{
#ifdef NIDAQ_IS_AVALIABLE
    // Transform gas signal
    // If voltage results in no gas


    if (voltage[0] <= m_voltReverseThreshold && voltage[0] >= m_voltGasThreshold)
    {
        voltage[0] = 0;
    }
    // If voltage is break/reverse
    else if (voltage[0] > m_voltReverseThreshold && voltage[0] <= m_maxGasVolt)
    //if (voltage[0] > m_gasNeutral && voltage[0] <= m_maxGasVolt)
    {
        voltage[0] = -(voltage[0] - m_voltReverseThreshold) / (m_maxGasVolt - m_voltReverseThreshold);
    }
    // If voltage is 0 - m_linearizationBreak
    else if (voltage[0] < m_voltGasThreshold && voltage[0] >= m_voltGasIntervall)
    //else if (voltage[0] < m_gasNeutral && voltage[0] >= m_voltGasIntervall)
    {
        voltage[0] = m_linearizationBreak * (m_voltGasThreshold - voltage[0]) / (m_voltGasThreshold - m_voltGasIntervall);
    }
    // If voltage is m_linearizationBreak - 100%
    else if (voltage[0] < m_voltGasIntervall && voltage[0] >= m_minGasVolt)
    {
        voltage[0] = m_linearizationBreak + (1 - m_linearizationBreak) * (m_voltGasIntervall - voltage[0]) / (m_voltGasIntervall - m_minGasVolt);
    }
    // If voltage is out of bounds
    else
    {
        std::cout << "Gas signal is out of bounds: " << voltage[0] << "A" << std::endl;
    }

    // Transform turn signal
    if (voltage[1] == m_turnNeutral)
    {
        voltage[1] = 0;
    }
    else if (voltage[1] > m_turnNeutral && voltage[1] <= m_maxTurnVolt)
    {
        voltage[1] = (voltage[1] - m_turnNeutral) / (m_maxTurnVolt - m_turnNeutral);
    }
    else if (voltage[1] < m_turnNeutral && voltage[1] >= m_minTurnVolt)
    {
        voltage[1] = (voltage[1] - m_turnNeutral) / (m_turnNeutral - m_minTurnVolt);
    }
    else
    {
        std::cout << "Turn signal is out of bounds: " << voltage[1] << "A" << std::endl;
    }
#endif
}

// Map interval [-1,1] to [m_minVal,m_maxVal].
void IOControl::decimalToVoltage(float64 *decimal)
{
    // Transform gas signal
    // If gas is 0

    if (decimal[0] == 0)
    {
        decimal[0] = m_gasNeutral;
    }
    // If gas is 0 - m_linearizationBreak
    else if (decimal[0] > 0 && decimal[0] <= m_linearizationBreak)
    {
        decimal[0] = m_voltGasThreshold - decimal[0] / m_linearizationBreak * (m_voltGasThreshold - m_voltGasIntervall);
    }
    // If gas is m_linearizationBreak - 100%
    else if (decimal[0] > m_linearizationBreak && decimal[0] <= 1)
    {
        decimal[0] = m_voltGasIntervall - (decimal[0] - m_linearizationBreak) / (1 - m_linearizationBreak)* (m_voltGasIntervall - m_minGasVolt);
    }
    // If break/reverse
    else if (decimal[0] < 0 && decimal[0] >= -1)
    {
        decimal[0] = m_voltReverseThreshold + -decimal[0] * (m_maxGasVolt - m_voltReverseThreshold);
    }
    // If out of bounds
    else
    {
        std::cout << "Gas signal is out of bounds: " << decimal[0] << std::endl;
    }

    // Transform turn signal
    if (decimal[1] == 0)
    {
        decimal[1] = m_turnNeutral;
    }
    else if (decimal[1] > 0 && decimal[1] <= 1)
    {
        decimal[1] = m_turnNeutral + decimal[1] * (m_maxTurnVolt - m_turnNeutral);
        if (decimal[1] > m_maxTurnVolt)
        {
            std::cout << "Turn signal is too high: " << decimal[1] << std::endl;
        }
    }
    else if (decimal[1] < 0 && decimal[1] >= -1)
    {
        decimal[1] = m_turnNeutral - -decimal[1] * (m_turnNeutral - m_minTurnVolt);
        if (decimal[1] < m_minTurnVolt)
        {
            std::cout << "Turn signal is too low: " << decimal[1] << std::endl;
        }
    }
    else
    {
        std::cout << "Turn signal is out of bounds: " << decimal[1] << std::endl;
    }
}

void IOControl::receiveSignalsVolt(float64 *signal)
{
#ifdef NIDAQ_IS_AVALIABLE
    DAQmxReadAnalogF64(m_hTaskInput, 1, 10.0, DAQmx_Val_GroupByChannel, signal, 2, NULL, NULL);
    signal[0] = (signal[0] + m_prevSignals[0]) / 2;
    signal[1] = (signal[1] + m_prevSignals[1]) / 2;

    m_prevSignals[0] = signal[0];
    m_prevSignals[1] = signal[1];

#endif
}

// Add small offset and send signals.
void IOControl::sendSignalsVolt(float64 *signal)
{
#ifdef NIDAQ_IS_AVALIABLE
    if (signal[0] < m_minGasVolt | signal[0] > m_maxGasVolt | signal[1] < m_minTurnVolt | signal[1] > m_maxTurnVolt)
    {
        std::cout << "Output voltage is smaller than m_minVal, or larger than m_maxVal" << std::endl;
        std::cout << "Gas voltage: " << signal[0] << ", Turn voltage: " << signal[1] << std::endl;
    }
    else
    {
        DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL);
    }
#endif
}
