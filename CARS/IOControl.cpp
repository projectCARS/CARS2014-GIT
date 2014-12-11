#include "headers.h"
#include "definitions.h"
#include "IOControl.h"
#include "functions.h"

//test
#include "headers.h"
#include "definitions.h"
#include "classes.h"
#include "PIDadaptiveGain.h"
#include "functions.h"


IOControl::IOControl(int ID, HandController::Enum handController)
{

    m_handController = handController;
     m_ID = ID;
    switch (m_handController)
    {

    case HandController::HandControl_1:

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
            m_gasFull = 0.9;


            analog_output = "Dev1/ao0:1";
            analog_input = "Dev1/ai1:2";
            digital_output_power = "Dev1/port0/line7";
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
            m_gasFull = 0.9;

            analog_output = "Dev1/ao0:1";
            analog_input = "Dev1/ai1:2";
            digital_output_power = "Dev1/port0/line7";
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
            m_gasFull = 0.9;

            analog_output = "Dev1/ao0:1";
            analog_input = "Dev1/ai1:2";
            digital_output_power = "Dev1/port0/line7";
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
            m_gasFull = 0.9;

            analog_output = "";
            analog_input = "";
            digital_output_power = "";
            break;
        }
        writeVoltageLogMan = false;
        if (writeVoltageLogMan)
        {
            timeVoltageLog.start();
            std::stringstream str;
            str.clear();
            str.str(std::string());
            str << "outdata/logFiles/logVoltageManual.txt";
            std::ofstream LF;
            LF.open(str.str());
            LF << "time carID xPos yPos speed yaw yawVel Ugas Uturn\n";
            std::cout << "IOcontrol object:		Writing log to " << str.str() << std::endl;
            LF.close();
        }

        writeVoltageLogAss = false;
        if (writeVoltageLogAss)
        {
            timeVoltageLog.start();
            std::stringstream str;
            str.clear();
            str.str(std::string());
            str << "outdata/logFiles/logVoltageAss.txt";
            std::ofstream LF;
            LF.open(str.str());
            LF << "time carID xPos yPos speed yaw yawVel Ugas Uturn turn\n";
            std::cout << "IOcontrol object:		Writing log to " << str.str() << std::endl;
            LF.close();
        }
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        DAQmxCreateTask("", &m_hTaskInput);
        DAQmxCreateTask("", &m_c2TaskOutput);
        DAQmxCreateTask("", &m_hTaskPower);
#endif
        m_prevSignals[0] = 0;
        m_prevSignals[1] = 0;
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
            m_gasFull = 0.9;

            analog_input = "Dev1/ai3:4";            //ai3:4 corresponds to pin 30 and 28
            digital_output_power = "Dev1/port0/line3";
            digital_output_PWM = "Dev1/ctr0:1";     //ctr0:1 corresponds to pin 2 and pin 40

            m_freq[0] = 20000;
            m_freq[1] = 20000;
            m_dutyCycle[0] = m_gasFull/3.0;
            m_dutyCycle[1] = m_turnNeutral/3.0;
            m_factorVol2Duty = 3.0f;

            break;
        default:
            //TODO: do something
            break;
        }

        break;

    default:
        //TODO: do something or...
        break;
    }


}


IOControl::~IOControl()
{
}

void IOControl::init(void)
{
    switch (m_handController)
    {
    case HandController::HandControl_1:

#ifdef NIDAQ_IS_AVALIABLE
        DAQmxCreateAIVoltageChan(m_hTaskInput, analog_input, "", DAQmx_Val_RSE, m_minValChannel, m_maxValChannel, DAQmx_Val_Volts, NULL);
        DAQmxCreateAOVoltageChan(m_hTaskOutput, analog_output, "", m_minValChannel, m_maxValChannel, DAQmx_Val_Volts, "");
        DAQmxCreateDOChan(m_hTaskPower, digital_output_power, "", DAQmx_Val_ChanForAllLines);
#endif
        writeVoltageLog = false;
        if (writeVoltageLog)
        {

            timeVoltageLog.start();
            std::stringstream str;
            str.clear();
            str.str(std::string());
            str << "outdata/logFiles/logVoltage1_theOneAndOnly.txt";
            std::ofstream LF;
            LF.open(str.str());
            LF << "time carID xPos yPos speed yaw yawVel Ugas Uturn gas turn\n";
            std::cout << "IOcontrol object:		Writing log to " << str.str() << std::endl;
            LF.close();
        }
        break;

    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
         DAQmxCreateAIVoltageChan(m_hTaskInput, analog_input, "", DAQmx_Val_RSE, m_minValChannel, m_maxValChannel, DAQmx_Val_Volts, NULL);
        DAQmxCreateDOChan(m_hTaskPower, digital_output_power, "", DAQmx_Val_ChanForAllLines);
        DAQmxCreateCOPulseChanFreq(m_c2TaskOutput, digital_output_PWM, "", DAQmx_Val_Hz, DAQmx_Val_Low, 0.0f, m_freq[0], m_dutyCycle[0]);
        DAQmxCfgImplicitTiming(m_c2TaskOutput, DAQmx_Val_ContSamps, 10000);
#endif
        break;

    default:
        break;
    }
}

void IOControl::startController(void)
{
    switch (m_handController)
    {
    case HandController::HandControl_1:
#ifdef NIDAQ_IS_AVALIABLE
        DAQmxStartTask(m_hTaskInput);
        DAQmxStartTask(m_hTaskOutput);
        DAQmxStartTask(m_hTaskPower);
        controllerOn();
#endif
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        DAQmxStartTask(m_hTaskInput);
        DAQmxStartTask(m_c2TaskOutput);
        DAQmxStartTask(m_hTaskPower);
        controllerOn();
#endif
        break;

    default:
        break;
    }
}

void IOControl::stopController(void)
{
    switch(m_handController)
    {
    case HandController::HandControl_1:
#ifdef NIDAQ_IS_AVALIABLE
        controllerOff();
        DAQmxStopTask(m_hTaskInput);
        DAQmxStopTask(m_hTaskOutput);
        DAQmxStopTask(m_hTaskPower);

        DAQmxClearTask(m_hTaskInput);
        DAQmxClearTask(m_hTaskOutput);
        DAQmxClearTask(m_hTaskPower);
#endif
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        controllerOff();
        DAQmxStopTask(m_hTaskInput);
        DAQmxStopTask(m_c2TaskOutput);
        DAQmxStopTask(m_hTaskPower);

        DAQmxClearTask(m_hTaskInput);
        DAQmxClearTask(m_c2TaskOutput);
        DAQmxClearTask(m_hTaskPower);
#endif
        break;
    default:
        break;
    }
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

void IOControl::sendSignals(float gas, float turn, CarData &carData)
{
    switch (m_handController)
    {
    case HandController::HandControl_1:
#ifdef NIDAQ_IS_AVALIABLE
        m_tempArray[0] = gas;
        m_tempArray[1] = turn;
        decimalToVoltage(m_tempArray);
        sendSignalsVolt(m_tempArray);

        if (writeVoltageLog)
        {
            std::stringstream str;
            str.clear();
            str.str(std::string());
            str << "outdata/logFiles/logVoltage1_theOneAndOnly.txt";
            std::ofstream LF;
            LF.open(str.str(),std::ofstream::app);
            // "time carID xPos yPos speed yaw yawVel Ugas Uturn gas turn \n"

            LF << timeVoltageLog.elapsed()/1000.0 << " " << carData.id << " ";
            LF << carData.state[0] << " " << carData.state[1] << " " ;
            LF << carData.state[2] << " " << carData.state[3] << " " ;
            LF << carData.state[4] << " " ;
            LF << m_tempArray[0] << " " << m_tempArray[1] << " ";
            LF << gas << " " << turn << "\n" ;
            LF.close();
        }

#endif
        break;
    case HandController::HandControl_2:
        m_tempArray[0] = gas;
        m_tempArray[1] = turn;
        decimalToVoltage(m_tempArray);
        sendSignalsVolt(m_tempArray);
        break;
    default:
        break;
    }
}

void IOControl::controllerOn(void)
{
    float64 signal[2];
    uInt8 value[1];
    switch (m_handController)
    {
    case HandController::HandControl_1:
#ifdef NIDAQ_IS_AVALIABLE
        //float64 signal[2];
        // Start the controller with full gas. This ensures that the car starts in racing mode.
        signal[0] = m_gasFull;
        signal[1] = m_turnNeutral;
        DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL);

        Sleep(20);
        //uInt8 value[1];
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
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        //float64 signal[2];
        // Start the controller with full gas. This ensures that the car starts in racing mode.
        signal[0] = m_gasFull;
        signal[1] = m_turnNeutral;
        DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL);

        Sleep(20);
        //uInt8 value[1];
        value[0] = 1;
        // Turn on controller.
        DAQmxWriteDigitalLines(m_hTaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL);

        // Sleep to ensure that the signals have been written.
        Sleep(200);
        signal[0] = m_gasNeutral;
        signal[1] = m_turnNeutral;
        // Send neutral signals.
        DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL);
#endif

        break;
    default:
        break;
    }
}

void IOControl::controllerOff(void)
{
    float64 signal[2];
    uInt8 value[1];

    switch (m_handController)
    {
    case HandController::HandControl_1:
#ifdef NIDAQ_IS_AVALIABLE   
        signal[0] = m_gasNeutral;
        signal[1] = m_turnNeutral;
        DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL);
        value[0] = 0;
        DAQmxWriteDigitalLines(m_hTaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL);
        signal[0] = 0;
        signal[1] = 0;
        DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL);
#endif
        break;
    case HandController::HandControl_2:
        signal[0] = m_gasNeutral;
        signal[1] = m_turnNeutral;
        DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL);
        value[0] = 0;
        DAQmxWriteDigitalLines(m_hTaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL);
        signal[0] = 0;
        signal[1] = 0;
        DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL);
        break;
    default:
        break;
    }
}

void IOControl::manualControl( CarData &carData)
{
#ifdef NIDAQ_IS_AVALIABLE
    // Read input signals and save to m_tempArray
    receiveSignalsVolt(m_tempArray);

    // Send signals (m_tempControlSignals) back to the controller
    sendSignalsVolt(m_tempArray);

    if (writeVoltageLogMan)
    {
        std::stringstream str;
        str.clear();
        str.str(std::string());
        str << "outdata/logFiles/logVoltageManual.txt";
        std::ofstream LF;
        LF.open(str.str(),std::ofstream::app);
        // "time carID xPos yPos speed yaw yawVel Ugas Uturn\n"

        LF << timeVoltageLog.elapsed()/1000.0 << " " << carData.id << " ";
        LF << carData.state[0] << " " << carData.state[1] << " " ;
        LF << carData.state[2] << " " << carData.state[3] << " " ;
        LF << carData.state[4] << " " ;
        LF << m_tempArray[0] << " " << m_tempArray[1] << "\n";
        LF.close();
    }
#endif
}

// Sends the gas signal from the hand controller and the turn signal turnSignal to the car.
void IOControl::assistedControl(float turnSignal, CarData &carData)
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

    if (writeVoltageLogAss)
    {
        std::stringstream str;
        str.clear();
        str.str(std::string());
        str << "outdata/logFiles/logVoltageAss.txt";
        std::ofstream LF;
        LF.open(str.str(),std::ofstream::app);
        // "time carID xPos yPos speed yaw yawVel Ugas Uturn turn\n"

        LF << timeVoltageLog.elapsed()/1000.0 << " " << carData.id << " ";
        LF << carData.state[0] << " " << carData.state[1] << " " ;
        LF << carData.state[2] << " " << carData.state[3] << " " ;
        LF << carData.state[4] << " " ;
        LF << voltArray[0] << " " << voltArray[1] << " ";
        LF << turnSignal << "\n" ;
        LF.close();
    }

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
    else if (decimal[0] > m_linearizationBreak && decimal[0] <= 1)//m_linearizationBreak && decimal[0] <= 1)
    {
        decimal[0] = m_voltGasIntervall - (decimal[0] - m_linearizationBreak) / (1 - m_linearizationBreak)* (m_voltGasIntervall - m_minGasVolt);
        //decimal[0] = -0.33 + m_voltGasIntervall - (decimal[0] - m_linearizationBreak) / (1 - m_linearizationBreak)* (m_voltGasIntervall - m_minGasVolt);
        //decimal[0] = -0.513*decimal[0] + 1.423;
    }
    // If break/reverse
    else if (decimal[0] < 0 && decimal[0] >= -1)
    {
        decimal[0] = m_voltReverseThreshold + -decimal[0] * (m_maxGasVolt - m_voltReverseThreshold);
        //decimal[0] = -0.2 + m_voltReverseThreshold + -decimal[0] * (m_maxGasVolt - m_voltReverseThreshold)*4;
    }
    // If out of bounds
    else
    {
        decimal[0] = m_gasNeutral;
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

void IOControl::decimalToVoltageLinearMap(float64 *decimal)
{
    // Transform gas signal
    // If gas is 0
    float gas;
    float turn;

    /* gas = k*decimal[0] + m;
    turn = k2*decimal[1] + m2;
    decimal[0] = gas;
    decimal[1] = turn;*/
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
    switch (m_handController)
    {
    case HandController::HandControl_1:
#ifdef NIDAQ_IS_AVALIABLE
        if ((signal[0] < m_minGasVolt) | (signal[0] > m_maxGasVolt) | (signal[1] < m_minTurnVolt) | (signal[1] > m_maxTurnVolt))
        {
            std::cout << "Output voltage is smaller than m_minVal, or larger than m_maxVal" << std::endl;
            std::cout << "Gas voltage: " << signal[0] << ", Turn voltage: " << signal[1] << std::endl;
        }
        else
        {
            DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL);
        }
#endif
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        if ((signal[0] < m_minGasVolt) | (signal[0] > m_maxGasVolt) | (signal[1] < m_minTurnVolt) | (signal[1] > m_maxTurnVolt))
        {
            std::cout << "Output voltage is smaller than m_minVal, or larger than m_maxVal" << std::endl;
            std::cout << "Gas voltage: " << signal[0] << ", Turn voltage: " << signal[1] << std::endl;
        }
        else
        {
            voltageToDutyCycle(signal);
            DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL);
        }
#endif
        break;
    default:
        break;
    }
}

void IOControl::voltageToDutyCycle(float64 *values)
{
    float64 voltage[2];
    voltage[0] = values[0];
    voltage[1] = values[1];
    values[0] = voltage[0]/m_factorVol2Duty;
    values[1] = voltage[1]/m_factorVol2Duty;
    if ( (values[0]<0.0) | (values[0]>1.0) )
    {
        qDebug("gas duty cycle is out of bounds");
        qDebug() << values[0];
    }
    if ( (values[1]<0.0) | (values[1]>1.0) )
    {
        qDebug("turn duty cycle is out of bounds");
        qDebug() << values[1];
    }
    qDebug() << values[1];
}
