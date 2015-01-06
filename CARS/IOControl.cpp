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

// Stream used to write to logfile
std::ofstream LF;
// For analog input. Used when driving in assisted and manual mode.
bool firstCallingConstructor = true;
bool firstCallingStop = true;
TaskHandle TaskInput = 0;




// Function that reads the returned status message from a DAQmx function. If failed, displays the error message
// Should be called whenever a DAQmx function is called

IOControl::IOControl(int ID, HandController::Enum handController)
{
    if (firstCallingConstructor)
    {
        firstCallingConstructor = false;
        firstCallingStop = true;
        printError(DAQmxCreateTask("TaskInput", &TaskInput), 32, " in IOcontrol");
        analog_input = "Dev1/ai1:4";        // pin [33 65 30 28]
        printError(DAQmxCreateAIVoltageChan(TaskInput, analog_input, "", DAQmx_Val_RSE, m_minValChannel, m_maxValChannel, DAQmx_Val_Volts, NULL), 41, " in IOcontrol");
        printError(DAQmxStartTask(TaskInput), 42, " in IOcontrol");
    }

    m_handController = handController;
    m_ID = ID;
    switch (m_handController)
    {

    case HandController::HandControl_1:
        qDebug("handkontroll 1 startad till bil %i", ID);

#ifdef NIDAQ_IS_AVALIABLE
        printError(DAQmxClearTask(m_hTaskOutput), 42, " in IOcontrol");
        printError(DAQmxClearTask(m_hTaskPower), 43, " in IOcontrol");

        // When creating tasks, make sure to use an empty string as the name argument
        // This will cause a random name to be genererated. Using the same name several times
        // can cause the name to remain in memory and fail to start the next time
        printError(DAQmxCreateTask("", &m_hTaskOutput), 46, " in IOcontrol");
        printError(DAQmxCreateTask("", &m_hTaskPower), 47, " in IOcontrol");

        m_prevSignals[0] = 0;
        m_prevSignals[1] = 0;

        analog_output = "Dev1/ao0:1";
        digital_output_power = "Dev1/port0/line7";

#endif
        switch (m_ID)
        {
        case 0:
            m_voltGasIntervall = 1.4693;
            m_linearizationBreak = 0.35;

            m_voltGasMin = 1.4710;
            m_voltGasMax = 0.95;
            m_voltGasSlope = m_voltGasMax - m_voltGasMin;

            m_voltReverseThreshold = 1.68;
            m_voltBrakeThreshold = 1.472;

            m_reverseGasSlope = 0.51;

            m_minGasVolt = 0.91; // 1.14;
            m_maxGasVolt = 2.21; // 2.31;
            m_minTurnVolt = 0.11;
            m_maxTurnVolt = 2.97; //3.242;

            m_gasNeutral = 1.61;
            m_turnNeutral = 1.53;
            m_gasFull = 0.9;

            break;
        case 1:
            m_voltReverseThreshold = 1.68;
            m_voltBrakeThreshold = 1.472;

            m_reverseGasSlope = 0.47;

            m_voltGasIntervall = 1.4693;
            m_linearizationBreak = 0.38;

            m_voltGasMin = 1.4706;
            m_voltGasMax = 0.95;
            m_voltGasSlope = m_voltGasMax - m_voltGasMin;

            m_minGasVolt = 0.91; // 1.14;
            m_maxGasVolt = 2.10; // 2.31;
            m_minTurnVolt = 0.17;
            m_maxTurnVolt = 2.9; //3.242;

            m_gasNeutral = 1.58;
            m_turnNeutral = 1.53;
            m_gasFull = 0.9;

            break;
        case 2:
            m_voltReverseThreshold = 1.68;
            m_voltBrakeThreshold = 1.472;

            m_reverseGasSlope = 0.47;

            m_voltGasMin = 1.4706;
            m_voltGasMax = 0.95;
            m_voltGasSlope = m_voltGasMax - m_voltGasMin;

            m_linearizationBreak = 0.38;
            m_voltGasIntervall = 1.4693;

            m_minGasVolt = 0.91; // 1.14;
            m_maxGasVolt = 2.10; // 2.31;
            m_minTurnVolt = 0.17;
            m_maxTurnVolt = 2.9; //3.242;

            m_gasNeutral = 1.58;
            m_turnNeutral = 1.53;
            m_gasFull = 0.9;

            break;
        default:
            m_voltReverseThreshold = 1.68;
            m_voltBrakeThreshold = 1.472;

            m_reverseGasSlope = 0.47;

            m_linearizationBreak = 0.38;
            m_voltGasMin = 1.4706;
            m_voltGasMax = 0.95;
            m_voltGasSlope = m_voltGasMax - m_voltGasMin;

            m_voltGasIntervall = 1.4693;

            m_minGasVolt = 0.3;
            m_maxGasVolt = 2.7;
            m_minTurnVolt = 0.17;
            m_maxTurnVolt = 2.9;

            m_gasNeutral = 1.58;
            m_turnNeutral = 1.53;
            m_gasFull = 0.9;

            break;
        }
        if (writeVoltageLogAuto)
        {
            timeVoltageLog.start();
            std::stringstream str;
            str.clear();
            str.str(std::string());
            str << "outdata/logFiles/logVoltage1_theOneAndOnly.txt";
            LF.open(str.str());
            LF << "time carID xPos yPos speed yaw yawVel Ugas Uturn gas turn\n";
            std::cout << "IOcontrol object:		Writing log to " << str.str() << std::endl;
        }
        if (writeVoltageLogMan)
        {
            timeVoltageLog.start();
            std::stringstream str;
            str.clear();
            str.str(std::string());
            str << "outdata/logFiles/logVoltageManual.txt";
            LF.open(str.str());
            LF << "time carID xPos yPos speed yaw yawVel Ugas Uturn\n";
            std::cout << "IOcontrol object:		Writing log to " << str.str() << std::endl;
        }

        if (writeVoltageLogAss)
        {
            timeVoltageLog.start();
            std::stringstream str;
            str.clear();
            str.str(std::string());
            str << "outdata/logFiles/logVoltageAss.txt";
            LF.open(str.str());
            LF << "time carID xPos yPos speed yaw yawVel Ugas Uturn turn\n";
            std::cout << "IOcontrol object:		Writing log to " << str.str() << std::endl;
        }
        break;
    case HandController::HandControl_2:
        qDebug("handkontroll 2 startad till bil %i", ID);
#ifdef NIDAQ_IS_AVALIABLE

        printError(DAQmxClearTask(m_c2TaskOutput), 210, " in IOcontrol");
        printError(DAQmxClearTask(m_c2TaskPower), 211, " in IOcontrol");

        // When creating tasks, make sure to use an empty string as the name argument
        // This will cause a random name to be genererated. Using the same name several times
        // can cause the name to remain in memory and fail to start the next time
        printError(DAQmxCreateTask("", &m_c2TaskOutput), 214, " in IOcontrol");
        printError(DAQmxCreateTask("", &m_c2TaskPower), 215, " in IOcontrol");

        digital_output_power = "Dev1/port0/line3";
        digital_output_PWM = "Dev1/ctr0:1";     // pin [2  40]
#endif
        m_prevSignals[0] = 0;
        m_prevSignals[1] = 0;


        switch (m_ID)
        {
        case 0:
            m_linearizationBreak = 0.35;
            m_voltGasMin = 1.4710;
            m_voltGasMax = 0.95;
            m_voltGasSlope = m_voltGasMax - m_voltGasMin;

            m_voltGasIntervall = 1.4693;

            m_voltReverseThreshold = 1.63;
            m_voltBrakeThreshold = 1.472;

            m_reverseGasSlope = 0.47;

            m_minGasVolt = 0.91; // 1.14;
            m_maxGasVolt = 2.21; // 2.31;
            m_minTurnVolt = 0.11;
            m_maxTurnVolt = 2.97; //3.242;

            m_gasNeutral = 1.61;
            m_turnNeutral = 1.53;
            m_gasFull = 0.9;

            break;
        case 1:
            m_linearizationBreak = 0.38;
            m_voltGasMin = 1.4706;
            m_voltGasMax = 0.95;
            m_voltGasSlope = m_voltGasMax - m_voltGasMin;

            m_voltGasIntervall = 1.44;//1.4693;

            m_voltReverseThreshold = 1.57;//1.68;
            m_voltBrakeThreshold = 1.472;
            m_reverseGasSlope = 0.75;

            m_minGasVolt = 0.91; // 1.14;
            m_maxGasVolt = 3.5; // 2.31;
            m_minTurnVolt = 0.11;
            m_maxTurnVolt = 2.9; //3.242;

            m_gasNeutral = 1.54;
            m_turnNeutral = 1.53;
            m_gasFull = 0.9;

            break;
        case 2:
            m_linearizationBreak = 0.38;
            m_voltGasMin = 1.4706;
            m_voltGasMax = 0.95;
            m_voltGasSlope = m_voltGasMax - m_voltGasMin;
            m_voltGasIntervall = 1.4693;

            m_voltReverseThreshold = 1.68;
            m_voltBrakeThreshold = 1.472;

            m_minGasVolt = 0.91; // 1.14;
            m_maxGasVolt = 2.10; // 2.31;
            m_minTurnVolt = 0.17;
            m_maxTurnVolt = 2.9; //3.242;

            m_gasNeutral = 1.58;
            m_turnNeutral = 1.53;
            m_gasFull = 0.9;

            m_freq[0] = 20000;
            m_freq[1] = 20000;
            m_dutyCycle[0] = m_gasFull/3.0;
            m_dutyCycle[1] = m_turnNeutral/3.0;
            m_factorVol2Duty = 3.0f;
            break;
        default:
            m_linearizationBreak = 0.38;
            m_voltGasMin = 1.4706;
            m_voltGasMax = 0.95;
            m_voltGasSlope = m_voltGasMax - m_voltGasMin;
            m_voltGasIntervall = 1.4693;

            m_voltReverseThreshold = 1.68;
            m_voltBrakeThreshold = 1.472;

            m_minGasVolt = 0.3;
            m_maxGasVolt = 2.7;
            m_minTurnVolt = 0.17;
            m_maxTurnVolt = 2.9;

            m_gasNeutral = 1.58;
            m_turnNeutral = 1.53;
            m_gasFull = 0.9;
        }

        m_freq[0] = 20000;
        m_freq[1] = 20000;
        m_factorVol2Duty = 3.0f;
        m_dutyCycle[0] = m_gasFull/m_factorVol2Duty;
        m_dutyCycle[1] = m_turnNeutral/m_factorVol2Duty;

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
        printError(DAQmxCreateAOVoltageChan(m_hTaskOutput, analog_output, "", m_minValChannel, m_maxValChannel, DAQmx_Val_Volts, ""),322, " in IOcontrol");
        printError(DAQmxCreateDOChan(m_hTaskPower, digital_output_power, "", DAQmx_Val_ChanForAllLines),323, " in IOcontrol");
#endif

        break;

    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        printError(DAQmxCreateDOChan(m_c2TaskPower, digital_output_power, "", DAQmx_Val_ChanForAllLines), 346, " in IOcontrol");
        printError(DAQmxCreateCOPulseChanFreq(m_c2TaskOutput, digital_output_PWM, "", DAQmx_Val_Hz, DAQmx_Val_Low, 0.0f, m_freq[0], m_dutyCycle[0]), 347, " in IOcontrol");
        printError(DAQmxCfgImplicitTiming(m_c2TaskOutput, DAQmx_Val_ContSamps, 10000), 348, " in IOcontrol");
#endif
        break;

    default:
        break;
    }
}

// Start the tasks created for the relevant controller
void IOControl::startController(void)
{
    switch (m_handController)
    {
    case HandController::HandControl_1:
#ifdef NIDAQ_IS_AVALIABLE
        printError(DAQmxStartTask(m_hTaskOutput), 365, " in IOcontrol");
        printError(DAQmxStartTask(m_hTaskPower), 366, " in IOcontrol");
        controllerOn();
#endif
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        printError(DAQmxStartTask(m_c2TaskOutput), 373, " in IOcontrol");
        printError(DAQmxStartTask(m_c2TaskPower), 374, " in IOcontrol");
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
        if(writeVoltageLogAuto || writeVoltageLogMan || writeVoltageLogAss)
            LF.close(); //Do this only for controller 1, since LF is not initiated with controller 2

        printError(DAQmxStopTask(m_hTaskOutput), 434, " in IOcontrol");
        printError(DAQmxStopTask(m_hTaskPower), 435, " in IOcontrol");

        printError(DAQmxClearTask(m_hTaskOutput), 438, " in IOcontrol");
        printError(DAQmxClearTask(m_hTaskPower), 439, " in IOcontrol");
#endif
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        controllerOff();
        printError(DAQmxStopTask(m_c2TaskOutput), 446, " in IOcontrol");
        printError(DAQmxStopTask(m_c2TaskPower), 447, " in IOcontrol");

        printError(DAQmxClearTask(m_c2TaskOutput), 443, " in IOcontrol");
        printError(DAQmxClearTask(m_c2TaskPower), 444, " in IOcontrol");
#endif
        break;
    default:
        break;
    }
    if(firstCallingStop)
    {
        firstCallingStop = false;
        firstCallingConstructor = true;
        printError(DAQmxStopTask(TaskInput), 452, " in IOcontrol");
        printError(DAQmxClearTask(TaskInput), 453, " in IOcontrol");
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

void IOControl::sendSignals(float gas, float turn, CarData &carData, bool isBacking)
{
    switch (m_handController)
    {
    case HandController::HandControl_1:
#ifdef NIDAQ_IS_AVALIABLE
        m_tempArray[0] = gas;
        m_tempArray[1] = turn;
        decimalToVoltage(m_tempArray, isBacking);

        if (writeVoltageLogAuto)
        {
            LF << timeVoltageLog.elapsed()/1000.0 << " " << carData.id << " ";
            LF << carData.state[0] << " " << carData.state[1] << " " ;
            LF << carData.state[2] << " " << carData.state[3] << " " ;
            LF << carData.state[4] << " " ;
            LF << m_tempArray[0] << " " << m_tempArray[1] << " ";
            LF << gas << " " << turn << "\n" ;
        }

        sendSignalsVolt(m_tempArray);

#endif
        break;
    case HandController::HandControl_2:
        m_tempArray[0] = gas;
        m_tempArray[1] = turn;
        decimalToVoltage(m_tempArray, isBacking);
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
        printError(DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL), 482, " in IOcontrol");

        Sleep(20);
        //uInt8 value[1];
        value[0] = 1;
        // Turn on controller.
        printError(DAQmxWriteDigitalLines(m_hTaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL), 488, " in IOcontrol");
        qDebug("handcontroller 1 turned on");
        // Sleep to ensure that the signals have been written.
        Sleep(200);
        signal[0] = m_gasNeutral; //0.9;
        signal[1] = m_turnNeutral;
        // Send neutral signals.
        printError(DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL), 495, " in IOcontrol");
        qDebug("write neutral signals");
#endif
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        //float64 signal[2];
        // Start the controller with full gas. This ensures that the car starts in racing mode.

        signal[0] = m_gasFull;
        signal[1] = m_turnNeutral;
        voltageToDutyCycle(signal);
        printError(DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL), 505, " in IOcontrol");

        Sleep(20);
        //uInt8 value[1];
        value[0] = 1;
        // Turn on controller.
        printError(DAQmxWriteDigitalLines(m_c2TaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL), 511, " in IOcontrol");
        qDebug("handcontroller 2 turned on");

        // Sleep to ensure that the signals have been written.
        Sleep(200);
        signal[0] = m_gasNeutral;
        signal[1] = m_turnNeutral;
        voltageToDutyCycle(signal);
        // Send neutral signals.
        printError(DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL), 519, " in IOcontrol");
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
        printError(DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL), 607, " in IOcontrol");
        value[0] = 0;
        printError(DAQmxWriteDigitalLines(m_hTaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL), 609, " in IOcontrol");
        signal[0] = 0;
        signal[1] = 0;
        printError(DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL), 612, " in IOcontrol");
#endif
        break;
    case HandController::HandControl_2:
        signal[0] = m_gasNeutral;
        signal[1] = m_turnNeutral;
        voltageToDutyCycle(signal);
        printError(DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL), 618, " in IOcontrol");
        value[0] = 0;
        printError(DAQmxWriteDigitalLines(m_c2TaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL), 620, " in IOcontrol");
        signal[0] = 0.001;
        signal[1] = 0.001;
        printError(DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL), 623, " in IOcontrol");
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
        LF << timeVoltageLog.elapsed()/1000.0 << " " << carData.id << " ";
        LF << carData.state[0] << " " << carData.state[1] << " " ;
        LF << carData.state[2] << " " << carData.state[3] << " " ;
        LF << carData.state[4] << " " ;
        LF << m_tempArray[0] << " " << m_tempArray[1] << "\n";
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
    decimalToVoltage(voltArray, false);

    // Write gas signal from hand controller.
    voltArray[0] = m_tempArray[0];

    // Send signals to the car.
    sendSignalsVolt(voltArray);

    if (writeVoltageLogAss)
    {
        LF << timeVoltageLog.elapsed()/1000.0 << " " << carData.id << " ";
        LF << carData.state[0] << " " << carData.state[1] << " " ;
        LF << carData.state[2] << " " << carData.state[3] << " " ;
        LF << carData.state[4] << " " ;
        LF << voltArray[0] << " " << voltArray[1] << " ";
        LF << turnSignal << "\n" ;
    }

#endif
}

void IOControl::voltageToDecimal(float64 *voltage)
{
#ifdef NIDAQ_IS_AVALIABLE
    // Transform gas signal
    // If voltage results in no gas


    if (voltage[0] <= m_voltReverseThreshold && voltage[0] >= m_voltGasMin)
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
    else if (voltage[0] < m_voltGasMin && voltage[0] >= m_voltGasIntervall)
        //else if (voltage[0] < m_gasNeutral && voltage[0] >= m_voltGasIntervall)
    {
        voltage[0] = m_linearizationBreak * (m_voltGasMin - voltage[0]) / (m_voltGasMin - m_voltGasIntervall);
    }
    // If voltage is m_linearizationBreak - 100%
    else if (voltage[0] < m_voltGasIntervall && voltage[0] >= m_minGasVolt)
    {
        voltage[0] = m_linearizationBreak + (1 - m_linearizationBreak) * (m_voltGasIntervall - voltage[0]) / (m_voltGasIntervall - m_minGasVolt);
    }
    // If voltage is out of bounds
    else
    {
        std::cout << "Gas signal is out of bounds: " << voltage[0] << std::endl;
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
void IOControl::decimalToVoltage(float64 *decimal, bool isBacking)
{
    //qDebug() << "gas i decimal:  " << decimal[0];

    // Transform gas signal if gas is 0
    if (decimal[0] == 0)
    {
        decimal[0] = m_gasNeutral;
    }
    else if (decimal[0] > 0 && decimal[0] <= 1) // If positive gas signal
    {
        decimal[0] = m_voltGasIntervall + decimal[0] * m_voltGasSlope;
    }
    // If break/reverse
    else if (isBacking && decimal[0] < 0 && decimal[0] >= -1)
    {
        decimal[0] = m_voltReverseThreshold  - decimal[0] *  m_reverseGasSlope;
    }
    else if (decimal[0] < 0 && decimal[0] >= -1)
    {
        decimal[0] = m_voltBrakeThreshold  - decimal[0] * m_reverseGasSlope;
    }
    // If out of bounds
    else
    {
        decimal[0] = m_gasNeutral;
        std::cout << "Gas signal is out of bounds: " << decimal[0] << std::endl;
    }
    //qDebug() << "gas i voltage:  " << decimal[0];

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


/*Reads signals from analog input pins in the DAQ, and only saves the values tha belongs to the calling handcontroller.
 * If another handcontroller is installed. 1, increase size of tempArray. 2, increase buffer size in DAQmxReadAnalogF64.
 * 3, extend switch/case structure.
 * */
void IOControl::receiveSignalsVolt(float64 *signal)
{
    float64 tempArray[4];
#ifdef NIDAQ_IS_AVALIABLE
    printError(DAQmxReadAnalogF64(TaskInput, 1, 10.0, DAQmx_Val_GroupByChannel, tempArray, 4, NULL, NULL), 797, " in IOcontrol");
    switch(m_handController)
    {
    case (HandController::HandControl_1) :
        signal[0] = tempArray[0];
        signal[1] = tempArray[1];
        break;
    case (HandController::HandControl_2) :
        signal[0] = tempArray[2];
        signal[1] = tempArray[3];
        break;
    }
    signal[0] = (signal[0] + m_prevSignals[0]) / 2;
    signal[1] = (signal[1] + m_prevSignals[1]) / 2;
    //qDebug("gas: %f \t \t Turn: %f", signal[0], signal[1]);
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
        printError(DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL),866, " in IOcontrol");
        //qDebug() << "gas voltage: " << signal[0];
#endif
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        //qDebug() << "gas voltage: " << signal[0];
        voltageToDutyCycle(signal);
        printError(DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL), 887, " in IOcontrol");
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
    if ( (values[0]<=0.001))
    {
        qDebug("gas duty cycle is out of bounds");
        qDebug() << values[0];
        values[0] = 0.001;
    }
    else if(values[0]>=0.99)
    {
        qDebug("gas duty cycle is out of bounds");
        qDebug() << values[0];
        values[0] = 0.99;
    }
    if ( (values[1]<=0.001))
    {
        qDebug("turn duty cycle is out of bounds");
        qDebug() << values[1];
        values[1] = 0.001;
    }
    else if(values[1]>=0.99)
    {
        qDebug("turn duty cycle is out of bounds");
        qDebug() << values[1];
        values[1] = 0.99;
    }
}
