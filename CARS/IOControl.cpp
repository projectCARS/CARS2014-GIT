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

void printError(int32 status, int line)
{
    if( DAQmxFailed(status) )
    {
        char errBuff[2048];
        DAQmxGetExtendedErrorInfo(errBuff,2048);
        qDebug() << errBuff << "at line " << line << "\n\n";
    }
}

IOControl::IOControl(int ID, HandController::Enum handController)
{

    m_handController = handController;
     m_ID = ID;
    switch (m_handController)
    {

    case HandController::HandControl_1:
        qDebug("handkontroll 1 startad till bil %i", ID);

#ifdef NIDAQ_IS_AVALIABLE

        printError(DAQmxClearTask(m_hTaskInput), 41);
        printError(DAQmxClearTask(m_hTaskOutput), 42);
        printError(DAQmxClearTask(m_hTaskPower), 43);

        printError(DAQmxCreateTask("m_hTaskInput", &m_hTaskInput),27);
        printError(DAQmxCreateTask("m_hTaskOutput", &m_hTaskOutput),28);
        printError(DAQmxCreateTask("m_hTaskPower", &m_hTaskPower),29);
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
            m_voltGasSlope = -0.533;

            m_voltReverseThreshold = 1.68;

            m_reverseGasSlope = 0.47;

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

            m_voltReverseThreshold = 1.68;

            m_reverseGasSlope = 0.47;

            m_linearizationBreak = 0.38;
            m_voltGasThreshold = 1.4706;
            m_voltGasIntervall = 1.4693;
            m_voltGasSlope = -0.533;

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

            m_voltReverseThreshold = 1.68;

            m_reverseGasSlope = 0.47;

            m_linearizationBreak = 0.38;
            m_voltGasThreshold = 1.4706;
            m_voltGasIntervall = 1.4693;
            m_voltGasSlope = -0.533;

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

            m_reverseGasSlope = 0.47;

            m_linearizationBreak = 0.38;
            m_voltGasThreshold = 1.4706;
            m_voltGasIntervall = 1.4693;
            m_voltGasSlope = -0.533;

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
        qDebug("handkontroll 2 startad till bil %i", ID);
#ifdef NIDAQ_IS_AVALIABLE
        printError(DAQmxCreateTask("m_c2TaskInput", &m_c2TaskInput),168);
        printError(DAQmxCreateTask("m_c2TaskOutput", &m_c2TaskOutput),169);
        printError(DAQmxCreateTask("m_c2TaskPower", &m_c2TaskPower),170);
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
            m_voltGasSlope = -0.533;

            m_voltReverseThreshold = 1.63;

            m_reverseGasSlope = 0.47;

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
        case 1:
            m_minValChannel = 0;
            m_maxValChannel = 3;

            m_linearizationBreak = 0.38;
            m_voltGasThreshold = 1.4706;
            m_voltGasIntervall = 1.44;//1.4693;
            m_voltGasSlope = -0.59;

            m_voltReverseThreshold = 1.57;//1.68;
            m_reverseGasSlope = 0.75;

            m_minGasVolt = 0.91; // 1.14;
            m_maxGasVolt = 3.5; // 2.31;
            m_minTurnVolt = 0.17;
            m_maxTurnVolt = 2.9; //3.242;

            m_gasNeutral = 1.54;
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
        case 2:
            m_minValChannel = 0;
            m_maxValChannel = 3;

            m_linearizationBreak = 0.38;
            m_voltGasThreshold = 1.4706;
            m_voltGasIntervall = 1.4693;
            m_voltGasSlope = -0.533;

            m_voltReverseThreshold = 1.68;

            m_minGasVolt = 0.91; // 1.14;
            m_maxGasVolt = 2.10; // 2.31;
            m_minTurnVolt = 0.17;
            m_maxTurnVolt = 2.9; //3.242;

            m_gasNeutral = 1.58;
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
            m_minValChannel = 0;
            m_maxValChannel = 3;

            m_linearizationBreak = 0.38;
            m_voltGasThreshold = 1.4706;
            m_voltGasIntervall = 1.4693;
            m_voltGasSlope = -0.533;

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

            m_freq[0] = 20000;
            m_freq[1] = 20000;
            m_dutyCycle[0] = m_gasFull/3.0;
            m_dutyCycle[1] = m_turnNeutral/3.0;
            m_factorVol2Duty = 3.0f;

            break;
            //TODO: do something
            break;
        }

        break;

    default:
        //TODO: do something or...
        break;
    }
    qDebug("8");
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
        qDebug("init initiated");
        printError(DAQmxCreateAIVoltageChan(m_hTaskInput, analog_input, "", DAQmx_Val_RSE, m_minValChannel, m_maxValChannel, DAQmx_Val_Volts, NULL), 321);
        qDebug("ch1");
        printError(DAQmxCreateAOVoltageChan(m_hTaskOutput, analog_output, "", m_minValChannel, m_maxValChannel, DAQmx_Val_Volts, ""),322);
        qDebug("ch2");
        printError(DAQmxCreateDOChan(m_hTaskPower, digital_output_power, "", DAQmx_Val_ChanForAllLines),323);
        qDebug("analog output initiated");
#endif
        writeVoltageLog = true;
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
        printError(DAQmxCreateAIVoltageChan(m_c2TaskInput, analog_input, "", DAQmx_Val_RSE, m_minValChannel, m_maxValChannel, DAQmx_Val_Volts, NULL), 345);
        printError(DAQmxCreateDOChan(m_c2TaskPower, digital_output_power, "", DAQmx_Val_ChanForAllLines), 346);
        printError(DAQmxCreateCOPulseChanFreq(m_c2TaskOutput, digital_output_PWM, "", DAQmx_Val_Hz, DAQmx_Val_Low, 0.0f, m_freq[0], m_dutyCycle[0]), 347);
        printError(DAQmxCfgImplicitTiming(m_c2TaskOutput, DAQmx_Val_ContSamps, 10000), 348);

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
        printError(DAQmxStartTask(m_hTaskInput), 364);
        printError(DAQmxStartTask(m_hTaskOutput), 365);
        printError(DAQmxStartTask(m_hTaskPower), 366);
        controllerOn();
#endif
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        printError(DAQmxStartTask(m_c2TaskInput), 372);
        printError(DAQmxStartTask(m_c2TaskOutput), 373);
        printError(DAQmxStartTask(m_c2TaskPower), 374);
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
        printError(DAQmxStopTask(m_hTaskInput), 433);
        printError(DAQmxStopTask(m_hTaskOutput), 434);
        printError(DAQmxStopTask(m_hTaskPower), 435);

        printError(DAQmxClearTask(m_hTaskInput), 437);
        printError(DAQmxClearTask(m_hTaskOutput), 438);
        printError(DAQmxClearTask(m_hTaskPower), 439);
#endif
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        controllerOff();
        printError(DAQmxStopTask(m_c2TaskInput), 445);
        printError(DAQmxStopTask(m_c2TaskOutput), 446);
        printError(DAQmxStopTask(m_c2TaskPower), 447);

        printError(DAQmxClearTask(m_c2TaskInput), 449);
        printError(DAQmxClearTask(m_c2TaskOutput), 450);
        printError(DAQmxClearTask(m_c2TaskPower), 451);
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

        sendSignalsVolt(m_tempArray);



#endif
        break;
    case HandController::HandControl_2:
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
        printError(DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL), 482);

        Sleep(20);
        //uInt8 value[1];
        value[0] = 1;
        // Turn on controller.
        printError(DAQmxWriteDigitalLines(m_hTaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL), 488);
        qDebug("handcontroller 1 turned on");
        // Sleep to ensure that the signals have been written.
        Sleep(200);
        signal[0] = m_gasNeutral; //0.9;
        signal[1] = m_turnNeutral;
        // Send neutral signals.
        printError(DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL), 495);
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
        printError(DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL), 505);

        Sleep(20);
        //uInt8 value[1];
        value[0] = 1;
        // Turn on controller.
        printError(DAQmxWriteDigitalLines(m_c2TaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL), 511);
        qDebug("handcontroller 2 turned on");

        // Sleep to ensure that the signals have been written.
        Sleep(200);
        signal[0] = m_gasNeutral;
        signal[1] = m_turnNeutral;
        voltageToDutyCycle(signal);
        // Send neutral signals.
        printError(DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL), 519);
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
        printError(DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL), 607);
        value[0] = 0;
        printError(DAQmxWriteDigitalLines(m_hTaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL), 609);
        signal[0] = 0;
        signal[1] = 0;
        printError(DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL), 612);
#endif
        break;
    case HandController::HandControl_2:
        signal[0] = m_gasNeutral;
        signal[1] = m_turnNeutral;
        voltageToDutyCycle(signal);
        printError(DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL), 618);
        value[0] = 0;
        printError(DAQmxWriteDigitalLines(m_c2TaskPower, 1, 1, 10.0, DAQmx_Val_GroupByChannel, value, NULL, NULL), 620);
        signal[0] = 0.001;
        signal[1] = 0.001;
        printError(DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL), 623);
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
       qDebug("manual gas volt: %f", m_tempArray[0]);
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
    //qDebug() << "gas i decimal:  " << decimal[0];
    if (decimal[0] == 0)
    {
        decimal[0] = m_gasNeutral;
    }
    // If gas is 0 - m_linearizationBreak
    /*else if (0)//decimal[0] > 0 && decimal[0] <= m_linearizationBreak)
    {
        decimal[0] = m_voltGasThreshold - decimal[0] / m_linearizationBreak * (m_voltGasThreshold - m_voltGasIntervall);
    }*/
    // If gas is m_linearizationBreak - 100%
    else if (decimal[0] > 0 && decimal[0] <= 1)//m_linearizationBreak && decimal[0] <= 1)
    {
        //decimal[0] = m_voltGasIntervall - (decimal[0] - m_linearizationBreak) / (1 - m_linearizationBreak)* (m_voltGasIntervall - m_minGasVolt);
        //decimal[0] = -0.33 + m_voltGasIntervall - (decimal[0] - m_linearizationBreak) / (1 - m_linearizationBreak)* (m_voltGasIntervall - m_minGasVolt);
        decimal[0] = m_voltGasSlope*decimal[0] + m_voltGasIntervall;
    }
    // If break/reverse
    else if (decimal[0] < 0 && decimal[0] >= -1)
    {
        decimal[0] = m_voltReverseThreshold  - decimal[0] * m_reverseGasSlope;
        //decimal[0] = -0.2 + m_voltReverseThreshold + -decimal[0] * (m_maxGasVolt - m_voltReverseThreshold)*4;
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
    switch(m_handController)
    {
    case (HandController::HandControl_1) :
        printError(DAQmxReadAnalogF64(m_hTaskInput, 1, 10.0, DAQmx_Val_GroupByChannel, signal, 2, NULL, NULL), 836);
    break;
    case (HandController::HandControl_2) :
        printError(DAQmxReadAnalogF64(m_c2TaskInput, 1, 10.0, DAQmx_Val_GroupByChannel, signal, 2, NULL, NULL), 839);
    break;
    }
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
        //qDebug() << "gas voltage: " << signal[0];
        if ((signal[0] < m_minGasVolt) | (signal[0] > m_maxGasVolt) | (signal[1] < m_minTurnVolt) | (signal[1] > m_maxTurnVolt))
        {
            std::cout << "Output voltage is smaller than m_minVal, or larger than m_maxVal" << std::endl;
            std::cout << "Gas voltage: " << signal[0] << ", Turn voltage: " << signal[1] << std::endl;
        }
        else
        {
            printError(DAQmxWriteAnalogF64(m_hTaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, signal, NULL, NULL),866);
        }
#endif
        break;
    case HandController::HandControl_2:
#ifdef NIDAQ_IS_AVALIABLE
        //qDebug() << "gas voltage: " << signal[0];
        if ((signal[0] < m_minGasVolt) | (signal[0] > m_maxGasVolt))
        {
            std::cout << "Output gas voltage is smaller than m_minVal, or larger than m_maxVal" << std::endl;
            std::cout << "Gas voltage: " << signal[0] << std::endl;
        }
        if ((signal[1] < m_minTurnVolt) | (signal[1] > m_maxTurnVolt))
        {
            std::cout << "Output turn voltage is smaller than m_minVal, or larger than m_maxVal" << std::endl;
            std::cout << "Turn voltage: " << signal[1] << std::endl;
        }
        if((signal[0] >= m_minGasVolt) && (signal[0] <= m_maxGasVolt) && (signal[1] >= m_minTurnVolt) && (signal[1] <= m_maxTurnVolt))
        {
            voltageToDutyCycle(signal);
            qDebug() << "gas duty cycle: " << signal[0];
            printError(DAQmxWriteCtrFreq(m_c2TaskOutput, 1, 1, 10.0, DAQmx_Val_GroupByChannel, m_freq, signal, NULL, NULL), 887);
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
}
