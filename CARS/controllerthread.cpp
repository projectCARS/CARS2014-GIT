#include "controllerthread.h"

ControllerThread::ControllerThread(QObject *parent) :
    QThread(parent)
{
    m_doStop = false;
}

void ControllerThread::loadControllerSettings()
{
    // Delete previous controllers.
    std::vector<Controller*>::iterator it;
    for (it=m_controllers.begin(); it!=m_controllers.end(); )
    {
        delete *it;
        it = m_controllers.erase(it);
    }


    m_numCars = 0;
    // Load controller settings and populate controller vector.
    while (m_settings.contains(QString("car/id%1/mode").arg(m_numCars)))
    {
        m_settings.beginGroup(QString("car/id%1").arg(m_numCars));
        // Add car to vector m_cars.
        switch (static_cast<ControllerType::Enum>(m_settings.value("controller").toInt()))
        {
        case ControllerType::PIDController:
            m_controllers.push_back(new PIDController(m_numCars));
            break;
        case ControllerType::PIDControllerSR:
            m_controllers.push_back(new PIDControllerSR(m_numCars));
            break;
        case ControllerType::PIDadaptiveGain:
            m_controllers.push_back(new PIDadaptiveGain(m_numCars));
            break;
        default:
            qDebug() << "Error: Controller type not implemented, in loadControllerSettings(), controllerthread.cpp";
        }
        m_settings.endGroup();
        m_numCars++;
        //qDebug() << m_numCars;
    }

    // If no controller settings are found, create a default controller.
    if (m_numCars == 0)
    {
        // TODO, implement this properly!
        m_settings.beginGroup(QString("car/id%1").arg(m_numCars));
        m_controllers.push_back(new PIDController(m_numCars));
        m_settings.endGroup();
        m_numCars++;
    }
}

void ControllerThread::loadHandControllerSettings( std::vector<CarData> &carData)
{
    int numCars = 0;
    while (m_settings.contains(QString("car/id%1/handController").arg(numCars)))
    {
        m_settings.beginGroup(QString("car/id%1").arg(numCars));
        // Add car to vector m_cars.
        carData[numCars].handController = static_cast<HandController::Enum>(m_settings.value("handController").toInt());
        m_settings.endGroup();
        numCars++;
    }
}

void ControllerThread::loadModeSettings( std::vector<CarData> &carData)
{
    int numCars = 0;
    while (m_settings.contains(QString("car/id%1/mode").arg(numCars)))
    {
        m_settings.beginGroup(QString("car/id%1").arg(numCars));
        // Add car to vector m_cars.
        carData[numCars].mode = static_cast<CarMode::Enum>(m_settings.value("mode").toInt());
        m_settings.endGroup();
        numCars++;
    }
}

void ControllerThread::run()
{
    qDebug("cthread started");
    // Load controller settings from file.
    loadControllerSettings();
    // Could be a member variable.
    std::vector<Signal> signal(m_numCars);

    // Reserve memory for m_numCars cars.
    std::vector<CarData> carData(m_numCars);

    // Vector with one IO controller for each car.
    std::vector<IOControl> ioControls;

    // Load hand controller settings from file
    loadHandControllerSettings(carData);
    loadModeSettings(carData);

    for (int i = 0; i < m_numCars; i++)
    {
        ioControls.push_back(IOControl(i,carData[i].handController));
    }

    // Initialize and start IO controllers.
    for (int i = 0; i < m_numCars; i++)
    {
        if(carData[i].mode != CarMode::NotConnected)
        {
            ioControls[i].init();
            ioControls[i].startController();
        }
    }

    // Initialize backing controllers. Delete old data.
    m_backingSequence.resize(0);
    m_isBacking.resize(0);
    m_stuckCounter.resize(0);
    for (int i = 0; i < m_numCars; i++)
    {
        m_backingSequence.push_back(new AutoReverse(i));
        m_isBacking.push_back(false);
        m_stuckCounter.push_back(0);
    }

    while(1)
    {
        // Check if thread should stop running.
        m_doStopMutex.lock();
        if (m_doStop)
        {
            m_doStop = false;
            m_doStopMutex.unlock();
            break;
        }
        m_doStopMutex.unlock();


        // Wait for all data to be written to struct.
        WaitForSingleObject(hControllerThreadEvent1, INFINITE);

        // Copy data.
        EnterCriticalSection(&csControllerThreadData);
        carData = controllerThreadData.carData;
        LeaveCriticalSection(&csControllerThreadData);

        // Loop over all cars and send control signals.
        for (int i = 0; i < m_numCars; i++)
        {
            // Only send signals to active cars.
            if (carData[i].active)
            {
                // Check in which mode the car is operating.
                switch (carData[i].mode)
                {
                    // Send gas and turn signal from hand controller to car.
                case CarMode::Manual:
                    ioControls[i].manualControl(carData[i]);
                    ioControls[i].receiveSignals(signal[i].gas, signal[i].turn);
                    break;
                    // Send gas and turn signal from controller to car.
                case CarMode::Auto:

                    if (-0.02 < carData[i].state[2] && carData[i].state[2] < 0.02 && signal[i].gas > 0.005 && !m_isBacking[i])
                    {
                        m_stuckCounter[i]++;
                    }
                    else
                        m_stuckCounter[i] = 0;

                    if (m_stuckCounter[i] >= 30)
                        m_isBacking[i] = true;

                    if (m_isBacking[i])
                    {
                        m_backingSequence[i]->calcSignals(carData[i].state, signal[i].gas, signal[i].turn);
                        m_isBacking[i] = m_backingSequence[i]->isBacking();
                    }
                    else
                    {
                        m_controllers[i]->calcSignals(carData[i].state, signal[i].gas, signal[i].turn);
                    }

                    ioControls[i].sendSignals(signal[i].gas, signal[i].turn, carData[i]);

                    break;
                    // Send turn signal from controller and gas signal from hand controller to car.
                case CarMode::Assisted:
                    ioControls[i].receiveSignals(signal[i].gas, signal[i].turn);
                    m_controllers[i]->calcTurnSignal(carData[i].state, signal[i].turn);
                    ioControls[i].assistedControl(signal[i].turn, carData[i]);
                    break;
                case CarMode::NotConnected:
                    // Do nothing.
                    break;
                default:
                    std::cout << "Error: Car mode type not implemented, in run(), controllerthread.cpp" << std::endl;
                }
            }
        }

        // Wait for input signal values to be read from struct.

        WaitForSingleObject(hControllerThreadEvent_signalsRead, INFINITE);

        // Copy data.
        EnterCriticalSection(&csControllerThreadData);
        controllerThreadData.signal = signal;
        LeaveCriticalSection(&csControllerThreadData);

        // Tell the processing thread that the input signals has been read.

        SetEvent(hControllerThreadEvent_signalsWritten);
    }

    // Stop all cars running in Auto mode. This doesn't always work (only if the threads are terminated in a specific way.)
    for (int i = 0; i < m_numCars; i++)
    {
        if (carData[i].active && (carData[i].mode == CarMode::Auto))
        {
            signal[i].gas = -0.9f;
            signal[i].turn = 0;
            ioControls[i].sendSignals(signal[i].gas, signal[i].turn, carData[i]);
        }
    }
    // Sleep so that the cars have time to stop.
    Sleep(1000);

    // Stop IO controllers.
    for (int i = 0; i < m_numCars; i++)
    {
        if(carData[i].mode != CarMode::NotConnected)
        {
            ioControls[i].stopController();
        }
    }
}

bool ControllerThread::stop()
{
    QMutexLocker locker(&m_doStopMutex);
    if (m_doStop == false)
    {
        // Stop thread and return true.
        m_doStop = true;
        return true;
    }
    else
    {
        // Return false if thread is already stopped.
        return false;
    }
}
