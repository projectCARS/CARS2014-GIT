#include "processingthread.h"
#include "headers.h"
#include "definitions.h"
#include "classes.h"
#include "functions.h"
#include <algorithm>
#include <QDebug>

#include <iomanip>
#include <fstream>

ProcessingThread::ProcessingThread(QObject *parent) :
    QThread(parent)
{
    // Set stop variable to false.
    m_doStop = false;
    // The number of cars is set to zero.
    m_numCars = 0;
}

void ProcessingThread::run()
{
    // Create virtual sensor.
    VirtualSensor sensor;
    // Timing.
    QTime time;
    double ompTimeStart = 0, ompTimeMid = 0, ompTimeEnd = 0, t1 = 0, t2 = 0;
   //double t2,t3,t4 = 0;
    //clock_t timerTest1, timerTest2;
    double timeDiff, currTime;
    // Counts the number while loops.
    int loopCounter = 0;

    // Load car settings from file and create cars.
    loadCarSettings();
    // Load draw settings from file
    loadDrawSettings();
    // Load race settings from file
    loadRaceSettings();

    std::vector<float> markers;
    // Reserve memory for m_numCars cars with 6 markers.
    markers.reserve(m_numCars*6);
    std::vector<CarMeasurement> carMeasurements;
    // Reserve memory for m_numCars cars.
    carMeasurements.reserve(m_numCars);
    std::vector<CarData> carData(m_numCars);
    std::vector<CarData> oldCarData(m_numCars);
    std::vector<Signal> signal(m_numCars);

    // TODO: Should all resizing and initialization of global variables be done in the GUI thread when the program starts, instead of here?

    // Initiate vectors in structs.
    drawThreadData.carData.resize(m_numCars);
    //lapData.bestTimes.reserve(10);
    controllerThreadData.carData.resize(m_numCars);

    // Deciding which log file to write to
    int fileNo = 1;
    std::stringstream str;
    for (int i=0 ; i < 100 ; i++) // Ceiling for number of log files here.
    {
        str.clear();
        str.str(std::string());
        str << "outdata/logFiles/log" << std::setw(3) << std::setfill('0') << fileNo << ".txt";
        if (!fileExists(str.str()))
        {
            break;
        }
        fileNo++;
    }
    std::ofstream logFile;
    logFile.open(str.str());
    logFile << "sysTime carID xPos yPos speed yaw yawVel xPosRaw yPosRaw yawRaw gas turn\n";
    std::cout << "Main thread:		Writing log to " << str.str() << std::endl;

    // Resize vectors.
    for (int i = 0; i < m_numCars; i++)
    {
        // TODO: States should not be hard-coded. Fix this with getter methods in Car.
        carData[i].state.resize(5);
        drawThreadData.carData[i].state.resize(5);
        controllerThreadData.carData[i].state.resize(5);
        controllerThreadData.signal.resize(2*m_numCars);
    }

    // Number of iterations to average over when calculating FPS in main loop.
    int fpsInterval = 200;

    // Determines how often data should be logged to file.
    int logInterval = 10;

    // Start virtual sensor.
    sensor.startSensor();

    // Initiate timer for lap times
   // lapData.lapTimer.start();
    ompTimeStart = omp_get_wtime();
    time.start();

    while(1)
    {
        // Check if thread should stop running.
        m_doStopMutex.lock();
        if (m_doStop)
        {
            m_doStop = false;
            m_doStopMutex.unlock();
            // Step out of while loop.
            break;
        }
        m_doStopMutex.unlock();

        // Save car data from previous iteration.
        oldCarData = carData;



        if(m_cars[0].getFiltertype() != FilterType::Enum::ParticleFilter)
        {
            // Detect markers using virtual sensor. Output is written to vector markers in world coordinates.
            markers = sensor.detectMarkers();
            // Calculate position, angle and id.
            carMeasurements = findCars(markers);
        }
        else
        {
            t1 = omp_get_wtime();
            // Add a thresholded image to drawthreadData struct
            sensor.grabThresholdImage();
            t2 = omp_get_wtime();
            EnterCriticalSection(&csDrawThreadData);
            m_cars[0].addImageMeasurement(drawThreadData.processedImage);
            LeaveCriticalSection(&csDrawThreadData);

            //std::cout << "Measurement Update : " << omp_get_wtime() - t1 << std::endl;

        }

        // Add measurements to cars.
        for (int j = 0; j < carMeasurements.size(); j++)
        {
            // Add new measurement to car.
            if(m_cars[j].getFiltertype() != FilterType::Enum::ParticleFilter)
            {
                m_cars[carMeasurements[j].id].addMeasurement(carMeasurements[j].x, carMeasurements[j].y, carMeasurements[j].theta);
            }
        }

        // Wait for new input signal values to be written to struct.
        WaitForSingleObject(hControllerThreadEvent_signalsWritten, INFINITE);

        // Copy data.
        EnterCriticalSection(&csControllerThreadData);
        signal = controllerThreadData.signal;
        LeaveCriticalSection(&csControllerThreadData);

        // Tell controller thread that the input signals has been read.
        SetEvent(hControllerThreadEvent_signalsRead);

        // Loop over all cars and estimate states.
        for (int j = 0; j < m_numCars; j++)
        {
            // Add input signals to cars.
            m_cars[j].addInputSignals(signal[j].gas, signal[j].turn);
            // Update filter.
            ompTimeMid = omp_get_wtime();
            m_cars[j].updateFilter();
            ompTimeEnd = omp_get_wtime();
            /* Extract data from the car object and store in vector carData,
            which will be sent to other threads. */
            carData[j].id = m_cars[j].getId();
            carData[j].active = m_cars[j].isActive();
            carData[j].lost = m_cars[j].isLost();
            carData[j].mode = m_cars[j].getMode();
            carData[j].state = m_cars[j].getState();

            if(m_cars[j].getFiltertype() == FilterType::Enum::ParticleFilter)
            {
                float yaw = carData[j].state[3];
                float vel = carData[j].state[2];
                float angvel = carData[j].state[4];
                sensor.cameraToWorldCoordinates(carData[j].state);
                carData[j].state[2] = vel;
                carData[j].state[3] = yaw;
                carData[j].state[4] = angvel;
            }
            //std::cout << carData[j].state[0] << std::endl << carData[j].state[1] << std::endl << carData[j].state[3] << std::endl;
        }

        // Log data. Currently only log data of car with id 0.
        if (loopCounter%logInterval==0) // log only some loops
        {
            // Time elapsed since startup in seconds.
            currTime = (double)time.elapsed()/1000.0;
            logData(currTime, carData, carMeasurements, signal, &logFile);
        }

        // Use old and new states to draw traveled distance on evaluated image.
        EnterCriticalSection(&csDrawThreadData);
        for (int k = 0; k < drawThreadData.background.size(); k++)
        {
            drawStatesToImage(oldCarData, carData, drawThreadData.background[k], signal);
        }
        LeaveCriticalSection(&csDrawThreadData);

        // Copy data to struct.
        EnterCriticalSection(&csControllerThreadData);
        controllerThreadData.carData = carData;
        LeaveCriticalSection(&csControllerThreadData);

        // Tell controller thread that new data have been written.
        SetEvent(hControllerThreadEvent1);

        // Update lap times
        for(int i = 0; i < m_cars.size(); i++){
            if(m_carSpecificDrawSettings[i] & CarSpecificDrawSettings::Timer)
                updateLapData(carData[i]);
        }

        // Enter critical section and send data to draw thread.


        // Update Race conditions
        for(int i = 0; i < raceSettings.carID.size(); i++)
        {
                if(!raceSettings.raceDone && updateRaceData(carData[i], raceSettings))
                {
                    qDebug("Race is over!! A winner is Car %i", raceSettings.winnerID);
                }
        }

        EnterCriticalSection(&csDrawThreadData);
        drawThreadData.carData = carData;
        drawThreadData.raceData = raceSettings;
        LeaveCriticalSection(&csDrawThreadData);

        // Estimate FPS in processing thread.
        if ((loopCounter % fpsInterval) == 0)
        {
            //Calculate duration of one iteration using OMP
            timeDiff = (double)(omp_get_wtime()-ompTimeStart) / fpsInterval * 1000;
            //std::cout << "Measurement Update : " << t2 - t1 << std::endl;
            //std::cout << "Filter Update : " << ompTimeEnd - ompTimeMid << std::endl;
            //std::cout << "Total loop time : " << (omp_get_wtime() - ompTimeStart)/ fpsInterval << std::endl;

            ompTimeStart = omp_get_wtime();
            // Calculate duration of one iteration in ms.
            std::cout << "FPS main: " << 1000.0/timeDiff << std::endl;

        }

        // Increment the loop counter.
        loopCounter++;
    }
    // Stop virtual sensor.
    sensor.stopSensor();
}

bool ProcessingThread::stop()
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

QImage ProcessingThread::matToQImage(const cv::Mat &mat)
{
    if(mat.type() == CV_8UC1)
    {
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
        {
            colorTable.push_back(qRgb(i,i,i));
        }
        const uchar *qImageBuffer = (const uchar*)mat.data;
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        return img.rgbSwapped();
    }
    else
    {
        qDebug() << "Error: cv::Mat could not be converted to QImage.";
        return QImage();
    }
}

/* Draw states to image. This function needs to be called for each estimation
of the state vector (in order to obtain a smooth line on the image). */
void ProcessingThread::drawStatesToImage(const std::vector<CarData> &oldCarData, const std::vector<CarData> &carData, cv::Mat &evaluatedImage, std::vector<Signal> signal)
{
    float distSquared, maxDistSquared;
    float positionPix1[2], positionPix2[2];
    for (int j = 0; j < carData.size(); j++)
    {
        if (carData[j].active)
        {

            // Extract position and convert to pixel coordinates.
            // x coordinate.
            positionPix1[0] = oldCarData[j].state[0] * PIXELS_PER_METER;
            // y coordinate.
            positionPix1[1] = oldCarData[j].state[1] * PIXELS_PER_METER;
            // x coordinate.
            positionPix2[0] = carData[j].state[0] * PIXELS_PER_METER;
            // y coordinate.
            positionPix2[1] = carData[j].state[1] * PIXELS_PER_METER;

            // Calculate the square of the traveled distance.
            distSquared = (carData[j].state[0]-oldCarData[j].state[0])*(carData[j].state[0]-oldCarData[j].state[0])+
                    (carData[j].state[1]-oldCarData[j].state[1])*(carData[j].state[1]-oldCarData[j].state[1]);
            maxDistSquared = 0.5*0.5;

            // Gas signal.
            float gas = signal[j].gas;

            // Draw car path.
            if (m_carSpecificDrawSettings[j] & CarSpecificDrawSettings::Path)
            {
                // Only draw traveled path if squared distance is sufficiently small.
                if (distSquared < maxDistSquared)
                {
                    if (gas > 0)
                        line(evaluatedImage, cv::Point(positionPix1[0], positionPix1[1]), cv::Point(positionPix2[0], positionPix2[1]), cv::Scalar(20 + 235 * gas, 0, 0), 3);
                    else if (gas == 0)
                        line(evaluatedImage, cv::Point(positionPix1[0], positionPix1[1]), cv::Point(positionPix2[0], positionPix2[1]), cv::Scalar(0, 70, 0), 3);
                    else
                        line(evaluatedImage, cv::Point(positionPix1[0], positionPix1[1]), cv::Point(positionPix2[0], positionPix2[1]), cv::Scalar(0, 0, 255 * (1 + gas)), 3);
                }
            }

            // Draw car tracks.
            if (m_carSpecificDrawSettings[j] & CarSpecificDrawSettings::CarTracks)
            {
                // Get yaw to draw tire marks.
                float yaw1 = carData[j].state[3];
                float yaw2 = oldCarData[j].state[3];
                int r = 8;
                float pl1[2] = { positionPix1[0] - r*sin(yaw1), positionPix1[1] + r*cos(yaw1) };
                float pr1[2] = { positionPix1[0] + r*sin(yaw1), positionPix1[1] - r*cos(yaw1) };
                float pl2[2] = { positionPix2[0] - r*sin(yaw2), positionPix2[1] + r*cos(yaw2) };
                float pr2[2] = { positionPix2[0] + r*sin(yaw2), positionPix2[1] - r*cos(yaw2) };

                // Only draw tracks if squared distance is sufficiently small.
                if (distSquared < maxDistSquared)
                {

                    if (gas >= .9)
                    {
                        float col = (255 - 255 * gas);
                        line(evaluatedImage, cv::Point(pl1[0], pl1[1]), cv::Point(pl2[0], pl2[1]), cv::Scalar(col, col, col), 2);
                        line(evaluatedImage, cv::Point(pr1[0], pr1[1]), cv::Point(pr2[0], pr2[1]), cv::Scalar(col, col, col), 2);
                    }
                    else if (gas < .0)
                    {
                        float col = 54 + 40*gas;
                        line(evaluatedImage, cv::Point(pl1[0], pl1[1]), cv::Point(pl2[0], pl2[1]), cv::Scalar(col, col, col), 2);
                        line(evaluatedImage, cv::Point(pr1[0], pr1[1]), cv::Point(pr2[0], pr2[1]), cv::Scalar(col, col, col), 2);
                    }
                }
            }
        }
    }
}

void ProcessingThread::loadCarSettings()
{
    // Delete previous cars.
    m_cars.resize(0);
    m_numCars = 0;
    // Load car settings and populate car vector.
    while (m_settings.contains(QString("car/id%1/mode").arg(m_numCars)))
    {
        m_settings.beginGroup(QString("car/id%1").arg(m_numCars));
        // Add car to vector m_cars.
        m_cars.push_back(Car(m_numCars,
                             static_cast<CarMode::Enum>(m_settings.value("mode").toInt()),
                             static_cast<FilterType::Enum>(m_settings.value("filter").toInt()),
                             static_cast<MotionModelType::Enum>(m_settings.value("motion_model").toInt()))
                         );
        m_settings.endGroup();
        m_numCars++;
    }

    // If no car settings are found, create a default car.
    if (m_numCars == 0)
    {
        m_settings.beginGroup(QString("car/id%1").arg(m_numCars));
        m_cars.push_back(Car(m_numCars, CarMode::Auto, FilterType::EKF, MotionModelType::CTModel));
        m_settings.endGroup();
        m_numCars++;
    }
}

void ProcessingThread::loadDrawSettings(void)
{
    m_carSpecificDrawSettings.resize(0);
    // Number of cars read from settings file.
    int numCarsRead = 0;

    // Add default general draw settings if no such settings are found.
    if (!m_settings.contains("draw_settings/general_draw_settings"))
    {
        // General settings.
        m_generalDrawSettings = 0;
    }
    else
    {
        m_generalDrawSettings = m_settings.value("draw_settings/general_draw_settings").toUInt();
    }

    // Read all draw settings from settings file.
    while (m_settings.contains(QString("car/id%1/mode").arg(numCarsRead)))
    {
        if (m_settings.contains(QString("draw_settings/id%1/car_specific_draw_settings").arg(numCarsRead)))
        {
            m_settings.beginGroup(QString("draw_settings/id%1").arg(numCarsRead));
            m_carSpecificDrawSettings.push_back(m_settings.value("car_specific_draw_settings").toUInt());
            m_settings.endGroup();
        }
        else
        {
            m_carSpecificDrawSettings.push_back((unsigned int)0);
        }

        numCarsRead++;
    }
}

void ProcessingThread::loadRaceSettings()
{
    raceSettings.carID.clear();
    raceSettings.raceDone = false;
    if(m_settings.contains("race_settings/number_of_laps"))
    {
        qDebug("settings exist");
        raceSettings.numberOfLaps = m_settings.value("race_settings/number_of_laps").toInt();
        raceSettings.doRace = (bool)m_settings.value("race_settings/do_race").toBool();

        int j = 0;
        while(m_settings.contains(QString("race_settings/id%1/race").arg(j)))
        {
                m_settings.beginGroup(QString("race_settings/id%1").arg(j));
                raceSettings.carID.push_back(m_settings.value("race").toInt());
                m_settings.endGroup();
            j++;
        }
    }
    else
    {
        raceSettings.doRace = false;
    }

}

// Finds car patterns among markers.
std::vector<CarMeasurement> ProcessingThread::findCars(const std::vector<float> &dots)
{
    std::vector<CarMeasurement> carMeasurements;

    //Parameters to adjust:
    const int nrOfIdDots = 2;			//Number of id dots on each car (=2 for 3 cars, =4 for 15 cars
    const int useAdvancedCarSeparation = 1;//=0 No, =1 yes. removes cars that interfere with other cars... (little risky)

    const float idPointRadius = 0.025f;	//Radius, within we look for id-points
    const float m_long = 0.0725f;			//The long side of the triangle we look for
    const float m_short = 0.0335f;		//The short side of the triangle we look for
    const float m_devLong = 0.007f;		//Deviation
    const float m_devShort = 0.007f;     //Deviation

    // Variables.
    int calcNrOfCars = (int)(pow(2, nrOfIdDots) - 1);
    std::vector<float> carsTemp;

    float res1, res2, res3, distanceToIdPoint, headingInDegrees, headingInRadians, angleToIdPoint;
    int stopFlag = 0;
    int id = 0;

    float refX = 0;
    float refY = 0;
    float centerX = 0;
    float centerY = 0;

    std::vector<float> quality(calcNrOfCars, -1);
    float qualityTemp;

    int collisions = 0;
    int collisionBool = 0;
    int remove = 0;
    std::vector <int> carsToRemove;

    //--------------------------//

    //Main algorithm
    for (int i = 0; i < dots.size(); i += 2)
    {
        stopFlag = 0;
        for (int j = 0; j < dots.size(); j += 2)
        {
            if (stopFlag == 0) //just so we dont find the same triangle twice
            {
                res1 = calcDist(dots[i], dots[i + 1], dots[j], dots[j + 1]);
                if (res1 >= (m_long - m_devLong) && res1 <= (m_long + m_devLong))
                {
                    for (int k = 0; k < dots.size(); k += 2)
                    {
                        res2 = calcDist(dots[j], dots[j + 1], dots[k], dots[k + 1]);
                        if (res2 >= (m_short - m_devShort) && res2 <= (m_short + m_devShort))
                        {
                            res3 = calcDist(dots[i], dots[i + 1], dots[k], dots[k + 1]);
                            if (res3 >= (m_long - m_devLong) && res3 <= (m_long + m_devLong))
                            {
                                //std::cout << res1 << ", " << res2 << ", " << res3 << std::endl;
                                //We have found a triangle, now we look for position and heading
                                refX = dots[k] + 0.5*(dots[j] - dots[k]); //Reference dot, between the two forward dots of the triangle
                                refY = dots[k + 1] + 0.5*(dots[j + 1] - dots[k + 1]);
                                centerX = refX + 0.5*(dots[i] - refX);	//Center dot, between reference dot and the dot in the back of the car
                                centerY = refY + 0.5*(dots[i + 1] - refY);

                                headingInRadians = atan2(refY - centerY, refX - centerX); //In radians
                                headingInDegrees = headingInRadians * 180 / M_PI;	//In degrees

                                //Now we have position and heading, now we loook for pattern
                                id = 0; //Reset id for new found car
                                std::vector<int> idPoints;
                                for (int li = 0; li < dots.size(); li += 2) //Check ALL dots if they are id-dots
                                {
                                    if (li != i && li != j && li != k)	//if this potential id-dot isnt part of the triangle
                                    {
                                        distanceToIdPoint = calcDist(dots[li], dots[li + 1], centerX, centerY);
                                        if (distanceToIdPoint <= idPointRadius)
                                        {
                                            idPoints.push_back(li);
                                            angleToIdPoint = atan2(dots[li + 1] - centerY, dots[li] - centerX) - headingInRadians;
                                            if (angleToIdPoint < 0)
                                            {
                                                angleToIdPoint = 2.0*M_PI + angleToIdPoint;
                                            }
                                            if (nrOfIdDots == 2)
                                            {
                                                if (angleToIdPoint >= 0 && angleToIdPoint <= M_PI) { id += 2; }
                                                else { id += 1; }
                                            }
                                            else
                                            {
                                                if (angleToIdPoint >= 0 && angleToIdPoint < 0.5*M_PI) { id += 2; }
                                                if (angleToIdPoint >= 0.5*M_PI && angleToIdPoint < M_PI) { id += 8; }
                                                if (angleToIdPoint >= M_PI && angleToIdPoint < 1.5*M_PI) { id += 4; }
                                                if (angleToIdPoint >= 1.5*M_PI && angleToIdPoint < 2.0*M_PI) { id += 1; }
                                            }
                                        }
                                    }
                                }

                                //Put car into return vector, but only one car per id can be returned
                                if (id != 0 && id <= m_numCars)
                                {
                                    qualityTemp = abs((res1 - m_long)*(res1 - m_long) + (res2 - m_short)*(res2 - m_short) + (res3 - m_long)*(res3 - m_long)); //calculate some quality measure

                                    carsTemp.push_back(float(id));
                                    carsTemp.push_back(centerX);
                                    carsTemp.push_back(centerY);
                                    carsTemp.push_back(headingInRadians);  //specify here if you want headingInRadians or headingInDegrees
                                    carsTemp.push_back(float(i));
                                    carsTemp.push_back(float(j));
                                    carsTemp.push_back(float(k));
                                    for (int rst = 0; rst < 4; rst++)
                                    {
                                        if (rst < idPoints.size())
                                        {
                                            carsTemp.push_back(float(idPoints[rst]));
                                        }
                                        else
                                        {
                                            carsTemp.push_back(-1.0);
                                        }
                                    }
                                    carsTemp.push_back(qualityTemp);
                                    stopFlag = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if (useAdvancedCarSeparation == 1)
    {
        for (int i = 0; i < carsTemp.size(); i += 12) //For every car
        {
            collisions = 0;
            for (int j = 0; j < carsTemp.size(); j += 12) //For every other car
            {
                remove = isThisIntInVector(j, carsToRemove);
                if (i != j && remove == 0)
                {
                    collisionBool = 0;
                    for (int k = 4; k < 11; k++)
                    {
                        for (int k2 = 4; k2 < 11; k2++)
                        {
                            if (carsTemp[i + k] != -1.0)
                            {
                                if (carsTemp[i + k] == carsTemp[j + k2]) { collisionBool = 1; }
                            }
                        }
                    }
                    if (collisionBool == 1) { collisions++; }
                }
            }
            if (collisions >= 2)
            {
                //Romove this car
                carsToRemove.push_back(i);
            }

        }
        for (int i = 0; i < carsTemp.size(); i += 12)
        {
            remove = isThisIntInVector(i, carsToRemove); //was this car removed?
            if (remove == 0)
            {
                // Struct to add car id, position and angle to.
                CarMeasurement carMeasurement;

                id = carsTemp[i];
                centerX = carsTemp[i + 1];
                centerY = carsTemp[i + 2];
                headingInDegrees = carsTemp[i + 3];
                qualityTemp = carsTemp[i + 11];

                if (quality[id - 1] == -1) //If no car with this id have been found before
                {
                    quality[id - 1] = qualityTemp;
                    carMeasurement.id = id - 1;
                    carMeasurement.x = centerX;
                    carMeasurement.y = centerY;
                    carMeasurement.theta = headingInDegrees;
                    carMeasurements.push_back(carMeasurement);
                }
                else
                {
                    if (quality[id - 1] > qualityTemp) //is this new found car better than the best found car with this id?
                    {
                        quality[id - 1] = qualityTemp;
                        carMeasurement.id = id - 1;
                        carMeasurement.x = centerX;
                        carMeasurement.y = centerY;
                        carMeasurement.theta = headingInDegrees;
                        carMeasurements.push_back(carMeasurement);
                    }
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < carsTemp.size(); i += 12)
        {

            // Struct to add car id, position and angle to.
            CarMeasurement carMeasurement;

            id = int(carsTemp[i]);
            centerX = carsTemp[i + 1];
            centerY = carsTemp[i + 2];
            headingInDegrees = carsTemp[i + 3];
            qualityTemp = carsTemp[i + 7];
            if (quality[id - 1] == -1) //If no car with this id have been found before
            {
                quality[id - 1] = qualityTemp;
                carMeasurement.id = id - 1;
                carMeasurement.x = centerX;
                carMeasurement.y = centerY;
                carMeasurement.theta = headingInDegrees;
                carMeasurements.push_back(carMeasurement);
            }
            else
            {
                if (quality[id - 1] > qualityTemp) //is this new found car better than the best found car with this id?
                {
                    quality[id - 1] = qualityTemp;
                    carMeasurement.id = id - 1;
                    carMeasurement.x = centerX;
                    carMeasurement.y = centerY;
                    carMeasurement.theta = headingInDegrees;
                    carMeasurements.push_back(carMeasurement);
                }
            }
        }
    }
    return carMeasurements;
}

// Used in findCars().
float ProcessingThread::calcDist(float x, float y, float x2, float y2)
{
    float dist = sqrt(pow(x - x2, 2) + pow(y - y2, 2));
    return dist;
}

int ProcessingThread::isThisIntInVector(int i, std::vector<int> vector)
{
    int remove = 0;
    for (int j = 0; j < vector.size(); j++)
    {
        if (vector[j] == i) { remove++; }
    }
    return remove;
}
