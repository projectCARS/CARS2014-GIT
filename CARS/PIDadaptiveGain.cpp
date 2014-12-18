#include "headers.h"
#include "definitions.h"
#include "classes.h"
#include "PIDadaptiveGain.h"
#include "functions.h"

#include <fstream>

PIDadaptiveGain::PIDadaptiveGain(int ID)
{
    m_ID = ID;
    m_startInd = 0;
    m_offset = 0.0f;
    m_onPath = false;   //dont update speed reference until car is near the reference path
    m_vRef = vRef;   //m_vRef is uniqe to each car
    m_aRef = aRef;
    m_gain = 1.0f;

    m_checkPoint = false;
    m_firstLapDone = false;
    timer.start();
    m_bLap = 1000.0;

    srand(time(NULL));

    m_prevI = 0;
    m_prevD = 0;
    m_prevAngError = 0;

    //used for section
    m_numOfInterval = 10.0;
    m_noIncreaseCounter.resize(0);
    m_noIncreaseCounter.resize((int)m_numOfInterval);
    m_noDecreaseCounter.resize(0);
    m_noDecreaseCounter.resize((int)m_numOfInterval);
    m_intervalLength = gRefLen / m_numOfInterval ;
    m_length2mid = (int) ( m_intervalLength / 2 );


    m_intervalStartIndexes.resize(0);
    m_intervalStartIndexes.resize((int)m_numOfInterval);
    m_intervalMidIndexes.resize(0);
    m_intervalMidIndexes.resize((int)m_numOfInterval);
    m_refSpeedShort.resize(0);
    m_refSpeedShort.resize((int)m_numOfInterval);
    m_refSpeedShortBest.resize(0);
    m_refSpeedShortBest.resize((int)m_numOfInterval);
    m_timerTimes.resize(0);
    m_timerTimes.resize((int)m_numOfInterval);
    m_times.resize(0);
    m_times.resize((int)m_numOfInterval);
    m_timesBest.resize(0);
    m_timesBest.resize((int)m_numOfInterval);
    m_sectionVisited.resize(0);
    m_sectionVisited.resize((int)m_numOfInterval);

    for (int i = 0; i<m_numOfInterval ; i++ )
    {
        m_intervalStartIndexes[i] = (int) (i * m_intervalLength);
        m_intervalMidIndexes[i] = m_intervalStartIndexes[i] + m_length2mid;
        m_refSpeedShort[i] = m_vRef[m_intervalMidIndexes[i]];
        m_sectionVisited[i] = false;
        m_noDecreaseCounter[i] = 0;
        m_noIncreaseCounter[i] = 0;
    }
    m_refSpeedShortBest = m_refSpeedShort;

    timerSection.start();
    m_firstLapStarted = false;
    //end of: used for section

    EnterCriticalSection(&csPlotData);
    if (AdaptiveRefPlotData.makePlot)
    {
        AdaptiveRefPlotData.newData2plot1 = true;
        AdaptiveRefPlotData.Xvalues.resize(0);
        AdaptiveRefPlotData.Xvalues.resize((int)m_numOfInterval);
        AdaptiveRefPlotData.Xvalues2.resize(0);
        AdaptiveRefPlotData.Xvalues2.resize(gRefLen);
        AdaptiveRefPlotData.Yvalues1.resize(0);
        AdaptiveRefPlotData.Yvalues1.resize((int)m_numOfInterval);
        AdaptiveRefPlotData.Yvalues2.resize(0);
        AdaptiveRefPlotData.Yvalues2.resize((int)m_numOfInterval);
        AdaptiveRefPlotData.Yvalues3.resize(0);
        AdaptiveRefPlotData.Yvalues3.resize((int)m_numOfInterval);
        AdaptiveRefPlotData.Yvalues4.resize(0);
        AdaptiveRefPlotData.Yvalues4.resize((int)gRefLen);

        AdaptiveRefPlotData.axisRange.resize(4);
        AdaptiveRefPlotData.axisRange[0] = 0;
        AdaptiveRefPlotData.axisRange[1] = gRefLen;
        AdaptiveRefPlotData.axisRange[2] = 0;
        AdaptiveRefPlotData.axisRange[3] = 3.5;

        for (int i = 0; i<m_numOfInterval ; i++ )
        {
            AdaptiveRefPlotData.Xvalues[i] = m_intervalMidIndexes[i];
            AdaptiveRefPlotData.Yvalues1[i] = m_refSpeedShort[i];
        }
        for(int i = 0; i < gRefLen; i++)
        {
            AdaptiveRefPlotData.Xvalues2[i] = i;
        }

    }
    if (AdaptiveTimePlotData.makePlot)
    {
        AdaptiveTimePlotData.Xvalues.resize(0);
        AdaptiveTimePlotData.Xvalues.resize(m_numOfInterval);
        AdaptiveTimePlotData.Xvalues2.resize(0);
        AdaptiveTimePlotData.Xvalues2.resize(gRefLen);
        AdaptiveTimePlotData.Yvalues1.resize(0);
        AdaptiveTimePlotData.Yvalues1.resize(m_numOfInterval);
        AdaptiveTimePlotData.Yvalues2.resize(0);
        AdaptiveTimePlotData.Yvalues2.resize(m_numOfInterval);

        AdaptiveTimePlotData.axisRange.resize(4);
        AdaptiveTimePlotData.axisRange[0] = 0;
        AdaptiveTimePlotData.axisRange[1] = m_numOfInterval;
        AdaptiveTimePlotData.axisRange[2] = 0;
        AdaptiveTimePlotData.axisRange[3] = 1.5;

        for (int i = 0; i<m_numOfInterval ; i++ )
        {
            AdaptiveTimePlotData.Xvalues[i] = i + 1;
        }

    }
    LeaveCriticalSection(&csPlotData);



    m_turnPID.resize(3);
#ifdef safeMode
#define minSpeed 0.5
#define maxSpeed 1.6

    m_turnPID[0] = 1.8f;	//P
    m_turnPID[1] = 0.2f;		//I
    m_turnPID[2] = 0.0;		//D

    Kp_forward = 0.18f;
    Kp_brake = 5.8f;
    Ki = 0.0f;
    Kd = 0.008f;
#endif


    // Deciding which log file to write to
    timeSRgain.start();
    int fileNo = 1;
    std::stringstream str;
    for (int i=0 ; i < 100 ; i++) // Ceiling for number of log files here. (i < ceiling)
    {
        str.clear();
        str.str(std::string());
        str << "outdata/logFiles/logAdaptiveSection" << std::setw(3) << std::setfill('0') << fileNo << ".txt";
        if (!fileExists(str.str()))
        {
            break;
        }
        fileNo++;
    }
    logFileAdaptive.open(str.str());

    logFileAdaptive << "m_numOfInterval gRefLen newline m_intervalMidIndexes  m_refSpeedShort andThen m_refSpeedShortBest  m_times m_timesBest \n";
    logFileAdaptive << m_numOfInterval << " " << gRefLen << " 0" << "\n" ;
    for (int i = 0; i<m_numOfInterval ; i++ )
    {
        logFileAdaptive <<  m_intervalMidIndexes[i] << " " << m_refSpeedShort[i] << " 0" << "\n";
    }

    // logFileSRgain << "sysTime carID xPos yPos speed lateralError refInd vRef[refInd]_before vRef[refInd]_after \n";
    std::cout << "PIDadaptiveGain object:		Writing log to " << str.str() << std::endl;

}

PIDadaptiveGain::~PIDadaptiveGain()
{
}

void PIDadaptiveGain::calcSignals(std::vector<float> &state, float &gas, float &turn)
{
    bool adaptiveGain, section, staticGain;
    adaptiveGain = false;
    section = true;
    staticGain = false;

    // Find point on reference curve.
    m_refIndCircle = findIntersection(state, m_startInd);
    findClosestReferencePoint(state);  //updates m_refIndClosest and m_dist_lateral

    EnterCriticalSection(&csPlotData);
    if (AdaptiveRefPlotData.makePlot)
    {
        AdaptiveRefPlotData.Yvalues4[m_refIndClosest] = state[2];
        AdaptiveRefPlotData.newData2plot4 = true;
    }
    LeaveCriticalSection(&csPlotData);

    //prepare reference speed
    if (adaptiveGain)
        m_refSpeed = calcRefSpeed(state, m_refIndClosest, m_gain, m_offset);
    else if (staticGain)
        m_refSpeed = calcRefSpeed(state, m_refIndClosest, 1.2, 0.3);    //change gain and offset
    else if (section)
        m_refSpeed = calcRefSpeed(state, m_refIndClosest, 1, 0);

    // Calculate gas and turn signal.
    gas = calcGasSignal(state, m_refSpeed);
    turn = calcTurnSignal(state, m_refIndCircle);

    if (adaptiveGain)
    {
        //update gain on speed profile. One gain for the whole lap
        if (lapDone(state))
        {
            updateSpeedReferenceGain();
        }
    }


    if (section)
    {
        //update speed ref value in section. Number of sections given by: m_numOfInterval
        // m_refIndClosest - index to closest ref point
        m_IndexSection = findClosestSection(m_refIndClosest);
        if (newSectionEntered(m_IndexSection))
        {
            updateSectionTimers(m_IndexSection); //if Indexsection == 0, this function calls updateSpeedReference
            //updateSpeedReference(m_IndexSection);
        }
        else if(m_IndexSection == 0 && visitedAllSections())
        {
            newLapStarted();
        }
    }
}

bool PIDadaptiveGain::visitedAllSections()
{
    bool r = true;

    for (int i = 0; i < m_numOfInterval; i++)
    {
        r &= m_sectionVisited[i];
    }
    return r;
}

void PIDadaptiveGain::newLapStarted()
{
    for (int i = 1; i < m_numOfInterval; i++)
    {
        m_sectionVisited[i] = false;
    }

    if ( m_firstLapDone )
    {
            m_timerTimes[m_numOfInterval-1] = timerSection.elapsed()/1000.0;
            timerSection.restart();     //restart timer on every new lap
            updateSpeedReference();

    }
    else if ( m_firstLapStarted )
    {

            m_timerTimes[m_numOfInterval-1] = timerSection.elapsed()/1000.0;
            timerSection.restart();     //restart timer on every new lap
            m_firstLapDone = true;
            qDebug() << "m_firstLapDone = true;";

            //save first laps times as best times
            m_timesBest[0] = m_timerTimes[0];
            for (int i = 1; i<m_numOfInterval; i++)
            {
                m_timesBest[i] = m_timerTimes[i] - m_timerTimes[i-1];
            }
            updateSpeedReference();
    }
    else
    {
        timerSection.restart();
        m_firstLapStarted = true;
    }

}

int PIDadaptiveGain::findClosestSection(int index)
{
    for (int i = 0; i < m_numOfInterval - 1; i++)
    {
        if ((index >= m_intervalStartIndexes[i]) && (index < m_intervalMidIndexes[i+1]))
            return i;
    }
    if (index>=m_intervalMidIndexes[m_numOfInterval-1])
        return m_numOfInterval-1;
    return -1;
}

bool PIDadaptiveGain::newSectionEntered(int IndexCurrent)
{
    if (!m_sectionVisited[IndexCurrent])
    {
        m_sectionVisited[IndexCurrent] = true;
        return true;
    }
    else
        return false;
}

void PIDadaptiveGain::updateSectionTimers(int IndexSection)
{
    if (m_firstLapStarted)
        m_timerTimes[IndexSection-1] = timerSection.elapsed()/1000.0;

}

void PIDadaptiveGain::updateSpeedReference()
{
    //qDebug() << "updateSpeedReference";
    // make m_times from m_timerTimes
    m_times[0] = m_timerTimes[0];
    for (int i = 1; i < m_numOfInterval; i++)
    {
        m_times[i] = m_timerTimes[i] - m_timerTimes[i-1];
        m_sectionVisited[i] = false;
    }

    // update m_timesBest and m_refSpeedShortBest;
    for (int i = 0; i < m_numOfInterval; i++)
    {
        if (( m_times[i] > 0) && (m_times[i] < m_timesBest[i] )) //time för section must be postive. Else something has gone wrong...
        {
            m_timesBest[i] = m_times[i];
            m_refSpeedShortBest[i] = m_refSpeedShort[i];
            m_noIncreaseCounter[i] = 0;
            m_noDecreaseCounter[i] = 0;
        }
        float K = 0.17;
        float K2 = 0.1;

        // make new m_refSpeedShort
        if (rand()/(float)RAND_MAX < .7)
        {

            newIncrease = K2/(0.15*m_noIncreaseCounter[i]+1) * (rand()/((float)RAND_MAX) + .3);
            m_noIncreaseCounter[i]++;
        }
        else
        {
            newIncrease = -K2/(0.15*m_noIncreaseCounter[i]+1) * (rand()/((float)RAND_MAX) + .3);
            m_noDecreaseCounter[i]++;
        }
        // Update current section with current and previous section data
        m_refSpeedShort[i] = m_refSpeedShortBest[i] + newIncrease;
        m_refSpeedShort[i] += K * oldIncrease;

        // Update previous section with new data
        if(i == 0)
        {
            m_refSpeedShort[m_numOfInterval - 1] = m_refSpeedShort[m_numOfInterval - 1] + K * newIncrease;
            if (m_refSpeedShort[m_numOfInterval - 1] < 0.3)
                m_refSpeedShort[m_numOfInterval - 1] = 0.3f;
        }
        else
        {
            m_refSpeedShort[i - 1] = m_refSpeedShort[i - 1] + K * newIncrease;
            if (m_refSpeedShort[i - 1] < 0.3)
                m_refSpeedShort[i - 1] = 0.3f;
        }
        oldIncrease = newIncrease;

        if (m_refSpeedShort[i] < 0.3)   //dont allow negativ and close to zero speed...
            m_refSpeedShort[i] = 0.3f;
    }

    // update plotdata with current refSpeedShort
    EnterCriticalSection(&csPlotData);

    if (AdaptiveRefPlotData.makePlot)
    {
        AdaptiveRefPlotData.newData2plot2 = true;
        AdaptiveRefPlotData.newData2plot3 = true;
        AdaptiveRefPlotData.newData2plot4 = true;
        for (int i = 0; i < m_numOfInterval ; i++ )
        {
            AdaptiveRefPlotData.Yvalues2[i] = m_refSpeedShort[i];
            AdaptiveRefPlotData.Yvalues3[i] = m_refSpeedShortBest[i];
        }
        for (int i = 0; i < gRefLen; i ++)
        {
           // AdaptiveRefPlotData.Yvalues4[i] = 0;
        }
    }
    if (AdaptiveTimePlotData.makePlot)
    {
        AdaptiveTimePlotData.newData2plot1 = true;
        AdaptiveTimePlotData.newData2plot2 = true;
        for (int i = 0; i < m_numOfInterval ; i++ )
        {
            AdaptiveTimePlotData.Yvalues1[i] = m_timesBest[i];
            AdaptiveTimePlotData.Yvalues2[i] = m_times[i];
        }
    }
    LeaveCriticalSection(&csPlotData);


    // construct m_vRef from m_refSpeedShort with cubic spline - not a knot BC
    double *x,*f,*b,*c,*d;
    int n;
    n = m_numOfInterval+2;
    x = new double [n];
    f = new double [n];
    b = new double [n];
    c = new double [n];
    d = new double [n];

    x[0] =(int) m_intervalMidIndexes[i] - m_intervalLength;
    for (int i = 0; i<m_numOfInterval;i++)
    {
        x[i] = (double) m_intervalMidIndexes[i];
        f[i] = m_refSpeedShort[i];
    }

    cubic_nak(n, x, f, b, c, d);
    for (int i = 0; i<gRefLen; i++)
    {
        m_vRef[i] = spline_eval(n, x, f, b, c, d, (double)i);
        qDebug() << m_vRef[i];
    }
    for (int i = 0; i<m_numOfInterval; i++)
    {
        logFileAdaptive << m_refSpeedShortBest[i] << " " << m_times[i] << " " << m_timesBest[i] << "\n";
    }
}


void PIDadaptiveGain::calcTurnSignal(std::vector<float> &state, float &turn)
{
    m_refIndCircle = findIntersection(state, m_startInd);
    turn = calcTurnSignal(state, m_refIndCircle);
}



float PIDadaptiveGain::calcGasSignal(std::vector<float> &state, float refSpeed){

    float error = (refSpeed - state[2]);

    //qDebug() << "refSpeed: " << refSpeed;

    start = std::chrono::system_clock::now();
    std::chrono::duration<double> T = start - end;
    end = std::chrono::system_clock::now();
    float dt = (float)T.count();
    if (dt < 0.0001 || dt > 0.1) {
        dt = 0.007f;
        m_prevI = 0;
    }
    float KP;
    if(error > 0)
        KP = Kp_forward;
    else
        KP = Kp_brake;

    float Ierror = (m_prevI + error*dt);
    if (Ierror > 8) Ierror = 8;
    float Derror = 0.5*m_prevD + 0.5*(m_prevAngError - error) / dt; //

    float signal = KP * error +
                   Ki * Ierror +
                   Kd * Derror;

    m_prevAngError = error;
    m_prevI = Ierror;
    m_prevD = Derror;

    if (signal > 1)
        signal = 1;
    else if (signal < -1)
        signal = -1;
    return signal;
}


// Finds the intersection between the reference curve
// and the circle with gCarRadius.
int PIDadaptiveGain::findIntersection(std::vector<float> &state, int startInd)
{
    int i, ind1, ind2;
    float dist1, dist2, diffX, diffY, carX, carY;

    // x and y coordinate of the car.
    carX = state[0];
    carY = state[1];

    // Start searching in gRef on startInd minus 5 indicies.
    ind1 = (gRefLen + startInd-5) % gRefLen;
    // Distance between the car and a point with index ind1 on the reference curve, in x and y direction.
    diffX = gRef[2 * ind1] - carX;
    diffY = gRef[2 * ind1 + 1] - carY;
    // Squared distance between the point on the reference curve and the car.
    dist1 = diffX*diffX + diffY*diffY;
    for (i = 0; i < gRefLen; i++)
    {
        // Next point on the reference curve.
        ind2 = (ind1 + 1) % gRefLen;
        // Distance between the car and a point with index ind2 on the reference curve, in x and y direction.
        diffX = gRef[2 * ind2] - carX;
        diffY = gRef[2 * ind2 + 1] - carY;
        // Squared distance between the point on reference curve and the car.
        dist2 = diffX*diffX + diffY*diffY;
        // Check if point with index ind1 is at the intersection between
        // the circle and the reference curve.
        if (dist1 <= gCarRadius && dist2 > gCarRadius)
        {
            // Break the loop if intersection is found.
            break;
        }

        // Save computation by assigning dist2 to dist1.
        dist1 = dist2;
        // Increment ind1 periodically.
        ind1 = (ind1 + 1) % gRefLen;
    }

    // Check if no point was found.
    if (i == gRefLen)
    {
        // At the next iteration, start searching in the same place on the reference curve.
        ind1 = m_startInd;
        //std::cout << "Warning, no intersection point found, in PIDadaptiveGain.cpp" << std::endl;
    }
    else
    {
        // Index to start searching for the intersection point at the next iteration.
        m_startInd = ind1;
    }

    // Return index of point on reference curve (ind1 within the circle, ind2 outside the circle).
    return ind1;
}


/*Find the the point on the reference path closest to the car
 */
void PIDadaptiveGain::findClosestReferencePoint(std::vector<float> &state)
{
    int i, index1, index2;
    float dist1, dist2, dist3, diffX, diffY, carX, carY;
    float x1,y1,x2,y2,t,Qx,Qy,d2;
    // x and y coordinates of the car.
    carX = state[0];
    carY = state[1];

    dist1 = 1000;  // A big number

    //sweep all points on refernce path, find shortest distance and correspoding index
    for (i = 0; i < gRefLen; i++)
    {
        diffX = gRef[2 * i] - carX;
        diffY = gRef[2 * i + 1] - carY;
        dist2 = diffX*diffX + diffY*diffY;
        if (dist2<dist1)    //if new distance is shorter save distance and corresponding index.
        {
            dist1 = dist2;
            index1 = i;
        }
    } //index1 is now refering to the closest reference point

    diffX = gRef[2 * index1 + 2] - carX;
    diffY = gRef[2 * index1 + 3] - carY;
    dist2 = diffX*diffX + diffY*diffY;      //distance to next reference point
    diffX = gRef[2 * index1 - 2] - carX;
    diffY = gRef[2 * index1 - 1] - carY;
    dist3 = diffX*diffX + diffY*diffY;      //distance to previus reference point

    if (dist2>dist3)                        //find second closest reference point
        index2 = index1 + 1;
    else
        index2 = index1 - 1;
    //index2 is second closest reference point.

    //find distance to projection on reference curve. Approximating curve to be a line(L) pasing through closest and second closest reference points
    x1 = gRef[2 * index1];        //coordinates for point 1 on line L
    y1 = gRef[2 * index1 + 1];
    x2 = gRef[2 * index2];        //coordinates for point 2 on line L
    y2 = gRef[2 * index2 + 1];
    // L : {x = x1 + t(x2-x1), y = y1 + t(y2-y1).  Equation for line, with parameter t all points on the line could be reached
    // with t = ... we find the point Q on L which is closest to the car
    t = (-(x1-carX)*(x2-x1) - (y1-carY)*(y2-y1))/((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    //squered distance from car to Q
    Qx = x1 + t*(x2-x1);
    Qy = y1 + t*(y2-y1);
    d2 = sqrt((carX-Qx)*(carX-Qx)+(carY-Qy)*(carY-Qy));

    //update varibles with new information
    m_refIndClosest = index1;
    m_dist_lateral = d2;
}


// Calculate Turning voltage.
float PIDadaptiveGain::calcTurnSignal(std::vector<float> &state, int refInd)
{
    float carX, carY, diffX, diffY, refAngle, carAngle, diffAngle;
    float turnSignal = 0;

    start = std::chrono::system_clock::now();
    std::chrono::duration<double> T = start - end;
    end = std::chrono::system_clock::now();
    float dt = (float)T.count();
    if (dt < 0.0001 || dt > 0.1) {
        dt = 0.007f;
    }

    // x and y coordinate of the car.
    carX = state[0];
    carY = state[1];
    // Angle of the car in radians.
    carAngle = state[3];
    // Distance from car to reference point along x- and y-axis.
    diffX = gRef[2 * refInd] - carX;
    diffY = gRef[2 * refInd + 1] - carY;
    // Angle to reference point.
    refAngle = std::atan2(diffY, diffX);
    // Difference between angles.
    diffAngle = refAngle - carAngle;
    // Handel discontinuity of arctan.
    if (diffAngle > M_PI)
    {
        diffAngle -= 2.0f * M_PI;
    }
    else if (diffAngle < -M_PI)
    {
        diffAngle += 2.0f * M_PI;
    }
    /* Calculate input voltage. Note that -M.PI <= diffAngle <= M_PI.
    It is assumed here that when diffAngle > 0 the controller
    should turn right, and when diffAngle < 0 the controller
    should turn left.*/

    if (diffAngle <= -M_PI_2)
    {
        turnSignal = -1.0;
    }
    else if (diffAngle >= M_PI_2)
    {
        turnSignal = 1.0;
    }
    else
    {
        float KturnVel = 0.1;
        float I = m_turnPID[1]*(m_prevIturn + diffAngle*dt);
        float P = m_turnPID[0]*diffAngle / M_PI_2 - KturnVel*state[4];
        turnSignal = I + P;
        if (turnSignal > 1)
        {
            turnSignal = 1;
        }
        else if (turnSignal < -1)
        {
            turnSignal = -1;
        }
        m_prevIturn = I;
        //qDebug() << "P: " << P << "I: " << I;
    }

    // Return Turning voltage.
    return turnSignal;
}

// Calculate/Get reference speed
float PIDadaptiveGain::calcRefSpeed(std::vector<float> &state, int refInd, double gain, double offset)
{
    float refSpeed;
    refSpeed = m_vRef[refInd]*gain + offset;
    //qDebug() << refSpeed;
    //refSpeed = m_vRef[refInd];
    /*if(refSpeed > state[2] + 0.5)
    {
        refSpeed = state[2] + 0.5;
        //qDebug() << "limited";
    }*/
    // Return refSpeed.
    return refSpeed;
}


// Update speed reference gain
void PIDadaptiveGain::updateSpeedReferenceGain()
{
    if (m_firstLapDone)
    {
        m_lLap = timer.elapsed()/1000.0;
        timer.restart();
        qDebug() <<"last: " << m_lLap << "best: " << m_bLap;

        if (m_lLap<m_bLap)
        {
            m_bLap = m_lLap;
            m_bGain = m_gain;
        }
        m_gain = m_bGain + 0.15*(rand()/((float)RAND_MAX)- 0.3);
        qDebug() << "bestGain: " << m_bGain << "new gain: " << m_gain;

    }
    else
    {
        timer.restart();
        m_firstLapDone = true;
    }
    // qDebug() << "Gain: " <<m_gain;

    /*
    double currTime = (double)timeSRgain.elapsed()/1000.0;
    //"sysTime carID xPos yPos speed lateralError || refInd vRef[refInd]_before vRef[refInd]_after \n";
    (logFileSRgain) << currTime << " " << m_ID << " " << state[0] << " " << state[1] << " " << state[2] << " " << lateralError << " ";
    (logFileSRgain) << refInd << " " << old_vRef << " " << m_vRef[refInd] << " ";
    (logFileSRgain) << "\n";
    */
}

bool PIDadaptiveGain::lapDone(std::vector<float> &state)
{
    float carX, carY;
    bool result = false;
    carX = state[0];
    carY = state[1];

    if (!m_checkPoint && carX>2)
    {
        qDebug() << "set check true";
        m_checkPoint = true;
    }
    else if (m_checkPoint && carY<.5 && carX>1.48 && carX<1.52)
    {
        m_checkPoint = false;
        result = true;
        qDebug() << "lapDone = true";
    }
    return result;
}





