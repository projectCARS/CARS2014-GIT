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
    m_onPath = false;   //dont update speed reference until car is near the reference path
    m_vRef = vRef;      //m_vRef is uniqe to each car
    m_gain = .8f;
    m_offset = 0.4f;
    m_checkPoint = false;
    m_firstLapDone = false;
    timer.start();
    m_bLap = 1000.0;

    srand(time(NULL));

    m_prevI = 0;
    m_prevD = 0;
    m_prevAngError = 0;


    m_turnPID.resize(3);
    m_speedPID.resize(3);
#ifdef safeMode
#define minSpeed 0.5
#define maxSpeed 1.6

    m_turnPID[0] = 1.5f;	//P
    m_turnPID[1] = 0.0;		//I
    m_turnPID[2] = 0.0;		//D

    m_speedPID[0] = 0.8f;	//P     0.2;
    m_speedPID[1] = 0.01f;	//I
    m_speedPID[2] = 0.008f;	//D
#endif


    // Deciding which log file to write to
    timeSRgain.start();
    int fileNo = 1;
    std::stringstream str;
    for (int i=0 ; i < 100 ; i++) // Ceiling for number of log files here. (i < ceiling)
    {
        str.clear();
        str.str(std::string());
        str << "outdata/logFiles/logSRgain" << std::setw(3) << std::setfill('0') << fileNo << ".txt";
        if (!fileExists(str.str()))
        {
            break;
        }
        fileNo++;
    }
    logFileSRgain.open(str.str());
    logFileSRgain << "sysTime carID xPos yPos speed lateralError refInd vRef[refInd]_before vRef[refInd]_after \n";
    std::cout << "PIDadaptiveGain object:		Writing log to " << str.str() << std::endl;

}

PIDadaptiveGain::~PIDadaptiveGain()
{
}

void PIDadaptiveGain::calcSignals(std::vector<float> &state, float &gas, float &turn)
{
    // Find point on reference curve.
    m_refIndCircle = findIntersection(state, m_startInd);
    findClosestReferencePoint(state);  //updates m_refIndClosest and m_dist_lateral

    //prepare reference speed
    m_refSpeed = calcRefSpeed(state, m_refIndClosest, m_gain, m_offset);

    // Calculate gas and turn signal.
    gas = calcGasSignal(state, m_refSpeed);
    turn = calcTurnSignal(state, m_refIndCircle);

    //update gain
    if (lapDone(state))
    {
        updateSpeedReferenceGain();
    }
    //qDebug() << m_vRef[0] << m_vRef[10] << m_vRef[20] << m_vRef[30] << m_vRef[40] << m_vRef[50];
}


void PIDadaptiveGain::calcTurnSignal(std::vector<float> &state, float &turn)
{
    m_refIndCircle = findIntersection(state, m_startInd);
    turn = calcTurnSignal(state, m_refIndCircle);
}



float PIDadaptiveGain::calcGasSignal(std::vector<float> &state, float refSpeed){

    float error = (refSpeed - state[2]);

    start = std::chrono::system_clock::now();
    std::chrono::duration<double> T = start - end;
    end = std::chrono::system_clock::now();
    float dt = (float)T.count();
    if (dt < 0.0001 || dt > 0.1) {
        dt = 0.007f;
        m_prevI = 0;
    }

    float P = error;
    float I = (m_prevI + error*dt);
    if (I > 8) I = 8;
    float D = 0.5*m_prevD + 0.5*(m_prevAngError - error) / dt; //


    float signal = P * m_speedPID[0] +
            I * m_speedPID[1] +
            D * m_speedPID[2];

    m_prevAngError = error;
    m_prevI = I;
    m_prevD = D;

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
        turnSignal = m_turnPID[0]*diffAngle / M_PI_2;
        if (turnSignal > 1)
        {
            turnSignal = 1;
        }
        else if (turnSignal < -1)
        {
            turnSignal = -1;
        }
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
    if(refSpeed > state[2] + 0.5)
    {
        refSpeed = state[2] + 0.5;
        //qDebug() << "limited";
    }
    // Return refSpeed.
    return refSpeed;
}


// Update speed reference vector


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
            m_gain = m_bGain + 0.05*(rand()-0.5);
        }
        else
        {
            m_gain = m_bGain + 0.05*(rand()-0.5);
        }
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





