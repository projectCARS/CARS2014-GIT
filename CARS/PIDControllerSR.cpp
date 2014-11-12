#include "headers.h"
#include "definitions.h"
#include "PIDControllerSR.h"
//#include <QDebug>


PIDControllerSR::PIDControllerSR(int ID)
{
    m_ID = ID;
    m_startInd = 0;

    m_prevI = 0;
    m_prevD = 0;
    m_prevAngError = 0;

    switch (m_ID) {
        case 0:
            m_turnPID.resize(3);
            m_speedPID.resize(3);
#ifdef safeMode
            #define minSpeed 0.5
            #define maxSpeed 1.6

            m_turnPID[0] = 1.5;		//P
            m_turnPID[1] = 0.0;		//I
            m_turnPID[2] = 0.0;		//D

            m_speedPID[0] = 0.8;	//P     0.2;
            m_speedPID[1] = 0.01;	//I
            m_speedPID[2] = 0.008;	//D
#endif

        #ifdef fullForce
            #define minSpeed 0.8
            #define maxSpeed 1.8

            m_turnPID[0] = 1.3;		//P
            m_turnPID[1] = 0.;		//I
            m_turnPID[2] = 0.;		//D

            m_speedPID[0] = 0.15;	//P     0.2;
            m_speedPID[1] = 0.05;	//I
            m_speedPID[2] = 0.001;	//D
        #endif


            break;
        case 1:
            m_turnPID.resize(3);
            m_turnPID[0] = 1.3;		//P
            m_turnPID[1] = 0;		//I
            m_turnPID[2] = 0;		//D

            m_speedPID.resize(3);
            m_speedPID[0] = 0.6;	//P     0.2;
            m_speedPID[1] = 0.001;	//I
            m_speedPID[2] = 0.001;	//D
            break;
        case 2:
            m_turnPID.resize(3);
            m_turnPID[0] = 1.3;		//P
            m_turnPID[1] = 0;		//I
            m_turnPID[2] = 0;		//D

            m_speedPID.resize(3);
            m_speedPID[0] = 0.6;	//P     0.2;
            m_speedPID[1] = 0.001;	//I
            m_speedPID[2] = 0.001;	//D
            break;
        default:
            m_turnGain = 1;
            break;
    }
}

PIDControllerSR::~PIDControllerSR()
{
}

void PIDControllerSR::calcTurnSignal(std::vector<float> &state, float &turn)
{
    m_refIndCircle = findIntersection(state, m_startInd);
    turn = calcTurnSignal(state, m_refIndCircle);
}

void PIDControllerSR::calcSignals(std::vector<float> &state, float &gas, float &turn)
{
    // Find point on reference curve.
    m_refIndCircle = findIntersection(state, m_startInd);
    m_refIndClosest = findClosestReferencePoint(state);
    m_refSpeed = calcRefSpeed(state, m_refIndClosest);

    if(m_refSpeed > state[2] + 0.5)
    {
        m_refSpeed = state[2] + 0.5;
    }
    // Calculate gas and turn signal.
    //gas = calcGasSignal(state, m_refGas);
    gas = calcGasSignalAlt(state, m_refSpeed);
    //gas = m_refGas;
    turn = calcTurnSignal(state, m_refIndCircle);
}

float PIDControllerSR::calcGasSignalAlt(std::vector<float> &state, float refSpeed){

    float error = (refSpeed - state[2]);

    start = std::chrono::system_clock::now();
    std::chrono::duration<double> T = start - end;
    end = std::chrono::system_clock::now();
    float dt = (float)T.count();
    if (dt < 0.0001 || dt > 0.1) {
        dt = 0.007;
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

float PIDControllerSR::findSpeed(std::vector<float> &state)
{
    uchar val = speedProfile.at<uchar>(state[1]*PIXELS_PER_METER, state[0]*PIXELS_PER_METER);
    float speed = (float)val;

    if (speed < 0)
        speed = ((speed / 127.5) - 1)*.9;
    else
        speed = ((speed / 127.5) - 1)*.6;

    std::cout << "Speed: " << speed << '\xd';
    return speed;
}

// Finds the intersection between the reference curve
// and the circle with gCarRadius.
int PIDControllerSR::findIntersection(std::vector<float> &state, int startInd)
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
        //std::cout << "Warning, no intersection point found, in PIDControllerSR.cpp" << std::endl;
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
int PIDControllerSR::findClosestReferencePoint(std::vector<float> &state)
{
    int index, i;
    float dist1, dist2, diffX, diffY, carX, carY;
    // x and y coordinates of the car.
    carX = state[0];
    carY = state[1];

    dist1 = 1000;  // A big number

    //sweep all points in refernce path and find shortest distance
    for (i = 0; i < gRefLen; i++)
    {
        diffX = gRef[2 * i] - carX;
        diffY = gRef[2 * i + 1] - carY;
        dist2 = diffX*diffX + diffY*diffY;
        if (dist2<dist1)    //if new distance is shorter save distance and corresponding index.
        {
            dist1 = dist2;
            index = i;
        }
    }
    return index;
}


// Calculate Turning voltage.
float PIDControllerSR::calcTurnSignal(std::vector<float> &state, int refInd)
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
        diffAngle -= 2.0 * M_PI;
    }
    else if (diffAngle < -M_PI)
    {
        diffAngle += 2.0 * M_PI;
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

// Calculate ref speed. Or more like get Ref speed from vector vRef...
float PIDControllerSR::calcRefSpeed(std::vector<float> &state, int refInd)
{
    float refSpeed = 0;
    refSpeed = vRef[refInd];

    // Return refSpeed.
    return refSpeed;
}
