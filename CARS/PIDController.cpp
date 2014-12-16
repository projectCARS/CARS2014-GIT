#include "headers.h"
#include "definitions.h"
#include "PIDController.h"


PIDController::PIDController(int ID)
{ 
    m_ID = ID;
    m_startInd = 0;

    m_prevI = 0;
    m_prevD = 0;
    m_prevAngError = 0;

    switch (m_ID) {
        case 0:
            m_turnPID.resize(3);
#ifdef safeMode
            #define minSpeed 0.5f
            #define maxSpeed 1.6f

            m_turnPID[0] = 1.5f;// 1.5f;    //P
            m_turnPID[1] = 0.3f;            //I
            m_turnPID[2] = 0.0f;            //D

            Kp_forward = 0.26f;
            Kp_brake = 0.8f;
            Ki = 0.05f;
            Kd = 0.001f;
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
#define minSpeed 0.5f
#define maxSpeed 1.6f
            m_turnPID.resize(3);
            m_turnPID[0] = 1.3f;		//P
            m_turnPID[1] = 0.3;		//I
            m_turnPID[2] = 0;		//D

            Kp_forward = 0.16f;
            Kp_brake = 0.8f;
            Ki = 0.01f;
            Kd = 0.008f;
            break;
        case 2:
            m_turnPID.resize(3);
            m_turnPID[0] = 1.3f;		//P
            m_turnPID[1] = 0.1;		//I
            m_turnPID[2] = 0;		//D

            Kp_forward = 0.16f;
            Kp_brake = 0.8f;
            Ki = 0.01f;
            Kd = 0.008f;
            break;
        default:
            m_turnGain = 1;
            break;
    }
}

PIDController::~PIDController()
{
}

void PIDController::calcTurnSignal(std::vector<float> &state, float &turn)
{
    m_refInd = findIntersection(state, m_startInd);
    turn = calcTurnSignal(state, m_refInd);
}

void PIDController::calcSignals(std::vector<float> &state, float &gas, float &turn)
{
    // Find point on reference curve.
    m_refInd = findIntersection(state, m_startInd);
    m_refSpeed = calcRefSpeed(state, m_refInd);

    if(m_refSpeed > state[2] + 0.5)
    {
        m_refSpeed = state[2] + 0.5;
    }
    // Calculate gas and turn signal.
    //gas = calcGasSignal(state, m_refGas);
    gas = calcGasSignalAlt(state, m_refSpeed);
    //gas = m_refGas;
    turn = calcTurnSignal(state, m_refInd);
}

float PIDController::calcGasSignalAlt(std::vector<float> &state, float refSpeed){

    float error = (refSpeed - state[2]);

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

float PIDController::findSpeed(std::vector<float> &state)
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
int PIDController::findIntersection(std::vector<float> &state, int startInd)
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
        //std::cout << "Warning, no intersection point found, in PIDController.cpp" << std::endl;
	}
    else
    {
        // Index to start searching for the intersection point at the next iteration.
        m_startInd = ind1;
    }

    // Return index of point on reference curve (ind1 within the circle, ind2 outside the circle).
	return ind1;
}

// Calculate Turning voltage.
float PIDController::calcTurnSignal(std::vector<float> &state, int refInd)
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
    //m_prevI = 0;

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
        turnSignal = P + I;
		if (turnSignal > 1)
		{
			turnSignal = 1;
		}
		else if (turnSignal < -1)
		{
			turnSignal = -1;
        }
        m_prevIturn = I;
        //qDebug() << "I: " << I << "P: " << P;
	}
	// Return Turning voltage.
	return turnSignal;
}

// Calculate ref speed.
float PIDController::calcRefSpeed(std::vector<float> &state, int refInd)
{
    float carX, carY, diffX, diffY, refAngle, carAngle, diffAngle;
    float refSpeed = 0;

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
        refSpeed = minSpeed;
    }
    else if (diffAngle >= M_PI_2)
    {
        refSpeed = minSpeed;
    }
    else
    {
        //Convert diffAngle --> refSpeed
        refSpeed = minSpeed + (1 - abs(diffAngle) /M_PI_2) * (maxSpeed - minSpeed);

        if (refSpeed > maxSpeed)
        {
            refSpeed = maxSpeed;
        }
        else if (refSpeed < minSpeed)
        {
            refSpeed = minSpeed;
        }
    }

    // Return refSpeed.
    return refSpeed;
}
