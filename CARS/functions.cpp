// Function declarations.
#include "headers.h"
#include "classes.h"
#include "definitions.h"
#include "functions.h"
#include <Eigen/Dense>


/*#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/features2d.hpp"*/

// Returns true if the race is over
bool updateRaceData(CarData &cardata, RaceSettings &raceSettings)
{
    if (cardata.lapData.lapNumber >= raceSettings.numberOfLaps){
        raceSettings.winnerID = cardata.id;
        raceSettings.raceDone = true;
        return true;
    }
    return false;
}

// Updates the lapdata struct in the car
void updateLapData(CarData &cardata)
{
    float carX, carY, dist;
    carX = cardata.state[0];
    carY = cardata.state[1];

    //Dont do anything until car has passed start line for the first time.
    if (!cardata.lapData.firstLapStarted)
    {
        if (carY<.5 && carX > 1.3 && carX < 1.7) //only proced if car is near the finish line
        {
            //coordinates for point(Q) on finish line closest to car. Qx=1.5 and Qy=carY
            float Qx = 1.5;
            dist = abs(carX-Qx);
            if (dist<0.02)
            {
                cardata.lapData.lapTimer.restart();
                cardata.lapData.firstLapStarted = true;
                cardata.lapData.bestTime = 1000;
                //qDebug() << "start first lap";
            }
        }

    }
    else
    {
        cardata.lapData.lapTime = cardata.lapData.lapTimer.elapsed()/1000.0;
        if (carY<.5 && carX > 1.3 && carX < 1.7) //is car near finishing line?
        {
            //find distance to finnishing line
            float Qx = 1.5;
            dist = abs(carX-Qx);
            //is dist very small(and timer big) save lap time and reset timer
            if (dist < 0.02 && cardata.lapData.lapTime > 3)
            {
                cardata.lapData.lapNumber++;
                cardata.lapData.firstLapDone = true;
                cardata.lapData.lastLapTime = cardata.lapData.lapTime;
                cardata.lapData.lapTimer.restart();
                //qDebug() << "lapTime: " << cardata.lapData.lapTime;
                if (cardata.lapData.lapTime < cardata.lapData.bestTime)
                {
                    cardata.lapData.bestTime = cardata.lapData.lapTime;
                    //std::cout << "bestTime: " << cardata.lapData.bestTime << std::endl;
                }
            }
            //update LapData with information


        }
    }
}

// Used in class Calibrator
bool fileExists(const std::string& file) {
    struct stat buf;
    return (stat(file.c_str(), &buf) == 0);
}

void logData(float sysTime, std::vector<CarData> carData, std::vector<CarMeasurement> carMeasurements, std::vector<Signal> signal, std::ofstream *logFile)
{
    int precision = 3; //Floating point precision in logging

    for (int i = 0; i < carData.size(); i++)
    {
        if (i == 0) // For now, log only car '0'
        {
            (*logFile) << std::setprecision(5) << sysTime << " " << carData[i].id << " ";

            for (int j = 0; j < carData[i].state.size(); j++)
            {
                (*logFile) << std::setprecision(precision) << carData[i].state[j] << " ";
            }

            for (int j = 0; j < carMeasurements.size(); j++)
            {
                if (carMeasurements[j].id == 0) // For now, log only car '0'
                {
                    (*logFile) << std::setprecision(precision) << carMeasurements[j].x << " " << carMeasurements[j].y << " " << carMeasurements[j].theta << " ";
                }
            }
            if (carMeasurements.size() == 0)
            {
                (*logFile) << "NaN NaN NaN ";
            }

            (*logFile) << std::setprecision(precision) << signal[i].gas << " " << signal[i].turn << " ";

            (*logFile) << "\n";
        }
        //std::cout << "[" << carData[i].state.size() << ", " << carMeasurements.size() << ", " << signals.size() << "]" << std::endl;
    }
}


void logDataU(float sysTime, std::vector<CarData> carData, float64 gas, float64 turn, std::ofstream *logFile)
{
    (*logFile) << sysTime << " " << gas;
    (*logFile) << "\n";

}

void drawStatesToImg(Eigen::MatrixXf carPattern, float *posX, float *posY, float *yaw, cv::Mat img, int selection)
{
    int points = carPattern.rows();
    Eigen::Matrix2f rotate, translate;
    Eigen::MatrixXf pos;
    Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(2, points);
    switch (selection)
    {
    case 1:
    {
        for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
        {
            rotate << cos(yaw[i]), -sin(yaw[i]),
                    sin(yaw[i]), cos(yaw[i]);
            translate << posX[i], 0,
                    posY[i], 0;
            pos = rotate*carPattern.transpose() + translate*ones;

            for (int p = 0; p < points; p++)
            {
                if (pos(0, p) > 0 && pos(0, p) < img.cols && pos(1, p) > 0 && pos(1, p) < img.rows)
                {
                    cv::circle(img, cv::Point(pos(0, p), pos(1, p)), 1.5, cv::Scalar(255, 150, 150), 1, 8, 0);
                }
            }
        }
    }
        break;
    case 2:
    {
        for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
        {
            rotate << cos(yaw[i]), -sin(yaw[i]),
                    sin(yaw[i]), cos(yaw[i]);
            translate << posX[i], 0,
                    posY[i], 0;
            pos = rotate*carPattern.transpose() + translate*ones;

            for (int p = 0; p < points; p++)
            {
                if (pos(0, p) > 0 && pos(0, p) < img.cols && pos(1, p) > 0 && pos(1, p) < img.rows)
                {
                    cv::circle(img, cv::Point(pos(0, p), pos(1, p)), 4, cv::Scalar(0, 0, 255), -1, 8, 0);
                }
            }
        }
    }
        break;
    }
}

void drawCar(Eigen::MatrixXf carPattern, float x, float y, float yaw, cv::Mat img)
{
    int points = carPattern.rows();
    Eigen::Matrix2f rotate, translate;
    Eigen::MatrixXf pos;
    Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(2, points);


    rotate << cos(yaw), -sin(yaw),
            sin(yaw), cos(yaw);
    translate << x, 0,
            y, 0;
    pos = rotate*carPattern.transpose() + translate*ones;

    for (int p = 0; p < points; p++)
    {
        if (pos(0, p) > 0 && pos(0, p) < img.cols && pos(1, p) > 0 && pos(1, p) < img.rows)
        {
            cv::circle(img, cv::Point(pos(0, p), pos(1, p)), 4, cv::Scalar(255, 0, 0), -1, 8, 0);
        }
    }



}

void cubic_nak ( int n, double *x, double *f, double *b, double *c, double *d )

/*
     PURPOSE:
          determine the coefficients for the 'not-a-knot'
          cubic spline for a given set of data


     CALLING SEQUENCE:
          cubic_nak ( n, x, f, b, c, d );


     INPUTS:
          n		number of interpolating points
          x		array containing interpolating points
          f		array containing function values to
            be interpolated;  f[i] is the function
            value corresponding to x[i]
          b		array of size at least n; contents will
            be overwritten
          c		array of size at least n; contents will
            be overwritten
          d		array of size at least n; contents will
            be overwritten


     OUTPUTS:
          b		coefficients of linear terms in cubic
            spline
      c		coefficients of quadratic terms in
            cubic spline
      d		coefficients of cubic terms in cubic
            spline

     REMARK:
          remember that the constant terms in the cubic spline
          are given by the function values being interpolated;
          i.e., the contents of the f array are the constant
          terms

          to evaluate the cubic spline, use the routine
          'spline_eval'
*/

{
     double *h,
            *dl,
            *dd,
            *du;
     int i;

     h  = new double [n];
     dl = new double [n];
     dd = new double [n];
     du = new double [n];

     for ( i = 0; i < n-1; i++ )
         h[i] = x[i+1] - x[i];
     for ( i = 0; i < n-3; i++ )
         dl[i] = du[i] = h[i+1];

     for ( i = 0; i < n-2; i++ ) {
         dd[i] = 2.0 * ( h[i] + h[i+1] );
         c[i]  = ( 3.0 / h[i+1] ) * ( f[i+2] - f[i+1] ) -
                 ( 3.0 / h[i] ) * ( f[i+1] - f[i] );
     }
     dd[0] += ( h[0] + h[0]*h[0] / h[1] );
     dd[n-3] += ( h[n-2] + h[n-2]*h[n-2] / h[n-3] );
     du[0] -= ( h[0]*h[0] / h[1] );
     dl[n-4] -= ( h[n-2]*h[n-2] / h[n-3] );

     tridiagonal ( n-2, dl, dd, du, c );

     for ( i = n-3; i >= 0; i-- )
         c[i+1] = c[i];
     c[0] = ( 1.0 + h[0] / h[1] ) * c[1] - h[0] / h[1] * c[2];
     c[n-1] = ( 1.0 + h[n-2] / h[n-3] ) * c[n-2] - h[n-2] / h[n-3] * c[n-3];
     for ( i = 0; i < n-1; i++ ) {
         d[i] = ( c[i+1] - c[i] ) / ( 3.0 * h[i] );
         b[i] = ( f[i+1] - f[i] ) / h[i] - h[i] * ( c[i+1] + 2.0*c[i] ) / 3.0;
     }

     delete [] h;
     delete [] du;
     delete [] dd;
     delete [] dl;
}


double spline_eval ( int n, double *x, double *f, double *b, double *c,
                     double *d, double t )

/*
     PURPOSE:
          evaluate a cubic spline at a single value of
          the independent variable given the coefficients of
          the cubic spline interpolant (obtained from
          'cubic_nak' or 'cubic_clamped')


     CALLING SEQUENCE:
          y = spline_eval ( n, x, f, b, c, d, t );
          spline_eval ( n, x, f, b, c, d, t );


     INPUTS:
          n		number of interpolating points
          x		array containing interpolating points
          f		array containing the constant terms from
            the cubic spline (obtained from 'cubic_nak'
            or 'cubic_clamped'). NOTE: this is the same array
            used in cubic_nak or cubic_clamped containing
            function values to be interpolated
          b		array containing the coefficients of the
            linear terms from the cubic spline
            (obtained from 'cubic_nak' or 'cubic_clamped')
          c		array containing the coefficients of the
            quadratic terms from the cubic spline
            (obtained from 'cubic_nak' or 'cubic_clamped')
          d		array containing the coefficients of the
            cubic terms from the cubic spline
            (obtained from 'cubic_nak' or 'cubic_clamped')
          t		value of independent variable at which
            the interpolating polynomial is to be
            evaluated


     OUTPUTS:
          y		value of cubic spline at the specified
            value of the independent variable
*/

{
     int i,
         found;

     i = 1;
     found = 0;
     while ( !found && ( i < n-1 ) ) {
           if ( t < x[i] )
              found = 1;
           else
              i++;
     }
     t = f[i-1] + ( t - x[i-1] ) * ( b[i-1] + ( t - x[i-1] ) * ( c[i-1] +
                  ( t - x[i-1] ) * d[i-1] ) );
     return ( t );
}

void tridiagonal ( int n, double *c, double *a, double *b, double *r )

{
     int i;

     for ( i = 0; i < n-1; i++ ) {
         b[i] /= a[i];
         a[i+1] -= c[i]*b[i];
     }

     r[0] /= a[0];
     for ( i = 1; i < n; i++ )
         r[i] = ( r[i] - c[i-1] * r[i-1] ) / a[i];

     for ( i = n-2; i >= 0; i-- )
         r[i] -= r[i+1] * b[i];
}
