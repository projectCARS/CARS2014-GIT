// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



#include <tchar.h>
#include <iostream>
#include <cmath>								// Constant M_PI and functions.
#include <fstream>								// Input and output to file.
#include <time.h>								// clock().
#include <sys/stat.h>	// To check for file existense

#include "FlyCapture2.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include <windows.h>	// for fullscreen windows (no borders) and Sleep()
#include <process.h>
//#include "NIDAQmx.h"
#include<Eigen/Core>
//#include <Eigen/LU>		//for inverse
#include<Eigen/Dense>		//for inverse

#include <iomanip>//For write to indexed images

#include <opencv2/core/eigen.hpp> // To convert eigen matrices to cv::Mat

#include <random> // For monte carlo part in particle filter

#include <limits> // to check for infinites in states in particle filter

//#include <stdlib.h> // for sleep()