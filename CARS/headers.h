// headers.h : include file for standard system include files,
// or project specific include files that are used frequently.

#ifndef HEADERS_H
#define HEADERS_H

#pragma once

#define _USE_MATH_DEFINES		// For macro M_PI.

#include <tchar.h>
#include <iostream>
#include <cmath>								// Constant M_PI and functions.
#include <fstream>								// Input and output to file.
#include <time.h>								// clock().
#include "FlyCapture2.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <windows.h>
#include <process.h>
#include "NIDAQmx.h"
#include <Eigen/Core>
#include <chrono>
#include <sys/stat.h>							// To check for file existense
#include <Eigen/Dense>							//for inverse
#include <iomanip>								//For writing to indexed images (calibration chessboards)
#include <vector>								//Used in Calibrator
#include "opencv2/calib3d/calib3d.hpp"			// Used in Calibrator
#include <opencv2/core/eigen.hpp>				// To convert between eigen and openCV

#endif // HEADERS_H
