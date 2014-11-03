// TestProject1.cpp : Defines the entry point for the console application.
#include "stdafx.h"
#include "classes.h"
#include "definitions.h"
#include "functions.h"

using namespace Bilbana;

// Initialize external declarations.
struct DrawThreadData drawThreadData;
struct RegulatorThreadData regulatorThreadData;
struct LapData lapData;
std::vector<float> gRef;
int gRefLen;

CRITICAL_SECTION csDrawData;
HANDLE hRegulatorThreadEvent1, hRegulatorThreadEvent2;
// remove: hRegulatorThreadEvent3, hRegulatorThreadEvent4;
HANDLE hDrawThreadEvent;

int RUN_TIME = 1500; // Number of samples to run
bool run;

// Declaration of thread functions.
unsigned __stdcall drawThread(void *ArgList);
unsigned __stdcall regulatorThread(void *ArgList);

// Main method.
int _tmain(int argc, _TCHAR* argv[])
{
	HANDLE hDrawThread, hRegulatorThread;
	VirtualSensor sensor;
	// Timing.
	int t1, t2;
	double tDiff;
	// Counts the number of main thread loops.
	int loopCount;

	std::vector<float> markers;
	// How much memory should be reserved?
	markers.reserve(100);
	std::vector<CarMeasurement> carMeasurements;
	// Reserve memory for 10 cars.
	carMeasurements.reserve(10);
	std::vector<CarData> carData;
	carData.reserve(10);
	std::vector<CarData> oldCarData;
	oldCarData.reserve(10);
	std::vector<Car> cars;
	cars.reserve(10);

	// Initiate vectors in structs.
	drawThreadData.carData.reserve(10);
	lapData.bestTimes.reserve(10);
	regulatorThreadData.carData.reserve(10);

	// Read reference signal from file.
	initReference();

	// TODO: read car settings from file and create car vector.
	// Vector with all car objects. 
	for (int id = 0; id < NUM_CARS; id++)
	{
		cars.push_back(Car(id, false, CarMode::Auto, FilterType::EKF, MotionModelType::CTModel));
	}

	// Used to count iterations in main loop.
	loopCount = 0;

	// Initialize critical section.
	InitializeCriticalSection(&csDrawData);
	// Create events. Initial state for hRegulatorThreadEvent1 is signaled.
	hRegulatorThreadEvent1 = CreateEvent(NULL, FALSE, TRUE, NULL);
	hRegulatorThreadEvent2 = CreateEvent(NULL, FALSE, FALSE, NULL);

	// For draw and regulator thread.
	run = TRUE;
	// Start virtual sensor.
	sensor.startSensor();

	// Create draw thread.
	hDrawThread = (HANDLE)_beginthreadex(NULL, 0, drawThread, NULL, 0, NULL);
	// Create controller thread.
	hRegulatorThread = (HANDLE)_beginthreadex(NULL, 0, regulatorThread, NULL, 0, NULL);

	std::cout << "Main thread:		Start" << std::endl;
	for (int i = 0; i < RUN_TIME;i++)
	{
		// Timing.
		t1 = clock();

		// Save car data from previous iteration.
		oldCarData = carData;

		// Detect markers using virtual sensor. Output is written to vector markers in world coordinates.
		markers = sensor.detectMarkers(); 

		// Calculate position, angle and id.
		carMeasurements = findCars(markers);

		// Set car as active or inactive and manual or auto in "states"
		//setOtherStates(states, cars);

		// Add measurements to cars.
		for (int i = 0; i < carMeasurements.size(); i++)
		{
			// Add new measurement to car.
			cars[carMeasurements[i].id].addMeasurement(carMeasurements[i].measurement);
		}

		// Loop over all cars and estimate states.
		for (int i = 0; i < NUM_CARS; i++)
		{
			// Update filter.
			cars[i].updateFilter();
			// Extract data from car.
			carData[i].state = cars[i].getState();
			carData[i].carAttributes = cars[i].getCarAttributes();
		}

		// Send all cars states and previous states to drawthread <------------------------------
		EnterCriticalSection(&csDrawData);
		// Use old and new states to draw traveled distance on evaluated image
		//drawStatesToImage(oldPositionPix, positionPix, drawThreadData.evaluatedImage);
		drawStatesToImage(oldCarData, carData, drawThreadData.evaluatedImage);
		LeaveCriticalSection(&csDrawData);


		// TODO: implement timing.
		//updateLapData(oldStates, states); //<------------------------FIXA -------------

		/* Probably better to have a critical section at regulatorThreadData, and 
		only SetEvent in main (so that main never needs to wait on regulator thread). */
		// Wait until regulator thread have copied data from struct.
		WaitForSingleObject(hRegulatorThreadEvent1, INFINITE);
		regulatorThreadData.carData = carData;
		// Tell regulator thread that data have been written.
		SetEvent(hRegulatorThreadEvent2);

		// Enter critical section and send data to draw thread.
		EnterCriticalSection(&csDrawData);
		drawThreadData.carData = carData;
		LeaveCriticalSection(&csDrawData);

		// Signal draw thread to display image every 4th iteration.
		loopCount++;
		if ((loopCount % 4) == 0)
		{
			// Signal draw thread to display image.
			SetEvent(hDrawThreadEvent);
		}

		t2 = clock();
		tDiff = ((double)(t2 - t1)) / CLOCKS_PER_SEC;
		// Elapsed time in milliseconds. 
		//std::cout << "ms, main: " << 1000*tDiff << std::endl; 
		// Rough estimation of main loop FPS. 
		//std::cout << "FPS, main: " << 1.0/tDiff << std::endl;
		// TODO: Implement better FPS estimation.
	}
	std::cout << "Main thread:		Stop measuring" << std::endl;

	// Signal draw and regulator thread to stop.
	run = FALSE;

	// Wait for draw and regulator thread to finish. 
	Sleep(500);
	SetEvent(hDrawThreadEvent);
	//WaitForSingleObject(hDrawThread, INFINITE);
	Sleep(500);
	//SetEvent(hRegulatorThreadEvent2);
	//SetEvent(hRegulatorThreadEvent4);
	WaitForSingleObject(hRegulatorThread, INFINITE);

	// Stop virtual sensor.
	sensor.stopSensor();

	// Clean-up
	// Destroy thread objects.
	//CloseHandle(hDrawThread);
	CloseHandle(hRegulatorThread);

	// Close event handles.
	CloseHandle(hRegulatorThreadEvent1);
	CloseHandle(hRegulatorThreadEvent2);
	CloseHandle(hDrawThreadEvent);

	// Release resources used by the critical section.
	DeleteCriticalSection(&csDrawData);

	/* TODO: delete all objects properly.
	Implement destructors properly. Objects allocated on stack are
	automatically destroyed. */

	std::cout << "Main thread:		Exiting" << std::endl;
	return 0;
}

// ---------- Thread functions ----------
// Regulator thread.
unsigned __stdcall regulatorThread(void *ArgList)
{
	Sleep(500);
	
	std::vector<float> signals(2);
	float turnSignal;
	std::vector<CarData> carData;
	// Reserve memory for 10 cars.
	carData.reserve(10);

	// Create IO controller.
	std::vector<IOControl> ioControls;
	for (int j = 0; j < 3; j++)
	{
		ioControls.push_back(IOControl(j));
	}

	// Vector with controllers. 
	std::vector<Controller*> controllers;
	controllers.reserve(NUM_CARS);
	// This doesnt work right? controllers[0] = PIDController(0);

	// Initialize controllers. 
	for (int i = 0; i < NUM_CARS; i++)
	{
		controllers[i] = new PIDController(i);
	}

	// Initialize and start input-output controllers.
	for (int i = 0; i < NUM_CARS; i++)
	{
		ioControls[i].init();
		ioControls[i].startController();
	}

	while (run)
	{
		// Wait for all data to be written to struct.
		WaitForSingleObject(hRegulatorThreadEvent2, INFINITE);
		// Copy data.
		carData = regulatorThreadData.carData;
		//regulatorThreadData.signals = ;
		// Tell main thread that the data have been copied.
		SetEvent(hRegulatorThreadEvent1);

		// Loop over all cars and send control signals.
		for (int i = 0; i < NUM_CARS; i++)
		{
			// Only send signals to active cars.
			if (!carData[i].carAttributes.active)
			{
				// Skip to next iteration of the for-loop.
				continue;
			}

			// Check in which mode the car is operating.
			switch (carData[i].carAttributes.mode)
			{
				// Send gas and turn signal from hand controller to car. 
				case CarMode::Manual:
					ioControls[i].manualControl();
					break;
				// Send gas and turn signal from controller to car.
				case CarMode::Auto:
					signals = controllers[i]->calcSignals(carData[i].state);
					ioControls[i].sendSignals(signals[0], signals[1]);
					break;
				// Send turn signal from controller and gas signal from hand controller to car.
				case CarMode::Assisted:
					turnSignal = controllers[i]->calcTurnSignal(carData[i].state);
					ioControls[i].assistedControl(turnSignal);
					break;
				default:
					std::cout << "Error: Car mode type not implemented, in main(), main.cpp" << std::endl;
			}
		}
	}

	// Stop IO controllers.
	for (int i = 0; i < NUM_CARS; i++)
	{
		ioControls[i].stopController();
	}

	return 0;
}

// Draw thread.
unsigned __stdcall drawThread(void *ArgList)
{
	// Testing latency
	int start = clock(), stop; double timeDiff;

	Sleep(500);
	cv::Mat tmpMat;
	//std::vector<float> states(CAR_STATE_LENGTH*NUM_CARS), positionPix(2);
	std::vector<float> positionPix(2);
	std::vector<CarData> carData;
	carData.reserve(10);
	
	float angle;

	cv::namedWindow("preProc", CV_WINDOW_NORMAL);
	cvMoveWindow("preProc", 0, 1024 / 2);
	cv::resizeWindow("preProc", 1280 / 2, 1024 / 2);

	cv::namedWindow("postProc", CV_WINDOW_NORMAL);
	cvMoveWindow("postProc", 1280 / 2, 1024 / 2);
	cv::resizeWindow("postProc", 1280 / 2, 1024 / 2);

	cv::namedWindow("evaluated", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("evaluated", 1920, 0);
	//cv::resizeWindow("evaluated", 1000, 800);

	//cvMoveWindow("evaluated", 0, 0);

	cvSetWindowProperty("evaluated", CV_WINDOW_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	HWND win_handle = FindWindow(0, L"evaluated");
	SetWindowLong(win_handle, GWL_STYLE, GetWindowLong(win_handle, GWL_EXSTYLE) | WS_EX_TOPMOST);
	ShowWindow(win_handle, SW_SHOW);


	//cv::namedWindow("project", CV_WINDOW_NORMAL);
	//cvMoveWindow("project", 0, 0);
	//cv::resizeWindow("project", 1000, 800);

	//cvSetWindowProperty("evaluated", CV_WINDOW_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	/*
	// get transformation parameters from input
	cv::namedWindow("photo", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("photo", 0, 0);
	cv::setMouseCallback("photo", trackMouse, NULL);

	cvSetWindowProperty("photo", CV_WINDOW_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	HWND win_handle = FindWindow(0, L"photo");
	SetWindowLong(win_handle, GWL_STYLE, GetWindowLong(win_handle, GWL_EXSTYLE) | WS_EX_TOPMOST);
	ShowWindow(win_handle, SW_SHOW);
	*/

	// ========================================== Loading background images
	drawThreadData.evaluatedImage = cv::imread("indata/texture.jpg", 1);
	if (drawThreadData.evaluatedImage.empty())
	{
		// If this error is displayed the program will crash
		std::cout << "Draw thread:		Cannnot open world map!" << std::endl;
	}

	if (!drawThreadData.image.empty())
	{
		std::cout << "Draw thread:		Saving first frame in 'Startup_frame.jpg'" << std::endl;
		imwrite("outdata/Startup_frame.jpg", drawThreadData.image);
	}
	//std::vector<cv::Mat> background;
	/*
	for (int i = 1;; i++)
	{
	std::stringstream str;
	str << "indata/background/texture" << std::setw(2) << std::setfill('0') << i << ".jpg";
	std::cout << "Loading " << str.str() << std::endl;
	if (fileExists(str.str()))
	{
	cv::Mat img = cv::imread(str.str(), 1);
	drawThreadData.background.push_back(img);
	//i++;
	}
	else
	{
	std::cout << "Drawthread: Found " << i << " background images." << std::endl;
	break;
	}
	}
	*/
	for (int i = 1;; i++)
	{
		std::stringstream str;
		str << "indata/background/texture" << std::setw(2) << std::setw(4) << std::setfill('0') << i << ".jpg";
		if (fileExists(str.str()))
		{
			cv::Mat img = cv::imread(str.str(), 1);
			drawThreadData.background.push_back(img);
		}
		else
		{
			std::cout << "Draw thread:		Found " << i - 1 << " background images." << std::endl;
			break;
		}
	}

	//cv::cvtColor(drawThreadData.evaluatedImage, drawThreadData.evaluatedImage, CV_GRAY2RGB);
	/*
	// If image is available, use as background and save once. Else, use white image.
	if (!drawThreadData.image.empty())
	{
	std::cout << "Saving first frame in 'Startup_frame.jpg'" << std::endl;
	EnterCriticalSection(&csDrawData);
	imwrite("./Startup_frame.jpg", drawThreadData.image);
	//imwrite("Startup_frame.jpg", drawThreadData.image);
	cv::cvtColor(drawThreadData.image, drawThreadData.evaluatedImage, CV_GRAY2RGB);
	LeaveCriticalSection(&csDrawData);
	// Some way of writing the finish line
	//line(drawThreadData.evaluatedImage, cv::Point(mainThreadData.checkPoint, 200), cv::Point(mainThreadData.checkPoint, 500), cv::Scalar(20, 200, 20));
	}
	else
	{
	std::cout << "No reference image found. Using white background." << std::endl;
	drawThreadData.evaluatedImage = cv::Mat(1024, 1280, CV_8UC3, cv::Scalar(255, 255, 255));
	}
	*/

	// Read and print reference track
	std::vector<float> track;
	// Prepare a pair of iterators to read the data from cin
	std::istream_iterator<double> eos;
	std::ifstream inputFile("reference.txt");
	std::istream_iterator<double> iit(inputFile);
	// No loop is necessary, because you can use copy()
	std::copy(iit, eos, std::back_inserter(track));
	for (int i = 1; i < track.size() - 2; i = i + 2)
	{
		for (int j = 0; j < drawThreadData.background.size(); j++)
		{
			line(drawThreadData.background[j], cv::Point(track[i], track[i + 1]), cv::Point(track[i + 2], track[i + 3]), cv::Scalar(0, 0, 255));
		}
		//std::cout << "[ " << track[i] << ", " << track[i+1] << "]"<< std::endl;
		//line(drawThreadData.evaluatedImage, cv::Point(track[i], track[i + 1]), cv::Point(track[i + 2], track[i + 3]), cv::Scalar(0, 0, 255));
	}

	int i = 0;
	while (run)
	{
		// For testing latency
		stop = clock();
		timeDiff = ((double)(stop - start)) / CLOCKS_PER_SEC;
		std::ostringstream time;
		time << "Time: " << timeDiff;
		std::string s = time.str();

		// Wait for main thread to send signal. 
		WaitForSingleObject(hDrawThreadEvent, INFINITE);

		// TODO: optimize use of critical section.
		// Enter critical section and print image.
		EnterCriticalSection(&csDrawData);
		// Get states from struct drawThreadData.
		carData = drawThreadData.carData;
		if (!drawThreadData.image.empty())
		{

			cv::putText(drawThreadData.image, s, cvPoint(30, 200), cv::FONT_HERSHEY_COMPLEX_SMALL, 8., cvScalar(255, 255, 255), 1, CV_AA);
			// Display image.
			cv::imshow("preProc", drawThreadData.image);
		}

		if (!drawThreadData.processedImage.empty())
		{
			// Display processed image.
			cv::imshow("postProc", drawThreadData.processedImage);
		}

		// Add features to momentary evaluated image
		// If statement is always true.
		//if (!drawThreadData.evaluatedImage.empty())
		//{
		// Copy evaluated image.
		//drawThreadData.evaluatedImage.copyTo(tmpMat);
		drawThreadData.background[i%drawThreadData.background.size()].copyTo(tmpMat);
		i++;
		LeaveCriticalSection(&csDrawData);


		// Draw car on track
		for (int j = 0; j < NUM_CARS; j++)
		{
			if (carData[j].carAttributes.active)
			{
				// Extract position and angle.
				positionPix[0] = carData[j].state[0]*PIXELS_PER_METER;
				positionPix[1] = carData[j].state[1]*PIXELS_PER_METER;
				angle = carData[j].state[2]*180.0/M_PI;
				// Convert to pixel coordinates.
				//metersToPixels(positionPix);

				cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(positionPix[0], positionPix[1]),
					cv::Size2f(55, 27
					), angle);
				cv::Point2f vertices2f[4];
				cv::Point vertices[4];
				rRect.points(vertices2f);
				for (int i = 0; i < 4; ++i){
					vertices[i] = vertices2f[i];
				}
				switch (j)
				{
				case 0:
					cv::fillConvexPoly(tmpMat, vertices, 4, cv::Scalar(204, 255, 51)); // Aqua
					break;
				case 1:
					cv::fillConvexPoly(tmpMat, vertices, 4, cv::Scalar(153, 255, 255)); // Yellow
					break;
				case 2:
					cv::fillConvexPoly(tmpMat, vertices, 4, cv::Scalar(51, 255, 0)); // Green
					break;
				default:
					cv::fillConvexPoly(tmpMat, vertices, 4, cv::Scalar(255, 255, 255)); // White
					break;
				}
			}
		}
		// For testing latency
		cv::putText(tmpMat, s, cvPoint(30, 200), cv::FONT_HERSHEY_COMPLEX_SMALL, 9., cvScalar(255, 255, 255), 1, CV_AA);

		/*
		std::ostringstream carFps;
		carFps << "Time: " << lapData.currTime;
		std::string s1 = carFps.str();
		cv::putText(tmpMat, s1, cvPoint(30, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(20, 20, 200), 1, CV_AA);

		std::ostringstream lap;
		lap << "Lap: " << lapData.lapTime;
		std::string s2 = lap.str();
		cv::putText(tmpMat, s2, cvPoint(30, 60),
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(20, 20, 200), 1, CV_AA);

		// Print best times
		for (int i = 0; i < lapData.bestTimes.size(); i++)
		{
		std::ostringstream bestlap;
		bestlap << lapData.bestTimes[i];
		std::string s6 = bestlap.str();
		cv::putText(tmpMat, s6, cvPoint(30, i * 30 + 140),
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(20, 20, 200), 1, CV_AA);
		}

		//std::ostringstream bestlap;
		//bestlap << "Best lap: " << mainThreadData.bestTime;
		//std::string s3 = bestlap.str();
		cv::putText(tmpMat, "5 Best laps: ", cvPoint(30, 110),
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(20, 20, 200), 1, CV_AA);

		// Print zone 1 or zone 2
		if (lapData.check1 == 1)
		cv::putText(tmpMat, "Zone: 1", cvPoint(30, 930),
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(20, 20, 200), 1, CV_AA);
		else
		cv::putText(tmpMat, "Zone: 2", cvPoint(30, 930),
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(20, 20, 200), 1, CV_AA);

		// Print position in meters
		//pixelsToMeters(drawThreadData.states, posMeters);
		std::ostringstream pos;
		pos << "Pos (m): [" << states[0] << ", " << states[1] << "]";
		std::string s4 = pos.str();
		cv::putText(tmpMat, s4, cvPoint(30, 900),
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(20, 20, 200), 1, CV_AA);
		*/
		cv::imshow("evaluated", tmpMat);
		//}

		// Wait in order to draw to window. Maximum framerate is 30 FPS.
		cv::waitKey(30);
	}
	std::cout << "Draw thread:		ending" << std::endl;
	return 0;
}