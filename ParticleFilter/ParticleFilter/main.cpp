// particleFilter.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "classes.h"
#include "functions.h"
#include "definitions.h"

using namespace Bilbana;
CRITICAL_SECTION csDrawData;
HANDLE hDrawThreadEvent;
struct DrawThreadData drawThreadData;
bool run;
int runTime = 1000;

// Declaration of thread functions.
unsigned __stdcall drawThread(void *ArgList);

int _tmain(int argc, _TCHAR* argv[])
{
	float speedExp = 10; // expected speed(ppx / timestep)
	int lim = 7; // How far around every point to search in hypothesis(+-border in x - and y - dimension respectively)

	// Read car pattern
	Eigen::MatrixXf carPattern;
	cv::FileStorage storage("indata/car02.yml", cv::FileStorage::READ);
	cv::Mat tmp;
	storage["pattern"] >> tmp;
	storage.release();
	cv::cv2eigen(tmp, carPattern);
	drawThreadData.pattern = carPattern;

	// Set up a particle filter
	ParticleFilter particleFilter(carPattern, speedExp, lim);

	// Initialize critical sections.
	InitializeCriticalSection(&csDrawData);
	// Create events. Initial state for hRegulatorThreadEvent1 is not signaled.
	hDrawThreadEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	run = TRUE;
	// Create draw thread.
	HANDLE hDrawThread = (HANDLE)_beginthreadex(NULL, 0, drawThread, NULL, 0, NULL);

	PGRCamera cam;
	RawData data;
	cam.connect();
	cam.startCamera();
	int lowerThresh = 150;
	//std::cout << "Threshold: " << lowerThresh << '\xd';
	int upperThresh = 255;
	int gaussSize = 3;
	float t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
	
	

	cv::Mat img; // current image
	cv::Mat showImg1, showImg2; // just for displaying purposes
	std::ofstream logFile;
	logFile.open("log.txt");
	
	// Save first image
	cam.grabImage(&data);
	img = cv::Mat(data.rows, data.cols, CV_8UC1, data.pData, cv::Mat::AUTO_STEP);
	imwrite("startup_image.png", img);

	for (int t = 0; ; t++)
	{
		clock_t start = clock();
		// Load the image where position is to be identified
		/*std::stringstream str;
		str << "indata/testImages/car" << t+1 << ".png"; // omitting image zero with gives pos
		if (fileExists(str.str()))
		{
			img = cv::imread(str.str(), CV_8UC1);
			
		}
		else
		{
			std::cout << "Class ParticleFilter exiting:	Found " << t << " images." << std::endl;
			break;
		}*/
		cam.grabImage(&data);
		img = cv::Mat(data.rows, data.cols, CV_8UC1, data.pData, cv::Mat::AUTO_STEP);
		EnterCriticalSection(&csDrawData);
		cvtColor(img, drawThreadData.image, CV_GRAY2BGR);
		LeaveCriticalSection(&csDrawData);

		

		cv::threshold(img, img, lowerThresh, upperThresh, cv::THRESH_BINARY);
		cv::GaussianBlur(img, img, cv::Size(gaussSize, gaussSize), 0, 0); //(3,3)
		cv::threshold(img, img, lowerThresh, upperThresh, cv::THRESH_BINARY);

		
		if (particleFilter.carGone())
		{
			std::cout << "Class ParticleFilter:	Could not find any matching hypotheses. Attempting extensive search" << std::endl;
			particleFilter.extensiveSearch(img);
		}
		else
		{
			t1 = clock();
			particleFilter.propagate();
			t2 = clock();
			particleFilter.update(img);
			t3 = clock();
			particleFilter.resample();
			t4 = clock();
		}
		clock_t stop = clock();
		
		

		EnterCriticalSection(&csDrawData);
		drawThreadData.posX = particleFilter.getPosX();
		drawThreadData.posY = particleFilter.getPosY();
		drawThreadData.yaw = particleFilter.getYaw();
		drawThreadData.posXPoints = particleFilter.getPosXPoints();
		drawThreadData.posYPoints = particleFilter.getPosYPoints();
		drawThreadData.yawPoints = particleFilter.getYawPoints();
		drawThreadData.sumStates = particleFilter.getSumStates();
		LeaveCriticalSection(&csDrawData);

		// Tell draw thread an estimation is done
		SetEvent(hDrawThreadEvent);

		// ====================== Just for logging purposes ======================
		particleFilter.logStates(&logFile);
		
		std::cout << "Time: " << (stop - start) / (float)CLOCKS_PER_SEC << "s" << std::endl;
		std::cout << "prop: " << (t2 - t1) / (float)CLOCKS_PER_SEC << "s" << std::endl;
		std::cout << "up: " << (t3 - t2) / (float)CLOCKS_PER_SEC << "s" << std::endl;
		std::cout << "res: " << (t4 - t3) / (float)CLOCKS_PER_SEC << "s" << std::endl;
	}
	std::cout << "Main thread:		Exiting" << std::endl;

	// Signal draw thread to stop.
	run = FALSE;

	// Wait for draw and regulator thread to finish. 
	Sleep(500);
	WaitForSingleObject(hDrawThread, INFINITE);

	// Fix thread objects, log file, camera
	CloseHandle(hDrawThread);
	CloseHandle(hDrawThreadEvent);
	DeleteCriticalSection(&csDrawData);


	logFile.close();

	return 0;
}

unsigned __stdcall drawThread(void *ArgList)
{
	float *states;
	while (run)
	{
		Sleep(10);
		// Wait for new input signal values to be written to struct.
		WaitForSingleObject(hDrawThreadEvent, INFINITE);
		
		EnterCriticalSection(&csDrawData);
		drawStatesToImg(drawThreadData.pattern, drawThreadData.posXPoints, drawThreadData.posYPoints, drawThreadData.yawPoints, drawThreadData.image, 1);
		
		drawStatesToImg(drawThreadData.pattern, drawThreadData.posX, drawThreadData.posY, drawThreadData.yaw, drawThreadData.image, 2);
		states = drawThreadData.sumStates;
		drawCar(drawThreadData.pattern, states[0] / NUMBER_OF_PARTICLES, states[1] / NUMBER_OF_PARTICLES, states[2] / NUMBER_OF_PARTICLES, drawThreadData.image);

		LeaveCriticalSection(&csDrawData);
		cv::imshow("image", drawThreadData.image);
		cv::waitKey(10);
	}
	std::cout << "Draw thread:		ending" << std::endl;
	return 0;
}