#pragma once

#define NUMBER_OF_PARTICLES 1000

struct DrawThreadData
{
	float *posX;
	float *posY;
	float *yaw;
	float *posXPoints;
	float *posYPoints;
	float *yawPoints;
	float *sumStates;
	cv::Mat image;
	Eigen::MatrixXf pattern;
};

// RawData struct - passed from PGRCamera.
struct RawData
{
	unsigned char *pData;
	unsigned int dataSize;
	unsigned int rows;
	unsigned int cols;
};