#pragma once

#include <Eigen/Core>
#include "PGRCamera.h"

class VirtualSensor
{
private:
	PGRCamera m_pgrCamera;		// Is pointer better?
	RawData m_rawData;

    double t1 = 0;
    double t0 = 0;

	cv::Mat mask;
		
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Mat cameraToWorldMatrix;
	Eigen::Matrix3f cameraToWorldMatrix_Eigen;
	cv::Mat worldToCameraMatrix;
	Eigen::Matrix3f worldToCameraMatrix_Eigen;

    // Used by imageToMarkers2.
    Eigen::Matrix<int, 7, 7, Eigen::RowMajor> m_mask;

    void nonLinearUndistort(float input[2], float output[2]);

    // Used by imageToMarkers2.
    double maskFunction(int y, int x, int size);
    // Used by imageToMarkers2.
    void initMask(void);

public:
	VirtualSensor();
	~VirtualSensor();

	// Start virtual sensor. The camera starts capturing images.
	void startSensor(void);
	// Stop virtual sensor. The camera stops capturing images.
	void stopSensor(void);
    // Grabs an image from the camera, then finds and returns midpoints of markers.
	std::vector<float> detectMarkers(void);
    void grabThresholdImage();
    void imageToMarkers(cv::Mat tempMat, std::vector<float> &markers);
    void VirtualSensor::cameraToWorldCoordinates(std::vector<float> &data);

    // Alternative to imageToMarkers. This function is based on the marker detection algorithm used at LiU.
    // Note that it needs further testing.
	void imageToMarkers2(cv::Mat &img, std::vector<float> &allMarkers);
};
