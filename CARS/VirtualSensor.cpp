#include "headers.h"
#include "classes.h"
#include "definitions.h"
#include "functions.h"
#include <QDebug>

int count = 0;

VirtualSensor::VirtualSensor()
{
#ifdef CAMERA_IS_AVALIABLE
	m_pgrCamera.connect();
#endif
	initMask();
	if (fileExists("indata/calibrationData.yml"))
	{
		std::cout << "Class VirtualSensor:	Loading calibration coefficients" << std::endl;
		cv::FileStorage storage("indata/calibrationData.yml", cv::FileStorage::READ);
		storage["distCoeffs"] >> distCoeffs;
		storage["cameraMatrix"] >> cameraMatrix;
        storage["cameraToWorldMatrix"] >> cameraToWorldMatrix;
		storage.release();
        cv::cv2eigen(cameraToWorldMatrix, cameraToWorldMatrix_Eigen);
	}
	else
	{
		std::cout << "Class VirtualSensor:	Cannot load calibrationData.yml" << std::endl;
	}
	mask = cv::imread("indata/mask.jpg", 1);
	if (mask.empty())
	{
		std::cout << "Class VirtualSensor:	Cannot load mask.jpg" << std::endl;
	}
    cv::cvtColor(mask, mask, CV_RGB2GRAY);
}

VirtualSensor::~VirtualSensor()
{
#ifdef CAMERA_IS_AVALIABLE
	m_pgrCamera.disconnect();
#endif
}

void VirtualSensor::startSensor(void)
{
#ifdef CAMERA_IS_AVALIABLE
	m_pgrCamera.startCamera();
#endif
}

void VirtualSensor::stopSensor(void)
{
#ifdef CAMERA_IS_AVALIABLE
	m_pgrCamera.stopCamera();
#endif
}


void VirtualSensor::grabThresholdImage()
{
#ifdef CAMERA_IS_AVALIABLE
    // Grab image from camera.

    //double t0 = omp_get_wtime();

    m_pgrCamera.grabImage(&m_rawData);
    // Create cv::Mat object. Data is not copied - pData is simply stored in tempMat.
    cv::Mat tempMat = cv::Mat(m_rawData.rows, m_rawData.cols, CV_8UC1, m_rawData.pData, cv::Mat::AUTO_STEP);

    //std::cout << "img grab time : " << omp_get_wtime() - t0 << std::endl;

    if (tempMat.empty())
    {
        std::cout << "Error: Failed to aquire image from camera. Try replugging the camera." << std::endl;
    }
#else
    // Read image from file.
    cv::Mat tempMat = cv::imread("indata/ImageWithCar.jpg", CV_LOAD_IMAGE_GRAYSCALE);
#endif
    // Enter critical section and copy image to struct drawThreadData.

    //double t = omp_get_wtime();

    EnterCriticalSection(&csDrawThreadData);
    tempMat.copyTo(drawThreadData.image);
    LeaveCriticalSection(&csDrawThreadData);
    tempMat = tempMat - mask * .5;

    //std::cout << "copy time : " << omp_get_wtime() - t << std::endl;

    int lowerThresh = 140;
    int upperThresh = 255;
    int gaussSize = 3;

    //t = omp_get_wtime();

    cv::threshold(tempMat, tempMat, lowerThresh, upperThresh, cv::THRESH_BINARY);
    cv::GaussianBlur(tempMat, tempMat, cv::Size(gaussSize, gaussSize), 0, 0); //(3,3)
    cv::threshold(tempMat, tempMat, lowerThresh, upperThresh, cv::THRESH_BINARY);

    //std::cout << "threshold time : " << omp_get_wtime() - t << std::endl;

    //t = omp_get_wtime();

    // Enter critical section and copy the processed image (tempMat) to processedImage.
    EnterCriticalSection(&csDrawThreadData);
    tempMat.copyTo(drawThreadData.processedImage);
    LeaveCriticalSection(&csDrawThreadData);

    //std::cout << "copy again time : " << omp_get_wtime() - t << std::endl;
}

std::vector<float> VirtualSensor::detectMarkers()
{
    std::vector<float> allMarkers;
#ifdef CAMERA_IS_AVALIABLE

    //double t00 = omp_get_wtime();
    // Grab image from camera.
    m_pgrCamera.grabImage(&m_rawData);
    //double t1 = omp_get_wtime();
    //qDebug() << "cameraTime: " << t1 - t0;
    // Create cv::Mat object. Data is not copied - pData is simply stored in tempMat.
    //double t0 = omp_get_wtime();
    cv::Mat tempMat = cv::Mat(m_rawData.rows, m_rawData.cols, CV_8UC1, m_rawData.pData, cv::Mat::AUTO_STEP);
    //double t1 = omp_get_wtime();
    //qDebug() << "t1: " << t1 - t0;
    if (tempMat.empty())
    {
        std::cout << "Error: Failed to aquire image from camera. Try replugging the camera." << std::endl;
    }
#else
    // Read image from file.
    cv::Mat tempMat = cv::imread("indata/ImageWithCar.jpg", CV_LOAD_IMAGE_GRAYSCALE);
#endif

    //double t2 = omp_get_wtime();
    // Enter critical section and copy image to struct drawThreadData.
    EnterCriticalSection(&csDrawThreadData);
    //double t3 = omp_get_wtime();
    tempMat.copyTo(drawThreadData.image);
    //double t4 = omp_get_wtime();
    LeaveCriticalSection(&csDrawThreadData);
    //double t5 = omp_get_wtime();
    tempMat = tempMat - mask * .5;
    //double t6 = omp_get_wtime();
    //qDebug() << "t1: " << t1 - t0 << "t2: " << t2 - t1 << "t3: " << t3 - t2 << "t4: " << t4 - t3;

    int lowerThresh = 120; // 140 original
    int upperThresh = 255;
    int gaussSize = 3;

    //double t7 = omp_get_wtime();
    cv::threshold(tempMat, tempMat, lowerThresh, upperThresh, cv::THRESH_BINARY);
    cv::GaussianBlur(tempMat, tempMat, cv::Size(gaussSize, gaussSize), 0, 0); //(3,3)
    cv::threshold(tempMat, tempMat, lowerThresh, upperThresh, cv::THRESH_BINARY);
    //double t8 = omp_get_wtime();


    //double t9 = omp_get_wtime();
    // Enter critical section and copy the processed image (tempMat) to processedImage.
    EnterCriticalSection(&csDrawThreadData);
    tempMat.copyTo(drawThreadData.processedImage);
    LeaveCriticalSection(&csDrawThreadData);

    //double t10 = omp_get_wtime();


    /* Calculate markers from tempMat. Markers are given in camera coordinates. */
    imageToMarkers(tempMat, allMarkers);
    //double t11 = omp_get_wtime();
    // Converting to meters

    cameraToWorldCoordinates(allMarkers);
    //double t12 = omp_get_wtime();

    //if (count > 300){
        //qDebug() << "t1: " << t12 - t00;
      //  count = 0;
    //}

    count++;

    return allMarkers;
}

/* Calculate markers from tempMat and rewrites tempMat to processed image.
Markers are returned in pixel coordinates. */
void VirtualSensor::imageToMarkers(cv::Mat tempMat, std::vector<float> &allMarkers)
{
    // Find contours and write to vector with cv::Points
    cv::vector<cv::vector<cv::Point>> contours;
    cv::vector<cv::Vec4i> hierarchy;
    if (!tempMat.empty())
    {
        findContours(tempMat, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    }
    else
    {
        std::cout << "tempMat empty!" << std::endl;
    }

    // Merge connected components
    int i, j;
    if (!contours.empty())
    {
        for (i = 0; i < contours.size(); i++)
        {
            int x = 0;
            int y = 0;
            int len = contours[i].size();
            // naive approach by averaging contour points
            for (j = 0; j < len; j++)
            {
                x = x + contours[i][j].x;
                y = y + contours[i][j].y;
            }
            allMarkers.push_back((float)(x / len));
            allMarkers.push_back((float)(y / len));
        }
    }
    else
    {
        //std::cout << "Warning: No markers found!" << std::endl;
    }
}

void VirtualSensor::cameraToWorldCoordinates(std::vector<float> &data)// float cameraPoint[2], float worldPoint[2])
{
	float cameraPoint[2];
	float nonLinearUndistorted[2];

	Eigen::Vector3f worldPoint_Eigen;

	for (int i = 0; i < data.size(); i = i +2)
	{
		cameraPoint[0] = data[i];
		cameraPoint[1] = data[i + 1];
		nonLinearUndistort(cameraPoint, nonLinearUndistorted);

        Eigen::Vector3f undistCameraPoint;
		undistCameraPoint << nonLinearUndistorted[0], nonLinearUndistorted[1], 1;

		worldPoint_Eigen = cameraToWorldMatrix_Eigen*undistCameraPoint;
		worldPoint_Eigen = worldPoint_Eigen / worldPoint_Eigen[2];
		data[i] = worldPoint_Eigen[0]/PIXELS_PER_METER;
		data[i + 1] = worldPoint_Eigen[1]/PIXELS_PER_METER;
	}
}

void VirtualSensor::nonLinearUndistort(float input[2], float output[2])
{
    double k1, k2, p1, p2, k3, fx, cx, fy, cy, z, x, y, r2, dx, dy, scale, xBis, yBis;
    // Following a somewhat standardized notation for distortion coefficients, the matrix elements are as follows:
    k1 = distCoeffs.at<double>(0, 0);
    k2 = distCoeffs.at<double>(0, 1);
    p1 = distCoeffs.at<double>(0, 2);
    p2 = distCoeffs.at<double>(0, 3);
    k3 = distCoeffs.at<double>(0, 4);
    fx = cameraMatrix.at<double>(0, 0);
    cx = cameraMatrix.at<double>(0, 2);
    fy = cameraMatrix.at<double>(1, 1);
    cy = cameraMatrix.at<double>(1, 2);
    z = 1.;

    x = (input[0] - cx) / fx;
    y = (input[1] - cy) / fy;
    r2 = x*x + y*y;

    dx = 2 * p1*x*y + p2*(r2 + 2 * x*x);
    dy = p1*(r2 + 2 * y*y) + 2 * p2*x*y;
    scale = (1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);

    xBis = x*scale + dx;
    yBis = y*scale + dy;

    output[0] = xBis*fx + cx;
    output[1] = yBis*fy + cy;
}

void VirtualSensor::imageToMarkers2(cv::Mat &img, std::vector<float> &allMarkers)
{
    // void imageToMarkers2(cv::Mat &img, std::vector<int> &allMarkers)
    int nRows, nCols, nMask, nMaskHalf, maskOffsetX, maskOffsetY, threshold, maskThreshold;
    int i, j, k, l;
    int cSum, cSumMax, markerX, markerY;
    // Pointer to 32-bit signed int.
    int *pRow, *pMask;
    cv::Point rectPoint1, rectPoint2;
    Eigen::Matrix<int, 7, 7, Eigen::RowMajor> C;

    /* Convert to 16-bit unsigned int. Is it better to
    convert in each loop iteration? */
    img.convertTo(img, CV_32S);

    // Can we really gain something by assuming this?
    CV_Assert(img.isContinuous());
    CV_Assert(img.depth() == CV_32SC1);

    nRows = img.rows;
    nCols = img.cols;

    /* Size of mask is nMask x nMask.
    The virtual sensor should have a function that creates the mask (uint16),
    and one that creates the threshold. These should be memember variables. */
    nMask = 7;
    maskOffsetX = nMask - 1;
    maskOffsetY = (nMask - 1) / 2 - 1;
    nMaskHalf = (nMask - 1) / 2;

    /* Pixel threshold. This shoud be a Gaussian in future implementations.
    Is it best to calculate a threshold matrix of size 1024x1280,
    rather than using a function? (Probably)s
    Should a LUT be used?
    Note: a constant could be added to the Gaussian as a parameter once a shape has been determined. */
    threshold = 100;
    maskThreshold = 25000;

    // Loop over rows in the image.
    for (i = maskOffsetY; i < nRows - nMask + 1; i++)
    {
        // Row pointer.
        pRow = img.ptr<int>(i);
        // Loop over columns in the image.
        for (j = maskOffsetX; j < nCols - nMask + 1; j++)
        {
            // Check if pixel value is larger than a threshold.
            if (pRow[j] >= threshold)
            {
                // Reset variable that checks for local maximum.
                cSumMax = 0;
                // Scan area using a mask.
                for (k = i - maskOffsetY; k < i+1; k++)
                {
                    pMask = img.ptr<int>(k);
                    for (l = j - maskOffsetX; l < j+1; l++)
                    {
                        /* Multiply the submatrix of the image with upper left corner in (k,l) of
                        size nMask x nMask by the mask. The multiplication is done element-wise.
                        Note that a typecast to integer is needed to prevent saturation. */
                        Eigen::Map<Eigen::Matrix<int, 7, 7, Eigen::RowMajor>, Eigen::Unaligned, Eigen::Stride<1280, 1>> localMat(pMask+l);
                        C = m_mask.cwiseProduct(localMat);
                        // Add all elements of the resulting matrix.
                        cSum = C.sum();

                        // Check if new maximum is found.
                        if (cSum > cSumMax)
                        {
                            cSumMax = cSum;
                            // Assuming that the upper left most pixel is (0,0), not (1,1).
                            markerY = k + nMaskHalf;
                            markerX = l + nMaskHalf;
                        }

                    } // l-loop
                } // k-loop

                // Marker found if cSumMax exceeds markerThreshold.
                if (cSumMax >= maskThreshold)
                {
                    allMarkers.push_back(markerX);
                    allMarkers.push_back(markerY);

                    // Erase marker from image by drawing a black rectangle.
                    rectPoint1.x = markerX-nMaskHalf;
                    rectPoint1.y = markerY-nMaskHalf;
                    rectPoint2.x = rectPoint1.x + nMask;
                    rectPoint2.y = rectPoint1.y + nMask;
                    cv::rectangle(img, rectPoint1, rectPoint2, cv::Scalar(0, 0, 0), -1);
                }
            }
        } // j-loop
    } // i-loop
}

// Used by imageToMarkers2.
double VirtualSensor::maskFunction(int y, int x, int n)
{
    // Maximum is attained in the middle of the mask.
    double maxValMask = 10.0;
    // Minimum is attained in the corners of the mask.
    double minValMask = 1.0;

    // Square shaped mask is assumed.
    double k = (maxValMask - minValMask) / (((n + 1) / 2 - 1) * std::sqrt(2));
    return maxValMask - k*std::sqrt(std::pow((n + 1) / 2 - y, 2) + std::pow((n + 1) / 2 - x, 2));
}

// Used by imageToMarkers2.
void VirtualSensor::initMask(void)
{
    // Size of mask is n x n.
    int n = 7;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            m_mask(i, j) = (int)(maskFunction((double)(i+1), (double)(j+1), 7)+0.0001);
        }
    }
}
