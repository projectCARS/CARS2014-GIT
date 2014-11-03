#include "headers.h"
#include "definitions.h"
#include "Calibrator.h"
#include "PGRCamera.h" // To get images in Calibrator::saveStills
#include "functions.h" // to check for file existence in constructor and callbackfunction for mouse movements


// Aquire calibration images if they are already present
Calibrator::Calibrator()
{
    flag = 0;
    if (fileExists("indata/calibrationData.yml") )
    {
        std::cout << "Class Calibrator: Loading calibration coefficients" << std::endl;
        cv::FileStorage storage("indata/calibrationData.yml", cv::FileStorage::READ);
        storage["distCoeffs"] >> distCoeffs;
        storage["cameraMatrix"] >> cameraMatrix;
        storage["cameraToWorldMatrix"] >> cameraToWorldMatrix;
        storage["worldToCameraMatrix"] >> worldToCameraMatrix;
        storage.release();
        cv::cv2eigen(cameraToWorldMatrix, cameraToWorldMatrix_Eigen);
        cv::cv2eigen(worldToCameraMatrix, worldToCameraMatrix_Eigen);
    }
}

Calibrator::~Calibrator()
{
}

// Open chessboard images and extract corner points
int Calibrator::addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize)
{
    // the points on the chessboard
    std::vector<cv::Point2f> imageCorners;
    std::vector<cv::Point3f> objectCorners;

    // 3D Scene Points:
    // Initialize the chessboard corners
    // in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z)= (i,j,0)
    for (int i = 0; i<boardSize.height; i++)
    {
        for (int j = 0; j<boardSize.width; j++)
        {
            objectCorners.push_back(cv::Point3f(i, j, 0.0f));
        }
    }

    // 2D Image points:
    cv::Mat image; // to contain chessboard image
    int successes = 0;
    // for all viewpoints
    for (int i = 0; i<filelist.size(); i++)
    {
        std::cout << "Processing image " << i + 1 << "/" << filelist.size() << '\xd';
        // Open the image
        image = cv::imread(filelist[i], 0);

        // Get the chessboard corners
        bool found = cv::findChessboardCorners(image, boardSize, imageCorners);

        /*
        // Get subpixel accuracy on the corners
        cv::cornerSubPix(image, imageCorners,
            cv::Size(5, 5),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::MAX_ITER +
            cv::TermCriteria::EPS,
            30,// max number of iterations
            0.1));// min accuracy
        */

        // If we have a good board, add it to our data
        if (imageCorners.size() == boardSize.area())
        {
            // Add image and scene points from one view
            addPoints(imageCorners, objectCorners);
            successes++;
        }

        //Draw the corners
        cv::drawChessboardCorners(image, boardSize, imageCorners, found);
        cv::imshow("Found chessboard pattern", image);
        cv::waitKey(100);
    }
    cv::destroyWindow("Found chessboard pattern");
    return successes;
}

// Add scene points and corresponding image points
void Calibrator::addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners)
{
    // 2D image points from one view
    imagePoints.push_back(imageCorners);
    // corresponding 3D scene points
    objectPoints.push_back(objectCorners);
}

// Calibrate the camera
// returns the re-projection error
double Calibrator::calibrate(cv::Size &imageSize)
{
    //Output rotations and translations
    std::vector<cv::Mat> rvecs, tvecs;

    // start calibration
    return
        calibrateCamera(objectPoints, // the 3D points
        imagePoints,  // the image points
        imageSize,    // image size
        cameraMatrix, // output camera matrix
        distCoeffs,   // output distortion matrix
        rvecs, tvecs, // Rs, Ts
        flag);        // set options
    //﻿  ﻿  ﻿  ﻿  ﻿  ,CV_CALIB_USE_INTRINSIC_GUESS);

}

// remove distortion in an image (after calibration)
cv::Mat Calibrator::remap(const cv::Mat &image)
{
    cv::Mat undistorted;

    cv::initUndistortRectifyMap(
        cameraMatrix,  // computed camera matrix
        distCoeffs,    // computed distortion matrix
        cv::Mat(),     // optional rectification (none)
        cv::Mat(),     // camera matrix to generate undistorted
        cv::Size(image.cols, image.rows),
        //            image.size(),  // size of undistorted
        CV_32FC1,      // type of output map
        map1, map2);   // the x and y mapping functions

    // Apply mapping functions
    cv::remap(image, undistorted, map1, map2,
        cv::INTER_LINEAR); // interpolation type

    return undistorted;
}

cv::Mat Calibrator::undistort(const cv::Mat &image)
{
    cv::Mat undistorted;
    cv::undistort(image, undistorted, cameraMatrix, distCoeffs);
    return undistorted;
}

// Set the calibration options
// 8radialCoeffEnabled should be true if 8 radial coefficients are required (5 is default)
// tangentialParamEnabled should be true if tangeantial distortion is present
void Calibrator::setCalibrationFlag(bool radial8CoeffEnabled, bool tangentialParamEnabled)
{
    // Set the flag used in cv::calibrateCamera()
    flag = 0;
    if (!tangentialParamEnabled) flag += CV_CALIB_ZERO_TANGENT_DIST;
    if (radial8CoeffEnabled) flag += CV_CALIB_RATIONAL_MODEL;
}

void Calibrator::saveStills(std::string path, int noImages, float alpha, float beta)
{

    PGRCamera cam;
    RawData data;
    cam.connect();
    cam.startCamera();

    //cam.setup();
    cv::namedWindow("Recording calibration patterns");
    for (int i = 1; i <= noImages; i++)
    {
        cv::waitKey(2000);

        cam.grabImage(&data);
        cv::Mat image = cv::Mat(data.rows, data.cols, CV_8UC1, data.pData, cv::Mat::AUTO_STEP);
        image.convertTo(image, -1, alpha, beta);
        //cv::medianBlur(image, image, 3);
        cv::GaussianBlur(image, image, cv::Size(3, 3), 0, 0);
        //cv::waitKey(30);

        cv::imshow("Recording calibration patterns", image);
        std::cout << "Image " << i << "/" << noImages << "             " << std::endl;

        std::stringstream str;
        str << path << "chessboard" << std::setw(2) << std::setfill('0') << i << ".jpg";
        imwrite(str.str(), image);
        cv::waitKey(30);
        //cam.printCameraProperties();
    }
    cam.stopCamera();
    cam.disconnect();
    cv::destroyWindow("Recording calibration patterns");
}
std::vector<std::string> Calibrator::loadStills(std::string path, int noImages)
{
    cv::namedWindow("Loading calibration patterns");
    cv::Mat image;
    std::vector<std::string> filelist;
    for (int i = 1; i <= noImages; i++)
    {
        std::cout << "Loading image " << i << "/" << noImages << '\xd';
        std::stringstream str;
        str << "indata/chessboards/chessboard" << std::setw(2) << std::setfill('0') << i << ".jpg";
        //std::cout << str.str() << std::endl;

        filelist.push_back(str.str());
        image = cv::imread(str.str(), 0);
        cv::imshow("Loading calibration patterns", image);
        //std::cout << "size: " << image.cols << "x" << image.rows << std::endl;
        cv::waitKey(100);
    }
    cv::destroyWindow("Loading calibration patterns");
    return filelist;
}

void Calibrator::saveMatrix(const std::string& path, cv::Mat& mat) // Not used anymore!
{

    if (!mat.empty())
    {
        std::cout << "Saving " << path << std::endl;
        std::ofstream matFile;
        matFile.open(path);
        for (int j = 0; j < mat.rows; j++)
        {
            for (int i = 0; i < mat.cols; i++)
            {
                matFile << mat.at<double>(j, i) << "	";
                //std::cout << "At [" << i << ", " << j << "] = " << mat.at<double>(j, i) << std::endl;
            }
            matFile << "\n";
        }
        matFile.close();
    }
    else
    {
        std::cout << "Calibrator error: Atempt at saving empty matrix " << path << std::endl;
    }
}

void Calibrator::updateCorners(void)
{
    // Load and undistort one image for projection calibration.
    PGRCamera cam;
    RawData data;
    cam.connect();
    cam.startCamera();

    cam.grabImage(&data);
    cv::Mat image = cv::Mat(data.rows, data.cols, CV_8UC1, data.pData, cv::Mat::AUTO_STEP);
    cv::imwrite("outdata/track.jpg", image);
    image.convertTo(image, -1, 2, 1); // Brighten for visibility
    cvtColor(image, image, CV_GRAY2RGB); // Color to get red text visible
    cv::waitKey(30); // Give the camera some time before quitting
    cam.stopCamera();
    cam.disconnect();

    cv::namedWindow("Detecting corners", CV_WINDOW_NORMAL); // so openCV does not distort pixel values!!!111
    //cv::namedWindow("Detecting corners", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("Detecting corners", 0, 0);
    cv::setMouseCallback("Detecting corners", mouseWrapper, (void*)this);

    // To remove borders
    /*
    cvSetWindowProperty("photo", CV_WINDOW_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    HWND win_handle = FindWindow(0, L"Detecting corners");
    SetWindowLong(win_handle, GWL_STYLE, GetWindowLong(win_handle, GWL_EXSTYLE) | WS_EX_TOPMOST);
    ShowWindow(win_handle, SW_SHOW);
    */

    while (pointCounter < 8)
    {
        cv::Mat tmp;
        image.copyTo(tmp);
        std::ostringstream pos;
        pos << "pos (ppx) = [" << currPos[0] << ", " << currPos[1] << "]";
        std::string s = pos.str();
        cv::putText(tmp, s, cvPoint(30, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(20, 20, 200), 1, CV_AA);
        cv::imshow("Detecting corners", tmp);
        cv::waitKey(30);
    }
    cv::destroyWindow("Detecting corners");
}

void Calibrator::mouseWrapper(int event, int x, int y, int flags, void *param)
{
    Calibrator *self = static_cast<Calibrator*>(param);
    self->trackMouse(event, x, y, flags);
}

void Calibrator::trackMouse(int event, int x, int y, int flags)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        std::cout << "point " << pointCounter / 2 + 1 << " @ [" << x << ", " << y << "]" << std::endl;
        cornerPoints[pointCounter] = x;
        cornerPoints[pointCounter + 1] = y;
        pointCounter = pointCounter + 2;
    }
    else if (event == cv::EVENT_MOUSEMOVE)
    {
        //std::cout << "MouseOver - pos [" << x << ", " << y << "]" << '\xd';
        currPos[0] = x;
        currPos[1] = y;
    }
}

void Calibrator::calculateCameraToWorldMatrix(cv::Mat mapImage, int* corners)
{
    float pPhotoSorted[8];
    polarSort(corners, pPhotoSorted);

    float pMap[8] = { 0, mapImage.rows, 0, 0, mapImage.cols, 0, mapImage.cols, mapImage.rows };

    std::cout << std::endl << "Desired mapping: " << std::endl;
    for (int i = 0; i < 8; i = i + 2)
    {
        std::cout << "[" << pPhotoSorted[i] << ", " << pPhotoSorted[i + 1] << "] <-> [" << pMap[i] << ", " << pMap[i + 1] << "]" << std::endl;
    }

    Eigen::Matrix3f A = basicMap(pPhotoSorted);
    Eigen::Matrix3f B = basicMap(pMap);
    Eigen::Matrix3f C = B*A.inverse();
    cv::eigen2cv(C, cameraToWorldMatrix);
}

void Calibrator::calculateWorldToCameraMatrix(cv::Mat mapImage, int* corners)
{
    float pPhotoSorted[8];
    polarSort(corners, pPhotoSorted);

    float pMap[8] = { 0, mapImage.rows, 0, 0, mapImage.cols, 0, mapImage.cols, mapImage.rows };

    std::cout << std::endl << "Desired mapping: " << std::endl;
    for (int i = 0; i < 8; i = i + 2)
    {
        std::cout << "[" << pPhotoSorted[i] << ", " << pPhotoSorted[i + 1] << "] <-> [" << pMap[i] << ", " << pMap[i + 1] << "]" << std::endl;
    }

    Eigen::Matrix3f A = basicMap(pMap);
    Eigen::Matrix3f B = basicMap(pPhotoSorted);
    Eigen::Matrix3f C = B*A.inverse();
    cv::eigen2cv(C, worldToCameraMatrix);
}

void Calibrator::polarSort(const int corners[8], float pPhotoSorted[8])
{
    // Find center of measured points
    float center[2] = { 0., 0. };
    for (int i = 0; i < 8; i = i + 2)
    {
        center[0] = center[0] + (float)corners[i];
        center[1] = center[1] + (float)corners[i + 1];
    }
    center[0] = center[0] / 4;
    center[1] = center[1] / 4;
    std::cout << "Center determined at: [" << center[0] << ", " << center[1] << "]" << std::endl;

    // Sort measured points clockwise
    float verticeA[2], verticeB[2];
    int index, sortedIndex[4];
    for (int i = 0; i < 8; i = i + 2)
    {
        index = 0;
        verticeA[0] = (float)corners[i];
        verticeA[1] = (float)corners[i + 1];
        for (int j = 0; j < 8; j = j + 2)
        {
            verticeB[0] = (float)corners[j];
            verticeB[1] = (float)corners[j + 1];
            if (j != i && leftOf(verticeA, verticeB, center))
            {
                index = index + 1;
            }
        }
        sortedIndex[i / 2] = index;
        //sortedIndex[i+1] = index;
    }
    for (int i = 0; i < 8; i = i + 2)
    {
        std::cout << "Point: [" << corners[i] << ", " << corners[i + 1] << "] has place [" << sortedIndex[i / 2] << "]" << std::endl;
    }
    std::cout << std::endl;
    //float pPhotoSorted[8];

    // Create sorted array
    for (int i = 0; i < 8; i = i + 2)
    {
        pPhotoSorted[sortedIndex[i / 2] * 2] = corners[i];
        pPhotoSorted[sortedIndex[i / 2] * 2 + 1] = corners[i + 1];
        std::cout << "Sorting index: [" << sortedIndex[i / 2] * 2 << ", " << sortedIndex[i / 2] * 2 + 1 << "] to [" << i << ", " << i + 1 << "]" << std::endl;
    }
}

bool Calibrator::leftOf(const float a[2], const float b[2], const float center[2])
{
    if (a[0] - center[0] >= 0 && b[0] - center[0] < 0)
        return true;
    if (a[0] - center[0] < 0 && b[0] - center[0] >= 0)
        return false;
    if (a[0] - center[0] == 0 && b[0] - center[0] == 0) {
        if (a[1] - center[1] >= 0 || b[1] - center[1] >= 0)
            return a[1] > b[1];
        return b[1] > a[1];
    }

    // compute the cross product of vectors (center -> a) x (center -> b)
    int det = (a[0] - center[0]) * (b[1] - center[1]) - (b[0] - center[0]) * (a[1] - center[1]);
    if (det < 0)
        return true;
    if (det > 0)
        return false;

    // points a and b are on the same line from the center
    // check which point is closer to the center
    int d1 = (a[0] - center[0]) * (a[0] - center[0]) + (a[1] - center[1]) * (a[1] - center[1]);
    int d2 = (b[0] - center[0]) * (b[0] - center[0]) + (b[1] - center[1]) * (b[1] - center[1]);
    return d1 > d2;
}

Eigen::Matrix3f Calibrator::basicMap(float p[8])
{
    Eigen::Matrix3f M;
    M << p[0], p[2], p[4],
        p[1], p[3], p[5],
        1, 1, 1;
    Eigen::Vector3f v;
    v << p[6], p[7], 1;

    Eigen::Vector3f x = M.colPivHouseholderQr().solve(v);
    //Eigen::Vector3f x = M.inverse()*v; // gives the same result as colPivHouseholderQr
    Eigen::Matrix3f A;
    A << x[0] * p[0], x[1] * p[2], x[2] * p[4],
        x[0] * p[1], x[1] * p[3], x[2] * p[5],
        x[0], x[1], x[2];
    //Eigen::VectorXd V2 = M.inverse()*V;
    return A;
}

void Calibrator::cameraToWorldCoordinates(float cameraPoint[2], float worldPoint[2])
{
    float nonLinearUndistorted[2];
    nonLinearUndistort(cameraPoint, nonLinearUndistorted);

    Eigen::Vector3f undistCameraPoint;
    //mapPoint << worldPoint[0], worldPoint[1], 1;
    undistCameraPoint << nonLinearUndistorted[0], nonLinearUndistorted[1], 1;

    Eigen::Vector3f worldPoint_Eigen = cameraToWorldMatrix_Eigen*undistCameraPoint;
    worldPoint_Eigen = worldPoint_Eigen / worldPoint_Eigen[2];
    worldPoint[0] = worldPoint_Eigen[0];
    worldPoint[1] = worldPoint_Eigen[1];
}
void Calibrator::worldToCameraCoordinates(float worldPoint[2], float cameraPoint[2])
{
    // Note that worldToCameraCoordinates does NOT utilize the non-linear transformation distCoeffs
    // In order to use the same precision in transforming to camera coordinates to polynomial described
    // by distCoeffs would need to be solved (somewhat precise) by some sort of Calibrator::nonLinearDistort()
    /*
    Eigen::Vector3f worldPoint_Eigen;
    //mapPoint << worldPoint[0], worldPoint[1], 1;
    worldPoint_Eigen << worldPoint[0], worldPoint[1], 1;

    Eigen::Vector3f cameraPoint_Eigen = worldToCameraMatrix_Eigen*worldPoint_Eigen;
    cameraPoint_Eigen = cameraPoint_Eigen / cameraPoint_Eigen[2];
    cameraPoint[0] = cameraPoint_Eigen[0];
    cameraPoint[1] = cameraPoint_Eigen[1];
    */

    Eigen::Vector3f undistCameraPoint;
    //mapPoint << worldPoint[0], worldPoint[1], 1;
    undistCameraPoint << worldPoint[0], worldPoint[1], 1;

    Eigen::Vector3f worldPoint_Eigen = worldToCameraMatrix_Eigen*undistCameraPoint;
    worldPoint_Eigen = worldPoint_Eigen / worldPoint_Eigen[2];
    cameraPoint[0] = worldPoint_Eigen[0];
    cameraPoint[1] = worldPoint_Eigen[1];

}
void Calibrator::nonLinearUndistort(float input[2], float output[2])
{
    // Following a somewhat standardized notation for distortion coefficients, the matrix elements are as follows:
    double k1 = distCoeffs.at<double>(0, 0);
    double k2 = distCoeffs.at<double>(0, 1);
    double p1 = distCoeffs.at<double>(0, 2);
    double p2 = distCoeffs.at<double>(0, 3);
    double k3 = distCoeffs.at<double>(0, 4);
    double  fx = cameraMatrix.at<double>(0, 0);
    double  cx = cameraMatrix.at<double>(0, 2);
    double  fy = cameraMatrix.at<double>(1, 1);
    double  cy = cameraMatrix.at<double>(1, 2);
    double z = 1.;

    double x = (input[0] - cx) / fx;
    double y = (input[1] - cy) / fy;
    double r2 = x*x + y*y;

    double dx = 2 * p1*x*y + p2*(r2 + 2 * x*x);
    double dy = p1*(r2 + 2 * y*y) + 2 * p2*x*y;
    double scale = (1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);

    double xBis = x*scale + dx;
    double yBis = y*scale + dy;

    output[0] = xBis*fx + cx;
    output[1] = yBis*fy + cy;
}
