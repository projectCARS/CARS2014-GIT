#pragma once
#include "definitions.h"

class Calibrator
{
private:
    // ========== openCV specific camera calibration =======
    // input points
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    // output Matrices
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    // flag to specify how calibration is done
    int flag;
    // used in image undistortion
    cv::Mat map1, map2;
    // ========== for affine remapping of track map ========
    int pointCounter = 0, currPos[2], cornerPoints[8];
    cv::Mat cameraToWorldMatrix;
    Eigen::Matrix3f cameraToWorldMatrix_Eigen;
    cv::Mat worldToCameraMatrix;
    Eigen::Matrix3f worldToCameraMatrix_Eigen;
    void polarSort(const int pPhoto[8], float pPhotoSorted[8]);
    bool leftOf(const float a[2], const float b[2], const float center[2]);
    Eigen::Matrix3f basicMap(float p[8]);
    void nonLinearUndistort(float input[2], float output[2]);

public:
    Calibrator();
    ~Calibrator();

    // Start/stop camera and aquire calibration images if they are not already present
    void getStills();
    // Open the chessboard images and extract corner points
    int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize);
    // Add scene points and corresponding image points
    void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);
    // Calibrate the camera
    double calibrate(cv::Size &imageSize);
    // Set the calibration flag
    void setCalibrationFlag(bool radial8CoeffEnabled = false, bool tangentialParamEnabled = false);
    // Remove distortion in an image (after calibration)
    cv::Mat Calibrator::remap(const cv::Mat &image);
    // Remove distortion without using affine transformation coefficients
    cv::Mat Calibrator::undistort(const cv::Mat &image);


    void saveStills(std::string path, int noImages, float alpha, float beta);
    std::vector<std::string> loadStills(std::string path, int noImages);
    void saveMatrix(const std::string& path, cv::Mat& mat);
    cv::Mat loadMatrix(const std::string& path, cv::Size& size);

    void updateCorners();
    static void mouseWrapper(int event, int x, int y, int flags, void* param);
    void trackMouse(int event, int x, int y, int flags);

    void calculateCameraToWorldMatrix(cv::Mat mapImage, int* corners);
    void calculateWorldToCameraMatrix(cv::Mat mapImage, int* corners);

    void cameraToWorldCoordinates(float cameraPoint[2], float worldPoint[2]);
    void worldToCameraCoordinates(float worldPoint[2], float cameraPoint[2]);

    // Getters
    cv::Mat getCameraMatrix() { return cameraMatrix; }
    cv::Mat getDistCoeffs()   { return distCoeffs; }
    int* getCornerPoints() { return cornerPoints; }
    cv::Mat getCameraToWorldMatrix() { return cameraToWorldMatrix; }
    cv::Mat getWorldToCameraMatrix() { return cameraToWorldMatrix; }

};
