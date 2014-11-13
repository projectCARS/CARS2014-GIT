#pragma once

namespace Bilbana
{
	bool fileExists(const std::string& file);
	void drawStatesToImg(Eigen::MatrixXf carPattern, float *posX, float *posY, float *yaw, cv::Mat img, int selection);
	void drawCar(Eigen::MatrixXf carPattern, float x, float y, float yaw, cv::Mat img);
}
