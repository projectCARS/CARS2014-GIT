#include "stdafx.h"
#include "definitions.h"

namespace Bilbana
{
	bool fileExists(const std::string& file)
	{
		struct stat buf;
		return (stat(file.c_str(), &buf) == 0);
	}

	void drawStatesToImg(Eigen::MatrixXf carPattern, float *posX, float *posY, float *yaw, cv::Mat img, int selection)
	{
		int points = carPattern.rows();
		Eigen::Matrix2f rotate, translate;
		Eigen::MatrixXf pos;
		Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(2, points);
		switch (selection)
		{
		case 1:
		{
			for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
			{
				rotate << cos(yaw[i]), -sin(yaw[i]),
					sin(yaw[i]), cos(yaw[i]);
				translate << posX[i], 0,
					posY[i], 0;
				pos = rotate*carPattern.transpose() + translate*ones;

				for (int p = 0; p < points; p++)
				{
					if (pos(0, p) > 0 && pos(0, p) < img.cols && pos(1, p) > 0 && pos(1, p) < img.rows)
					{
						cv::circle(img, cv::Point(pos(0, p), pos(1, p)), 1.5, cv::Scalar(255, 150, 150), 1, 8, 0);
					}
				}
			}
		}
			break;
		case 2:
		{
			for (int i = 0; i < NUMBER_OF_PARTICLES; i++)
			{
				rotate << cos(yaw[i]), -sin(yaw[i]),
					sin(yaw[i]), cos(yaw[i]);
				translate << posX[i], 0,
					posY[i], 0;
				pos = rotate*carPattern.transpose() + translate*ones;

				for (int p = 0; p < points; p++)
				{
					if (pos(0, p) > 0 && pos(0, p) < img.cols && pos(1, p) > 0 && pos(1, p) < img.rows)
					{
						cv::circle(img, cv::Point(pos(0, p), pos(1, p)), 4, cv::Scalar(0, 0, 255), -1, 8, 0);
					}
				}
			}
		}
			break;
		}
	}

	void drawCar(Eigen::MatrixXf carPattern, float x, float y, float yaw, cv::Mat img)
	{
		int points = carPattern.rows();
		Eigen::Matrix2f rotate, translate;
		Eigen::MatrixXf pos;
		Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(2, points);


		rotate << cos(yaw), -sin(yaw),
			sin(yaw), cos(yaw);
		translate << x, 0,
			y, 0;
		pos = rotate*carPattern.transpose() + translate*ones;

		for (int p = 0; p < points; p++)
		{
			if (pos(0, p) > 0 && pos(0, p) < img.cols && pos(1, p) > 0 && pos(1, p) < img.rows)
			{
				cv::circle(img, cv::Point(pos(0, p), pos(1, p)), 4, cv::Scalar(255, 0, 0), -1, 8, 0);
			}
		}



	}
}