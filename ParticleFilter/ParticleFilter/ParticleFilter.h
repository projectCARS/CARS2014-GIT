#pragma once
#include "definitions.h"

namespace Bilbana
{
	class ParticleFilter
	{
	private:
		float posX[NUMBER_OF_PARTICLES];
		float posY[NUMBER_OF_PARTICLES];
		float yaw[NUMBER_OF_PARTICLES];
		float posXPoints[NUMBER_OF_PARTICLES];
		float posYPoints[NUMBER_OF_PARTICLES];
		float yawPoints[NUMBER_OF_PARTICLES];
		float weights[NUMBER_OF_PARTICLES];
		float cumulativeWeights[NUMBER_OF_PARTICLES];
		float sumStates[3];

		Eigen::MatrixXf carPattern;
		float expectedSpeed;
		int limit;
		bool noCar;

		float gaussianNoise(void);
		int findFirst(const float value);

	public:
		ParticleFilter(Eigen::MatrixXf ID, float speed, int lim);
		~ParticleFilter();

		void setState(float state[3]);
		void extensiveSearch(cv::Mat img);
		void propagate(void);
		void update(const cv::Mat img);
		void resample(void);

		
		
		void logStates(std::ofstream *logFile);
		bool carGone(void){ return noCar; }

		void markCar(const float state[3], cv::Mat img);




		float *getPosX(void){ return posX; }
		float *getPosY(void){ return posY; }
		float *getYaw(void){ return yaw; };
		float *getPosXPoints(void){ return posXPoints; };
		float *getPosYPoints(void){ return posYPoints; };
		float *getYawPoints(void){ return yawPoints; };
		float *getSumStates(void){ return sumStates; };


	};
}