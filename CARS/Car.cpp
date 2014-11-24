#include "headers.h"
#include "Car.h"
#include "definitions.h"
#include "classes.h"

#include <QDebug>

Car::Car(int id, CarMode::Enum mode, FilterType::Enum filterType, MotionModelType::Enum motionModelType)
{
	// Assign car attributes.
	m_id = id;
	m_active = false;
	m_lost = false;
	m_mode = mode;
    m_filtertype = filterType;


    Eigen::MatrixXf carPattern;
    cv::FileStorage storage("indata/car01.yml", cv::FileStorage::READ);
    cv::Mat tmp;
    storage["pattern"] >> tmp;
    storage.release();
    cv::cv2eigen(tmp, carPattern);
    drawThreadData.pattern = carPattern;
    float speed = 10;

	// Initialize filter.
	switch (filterType)
	{
		case FilterType::EKF:
            m_filter = new EKF(motionModelType);
			break;
        case FilterType::ParticleFilter:
        {
            //TODO make this id-dependent
            m_filter = new ParticleFilter(carPattern, speed, 5, motionModelType);
        }
            break;
        case FilterType::NoFilter:
            m_filter = new NoFilter();
            break;
		default:
			std::cout << "Error: Filter type not implemented, in Car::Car(), Car.cpp" << std::endl;
	}
}

Car::~Car()
{
    // Already deleted?
    //delete m_filter;
}

// Add measurement to filter.
void Car::addMeasurement(float x, float y, float theta)
{
	if (m_active)
	{
		m_lost = false;
		m_inactiveTimer = 0;
        m_filter->addMeasurement(x, y, theta);

	}
	else
	{
		m_activeTimer++;
		m_numberOfFalseDetections++;
		if (m_activeTimer == m_activeTimerMax)
		{
			m_active = true;
			m_activeTimer = 0;
			m_numberOfFalseDetections -= m_activeTimerMax;
            m_filter->addMeasurement(x, y, theta);
		}
	}
}

void Car::addImageMeasurement(cv::Mat img)
{
    if (m_active)
    {
        m_lost = false;
        m_inactiveTimer = 0;
        m_filter->addImageMeasurement(img);

    }
    else
    {
        m_activeTimer++;
        m_numberOfFalseDetections++;
        if (m_activeTimer == m_activeTimerMax)
        {
            m_active = true;
            m_activeTimer = 0;
            m_numberOfFalseDetections -= m_activeTimerMax;
            m_filter->addImageMeasurement(img);
        }
    }
}

void Car::addInputSignals(float gas, float turn){
    m_filter->addInputSignals(gas, turn);
}

// Update the filter. 
void Car::updateFilter()
{
	// Only update the filter if the car is active. 
	if (m_active)
	{
		// Tick timer if the car is not detected.
		if (!m_filter->hasNewMeasurement())
		{
			tickTimer();
		}
		// Update the filter.
		m_filter->updateFilter();
	}
}

void Car::tickTimer()
{
	m_inactiveTimer++;
	m_lost = true;
	m_numberOfLosses++;
	if (m_inactiveTimer == m_inactiveTimerMax)
	{
		m_active = false;
		m_lost = false;
		m_numberOfLosses -= m_inactiveTimerMax;
	}
}
