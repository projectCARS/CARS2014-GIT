#include "stdafx.h"
#include "definitions.h"
#include "PGRCamera.h"

namespace Bilbana
{
	// Constructor.
	PGRCamera::PGRCamera()
	{
		m_shutter = 30;
		//m_autoExposure = false;
		m_exposure = -3.5;
		m_gain = 10.5;
		m_sharpness = 1500;
	}

	// Destructor.
	PGRCamera::~PGRCamera()
	{
	}

	// Connect the camera.
	void PGRCamera::connect()
	{
		FlyCapture2::Error error;

		// Exactly one camera is assumed to be connected.
		error = m_busMgr.GetCameraFromIndex(0, &m_guid);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
			// throw error;
		}

		error = m_cam.Connect(&m_guid);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}

		// Print some camera properties.
		printCameraProperties();
	}

	// Disconnect the camera.
	void PGRCamera::disconnect()
	{
		FlyCapture2::Error error;

		error = m_cam.Disconnect();
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}
	}

	// Start capturing images.
	void PGRCamera::startCamera(void)
	{
		FlyCapture2::Error error;

		error = m_cam.StartCapture();
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}
	}

	// Stop capturing.
	void PGRCamera::stopCamera(void)
	{
		FlyCapture2::Error error;

		error = m_cam.StopCapture();
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}
	}

	void PGRCamera::printCameraProperty(FlyCapture2::PropertyType propertyType, float &value)
	{
		FlyCapture2::Error error;
		FlyCapture2::Property prop;

		// Set property fields to zero.
		memset(&prop, 0, sizeof(prop));
		prop.type = propertyType;
		error = m_cam.GetProperty(&prop);

		value = prop.absValue;
	}

	void PGRCamera::printCameraProperties(void)
	{
		float value;

		printCameraProperty(FlyCapture2::PropertyType::SHUTTER, value);
		// Print property value.
		std::cout << "Shutter: " << value << std::endl;

		printCameraProperty(FlyCapture2::PropertyType::BRIGHTNESS, value);
		// Print property value.
		std::cout << "Brightness: " << value << std::endl;

		printCameraProperty(FlyCapture2::PropertyType::GAIN, value);
		// Print property value.
		std::cout << "Gain: " << value << std::endl;

		printCameraProperty(FlyCapture2::PropertyType::AUTO_EXPOSURE, value);
		// Print property value.
		std::cout << "Auto exposure: " << value << std::endl;

		printCameraProperty(FlyCapture2::PropertyType::SHARPNESS, value);
		// Print property value.
		std::cout << "Sharpness: " << value << std::endl;
		//printCameraProperty(FlyCapture2::PropertyType::TRIGGER_MODE, value);
		// Print property value.
		//std::cout << "Trigger mode: " << value << std::endl;

		//printCameraProperty(FlyCapture2::PropertyType::TRIGGER_DELAY, value);
		// Print property value.
		//std::cout << "Trigger delay: " << value << std::endl;

		//printCameraProperty(FlyCapture2::PropertyType::TEMPERATURE, value);
		// Print property value.
		//std::cout << "Temperature: " << value << std::endl;

		printCameraProperty(FlyCapture2::PropertyType::FRAME_RATE, value);
		// Print property value.
		std::cout << "Frame rate: " << value << std::endl;
	}

	// Camera setup. Not fully implemented.
	void PGRCamera::setup(void)
	{
		FlyCapture2::Error error;

		// Specify exposure value.
		FlyCapture2::Property exp;
		memset(&exp, 0, sizeof(exp));
		exp.type = FlyCapture2::AUTO_EXPOSURE;
		exp.onOff = true;

		exp.autoManualMode = false;
		exp.absControl = true;
		exp.absValue = m_exposure;
		error = m_cam.SetProperty(&exp);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}

		// Specify shutter time.
		FlyCapture2::Property shut;
		memset(&shut, 0, sizeof(shut));
		shut.type = FlyCapture2::SHUTTER;
		shut.onOff = true;

		shut.autoManualMode = false;
		shut.absControl = true;
		shut.absValue = m_shutter;
		
		error = m_cam.SetProperty(&shut);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}

		// Specify gain.
		FlyCapture2::Property gain;
		memset(&gain, 0, sizeof(gain));
		gain.type = FlyCapture2::GAIN;
		gain.autoManualMode = false;
		gain.absControl = true;

		gain.absValue = m_gain;
		error = m_cam.SetProperty(&gain);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}

		// Specify sharpness
		FlyCapture2::Property sharp;
		sharp.type = FlyCapture2::SHARPNESS;
		sharp.onOff = true;
		sharp.autoManualMode = false;
		sharp.valueA = m_sharpness;
		error = m_cam.SetProperty(&sharp);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}
	}

	// Grabs an image from the camera. The image is stored in m_processedImage.
	void PGRCamera::grabImage(void)
	{
		FlyCapture2::Error error;
		FlyCapture2::Image buffImage;

		error = m_cam.RetrieveBuffer(&buffImage);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}

		// TODO: lock m_csRawImage?
		m_rawImage = buffImage;
		// Unlock.

		// Is it really necessary to convert the image?
		error = m_rawImage.Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &m_processedImage);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}
	}

	// Does the same as grabImage(), but also writes image info
	// to the struct rawData.
	void PGRCamera::grabImage(RawData *rawData)
	{
		FlyCapture2::Error error;
		FlyCapture2::Image buffImage;

		error = m_cam.RetrieveBuffer(&buffImage);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}

		// TODO: lock m_csRawImage?
		m_rawImage = buffImage;
		// Unlock.

		// Is it really necessary to convert the image?
		error = m_rawImage.Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &m_processedImage);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}

		// Save information to struct.
		rawData->cols = m_processedImage.GetCols();
		rawData->rows = m_processedImage.GetRows();
		rawData->dataSize = m_processedImage.GetDataSize();
		rawData->pData = m_processedImage.GetData();
		//rawData->dataSize = m_processedImage.GetReceivedDataSize();
	}

	// Saves the image m_processedImage to a file. 
	// Supported file extensions: PGM, PPM, BMP,
	// JPEG, JPEG2000, TIFF, PNG, RAW.
	void PGRCamera::saveImage(const char *name)
	{
		FlyCapture2::Error error;

		error = m_processedImage.Save(name);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}
	}
	
	// Grabs an image and saves to a file. 
	// Supported file extensions: PGM, PPM, BMP,
	// JPEG, JPEG2000, TIFF, PNG, RAW.
	void PGRCamera::grabAndSaveImage(const char *name)
	{
		FlyCapture2::Error error;

		// Grab image from camera.
		this->grabImage();

		error = m_processedImage.Save(name);
		if (error != FlyCapture2::PGRERROR_OK)
		{
			printError(error);
		}
	}

	// Help function to print error messages.
	void PGRCamera::printError(FlyCapture2::Error error)
	{
		error.PrintErrorTrace();
	}
}
