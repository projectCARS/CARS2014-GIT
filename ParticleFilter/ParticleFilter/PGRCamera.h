#pragma once
#include "definitions.h"

namespace Bilbana
{
	class PGRCamera
	{
	private:
		FlyCapture2::Camera m_cam;
		FlyCapture2::PGRGuid m_guid;
		FlyCapture2::Property m_prop;
		FlyCapture2::BusManager m_busMgr;
		FlyCapture2::Image m_rawImage;
		FlyCapture2::Image m_processedImage;

		float m_shutter;
		//bool m_autoExposure;
		float m_exposure;
		float m_gain;
		int m_sharpness;

	public:
		PGRCamera();
		~PGRCamera();

		// TODO: implement function that controls shutter time.

		// Setup bus manager and connect to camera with index 0.
		void connect(void);
		void disconnect(void);
		void startCamera(void);
		void stopCamera(void);
		/* Print camera property. For example, printCameraPropery(FlyCapture2::SHUTTER) 
		prints information about the shutter property. */
		void printCameraProperty(FlyCapture2::PropertyType propertyType, float &value);
		// Prints some camera properties.
		void printCameraProperties(void);
		// Camera setup.
		// TODO: implement.
		void setup(void);
		// Grabs an image from the camera. The image is stored in m_processedImage.
		void grabImage(void);
		// Does the same as grabImage(), but also writes image info
		// to the struct rawData.
		void grabImage(RawData *rawData);
		// Saves the image m_processedImage to a file. 
		// Supported file extensions: PGM, PPM, BMP,
		// JPEG, JPEG2000, TIFF, PNG, RAW.
		void saveImage(const char *name);
		// Grabs an image and saves to a file. 
		// Supported file extensions: PGM, PPM, BMP,
		// JPEG, JPEG2000, TIFF, PNG, RAW.
		void grabAndSaveImage(const char *name);
		// Help function to print error messages.
		void printError(FlyCapture2::Error error);
	};
}




