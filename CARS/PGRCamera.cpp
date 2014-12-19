#include "headers.h"
#include "definitions.h"
#include "PGRCamera.h"

// Constructor.
PGRCamera::PGRCamera()
{
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
    // Setup camera, especially needed when computer is restarted
    setup();

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
	std::cout << "Class PGRCamera:	printing camera stats" << std::endl;

	printCameraProperty(FlyCapture2::PropertyType::SHUTTER, value);
	// Print property value.
	std::cout << "	Shutter: " << value << std::endl;

	printCameraProperty(FlyCapture2::PropertyType::BRIGHTNESS, value);
	// Print property value.
	std::cout << "	Brightness: " << value << std::endl;

	printCameraProperty(FlyCapture2::PropertyType::GAIN, value);
	// Print property value.
	std::cout << "	Gain: " << value << std::endl;

	printCameraProperty(FlyCapture2::PropertyType::AUTO_EXPOSURE, value);
	// Print property value.
	std::cout << "	Auto exposure: " << value << std::endl;

	printCameraProperty(FlyCapture2::PropertyType::SHARPNESS, value);
	// Print property value.
	std::cout << "	Sharpness: " << value << std::endl;

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
	std::cout << "	Frame rate: " << value << std::endl;
}

// Camera setup. Not fully implemented.
void PGRCamera::setup(void)
{
    FlyCapture2::Error error;

    bool valid;
    FlyCapture2::Format7PacketInfo info;

    FlyCapture2::Format7ImageSettings imageSettings;
    unsigned int packageSize = 24764;

    imageSettings.mode = FlyCapture2::Mode::MODE_0;
    imageSettings.width = 1280;
    imageSettings.height = 1024;
    imageSettings.offsetX = 0;
    imageSettings.offsetY = 0;
    imageSettings.pixelFormat = FlyCapture2::PixelFormat::PIXEL_FORMAT_RAW8;

    error = m_cam.ValidateFormat7Settings(&imageSettings, &valid, &info);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        printError(error);
    }

    if (valid)
    {
        std::cout << "First time settings are valid" << std::endl;
        std::cout << "Max bytes per packet: " << info.maxBytesPerPacket << std::endl;
    }
    else
    {
        std::cout << "First time settings are invaild" << std::endl;
        std::cout << "Recommended bytes per packet: " << info.recommendedBytesPerPacket << std::endl;
        std::cout << "Max bytes per packet: " << info.maxBytesPerPacket << std::endl;
        std::cout << "Unit bytes per packet: " << info.unitBytesPerPacket << std::endl;
    }

    error = m_cam.SetFormat7Configuration(&imageSettings, packageSize);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        std::cout << "Failed at first attempt to set image settings and package size" << std::endl;
        printError(error);
    }

    error = m_cam.ValidateFormat7Settings(&imageSettings, &valid, &info);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        printError(error);
    }

    if (valid)
    {
        std::cout << "Second time settings are vaild" << std::endl;
        std::cout << "Max bytes per packet: " << info.maxBytesPerPacket << std::endl;
    }
    else
    {
        std::cout << "Second time settings are invaild" << std::endl;
        std::cout << "Recommended bytes per packet: " << info.recommendedBytesPerPacket << std::endl;
        std::cout << "Max bytes per packet: " << info.maxBytesPerPacket << std::endl;
        std::cout << "Unit bytes per packet: " << info.unitBytesPerPacket << std::endl;
    }

    error = m_cam.SetFormat7Configuration(&imageSettings, packageSize);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        std::cout << "Failed at second attempt to set image settings and package size" << std::endl;
        printError(error);
    }

    FlyCapture2::Property frame_rate;
    frame_rate.type = FlyCapture2::PropertyType::FRAME_RATE;
    frame_rate.absControl = true;
    frame_rate.absValue = 150;
    frame_rate.autoManualMode = false;
    frame_rate.onOff = true;
    error = m_cam.SetProperty(&frame_rate);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        std::cout << "Error when setting frame rate." << std::endl;
        printError(error);
    }

    FlyCapture2::Property shutter;
    memset(&shutter, 0, sizeof(shutter));
    shutter.type = FlyCapture2::PropertyType::SHUTTER;
    shutter.absControl = false;
    //shutter.absValue = 6.61188;
    shutter.onOff = true;
    shutter.autoManualMode = true;
    error = m_cam.SetProperty(&shutter);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        std::cout << "Error when setting shutter time." << std::endl;
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
