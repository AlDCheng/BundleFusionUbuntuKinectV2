#pragma once

/************************************************************************/
/* Prime sense depth camera: Warning this is highly untested atm        */
/************************************************************************/

#include "GlobalAppState.h"

//Only working with OpenNI 2 SDK
#ifdef OPEN_NI

#include "RGBDSensor.h"
//#include <OpenNI.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>
#include <list>

class GenericSensor : public RGBDSensor
{
public:

	//! Constructor; allocates CPU memory and creates handles
	GenericSensor();

	//! Destructor; releases allocated ressources
	~GenericSensor();

	//! Initializes the sensor
	void createFirstConnected();

	//! Processes the depth data (and color)
	bool processDepth();
	
	bool receiveDepthAndColor(cv::Mat& rgb, cv::Mat& depth);
	
	std::string getSensorName() const {
		return "orbbec";
	}
	//! Processes the Kinect color data
	bool processColor()
	{
		return true;
	}

protected:
	//! reads depth and color from the sensor
	bool readDepthAndColor(const cv::Mat& rgbTest, const cv::Mat& depthTest,float* depthFloat, vec4uc* colorRGBX);

	int				m_picNum = 30;//  30;//4000;//6000;//11000;//6830;
	// to prevent drawing until we have data for both streams
	bool			m_bDepthReceived;
	bool			m_bColorReceived;

	bool			m_bDepthImageIsUpdated;
	bool			m_bDepthImageCameraIsUpdated;
	bool			m_bNormalImageCameraIsUpdated;

	bool			m_kinect4Windows;
};

#endif
