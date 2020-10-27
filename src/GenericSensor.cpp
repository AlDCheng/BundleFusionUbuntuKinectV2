
#include "stdafx.h"

#include "GenericSensor.h"

// Assume rgb and depth same resolution


GenericSensor::GenericSensor()
{
    m_bDepthReceived = false;
    m_bColorReceived = false;

    m_bDepthImageIsUpdated = false;
    m_bDepthImageCameraIsUpdated = false;
    m_bNormalImageCameraIsUpdated = false;
}

GenericSensor::~GenericSensor()
{
}

void GenericSensor::createFirstConnected()
{

    RGBDSensor::init (  GlobalAppState::get().s_rawDepthWidth, 
                        GlobalAppState::get().s_rawDepthHeight, 
                        GlobalAppState::get().s_rawDepthWidth,
                        GlobalAppState::get().s_rawDepthHeight, 
                        1 );
    initializeDepthIntrinsics ( GlobalAppState::get().s_cameraIntrinsicFx,
                                GlobalAppState::get().s_cameraIntrinsicFy,
                                GlobalAppState::get().s_cameraIntrinsicCx,
                                GlobalAppState::get().s_cameraIntrinsicCy );

    initializeColorIntrinsics ( GlobalAppState::get().s_cameraIntrinsicFx,
                                GlobalAppState::get().s_cameraIntrinsicFy,
                                GlobalAppState::get().s_cameraIntrinsicCx,
                                GlobalAppState::get().s_cameraIntrinsicCy );

    initializeColorExtrinsics ( mat4f::identity() );
    initializeDepthExtrinsics ( mat4f::identity() );


}

bool GenericSensor::receiveDepthAndColor ( cv::Mat& rgb, cv::Mat& depth )
{
    bool hr = true;

    m_bDepthImageIsUpdated = false;
    m_bDepthImageCameraIsUpdated = false;
    m_bNormalImageCameraIsUpdated = false;

    hr = readDepthAndColor ( rgb, depth, getDepthFloat(), m_colorRGBX );

    m_bDepthImageIsUpdated = true;
    m_bDepthImageCameraIsUpdated = true;
    m_bNormalImageCameraIsUpdated = true;

    m_bDepthReceived = true;
    m_bColorReceived = true;

    return hr;
}


bool GenericSensor::processDepth()
{

    bool hr = true;
    return hr;
}



bool GenericSensor::readDepthAndColor ( const cv::Mat& rgbTest, const cv::Mat& depthTest,float* depthFloat, vec4uc* colorRGBX )
{
    bool hr = true;

    if ( rgbTest.empty() )
    {
        std::cout << "no rgb!" << std::endl;
        hr = false;
    }
    if ( depthTest.empty() )
    {
        std::cout << "no depth!" << std::endl;
        hr = false;
    }

    if ( rgbTest.empty() || depthTest.empty() )
    {
        return false;
    }

    const uint16_t* pDepth = ( const uint16_t* ) depthTest.data;
    const uint8_t* pImage = ( const uint8_t* ) rgbTest.data;

    if ( !depthTest.empty() && !rgbTest.empty() )
    {
        unsigned int width = depthTest.cols;
        unsigned int nPixels = depthTest.cols * depthTest.rows;

        for ( unsigned int i = 0; i < nPixels; i++ )
        {
            const int x = i%width;
            const int y = i / width;
            const int src = y*width + ( width - 1 - x );
            const uint16_t& p = pDepth[src];

            float dF = ( float ) p*0.001f;
            if ( dF >= GlobalAppState::get().s_sensorDepthMin && dF <= GlobalAppState::get().s_sensorDepthMax )
            {
                depthFloat[i] = dF;
            }
            else
            {
                depthFloat[i] = -std::numeric_limits<float>::infinity();
            }
        }
        incrementRingbufIdx();
    }

    // check if we need to draw depth frame to texture
    //if (m_depthFrame.isValid() && m_colorFrame.isValid())
    if ( !depthTest.empty() && !rgbTest.empty() )
    {
        //unsigned int width = m_colorFrame.getWidth();
        //unsigned int height = m_colorFrame.getHeight();
        unsigned int width = rgbTest.cols;
        unsigned int height = rgbTest.rows;
        unsigned int nPixels = width*height;

        for ( unsigned int i = 0; i < nPixels; i++ )
        {
            const int x = i%width;
            const int y = i / width;

            if ( y >= 0 && y < ( int ) height )
            {
                unsigned int Index1D = y*width + ( width - 1 - x );	//x-flip here

                //const openni::RGB888Pixel& pixel = pImage[Index1D];

                unsigned int c = 0;
                c |= pImage[3*Index1D + 0];
                c <<= 8;
                c |= pImage[3*Index1D + 1];
                c <<= 8;
                c |= pImage[3*Index1D + 2];
                c |= 0xFF000000;

                ( ( LONG* ) colorRGBX ) [y*width + x] = c;
            }
        }
    }


    return hr;
}