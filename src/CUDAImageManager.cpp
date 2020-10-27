
#include "stdafx.h"

#include "CUDAImageManager.h"

bool		CUDAImageManager::ManagedRGBDInputFrame::s_bIsOnGPU = false;
unsigned int CUDAImageManager::ManagedRGBDInputFrame::s_width = 0;
unsigned int CUDAImageManager::ManagedRGBDInputFrame::s_height = 0;

float*		CUDAImageManager::ManagedRGBDInputFrame::s_depthIntegrationGlobal = NULL;
uchar4*		CUDAImageManager::ManagedRGBDInputFrame::s_colorIntegrationGlobal = NULL;

CUDAImageManager::ManagedRGBDInputFrame* CUDAImageManager::ManagedRGBDInputFrame::s_activeColorGPU = NULL;
CUDAImageManager::ManagedRGBDInputFrame* CUDAImageManager::ManagedRGBDInputFrame::s_activeDepthGPU = NULL;

CUDAImageManager::ManagedRGBDInputFrame* CUDAImageManager::ManagedRGBDInputFrame::s_activeColorCPU = NULL;
CUDAImageManager::ManagedRGBDInputFrame* CUDAImageManager::ManagedRGBDInputFrame::s_activeDepthCPU = NULL;

Timer CUDAImageManager::s_timer;

bool CUDAImageManager::process ( cv::Mat& rgb, cv::Mat& depth )
{
    if ( !m_RGBDSensor->receiveDepthAndColor ( rgb,depth ) )
    {
        return false;
    }

    if ( m_currFrame + 1 > GlobalBundlingState::get().s_maxNumImages * GlobalBundlingState::get().s_submapSize )
    {
        std::cout << "WARNING: reached max #images, truncating sequence  num:" << m_currFrame << std::endl;
        return false;
    }

    if ( GlobalBundlingState::get().s_enableGlobalTimings )
    {
        TimingLog::addLocalFrameTiming();
        cudaDeviceSynchronize();
        s_timer.start();
    }

    m_data.push_back ( ManagedRGBDInputFrame() );
    ManagedRGBDInputFrame& frame = m_data.back();
    frame.alloc();

    const unsigned int bufferDimColorInput = m_RGBDSensor->getColorWidth() *m_RGBDSensor->getColorHeight();
    MLIB_CUDA_SAFE_CALL ( cudaMemcpy ( d_colorInput, m_RGBDSensor->getColorRGBX(), sizeof ( uchar4 ) *bufferDimColorInput, cudaMemcpyHostToDevice ) );

    if ( ( m_RGBDSensor->getColorWidth() == m_widthIntegration ) && ( m_RGBDSensor->getColorHeight() == m_heightIntegration ) )
    {
        if ( ManagedRGBDInputFrame::s_bIsOnGPU )
        {
            CUDAImageUtil::copy<uchar4> ( frame.m_colorIntegration, d_colorInput, m_widthIntegration, m_heightIntegration );
            //std::swap(frame.m_colorIntegration, d_colorInput);
        }
        else
        {
            memcpy ( frame.m_colorIntegration, m_RGBDSensor->getColorRGBX(), sizeof ( uchar4 ) *bufferDimColorInput );
        }
    }
    else
    {
        if ( ManagedRGBDInputFrame::s_bIsOnGPU )
        {
            CUDAImageUtil::resampleUCHAR4 ( frame.m_colorIntegration, m_widthIntegration, m_heightIntegration, d_colorInput, m_RGBDSensor->getColorWidth(), m_RGBDSensor->getColorHeight() );
        }
        else
        {
            CUDAImageUtil::resampleUCHAR4 ( frame.s_colorIntegrationGlobal, m_widthIntegration, m_heightIntegration, d_colorInput, m_RGBDSensor->getColorWidth(), m_RGBDSensor->getColorHeight() );
            MLIB_CUDA_SAFE_CALL ( cudaMemcpy ( frame.m_colorIntegration, frame.s_colorIntegrationGlobal, sizeof ( uchar4 ) *m_widthIntegration*m_heightIntegration, cudaMemcpyDeviceToHost ) );
            frame.s_activeColorGPU = &frame;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // Process Depth
    ////////////////////////////////////////////////////////////////////////////////////

    const unsigned int bufferDimDepthInput = m_RGBDSensor->getDepthWidth() *m_RGBDSensor->getDepthHeight();
    MLIB_CUDA_SAFE_CALL ( cudaMemcpy ( d_depthInputRaw, m_RGBDSensor->getDepthFloat(), sizeof ( float ) *m_RGBDSensor->getDepthWidth() * m_RGBDSensor->getDepthHeight(), cudaMemcpyHostToDevice ) );


    if ( GlobalBundlingState::get().s_erodeSIFTdepth )
    {
        unsigned int numIter = 2;
        numIter = 2 * ( ( numIter + 1 ) / 2 );
        for ( unsigned int i = 0; i < numIter; i++ )
        {
            if ( i % 2 == 0 )
            {
                CUDAImageUtil::erodeDepthMap ( d_depthInputFiltered, d_depthInputRaw, 3,
                                               m_RGBDSensor->getDepthWidth(), m_RGBDSensor->getDepthHeight(), 0.05f, 0.3f );
            }
            else
            {
                CUDAImageUtil::erodeDepthMap ( d_depthInputRaw, d_depthInputFiltered, 3,
                                               m_RGBDSensor->getDepthWidth(), m_RGBDSensor->getDepthHeight(), 0.05f, 0.3f );
            }
        }
    }
    if ( GlobalBundlingState::get().s_depthFilter ) //smooth
    {
        CUDAImageUtil::gaussFilterDepthMap ( d_depthInputFiltered, d_depthInputRaw, GlobalBundlingState::get().s_depthSigmaD, GlobalBundlingState::get().s_depthSigmaR,
                                             m_RGBDSensor->getDepthWidth(), m_RGBDSensor->getDepthHeight() );
    }
    else
    {
        CUDAImageUtil::copy<float> ( d_depthInputFiltered, d_depthInputRaw, m_RGBDSensor->getDepthWidth(), m_RGBDSensor->getDepthHeight() );
    }

    if ( ( m_RGBDSensor->getDepthWidth() == m_widthIntegration ) && ( m_RGBDSensor->getDepthHeight() == m_heightIntegration ) )
    {
        if ( ManagedRGBDInputFrame::s_bIsOnGPU )
        {
            CUDAImageUtil::copy<float> ( frame.m_depthIntegration, d_depthInputFiltered, m_widthIntegration, m_heightIntegration );
            //std::swap(frame.m_depthIntegration, d_depthInput);
        }
        else
        {
            if ( GlobalBundlingState::get().s_erodeSIFTdepth )
            {
                MLIB_CUDA_SAFE_CALL ( cudaMemcpy ( frame.m_depthIntegration, d_depthInputFiltered, sizeof ( float ) *bufferDimDepthInput, cudaMemcpyDeviceToHost ) );
            }
            else
            {
                memcpy ( frame.m_depthIntegration, m_RGBDSensor->getDepthFloat(), sizeof ( float ) *bufferDimDepthInput );
            }
        }
    }
    else
    {
        if ( ManagedRGBDInputFrame::s_bIsOnGPU )
        {
            CUDAImageUtil::resampleFloat ( frame.m_depthIntegration, m_widthIntegration, m_heightIntegration, d_depthInputFiltered, m_RGBDSensor->getDepthWidth(), m_RGBDSensor->getDepthHeight() );
        }
        else
        {
            CUDAImageUtil::resampleFloat ( frame.s_depthIntegrationGlobal, m_widthIntegration, m_heightIntegration, d_depthInputFiltered, m_RGBDSensor->getDepthWidth(), m_RGBDSensor->getDepthHeight() );
            MLIB_CUDA_SAFE_CALL ( cudaMemcpy ( frame.m_depthIntegration, frame.s_depthIntegrationGlobal, sizeof ( float ) *m_widthIntegration*m_heightIntegration, cudaMemcpyDeviceToHost ) );
            frame.s_activeDepthGPU = &frame;
        }
    }

    if ( GlobalBundlingState::get().s_enableGlobalTimings )
    {
        cudaDeviceSynchronize();
        s_timer.stop();
        TimingLog::getFrameTiming ( true ).timeSensorProcess = s_timer.getElapsedTimeMS();
    }

    m_currFrame++;
    return true;
}
