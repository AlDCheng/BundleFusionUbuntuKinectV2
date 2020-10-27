#ifndef IMAGESTREAM_H
#define IMAGESTREAM_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <chrono>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <GlobalAppState.h>

using namespace std;

class ImageStream {
public:
    ImageStream();
    ~ImageStream();

    // Initialization functions
    void InitKinect2();

    // Frame Capture functions
    bool GetNext();
    bool GetNextTUM();
    bool GetNextKinect2();

    // Frame containers
    cv::Mat rgb, depth, ir;

private:
    // Dataset file storage
    ifstream assoc_list, assoc_list_source;
    string dataset, path, path_source;

    // Kinect V2 
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    libfreenect2::FrameMap frames;
    libfreenect2::Registration *registration;

    libfreenect2::SyncMultiFrameListener *listener;

    cv::Mat rgb_source, depth_source;
    double timestamp;
    string type;
};

#endif // IMAGESTREAM_H_