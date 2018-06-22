#ifndef TRACK_CONTROL_H
#define TRACK_CONTROL_H

#include<opencv2/opencv.hpp>

class TrackControl
{
public:
    TrackControl() = default;
    TrackControl(double cameraInfo,double maxV):camera_info(cameraInfo),max_v(maxV){}

    virtual ~TrackControl(){}

    using Vec4 = double[4];
    cv::Rect target;

    virtual const Vec4 &getVelocity(const double &) = 0;

    virtual TrackControl &setParams(double cameraInfo,double maxV){camera_info = cameraInfo;max_v = maxV;return *this;}
    virtual TrackControl &setTarget(const cv::Rect &r){target = r;return *this;}
    virtual TrackControl &setImgSize(const int &image_width,const int &image_height){imageWidth = image_width;imageHeight =image_height;return *this;}
protected:
    Vec4 velocity = {0,0,0,0};

    int imageWidth = 640;
    int imageHeight = 360;

    double camera_info = 1000.0; //投影中心到成像面的以像素为单位的距离
    double max_v = 2.0;
};

#endif

