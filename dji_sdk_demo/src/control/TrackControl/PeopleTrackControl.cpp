#include<PeopleTrackControl.h>
#ifndef PI
#define PI 3.1415926
#endif

const PeopleTrackControl::Vec4 &PeopleTrackControl::getVelocity(const double &curHeight)
{


    /*
     *
     *
     *
     * You'd better design your own control algorithm
     *
     *
     *
     * /

//    double curHeight_ = curHeight+3.0;

//    double forwardV,leftrV,heightV,yawV;
//    forwardV = leftrV = heightV = yawV = 0.0;
//    int center_X = target.x+target.width/2;
//    int center_Y = target.y+target.height/2;

//    double max_v_ = max_v;
//    max_v = max_v/2.5;

//    /*...............heightV.................*/
//    heightV = 0;

//    /*................forwardV--------------*/
//    if(abs(center_Y-imageHeight/2)<=25)
//        forwardV = 0;
//    else
//    {
//        double distance,distance1,distance2;
//        double angle1,angle2;
//        if(center_Y-imageHeight/2>25)
//        {
//            distance1 = curHeight_*sqrt(2.0)/(camera_info/(center_Y-imageHeight/2));
//            angle1 = atan(camera_info/(center_Y-imageHeight/2));
//            angle2 = 135.0/180.0*PI - angle1;
//            distance = sin(angle1)*distance1/sin(angle2);
//            forwardV = distance>max_v?max_v:distance;
//            forwardV = -2.5*forwardV;
//        }
//        else
//        {
//            distance1 = curHeight_*sqrt(2.0)/(camera_info/abs(center_Y-imageHeight/2));
//            angle1 = atan(camera_info/abs(center_Y-imageHeight/2));
//            angle2 = angle1 - atan(1.0);
//            distance2 = distance1/sin(angle2)*sin(atan(1.0));
//            distance = sqrt(distance1*distance1+distance2*distance2-2*cos(PI-angle1-angle2)*distance1*distance2);
//            forwardV = distance>max_v?max_v:distance;
//            forwardV = 2.5*forwardV;
//        }
//    }
//    max_v = max_v_;

//    /*-----------------leftrV-------------------*/
//    leftrV = 0;

//    /*-----------------yawV-------------------*/
//    yawV = atan((center_X-imageWidth/2)/(imageHeight-1-center_Y+0.00001))*180.0/PI;

//    velocity[0] = forwardV;
//    velocity[1] = leftrV;
//    velocity[2] = heightV;
//    velocity[3] = yawV;

//    return velocity;
}
