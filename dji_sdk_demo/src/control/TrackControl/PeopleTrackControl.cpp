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
     * Design your own control algorithm
     *
     *
     *
     * /

    double forwardV,leftrV,heightV,yawV;
    forwardV = leftrV = heightV = yawV = 0.0;

    velocity[0] = forwardV;
    velocity[1] = leftrV;
    velocity[2] = heightV;
    velocity[3] = yawV;

    return velocity;
}
