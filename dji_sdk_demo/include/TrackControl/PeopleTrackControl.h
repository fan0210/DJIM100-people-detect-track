#ifndef PEOPLE_TRACK_CONTROL_H
#define PEOPLE_TRACK_CONTROL_H

#include<TrackControl.hpp>

class PeopleTrackControl final : public TrackControl
{
public:
    PeopleTrackControl() = default;
    using TrackControl::TrackControl;
    ~PeopleTrackControl(){}

    const Vec4 &getVelocity(const double &)override;
};

#endif
