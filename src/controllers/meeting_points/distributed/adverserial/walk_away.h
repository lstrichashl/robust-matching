#ifndef GREEDY_MEETING_POINT_WALK_AWAY_H
#define GREEDY_MEETING_POINT_WALK_AWAY_H

#include "../greedy_meeting_points.h"

using namespace argos;


class CGreedyMeetingPointWalkAWay : public CGreedyMeetingPoint {

public:
    CGreedyMeetingPointWalkAWay();
    virtual ~CGreedyMeetingPointWalkAWay() {}
    virtual void headingUpdate();
    virtual void Reset();

};

#endif
