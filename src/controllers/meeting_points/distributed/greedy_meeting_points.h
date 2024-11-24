#ifndef GREEDY_MEETING_POINT_H
#define GREEDY_MEETING_POINT_H

#include "../meeting_points_epuck.h"

using namespace argos;

enum GreedyMeetingPointHandShake {
   IDLE = 0,
   SIN,
   SINACK,
   ACK,
   WAIT_FOR_PARTER_IN_TARGET
};

class CGreedyMeetingPoint : public CMeetingPointEpuck {

public:
    CGreedyMeetingPoint();
    virtual ~CGreedyMeetingPoint() {}
    virtual void ControlStep();
    virtual CVector2 FlockingVector();
    virtual void headingUpdate();


protected:
    GreedyMeetingPointHandShake handshake;
    int other_try_to_sin_robot_id;
    std::set<int> other_try_history;

    CVector2 m_prev_heading = CVector2::ZERO;

};

#endif
