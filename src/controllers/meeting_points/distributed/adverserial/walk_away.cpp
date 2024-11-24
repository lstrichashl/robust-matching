#include "walk_away.h"

CGreedyMeetingPointWalkAWay::CGreedyMeetingPointWalkAWay()
    :CGreedyMeetingPoint(){
    m_typename = "CGreedyMeetingPointWalkAWay";
}

void CGreedyMeetingPointWalkAWay::Reset() {
    CGreedyMeetingPoint::Reset();
    m_eState = STATE_ALONE;
    m_pcRABAct->SetData(0, STATE_ALONE);
    m_pcLedAct->SetAllRGBColors(CColor::RED);
    m_pcLedAct->SetAllRedLeds(true);
}

void CGreedyMeetingPointWalkAWay::headingUpdate(){
    CVector2 flocking = FlockingVector() * 10;
    m_heading *= -1;
    m_heading += flocking;
}


REGISTER_CONTROLLER(CGreedyMeetingPointWalkAWay, "greedy_meeting_point_walk_away_controller")
