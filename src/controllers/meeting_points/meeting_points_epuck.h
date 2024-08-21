#ifndef MEETING_POINT_EPUCK_H
#define MEETING_POINT_EPUCK_H

#include "src/controllers/abstract_controllers/base_controller.h"

using namespace argos;

enum MeetingPointPhase {
   Rotate = 0,
   Walk,
   Done
};

class CMeetingPointEpuck : public BaseConrtoller {

public:
    CMeetingPointEpuck();
    virtual ~CMeetingPointEpuck() {}
    virtual void ControlStep();
    virtual int GetEncoderDiffRight();
    virtual int GetEncoderDiffLeft();
    virtual void MoveForward();
    virtual void RotateToAngle();
    virtual void NewIteration();
    virtual CVector2 FlockingVector();
    virtual CVector2 GetDirectionFromEncoders();
    virtual CVector2 ToMateVector(CVector2 self_position, CVector2 meeting_point);

    int m_iPreviousEncoderRight, m_iPreviousEncoderLeft;

    Real m_fDistance;
    Real m_relative_orientation;

    MeetingPointPhase m_meeting_point_phase;

    CVector2 m_encoder_position;



};

#endif
