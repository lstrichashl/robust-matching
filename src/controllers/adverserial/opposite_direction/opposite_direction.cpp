#include "opposite_direction.h"

COppositeDirection::COppositeDirection() {}

void COppositeDirection::ControlStep() {
    CVector3 to_mate = ToMateVector();
    CRadians cZAngle = GetZAngleOrientation();
    to_mate.RotateZ(-cZAngle);
    SetWheelSpeedsFromVector(-to_mate);
}

void COppositeDirection::Reset() {
    m_pcLedAct->SetAllRGBColors(CColor::RED);
    m_pcLedAct->SetAllRedLeds(true);
}

REGISTER_CONTROLLER(COppositeDirection, "adverserial_opposite_direction_controller")
