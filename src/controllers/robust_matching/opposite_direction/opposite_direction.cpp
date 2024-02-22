#include "opposite_direction.h"

COppositeDirection::COppositeDirection(){
    m_typename = "COppositeDirection";
   }

void COppositeDirection::ControlStep() {
    CVector2 to_mate = ToMateVector();
    CRadians cZAngle = GetZAngleOrientation();
    to_mate.Rotate(-cZAngle);
    SetWheelSpeedsFromVector(-to_mate.Normalize());
}

void COppositeDirection::Reset() {
    m_pcLedAct->SetAllRGBColors(CColor::RED);
    m_pcLedAct->SetAllRedLeds(true);
}

REGISTER_CONTROLLER(COppositeDirection, "adverserial_opposite_direction_controller")
