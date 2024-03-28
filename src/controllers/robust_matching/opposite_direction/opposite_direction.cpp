#include "opposite_direction.h"

COppositeDirection::COppositeDirection(){
    m_typename = "COppositeDirection";
    m_heading = CVector2(0,0);
   }

void COppositeDirection::ControlStep() {
    m_heading = -m_heading.Normalize();
    CRobustMatching::ControlStep();
}

void COppositeDirection::Reset() {
    m_pcLedAct->SetAllRGBColors(CColor::RED);
    m_pcLedAct->SetAllRedLeds(true);
}

REGISTER_CONTROLLER(COppositeDirection, "adverserial_opposite_direction_controller")
