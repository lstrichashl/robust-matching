#include "opposite_direction.h"

COppositeDirection::COppositeDirection(){
    m_typename = "COppositeDirection";
   }

void COppositeDirection::ControlStep() {
    m_heading = -m_heading;
    CRobustMatching::ControlStep();
}

void COppositeDirection::Reset() {
    m_pcLedAct->SetAllRGBColors(CColor::RED);
    m_pcLedAct->SetAllRedLeds(true);
}

REGISTER_CONTROLLER(COppositeDirection, "adverserial_opposite_direction_controller")
