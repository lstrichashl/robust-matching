#include "matching_regular_bot.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "math.h"


CRobustMatching::CRobustMatching(){
   m_typename = "non_faulty";
}


void CRobustMatching::ControlStep() {
   m_heading = m_heading.Normalize();
   BaseConrtoller::ControlStep();
}

void CRobustMatching::Reset(){
   BaseConrtoller::Reset();
   m_eState = STATE_ALONE;
   m_pcLedAct->SetAllBlack();
   m_pcLedAct->SetAllRGBColors(CColor::GREEN);
}

REGISTER_CONTROLLER(CRobustMatching, "robust_matching_controller")
