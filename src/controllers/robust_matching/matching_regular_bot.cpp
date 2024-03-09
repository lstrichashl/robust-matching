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
   BaseConrtoller::ControlStep();
   //  if(to_mate.Length() > PAIRING_THRESHOLD){
   //       m_eState = STATE_ALONE;
   //       SetWheelSpeedsFromVector(to_mate.Normalize());
   //       m_pcLedAct->SetAllRGBColors(CColor::GREEN);
   //  }
   //  else {
   //       m_eState = STATE_PAIRED;
   //       m_pcWheels->SetLinearVelocity(0, 0);
   //       m_pcLedAct->SetAllBlack();
   //    //   m_pcLedAct->SetAllRGBColors(CColor::RED);
   //    //   m_pcLedAct->SetAllRedLeds(true);
   //  }
}

void CRobustMatching::Reset(){
   BaseConrtoller::Reset();
   m_eState = STATE_ALONE;
   m_pcLedAct->SetAllBlack();
   m_pcLedAct->SetAllRGBColors(CColor::GREEN);
}

REGISTER_CONTROLLER(CRobustMatching, "robust_matching_controller")
