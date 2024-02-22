#include "matching_regular_bot.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "math.h"


CRobustMatching::CRobustMatching(){
   m_typename = "non_faulty";
}


CRadians CRobustMatching::GetZAngleOrientation() {
   CRadians cZAngle, cYAngle, cXAngle;
   CEPuck2Entity self_entity = *dynamic_cast<CEPuck2Entity*>(&(&CSimulator::GetInstance())->GetSpace().GetEntity(CCI_Controller::GetId()));
   self_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   return cZAngle;
}

CVector2 CRobustMatching::ToMateVector() {
   if(mate != NULL) {
      CEPuck2Entity self_entity = *dynamic_cast<CEPuck2Entity*>(&(&CSimulator::GetInstance())->GetSpace().GetEntity(CCI_Controller::GetId()));
      CVector2 self_position(self_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                              self_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      CVector2 mate_position(mate->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                              mate->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      CVector2 to_mate = mate_position - self_position;
      return to_mate;
   }
   else {
      return CVector2(0,0);
   }
}

void CRobustMatching::ControlStep() {
   CVector2 to_mate = ToMateVector();
   CRadians cZAngle = GetZAngleOrientation();
   to_mate.Rotate(-cZAngle);
    if(to_mate.Length() > PAIRING_THRESHOLD){
         m_eState = STATE_ALONE;
         SetWheelSpeedsFromVector(to_mate.Normalize());
         m_pcLedAct->SetAllRGBColors(CColor::GREEN);
    }
    else {
         m_eState = STATE_PAIRED;
         m_pcWheels->SetLinearVelocity(0, 0);
         m_pcLedAct->SetAllBlack();
      //   m_pcLedAct->SetAllRGBColors(CColor::RED);
      //   m_pcLedAct->SetAllRedLeds(true);
    }

}

void CRobustMatching::Reset(){
   BaseConrtoller::Reset();
   m_eState = STATE_ALONE;
   m_pcLedAct->SetAllBlack();
   m_pcLedAct->SetAllRGBColors(CColor::GREEN);
}

REGISTER_CONTROLLER(CRobustMatching, "robust_matching_controller")
