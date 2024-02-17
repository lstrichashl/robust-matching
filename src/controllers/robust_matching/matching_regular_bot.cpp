#include "matching_regular_bot.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "math.h"


CRobustMatching::CRobustMatching() :
   m_pcWheels(NULL),
   m_pcLedAct(NULL)
{
   m_typename = "non_faulty";
}


void CRobustMatching::Init(TConfigurationNode& t_node) {
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcLedAct = GetActuator<CCI_EPuck2LEDsActuator>("epuck2_leds");
    try {
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }
    Reset();
}


void CRobustMatching::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

CRadians CRobustMatching::GetZAngleOrientation() {
   CRadians cZAngle, cYAngle, cXAngle;
   CEPuck2Entity self_entity = *dynamic_cast<CEPuck2Entity*>(&(&CSimulator::GetInstance())->GetSpace().GetEntity(CCI_Controller::GetId()));
   self_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   return cZAngle;
}

CVector3 CRobustMatching::ToMateVector() {
   if(mate != NULL) {
      CEPuck2Entity self_entity = *dynamic_cast<CEPuck2Entity*>(&(&CSimulator::GetInstance())->GetSpace().GetEntity(CCI_Controller::GetId()));
      CVector3 self_position = self_entity.GetEmbodiedEntity().GetOriginAnchor().Position;
      CVector3 mate_position = mate->GetEmbodiedEntity().GetOriginAnchor().Position;
      CVector3 to_mate = mate_position - self_position;
      return to_mate;
   }
   else {
      return CVector3(0,0,0);
   }
}

void CRobustMatching::ControlStep() {
   CVector3 to_mate = ToMateVector();
   CRadians cZAngle = GetZAngleOrientation();
   to_mate.RotateZ(-cZAngle);
    if(to_mate.Length() > 0.07){
         SetWheelSpeedsFromVector(to_mate.Normalize());
         m_pcLedAct->SetAllRGBColors(CColor::GREEN);
    }
    else {
        m_pcWheels->SetLinearVelocity(0, 0);
        m_pcLedAct->SetAllBlack();
      //   m_pcLedAct->SetAllRGBColors(CColor::RED);
      //   m_pcLedAct->SetAllRedLeds(true);
    }
}

void CRobustMatching::Reset(){
    m_pcLedAct->SetAllBlack();
    m_pcLedAct->SetAllRGBColors(CColor::GREEN);
}

// this function was copied from argos-examples/controllers/footbot_flocking
void CRobustMatching::SetWheelSpeedsFromVector(const CVector3& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.GetZAngle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

REGISTER_CONTROLLER(CRobustMatching, "robust_matching_controller")
