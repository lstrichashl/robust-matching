#include "virtual_forces_bot.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

CVirtualForces::CVirtualForces():
   m_pcWheels(NULL),
   m_pcLedAct(NULL)
{
    m_typename = "non_faulty";
}


void CVirtualForces::Init(TConfigurationNode& t_node) {
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcLedAct = GetActuator<CCI_EPuck2LEDsActuator>("epuck2_leds");
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator  >("range_and_bearing" );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor    >("range_and_bearing" );
    try {
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }

    Reset();
}

void CVirtualForces::Reset() {
    m_eState = STATE_ALONE;
    m_pcRABAct->SetData(0, STATE_ALONE);
    m_pcLedAct->SetAllRGBColors(CColor::GREEN);
}

void CVirtualForces::ControlStep() {
    switch (m_eState)
    {
    case STATE_ALONE:
        Alone();
        break;
    case STATE_PAIRED:
        Paired();
    default:
        break;
    }
}

void CVirtualForces::Alone(){
    if(m_eState != STATE_ALONE){
        m_eState = STATE_ALONE;
        m_pcRABAct->SetData(0, STATE_ALONE);
    }
    if(ShouldTransitionToPaired()) {
        Paired();
    }
    else {
        CVector2 cDirection = FlockingVector();
        SetWheelSpeedsFromVector(cDirection);
    }
}

void CVirtualForces::Paired(){
    if(m_eState != STATE_PAIRED){
        m_eState = STATE_PAIRED;
        m_pcRABAct->SetData(0, STATE_PAIRED);
    }
    m_pcWheels->SetLinearVelocity(0, 0);
}

bool CVirtualForces::ShouldTransitionToPaired(){
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(! tMsgs.empty()) {
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            if(tMsgs[i].Range < 8){
                return true;
            }
        }
    }
    return false;
}


CVector2 CVirtualForces::FlockingVector() {
    /* Get RAB messages from nearby eye-bots */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    /* Go through them to calculate the flocking interaction vector */
    if(! tMsgs.empty()) {
        /* This will contain the final interaction vector */
        CVector2 cAccum;
        /* Used to calculate the vector length of each neighbor's contribution */
        Real fLJ;
        /* A counter for the neighbors in state flock */
        UInt32 unPeers = 0;
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            /*
            * We consider only the neighbors in state flock
            */
            fLJ = ::pow(tMsgs[i].Range, -2);
            if(tMsgs[i].Data[0] == STATE_ALONE) {
                cAccum += CVector2(fLJ,
                            tMsgs[i].HorizontalBearing);
            }
            else {
                if(tMsgs[i].Range < 20){ 
                    cAccum += CVector2(0.1 * fLJ,
                                tMsgs[i].HorizontalBearing + tMsgs[i].HorizontalBearing.PI);
                }
            }
        }
    //   if(unPeers > 0) {
    //      /* Divide the accumulator by the number of flocking neighbors */
    //      cAccum /= unPeers;
    //      /* Limit the interaction force */
    //      if(cAccum.Length() > m_sFlockingParams.MaxInteraction) {
    //         cAccum.Normalize();
    //         cAccum *= m_sFlockingParams.MaxInteraction;
    //      }
    //   }
        /* All done */
        return cAccum * 250;
    }
    else {
        /* No messages received, no interaction */
        return CVector2();
    }
}




void CVirtualForces::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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

// this function was copied from argos-examples/controllers/footbot_flocking
void CVirtualForces::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
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

REGISTER_CONTROLLER(CVirtualForces, "virtual_forces_bot_controller")
