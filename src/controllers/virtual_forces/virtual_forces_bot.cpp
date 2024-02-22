#include "virtual_forces_bot.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

CVirtualForces::CVirtualForces(){
    m_typename = "non_faulty";
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
    if(ShouldTransitionToAlone()){
        Alone();
    }
    m_pcWheels->SetLinearVelocity(0, 0);
}

bool CVirtualForces::ShouldTransitionToAlone(){
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    bool has_near_robot = false;
    if(! tMsgs.empty()) {
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            if(tMsgs[i].Range/100 < PAIRING_THRESHOLD){
                has_near_robot = true;
            }
        }
    }
    return !has_near_robot;
}

bool CVirtualForces::ShouldTransitionToPaired(){
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(! tMsgs.empty()) {
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            if(tMsgs[i].Range/100 < PAIRING_THRESHOLD && tMsgs[i].Data[0] == STATE_ALONE){
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
            if(tMsgs[i].Data[0] == STATE_ALONE) {
                fLJ = ElectricalForce(tMsgs[i].Range);
                cAccum += CVector2(fLJ,
                            tMsgs[i].HorizontalBearing);
            }
            else {
                if(tMsgs[i].Range/100 < PAIRING_THRESHOLD * 3){
                    Real force = -0.05 * ElectricalForce(tMsgs[i].Range);
                    cAccum += CVector2(force,
                                tMsgs[i].HorizontalBearing);
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


REGISTER_CONTROLLER(CVirtualForces, "virtual_forces_bot_controller")
