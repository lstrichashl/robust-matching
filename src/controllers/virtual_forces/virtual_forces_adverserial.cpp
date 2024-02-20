#include "virtual_forces_adverserial.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

CAdverserialVirtualForces::CAdverserialVirtualForces(){
    m_typename = "CAdverserialVirtualForces";
}

void CAdverserialVirtualForces::Reset() {
    m_eState = STATE_ALONE;
    m_pcRABAct->SetData(0, STATE_PAIRED);
    m_pcLedAct->SetAllRGBColors(CColor::RED);
    m_pcLedAct->SetAllRedLeds(true);
}

void CAdverserialVirtualForces::ControlStep() {
    // switch (m_eState)
    // {
    // case STATE_ALONE:
    //     Alone();
    //     break;
    // case STATE_PAIRED:
    //     Paired();
    // default:
    //     break;
    // }
}

bool CAdverserialVirtualForces::ShouldTransitionToPaired(){
    return CVirtualForces::ShouldTransitionToPaired();
}

CVector2 CAdverserialVirtualForces::FlockingVector() {
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
            fLJ = LinearForce(tMsgs[i].Range);
            if(tMsgs[i].Data[0] == STATE_ALONE) {
                cAccum += CVector2(fLJ,
                            tMsgs[i].HorizontalBearing + tMsgs[i].HorizontalBearing.PI);
            }
            else {
                if(tMsgs[i].Range < 15){ 
                    cAccum += CVector2(0.05 * fLJ,
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

REGISTER_CONTROLLER(CAdverserialVirtualForces, "virtual_forces_adverserial_controller")
