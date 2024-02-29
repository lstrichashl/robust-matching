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



CVector2 CVirtualForces::FlockingVector() {
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(! tMsgs.empty()) {
        CVector2 localAttractionForce;
        CVector2 gaziForce;
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            if(tMsgs[i].Data[0] == STATE_ALONE) {
                Real fLJ = ElectricalForce(tMsgs[i].Range);
                localAttractionForce += CVector2(fLJ, tMsgs[i].HorizontalBearing);
            }
            else {
                Real force = GaziForce(tMsgs[i].Range);
                gaziForce += CVector2(force, tMsgs[i].HorizontalBearing);
            }
        }
        return localAttractionForce + gaziForce;
    }
    else {
        return CVector2();
    }
}


REGISTER_CONTROLLER(CVirtualForces, "virtual_forces_bot_controller")
