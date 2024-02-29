#include "virtual_forces_adverserial.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

CAdverserialVirtualForces::CAdverserialVirtualForces(){
    m_typename = "CAdverserialVirtualForces";
}

void CAdverserialVirtualForces::Reset() {
    CVirtualForces::Reset();
    m_eState = STATE_ALONE;
    m_pcRABAct->SetData(0, STATE_ALONE);
    m_pcLedAct->SetAllRGBColors(CColor::RED);
    m_pcLedAct->SetAllRedLeds(true);
}

CVector2 CAdverserialVirtualForces::FlockingVector() {
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(! tMsgs.empty()) {
        CVector2 cAccum;
        Real fLJ;
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            fLJ = -ElectricalForce(tMsgs[i].Range);
            cAccum += CVector2(fLJ, tMsgs[i].HorizontalBearing);
        }
        return cAccum;
    }
    else {
        return CVector2();
    }
}

REGISTER_CONTROLLER(CAdverserialVirtualForces, "virtual_forces_adverserial_controller")
