#include "forces_moving.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

CForcesMoving::CForcesMoving():
    BaseConrtoller(){
}

void CForcesMoving::ControlStep() {
    m_heading = FlockingVector();
    BaseConrtoller::ControlStep();
}

CVector2 CForcesMoving::FlockingVector() {
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
        CVector2 force = localAttractionForce + gaziForce;
        return force.Normalize();
    }
    else {
        return CVector2();
    }
}


REGISTER_CONTROLLER(CForcesMoving, "forces_moving_controller")
