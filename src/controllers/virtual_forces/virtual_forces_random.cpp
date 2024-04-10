#include "virtual_forces_random.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

CVirtualForcesRandom::CVirtualForcesRandom():
    CVirtualForces(){}

CVector2 CVirtualForcesRandom::FlockingVector() {
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(! tMsgs.empty()) {
        CVector2 localAttractionForce,gaziForceAttraction, gaziForceRepulsion, gaziForce;
        CVector2 forces = CVector2::ZERO, random = CVector2::ZERO;
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            if(tMsgs[i].Data[0] == STATE_ALONE) {
                Real fLJ = ElectricalForce(tMsgs[i].Range);
                localAttractionForce += CVector2(fLJ, tMsgs[i].HorizontalBearing);
            }
            else {
                gaziForceAttraction += CVector2(GaziAttraction(tMsgs[i].Range), tMsgs[i].HorizontalBearing);
                gaziForceRepulsion += CVector2(GaziRepultion(tMsgs[i].Range), tMsgs[i].HorizontalBearing);
            }
        }
        random = RandomWalk()/5;
        double alpha = ((double)m_time) / 3000;
        if(alpha > 1)
            alpha = 1;
        CVector2 force = random + localAttractionForce + gaziForceRepulsion;
        return force.Normalize();
    }
    else{
        return RandomWalk().Normalize();
    }
}

REGISTER_CONTROLLER(CVirtualForcesRandom, "virtual_forces_random_controller")
REGISTER_CONTROLLER(CVirtualForcesRandomCrash, "virtual_forces_random_crash_controller")
