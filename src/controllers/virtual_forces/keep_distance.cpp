#include "keep_distance.h"


argos::CVector2 CKeepDistance::FlockingVector(){
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(!tMsgs.empty()) {
        CVector2 cAccum;
        Real fLJ;
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            double distance = tMsgs[i].Range;
            if(tMsgs[i].Data[0] == STATE_ALONE) {
                fLJ = LennardJonesPotential(tMsgs[i].Range);
                cAccum += CVector2(fLJ, tMsgs[i].HorizontalBearing);
            }
            else{
                fLJ = distance * (0.0000005 - 40 * ::pow(M_E, -::pow(distance,2)/50)); // Gazi force
                cAccum += CVector2(fLJ, tMsgs[i].HorizontalBearing);
            }
        }
        return cAccum;
    }
    else {
        return CVector2();
    }
}

REGISTER_CONTROLLER(CKeepDistance, "keep_distance_controller")
