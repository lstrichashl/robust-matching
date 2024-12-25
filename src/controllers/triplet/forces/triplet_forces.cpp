#include "triplet_forces.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

CTripletForces::CTripletForces():
    BaseConrtoller(){
    m_typename = "non_faulty";
    number_of_robots_in_group = 1;
}

void CTripletForces::ControlStep() {
    m_heading = FlockingVector();
    m_eState = UpdateEState();
    m_pcRABAct->SetData(0, m_eState);
   int id = stoi(GetId());
    m_pcRABAct->SetData(1, id);
    if(m_eState != STATE_ALONE){
      m_heading = CVector2::ZERO;
    }
   if(m_is_crash){
      m_heading = CVector2::ZERO;
      m_pcLedAct->SetAllRGBColors(CColor::RED);
      m_pcLedAct->SetAllRedLeds(true);
   }
   m_pcRABAct->SetData(2, number_of_robots_in_group);
   SetWheelSpeedsFromVector(m_heading);
}

CVector2 CTripletForces::FlockingVector() {
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    if(! tMsgs.empty()) {
        CVector2 localAttractionForce;
        CVector2 gaziForce;
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            if(tMsgs[i].Data[0] != STATE_TRIPLET) {
                Real fLJ = 5 * ElectricalForce(tMsgs[i].Range);
                localAttractionForce += CVector2(fLJ, tMsgs[i].HorizontalBearing);
                // if(tMsgs[i].Data[0] == STATE_PAIRED){
                //     localAttractionForce *= 5;
                // }
            }
            else {
                Real force = GaziRepultion(tMsgs[i].Range);
                gaziForce += CVector2(force, tMsgs[i].HorizontalBearing);
            }
        }
        CVector2 force = localAttractionForce + gaziForce;
        return force;
    }
    else {
        return CVector2();
    }
}

EState CTripletForces::UpdateEState(){
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
    UInt8 touched_robots_counter = 0;
    vector<int> indexes_of_touched_robots;
    EState result = STATE_ALONE;
    if(! tMsgs.empty()) {
        for(size_t i = 0; i < tMsgs.size(); ++i) {
            if(tMsgs[i].Range/100 < PAIRING_THRESHOLD){
                touched_robots_counter++;
                if(tMsgs[i].Data[0] == STATE_TRIPLET){
                    return STATE_TRIPLET;
                }
            }
        }
    }
    if(touched_robots_counter == 0){
        return STATE_ALONE;
    }
    else if(touched_robots_counter == 1){
        return STATE_PAIRED;
    }
    else if(touched_robots_counter == 2){
        return STATE_TRIPLET;
    }
}


REGISTER_CONTROLLER(CTripletForces, "triplet_forces_controller")
