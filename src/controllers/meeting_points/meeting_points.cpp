#include "meeting_points.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include "math.h"


CMeetingPoint::CMeetingPoint():
   m_pcPosAct(NULL),
   m_pcPosSens(NULL),
   BaseConrtoller(){
   PAIRING_THRESHOLD = 0.07;
}

void CMeetingPoint::Init(TConfigurationNode& t_node){
    m_pcPosAct    = GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");
    m_pcPosSens   = GetSensor  <CCI_PositioningSensor        >("positioning"       );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator  >("range_and_bearing" );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor    >("range_and_bearing" );
    Reset();
}

void CMeetingPoint::ControlStep() {
    // m_heading = m_heading.Normalize();
    if(ShouldTransitionToPaired()){
        m_eState = STATE_PAIRED;
        cout << "paired" << endl;
    }
    else if(ShouldTransitionToAlone()){
        m_eState = STATE_ALONE;
    }
    m_pcRABAct->SetData(0, m_eState);
    // int id = stoi(GetId());
    // m_pcRABAct->SetData(1, id);
    // if(m_eState != STATE_PAIRED && !m_is_crash){
    // cout << m_target_position.GetX() << endl;
    m_pcPosAct->SetAbsolutePosition(m_target_position);
    // }
}

void CMeetingPoint::Reset(){
   m_target_position = CVector3(-2,1,1);
   m_eState = STATE_ALONE;
}


CMeetingPointCrash::CMeetingPointCrash():CMeetingPoint(){
    m_typename = "crash";
}

void CMeetingPointCrash::ControlStep(){
    // m_pcPosAct->SetAbsolutePosition(m_target_position);
}

REGISTER_CONTROLLER(CMeetingPointCrash, "meeting_point_crash_controller")
REGISTER_CONTROLLER(CMeetingPoint, "meeting_point_controller")
