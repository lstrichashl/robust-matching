#include "robust_matching.h"

CRobustMatching::CRobustMatching() :
   m_pcPosAct(NULL),
   m_pcPosSens(NULL),
   m_pcRABSens(NULL) {}


void CRobustMatching::Init(TConfigurationNode& t_node) {
   m_pcPosAct  = GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");
   m_pcPosSens = GetSensor  <CCI_PositioningSensor        >("positioning"       );
   m_pcRABSens = GetSensor  <CCI_RangeAndBearingSensor    >("range_and_bearing" );
}

void CRobustMatching::ControlStep() {
   m_pcPosAct->SetAbsolutePosition(m_cTargetPos);
}

REGISTER_CONTROLLER(CRobustMatching, "epuck2_matching_controller")
