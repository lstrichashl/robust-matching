#ifndef ROBUST_MATCHING_H
#define ROBUST_MATCHING_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

using namespace argos;

class CRobustMatching : public CCI_Controller {

public:
    CRobustMatching();
    virtual ~CRobustMatching() {}
    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();

    CVector3 m_cTargetPos;
    CVector3 self_position;
    CEPuck2Entity* mate;
    Real orientation;
protected:
   /* Pointer to the quadrotor position actuator */
   CCI_QuadRotorPositionActuator* m_pcPosAct;
   /* Pointer to the positioning sensor */
   CCI_PositioningSensor* m_pcPosSens;
   /* Pointer to the range-and-bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABSens;

};


#endif
