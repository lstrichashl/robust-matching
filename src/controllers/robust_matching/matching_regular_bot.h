#ifndef ROBUST_MATCHING_H
#define ROBUST_MATCHING_H

#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
#include "src/controllers/abstract_controllers/base_controller.h"


using namespace argos;

class CRobustMatching : public BaseConrtoller {

public:
    CRobustMatching();
    virtual ~CRobustMatching() {}
    virtual void ControlStep();
    virtual CRadians GetZAngleOrientation();
    virtual CVector2 ToMateVector();
    virtual void Reset();

};


#endif
