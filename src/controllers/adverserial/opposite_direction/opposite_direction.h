#ifndef OPPOSITE_DIRECTION_H
#define OPPOSITE_DIRECTION_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
#include "src/controllers/robust_matching/robust_matching.h"

using namespace argos;

class COppositeDirection: public CRobustMatching {
public:
    COppositeDirection();
    virtual ~COppositeDirection() {}
    virtual void ControlStep();
    virtual void Reset();
};

#endif