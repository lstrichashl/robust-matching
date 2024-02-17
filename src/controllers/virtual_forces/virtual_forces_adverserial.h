#ifndef ADVERSERIAL_H
#define ADVERSERIAL_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include "src/controllers/virtual_forces/virtual_forces_bot.h"

using namespace argos;

class CAdverserialVirtualForces: public CVirtualForces {

public:
    CAdverserialVirtualForces();
    virtual ~CAdverserialVirtualForces() {}
    virtual void ControlStep();
    virtual void Reset();
    // virtual bool ShouldTransitionToPaired();
    virtual CVector2 FlockingVector();
};

#endif