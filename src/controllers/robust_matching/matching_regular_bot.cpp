#include "matching_regular_bot.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "math.h"


CRobustMatching::CRobustMatching():BaseConrtoller(){}


void CRobustMatching::ControlStep() {
   m_heading = m_heading.Normalize();
   BaseConrtoller::ControlStep();
}

REGISTER_CONTROLLER(CRobustMatching, "robust_matching_controller")
