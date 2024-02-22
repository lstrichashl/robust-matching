#ifndef CRASH_BOT_H
#define CRASH_BOT_H

#include "src/controllers/abstract_controllers/base_controller.h"
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>

using namespace argos;

class CCrashController: public BaseConrtoller {
    public:
        CCrashController(){}
        virtual void Reset();

    protected:
        CCI_RangeAndBearingActuator* m_pcRABAct;
        CCI_EPuck2LEDsActuator *m_pcLedAct;
};

#endif