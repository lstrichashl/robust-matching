#ifndef CRASH_BOT_H
#define CRASH_BOT_H

#include "src/controllers/abstract_controllers/base_controller.h"

using namespace argos;

class CCrashController: public BaseConrtoller {
    public:
        virtual void Reset();
        virtual void ControlStep();
};

#endif