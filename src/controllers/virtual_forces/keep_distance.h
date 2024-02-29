#ifndef KEEP_DISTANCE_H
#define KEEP_DISTANCE_H

#include <argos3/core/control_interface/ci_controller.h>
#include "src/controllers/virtual_forces/virtual_forces_adverserial.h"

using namespace argos;

class CKeepDistance: public CAdverserialVirtualForces{
    public:
        virtual argos::CVector2 FlockingVector();
};

#endif