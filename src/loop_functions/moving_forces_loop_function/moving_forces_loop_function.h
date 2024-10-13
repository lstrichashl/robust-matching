#ifndef MOVING_FORCES_LOOP_FUNCTIONS_H
#define MOVING_FORCES_LOOP_FUNCTIONS_H

#include "src/loop_functions/basic_loop_functions/basic_loop_functions.h"
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/meeting_points/meeting_points_epuck.h"

using namespace argos;

class CMovingForces : public CBasicLoopFunctions {

    public:
        CMovingForces();
        virtual ~CMovingForces() {}
        virtual void PreStep();
};


#endif