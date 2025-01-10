#ifndef NEIBERG_LOOP_FUNCTIONS_H
#define NEIBERG_LOOP_FUNCTIONS_H

#include "src/loop_functions/print_experiment_loop_fuctions/print_experiment_loop_fuctions.h"
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/meeting_points/distributed/nieberg.h"
#include <unordered_map>

using namespace argos;

class CNeibergLoopFunctions : public CPrintExperimentFunctions {

    public:
        CNeibergLoopFunctions();
        virtual ~CNeibergLoopFunctions() {}
        virtual void PostStep();
        virtual void Reset();
        virtual void Init(TConfigurationNode& t_tree);
};


#endif