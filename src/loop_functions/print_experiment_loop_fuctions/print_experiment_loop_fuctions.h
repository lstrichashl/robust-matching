#ifndef PRINT_EXPERIMENT_FUNCTIONS_H
#define PRINT_EXPERIMENT_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/loop_functions/matching_result/matching_result.h"
#include "src/loop_functions/basic_loop_functions/basic_loop_functions.h"
#include <vector>


using namespace argos;

class CPrintExperimentFunctions : public CBasicLoopFunctions {

public:
   CPrintExperimentFunctions();
   virtual ~CPrintExperimentFunctions() {}
   virtual void Init(TConfigurationNode& t_tree);
   virtual void Destroy();
   virtual void PreStep();

protected:
   virtual void write_to_log(Clusters pairs);
};

#endif
