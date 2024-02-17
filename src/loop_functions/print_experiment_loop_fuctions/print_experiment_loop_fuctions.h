#ifndef PRINT_EXPERIMENT_FUNCTIONS_H
#define PRINT_EXPERIMENT_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/loop_functions/matching_result/matching_result.h"
#include <vector>


using namespace argos;

class CPrintExperimentFunctions : public CLoopFunctions {

public:
   CPrintExperimentFunctions();
   virtual ~CPrintExperimentFunctions() {}
   virtual void Init(TConfigurationNode& t_tree);
   virtual void Destroy();
   virtual void PreStep();
   virtual std::vector<CEPuck2Entity*> GetRobots(){
      return m_robots;
   }

protected:
   std::vector<CEPuck2Entity*> m_robots;
   std::vector<std::string> m_logs;

   std::string m_log_file_path;

   virtual void write_to_log(std::vector<std::pair<int,int>> matching);
};

#endif
