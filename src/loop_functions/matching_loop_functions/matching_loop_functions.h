#ifndef MATCHING_LOOP_FUNCTIONS_H
#define MATCHING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/loop_functions/minimum_cost_perfect_matching/Graph.h"
#include "src/loop_functions/minimum_cost_perfect_matching/Matching.h"
#include "src/loop_functions/basic_loop_functions/basic_loop_functions.h"

#include <vector>


using namespace argos;

class CMatchingLoopFunctions : public CBasicLoopFunctions {

public:
   CMatchingLoopFunctions();
   virtual ~CMatchingLoopFunctions() {}
   virtual void Init(TConfigurationNode& t_tree);
   virtual void Destroy();
   virtual void PreStep();
   virtual Graph GetGraph(){
      return m_robotGraph;
   }
   virtual vector<int> GetMatching(){
      return m_matching;
   }
   virtual vector<CEPuck2Entity*> GetRobotsInMatching(){
      return m_robots_in_matching;
   }

private:
   Graph m_robotGraph;
   vector<int> m_matching;
   vector<double> m_costs;
   vector<CEPuck2Entity*> m_robots_in_matching;
   bool m_isCommited;

   UInt32 m_repeat_interval;
};

#endif
