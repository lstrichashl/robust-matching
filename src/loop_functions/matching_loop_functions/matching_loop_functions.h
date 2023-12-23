#ifndef MATCHING_LOOP_FUNCTIONS_H
#define MATCHING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "minimum_cost_perfect_matching/Graph.h"
#include "minimum_cost_perfect_matching/Matching.h"
#include <vector>


using namespace argos;

class CMatchingLoopFunctions : public CLoopFunctions {

public:
   CMatchingLoopFunctions();
   virtual ~CMatchingLoopFunctions() {}
   virtual void Init(TConfigurationNode& t_tree);
   virtual void Destroy();
   virtual void PreStep();
   virtual bool IsExperimentFinished();
   virtual CRadians GetZAngleOrientation(CQuaternion orientation);
   virtual Graph GetGraph(){
      return m_robotGraph;
   }
   virtual pair< list<int>, double > GetMatching(){
      return m_solution;
   }
   virtual vector<CEPuck2Entity*> GetRobots(){
      return m_robots;
   }
   virtual std::string GetLogFileName();

private:
   Graph m_robotGraph;
   pair< list<int>, double > m_solution;
   vector<double> m_costs;
   vector<CEPuck2Entity*> m_robots;
   vector<std::string> m_logs;

   virtual void write_to_log(Graph graph, pair< list<int>, double > solution);
};

#endif
