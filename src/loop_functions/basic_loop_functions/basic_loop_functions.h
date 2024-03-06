#ifndef BASIC_LOOP_FUNCTIONS_H
#define BASIC_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/abstract_controllers/base_controller.h"
#include "src/loop_functions/matching_result/matching_result.h"
#include <vector>

using namespace argos;

class CBasicLoopFunctions : public CLoopFunctions {

public:
   CBasicLoopFunctions();
   virtual ~CBasicLoopFunctions() {}
   virtual void PreStep();
   virtual void Init(TConfigurationNode& t_tree);
   virtual bool IsExperimentFinished();
   virtual void PostExperiment();
   virtual void Reset();
   virtual vector<CEPuck2Entity*> GetRobots(){
      return m_robots;
   }
   virtual vector<CEPuck2Entity*> GetNFRobots();
   vector<CVector2> GetPositions(vector<CEPuck2Entity*> robots);
   Clusters GetRobotPairs(vector<CEPuck2Entity*> robots);
   virtual void add_log();
   virtual void PlaceCluster(TConfigurationNode& entityNode);
   virtual void RemoveAll(vector<CEntity*> entites);

protected:
   vector<CEPuck2Entity*> m_robots;
   vector<CEPuck2Entity*> m_nf_robots;
   vector<string> m_logs;

   string m_log_file_path;
   double m_range;

   virtual void write_all_logs(vector<string> logs, string params_string);

private:
    vector<CVector2> m_last_positions;
    vector<CVector2> m_new_positions;
};

#endif
