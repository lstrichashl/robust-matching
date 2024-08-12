#ifndef BASIC_LOOP_FUNCTIONS_H
#define BASIC_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/abstract_controllers/base_controller.h"
#include "src/loop_functions/matching_result/matching_result.h"
#include <vector>

using namespace argos;

struct DistributeConfig{
    TConfigurationNode robot_config;
    UInt32 unQuantity;

    DistributeConfig(TConfigurationNode& _robot_config, UInt32 _unQuantity){
        robot_config = _robot_config;
        unQuantity = _unQuantity;
    }
};

class CBasicLoopFunctions : public CLoopFunctions {

public:
   CBasicLoopFunctions();
   virtual ~CBasicLoopFunctions() {}
   virtual void PreStep();
   virtual void PostStep();
   virtual void Init(TConfigurationNode& t_tree);
   virtual bool IsExperimentFinished();
   virtual void PostExperiment();
   virtual void Reset();
   virtual vector<CEntity*> GetRobots(){
      return m_robots;
   }
   virtual vector<CEntity*> GetNFRobots();
   vector<CVector2> GetPositions(vector<CEntity*> robots);
   Clusters GetRobotPairs(vector<CEntity*> robots);
   virtual void add_log();
   virtual void PlaceCluster(TConfigurationNode& entityNode);
   virtual void PlaceCluster(vector<DistributeConfig> configs,Real robot_range, CRange<Real> arena_range, int base_id,vector<CEntity*> seed_entites={});
   virtual void RemoveAll(vector<CEntity*> entites);
   virtual bool _is_connected_graph(vector<CVector2> positions, Real robot_range);

protected:
   vector<CEntity*> m_robots;
   vector<CEntity*> m_nf_robots;
   vector<string> m_logs;

   string m_log_file_path;
   double m_range;

   virtual void write_all_logs(vector<string> logs, string params_string);

    vector<CVector2> m_init_positions;
    
// private:
    vector<CVector2> m_last_positions;
    vector<CVector2> m_new_positions;
};

#endif
