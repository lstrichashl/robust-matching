#include "matching_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/robust_matching/matching_regular_bot.h"
#include <list>

#include <iostream>
#include <fstream>


double get_costs_diff(vector<double> cost1, vector<double> cost2){
    double sum_diffs = 0;
    for(unsigned int i = 0; i < cost1.size(); i++){
        sum_diffs += abs(cost1[i] - cost2[i]);
    }
    return sum_diffs;
}


CMatchingLoopFunctions::CMatchingLoopFunctions():
    m_matching(0),
    m_costs(0),
    m_isCommited(false),
    m_repeat_interval(1),
    CBasicLoopFunctions()
    {}

void CMatchingLoopFunctions::Init(TConfigurationNode& t_tree) {
    CBasicLoopFunctions::Init(t_tree);
    TConfigurationNode& paramsNode = GetNode(t_tree, "params");
    try{
        bool is_commited;
        GetNodeAttribute(paramsNode, "is_commited", is_commited);
        m_isCommited = is_commited;
        GetNodeAttribute(paramsNode, "repeat_interval", m_repeat_interval);
        GetNodeAttribute(paramsNode, "range", m_range);
    }
    catch(...){
        cout << "error with loading params tag in CMatchingLoopFunctions class" << endl;
    }
}

void CMatchingLoopFunctions::Destroy(){
    std::string param_string = "{\"is_commited\":\""+to_string(m_isCommited)+"\"}";
    write_all_logs(m_logs, param_string);
}

void CMatchingLoopFunctions::PreStep(){
    CBasicLoopFunctions::PreStep();
    try{
        UInt32 time = GetSpace().GetSimulationClock();
        if((m_matching.size() == 0 || (time % m_repeat_interval == 0 && !m_isCommited))) {
            MatchingResult result = GetMatchingResult(m_robots, m_range);
            m_matching = result._matching;
            m_robotGraph = result._graph;
            m_robots_in_matching = result._robots;
        }
        vector<int> matching = m_matching;
        vector<int> nf_matchig;
        vector<int> nf_half_matchig;
        for(unsigned i = 0; i < m_robots.size(); i++) {
            BaseConrtoller& cController = dynamic_cast<BaseConrtoller&>(m_robots[i]->GetControllableEntity().GetController());
            cController.mate = NULL;
        }
        for(vector<int>::iterator it = matching.begin(); it != matching.end(); it++)
        {
            pair<int, int> edge = m_robotGraph.GetEdge( *it );
            CEPuck2Entity* robot1 = m_robots[edge.first];
            CEPuck2Entity* robot2 = m_robots[edge.second];
            BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(robot1->GetControllableEntity().GetController());
            BaseConrtoller& cController2 = dynamic_cast<BaseConrtoller&>(robot2->GetControllableEntity().GetController());
            cController1.mate = robot2;
            cController2.mate = robot1; 
        }
    }
   catch(std::exception& ex) {
    cout << "catch" << endl;
      THROW_ARGOSEXCEPTION_NESTED("error while matching loop function", ex);
   }
}

REGISTER_LOOP_FUNCTIONS(CMatchingLoopFunctions, "matching_loop_functions")
