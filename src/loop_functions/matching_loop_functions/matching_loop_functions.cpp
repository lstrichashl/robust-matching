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
    add_log();
    write_all_logs(m_logs, param_string);
}

CRadians GetZAngleOrientation(CEPuck2Entity* robot1) {
   CRadians cZAngle, cYAngle, cXAngle;
   robot1->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   return cZAngle;
}

CVector2 ToMateVector(CEPuck2Entity* robot1, CEPuck2Entity* robot2) {
    CVector2 self_position(robot1->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                            robot1->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    CVector2 mate_position(robot2->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                            robot2->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    CVector2 to_mate = mate_position - self_position;
    CRadians cZAngle = GetZAngleOrientation(robot1);
    return to_mate.Rotate(-cZAngle).Normalize();
}


void CMatchingLoopFunctions::PreStep(){
    CBasicLoopFunctions::PreStep();
    vector<int> robots_in_matching;
    vector<CEPuck2Entity*> robots_not_in_matching;
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
            cController.m_heading = CVector2::ZERO;
        }
        for(vector<int>::iterator it = matching.begin(); it != matching.end(); it++)
        {
            pair<int, int> edge = m_robotGraph.GetEdge( *it );
            CEPuck2Entity* robot1 = m_robots[edge.first];
            CEPuck2Entity* robot2 = m_robots[edge.second];
            BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(robot1->GetControllableEntity().GetController());
            BaseConrtoller& cController2 = dynamic_cast<BaseConrtoller&>(robot2->GetControllableEntity().GetController());
            cController1.m_heading = ToMateVector(robot1, robot2);
            cController2.m_heading = ToMateVector(robot2, robot1);
            robots_in_matching.push_back(edge.first);
            robots_in_matching.push_back(edge.second);
        }
        for(int i = 0; i < m_robots.size(); i++){
            if(find(robots_in_matching.begin(), robots_in_matching.end(), i) == robots_in_matching.end()){
                BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(m_robots[i]->GetControllableEntity().GetController());
                cout << cController1.GetType() << endl;
                cController1.m_heading = cController1.RandomWalk();
            }
        }
    }
   catch(std::exception& ex) {
    cout << "catch" << endl;
      THROW_ARGOSEXCEPTION_NESTED("error while matching loop function", ex);
   }
}

REGISTER_LOOP_FUNCTIONS(CMatchingLoopFunctions, "matching_loop_functions")
