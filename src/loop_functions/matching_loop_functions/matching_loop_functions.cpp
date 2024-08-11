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

CRadians GetZAngleOrientation(CEntity* robot1) {
   CRadians cZAngle, cYAngle, cXAngle;
   GetEmbodiedEntity3(robot1)->GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   return cZAngle;
}

CVector2 ToMateVector(CEntity* robot1, CEntity* robot2) {
    CVector2 self_position(GetEmbodiedEntity3(robot1)->GetOriginAnchor().Position.GetX(),
                            GetEmbodiedEntity3(robot1)->GetOriginAnchor().Position.GetY());
    CVector2 mate_position(GetEmbodiedEntity3(robot2)->GetOriginAnchor().Position.GetX(),
                            GetEmbodiedEntity3(robot2)->GetOriginAnchor().Position.GetY());
    CVector2 to_mate = mate_position - self_position;
    CRadians cZAngle = GetZAngleOrientation(robot1);
    return to_mate.Rotate(-cZAngle).Normalize();
}


Graph* GenerateGraph(vector<CEntity*> robots, double range){
    std::vector<CVector2> positions;
    std::vector<std::pair<int, CQuaternion>> orientations;
    int number_of_robots = robots.size();
    for (unsigned i=0; i<number_of_robots; i++){
        CVector2 position(GetEmbodiedEntity3(robots[i])->GetOriginAnchor().Position.GetX(),
                GetEmbodiedEntity3(robots[i])->GetOriginAnchor().Position.GetY());
        positions.push_back(position);
        orientations.push_back(std::make_pair(i, GetEmbodiedEntity3(robots[i])->GetOriginAnchor().Orientation));
    }
    Graph* G = new Graph(number_of_robots);
    for(unsigned i=0; i<number_of_robots; i++){
        for(unsigned j=0;j<number_of_robots;j++){
            if(i != j) {
                CRadians cZAngle1 = GetZAngleOrientation(orientations[i].second);
                CRadians cZAngle2 = GetZAngleOrientation(orientations[j].second);
                CVector2 distance_vector1 = (positions[i] - positions[j]);

                Real angle_distance = 1 - (abs(cZAngle1.GetValue() - cZAngle2.GetValue()) / 3.141592);
                if(distance_vector1.Length() < range){
                    G->AddEdge(i, j, distance_vector1.Length());
                }
                // cost[G.GetEdgeIndex(i,j)] += + 0.00605 * angle_distance;
                // cost[G.GetEdgeIndex(i,j)] += + 0.01 * angle_distance;
            }
        }
    }
    AddHopToGraph(G, positions);
    return G;
}

void CMatchingLoopFunctions::PreStep(){
    CBasicLoopFunctions::PreStep();
    vector<int> robots_in_matching;
    vector<CEntity*> robots_not_in_matching;
    try{
        UInt32 time = GetSpace().GetSimulationClock();
        if((m_matching.size() == 0 || (time % m_repeat_interval == 0 && !m_isCommited))) {
            Graph* g = GenerateGraph(m_robots, m_range);
            MatchingResult result = GetMatchingResult(g, m_robots, m_range);
            m_matching = result._matching;
            m_robotGraph = result._graph;
            m_robots_in_matching = result._robots;
        }
        vector<int> matching = m_matching;
        vector<int> nf_matchig;
        vector<int> nf_half_matchig;
        for(unsigned i = 0; i < m_robots.size(); i++) {
            BaseConrtoller& cController = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[i])->GetController());
            cController.m_heading = CVector2::ZERO;
        }
        for(vector<int>::iterator it = matching.begin(); it != matching.end(); it++)
        {
            pair<int, int> edge = m_robotGraph.GetEdge( *it );
            CEntity* robot1 = m_robots[edge.first];
            CEntity* robot2 = m_robots[edge.second];
            BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robot1)->GetController());
            BaseConrtoller& cController2 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robot2)->GetController());
            cController1.m_heading = ToMateVector(robot1, robot2);
            cController2.m_heading = ToMateVector(robot2, robot1);
            robots_in_matching.push_back(edge.first);
            robots_in_matching.push_back(edge.second);
        }
        for(int i = 0; i < m_robots.size(); i++){
            if(find(robots_in_matching.begin(), robots_in_matching.end(), i) == robots_in_matching.end()){
                BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[i])->GetController());
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
