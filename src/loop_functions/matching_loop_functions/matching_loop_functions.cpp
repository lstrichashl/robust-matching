#include "matching_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/robust_matching/robust_matching.h"
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
    m_solution(0,-1),
    m_costs(0),
    m_tickNumber(0)
    {}

void CMatchingLoopFunctions::Init(TConfigurationNode& t_tree) {
    CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType("e-puck2");
    for(CSpace::TMapPerType::iterator it = m_cRobots.begin(); it != m_cRobots.end(); ++it) {
        CEPuck2Entity* robot = any_cast<CEPuck2Entity*>(it->second);
        m_robots.push_back(robot);
    }
}

CRadians CMatchingLoopFunctions::GetZAngleOrientation(CQuaternion orientation) {
   CRadians cZAngle, cYAngle, cXAngle;
   orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   return cZAngle;
}

void CMatchingLoopFunctions::PreStep(){
    std::vector<std::pair<int, CVector2>> positions;
    std::vector<std::pair<int, CQuaternion>> orientations;
    int number_of_robots = m_robots.size();
    for (unsigned i=0; i<number_of_robots; i++){
        CVector2 position(m_robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                m_robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        positions.push_back(std::make_pair(i, position));
        orientations.push_back(std::make_pair(i, m_robots[i]->GetEmbodiedEntity().GetOriginAnchor().Orientation));
    }
    std::vector<std::pair<int, CVector2>>::iterator it1;
    std::vector<std::pair<int, CVector2>>::iterator it2;
    Graph G(number_of_robots);
	vector<double> cost(number_of_robots*number_of_robots);
    for(unsigned i=0; i<number_of_robots; i++){
        for(unsigned j=0;j<number_of_robots;j++){
            if(i != j) {
                CRadians cZAngle1 = GetZAngleOrientation(orientations[i].second);
                CRadians cZAngle2 = GetZAngleOrientation(orientations[j].second);
                CVector2 distance_vector1 = (positions[i].second - positions[j].second);

                Real angle_distance = 1 - (abs(cZAngle1.GetValue() - cZAngle2.GetValue()) / 3.141592);
                G.AddEdge(i, j);
                cost[G.GetEdgeIndex(i,j)] = distance_vector1.Length();
                cost[G.GetEdgeIndex(i,j)] += + 0.01 * angle_distance;
                // cout << "(" << i << "," << j << "): " << angle_distance << "," << distance_vector1.Length() << endl;
            }
        }
    }
    Matching M(G);
    pair< list<int>, double > solution = M.SolveMinimumCostPerfectMatching(cost);

    // if(m_costs.size() > 0 && get_costs_diff(cost, m_costs) < 0.05) {
    //     GetSimulator().Terminate();
    // }

	list<int> matching = solution.first;
	double matching_cost = solution.second;
    m_solution = solution;
    m_robotGraph = G;
    m_costs = cost;
    for(list<int>::iterator it = matching.begin(); it != matching.end(); it++)
    {
        pair<int, int> edge = G.GetEdge( *it );
        CEPuck2Entity* robot1 = m_robots[edge.first];
        CEPuck2Entity* robot2 = m_robots[edge.second];
        CRobustMatching& cController1 = dynamic_cast<CRobustMatching&>(robot1->GetControllableEntity().GetController());
        CRobustMatching& cController2 = dynamic_cast<CRobustMatching&>(robot2->GetControllableEntity().GetController());
        cController1.mate = robot2;
        cController2.mate = robot1; 
    }

    write_to_log(G, solution);
    m_tickNumber++;
}

void CMatchingLoopFunctions::write_to_log(Graph graph, pair< list<int>, double > solution){
    std::string cost_string = "\"cost\":\""+to_string(solution.second) + "\"";
    std::string tick_string = "\"tick\":\""+to_string(m_tickNumber)+"\"";
    list<int> matching = solution.first;
    std::string matcing_string = "\"matching\":\""; 
    for(list<int>::iterator it = matching.begin(); it != matching.end(); it++){
        pair<int, int> edge = graph.GetEdge( *it );
        matcing_string += "(" + to_string(edge.first) + "," + to_string(edge.second) + "),";
    }
    matcing_string.pop_back();
    matcing_string += "\"";
    std::string log =  "{" + matcing_string + "," + cost_string + "," + tick_string + "}";

    std::string file_name = GetSimulator().GetExperimentFileName();
    ofstream os(file_name + ".log", std::ios_base::app);
    os << log << endl;
    os.close();
}

REGISTER_LOOP_FUNCTIONS(CMatchingLoopFunctions, "matching_loop_functions")
