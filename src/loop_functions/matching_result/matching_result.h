#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/loop_functions/minimum_cost_perfect_matching/Graph.h"
#include "src/loop_functions/minimum_cost_perfect_matching/Matching.h"


#include "src/controllers/virtual_forces/virtual_forces_bot.h"

// #include <argos3/core/control_interface/ci_controller.h>
// #include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
// #include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
// #include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
// #include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
// #include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
// #include <argos3/core/utility/configuration/argos_configuration.h>
// #include <argos3/core/simulator/space/space.h>
// #include <argos3/core/simulator/simulator.h>
// #include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

#include "src/controllers/abstract_controllers/base_controller.h"
#include <list>
#include <iostream>
#include <fstream>

struct MatchingResult{
    Graph _graph;
    Matching _matching;
    vector<double> _cost;
    pair< list<int>, double > _solution;
    vector<CEPuck2Entity*> _robots;

    MatchingResult(Graph graph, Matching matching, vector<double> cost, pair< list<int>, double > solution, vector<CEPuck2Entity*> robots):
        _graph(graph),
        _matching(matching),
        _cost(cost),
        _solution(solution),
        _robots(robots)
    {}
};

CRadians GetZAngleOrientation(CQuaternion orientation) {
   CRadians cZAngle, cYAngle, cXAngle;
   orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   return cZAngle;
}


MatchingResult GetMatchingResult(vector<CEPuck2Entity*> robots) {
    std::vector<std::pair<int, CVector2>> positions;
    std::vector<std::pair<int, CQuaternion>> orientations;
    int number_of_robots = robots.size();
    for (unsigned i=0; i<number_of_robots; i++){
        CVector2 position(robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        positions.push_back(std::make_pair(i, position));
        orientations.push_back(std::make_pair(i, robots[i]->GetEmbodiedEntity().GetOriginAnchor().Orientation));
    }
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
                // cost[G.GetEdgeIndex(i,j)] += + 0.00605 * angle_distance;
                // cost[G.GetEdgeIndex(i,j)] += + 0.01 * angle_distance;
            }
        }
    }
    Matching M(G);
    pair< list<int>, double > solution = M.SolveMinimumCostPerfectMatching(cost);
    MatchingResult result = MatchingResult(G, M, cost, solution, robots);
    return result;
}



MatchingResult GetBestMatching(vector<CEPuck2Entity*> robots) {
    if(robots.size() % 2 == 0){
        return GetMatchingResult(robots);
    }
    vector<CEPuck2Entity*> robots_clone = robots;
    robots_clone.erase(robots_clone.begin());
    MatchingResult a = GetMatchingResult(robots_clone);
    MatchingResult* b = &a;
    unsigned j = 0;
    for(unsigned i = 1; i < robots.size(); i++) {
        robots_clone = robots;
        robots_clone.erase(std::next(robots_clone.begin(), i));
        MatchingResult result = GetMatchingResult(robots_clone);
        if(result._solution.second < b->_solution.second) {
            b = &result;
            j = i;
        }
    }
    robots_clone = robots;
    robots_clone.erase(robots_clone.begin()+j);
    MatchingResult bestMatchingResult = GetMatchingResult(robots_clone);
    return bestMatchingResult;
}
vector<int> Get_nf_matching(MatchingResult result){
    list<int> matching = result._solution.first;
    vector<int> nf_matchig;
    for(list<int>::iterator it = matching.begin(); it != matching.end(); it++)
    {
        pair<int, int> edge = result._graph.GetEdge( *it );
        CEPuck2Entity* robot1 = result._robots[edge.first];
        CEPuck2Entity* robot2 = result._robots[edge.second];
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(robot1->GetControllableEntity().GetController());
        BaseConrtoller& cController2 = dynamic_cast<BaseConrtoller&>(robot2->GetControllableEntity().GetController());
        if(cController1.GetType() == "non_faulty" && cController2.GetType() == "non_faulty"){
            nf_matchig.push_back(*it);
        }
    }
    return nf_matchig;
}

vector<int> Get_nf_half_matching(MatchingResult result){
    list<int> matching = result._solution.first;
    vector<int> nf_half_matchig;
    for(list<int>::iterator it = matching.begin(); it != matching.end(); it++)
    {
        pair<int, int> edge = result._graph.GetEdge( *it );
        CEPuck2Entity* robot1 = result._robots[edge.first];
        CEPuck2Entity* robot2 = result._robots[edge.second];
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(robot1->GetControllableEntity().GetController());
        BaseConrtoller& cController2 = dynamic_cast<BaseConrtoller&>(robot2->GetControllableEntity().GetController());
        if(cController1.GetType() == "non_faulty" || cController2.GetType() == "non_faulty"){
            nf_half_matchig.push_back(*it);
        }
    }
    return nf_half_matchig;
}

double GetMatchingCost(vector<int> nf_matchig, vector<double> costs){
    double nf_matching_cost = 0;
    for(vector<int>::iterator it = nf_matchig.begin(); it != nf_matchig.end(); it++)
		nf_matching_cost += costs[*it];
    return nf_matching_cost;
}

std::vector<std::pair<int, CVector2>> GetPositions(vector<CEPuck2Entity*> robots){
    std::vector<std::pair<int, CVector2>> positions;
    int number_of_robots = robots.size();
    for (unsigned i=0; i<number_of_robots; i++){
        CVector2 position(robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        positions.push_back(std::make_pair(i, position));
    }
    return positions;
}

std::vector<std::pair<int,int>> GetRobotPairs(vector<CEPuck2Entity*> robots) {
    std::vector<std::pair<int, CVector2>> positions = GetPositions(robots);
    std::vector<vector<int>> possible_robots_in_aggregarion_radios;
    std::vector<std::pair<int,int>> robot_pairs;
    for (unsigned i = 0; i < positions.size(); i++){
        double min_distance = 10000000;
        double min_robot = -1;
        std::vector<int> robots_in_radios;
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(robots[i]->GetControllableEntity().GetController());
        if(cController1.GetType() != "non_faulty"){
            continue;
        }
        for (unsigned j = 0; j < positions.size(); j++){
            BaseConrtoller& cController2 = dynamic_cast<BaseConrtoller&>(robots[j]->GetControllableEntity().GetController());
            if(cController2.GetType() != "non_faulty"){
                continue;
            }
            if(i == j) {
                continue;
            }
            double distance = (positions[i].second - positions[j].second).Length();
            if(distance < 0.1) {
                robots_in_radios.push_back(j);
            }
        }
        possible_robots_in_aggregarion_radios.push_back(robots_in_radios);
    }
    for(unsigned i = 0; i < possible_robots_in_aggregarion_radios.size(); i++){
        if(possible_robots_in_aggregarion_radios[i].size() == 1){
            int other_robot_index = possible_robots_in_aggregarion_radios[i][0];
            if(possible_robots_in_aggregarion_radios[other_robot_index][0] == i){
                if(i < other_robot_index){ // make sure we add the pair only once (for example instead of [(7,8),(8,7)] will be added [(7,8)])
                    robot_pairs.push_back(std::make_pair(i, other_robot_index));
                }
            }
        }
    }
    return robot_pairs;
}