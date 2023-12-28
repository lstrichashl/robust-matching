#include "matching_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/robust_matching/robust_matching.h"
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
    m_logs(0),
    m_isCommited(false)
    {}

void CMatchingLoopFunctions::Init(TConfigurationNode& t_tree) {
    CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType("e-puck2");
    for(CSpace::TMapPerType::iterator it = m_cRobots.begin(); it != m_cRobots.end(); ++it) {
        CEPuck2Entity* robot = any_cast<CEPuck2Entity*>(it->second);
        m_robots.push_back(robot);
    }
    try{
        TConfigurationNode& paramsNode = GetNode(t_tree, "params");
        bool is_commited;
        GetNodeAttribute(paramsNode, "is_commited", is_commited);
        m_isCommited = is_commited;
    }
    catch(...){
        cout << "error with loading params tag in CMatchingLoopFunctions class" << endl;
    }
}

void CMatchingLoopFunctions::Destroy(){
    ofstream os(GetLogFileName());

    std::string robot_types = "[";
    for (unsigned i=0; i<m_robots.size(); i++){
        CRobustMatching& cController1 = dynamic_cast<CRobustMatching&>(m_robots[i]->GetControllableEntity().GetController());
        robot_types += "{\"robot_id\":\""+to_string(i)+"\",\"type\":\""+cController1.GetType()+"\"},";
    }
    robot_types.pop_back();
    robot_types += "]";

    std::string all_log = "[";
    for(unsigned i = 0; i < m_logs.size(); i++){
        all_log += m_logs[i] + ",";
    }
    all_log.pop_back();
    all_log += "]";

    std::string param_string = "{\"is_commited\":\""+to_string(m_isCommited)+"\"}";

    std::string filecontent = "{\"params\":"+param_string+",\"robot_types\":"+robot_types+",\"logs\":"+all_log+"}";
    os << filecontent << endl;
    os.close();
}

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

void CMatchingLoopFunctions::PreStep(){
    if(m_solution.second == -1 || !m_isCommited) {
        MatchingResult result = GetBestMatching(m_robots);
        m_solution = result._solution;
        m_costs = result._cost;
        m_robotGraph = result._graph;
        m_robots_in_matching = result._robots;
    }
    list<int> matching = m_solution.first;
    vector<int> nf_matchig;
    vector<int> nf_half_matchig;
    for(unsigned i = 0; i < m_robots.size(); i++) {
        CRobustMatching& cController = dynamic_cast<CRobustMatching&>(m_robots[i]->GetControllableEntity().GetController());
        cController.mate = NULL;
    }
    for(list<int>::iterator it = matching.begin(); it != matching.end(); it++)
    {
        pair<int, int> edge = m_robotGraph.GetEdge( *it );
        CEPuck2Entity* robot1 = m_robots_in_matching[edge.first];
        CEPuck2Entity* robot2 = m_robots_in_matching[edge.second];
        CRobustMatching& cController1 = dynamic_cast<CRobustMatching&>(robot1->GetControllableEntity().GetController());
        CRobustMatching& cController2 = dynamic_cast<CRobustMatching&>(robot2->GetControllableEntity().GetController());
        if(cController1.GetType() == "CRobustMatching" && cController2.GetType() == "CRobustMatching"){
            nf_matchig.push_back(*it);
        }
        if(cController1.GetType() == "CRobustMatching" || cController2.GetType() == "CRobustMatching"){
            nf_half_matchig.push_back(*it);
        }
        cController1.mate = robot2;
        cController2.mate = robot1; 
    }
    double nf_matching_cost = 0;
    for(vector<int>::iterator it = nf_matchig.begin(); it != nf_matchig.end(); it++)
		nf_matching_cost += m_costs[*it];

    double nf_half_matching_cost = 0;
    for(vector<int>::iterator it = nf_half_matchig.begin(); it != nf_half_matchig.end(); it++)
		nf_half_matching_cost += m_costs[*it];

    write_to_log(m_robotGraph, m_solution, nf_matching_cost, nf_half_matching_cost);
}

void CMatchingLoopFunctions::write_to_log(Graph graph, pair< list<int>, double > solution, double nf_matching_cost, double nf_half_matching_cost){
    std::string cost_string = "\"cost\":\""+to_string(solution.second) + "\"";
    std::string tick_string = "\"tick\":\""+to_string(GetSpace().GetSimulationClock())+"\"";
    std::string nf_matching_cost_string = "\"nf_matching_cost\":\""+to_string(nf_matching_cost)+"\"";
    std::string nf_half_matching_cost_string = "\"nf_half_matching_cost\":\""+to_string(nf_half_matching_cost)+"\"";
    list<int> matching = solution.first;
    std::string matcing_string = "\"matching\":\"["; 
    for(list<int>::iterator it = matching.begin(); it != matching.end(); it++){
        pair<int, int> edge = graph.GetEdge( *it );
        matcing_string += "(" + to_string(edge.first) + "," + to_string(edge.second) + "),";
    }
    matcing_string.pop_back();
    matcing_string += "]\"";
    std::string log =  "{" + matcing_string + "," + cost_string + "," + tick_string + "," + nf_matching_cost_string + "," + nf_half_matching_cost_string + "}";

    m_logs.push_back(log);
}

std::string CMatchingLoopFunctions::GetLogFileName(){
    return GetSimulator().GetExperimentFileName() + ".log";
}

bool CMatchingLoopFunctions::IsExperimentFinished() {
    return false;
}

REGISTER_LOOP_FUNCTIONS(CMatchingLoopFunctions, "matching_loop_functions")
