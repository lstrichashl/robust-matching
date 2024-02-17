#include "matching_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/robust_matching/matching_regular_bot.h"
#include "src/loop_functions/matching_result/matching_result.h"
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
    m_logs(0),
    m_isCommited(false),
    m_repeat_interval(1)
    {}

void CMatchingLoopFunctions::Init(TConfigurationNode& t_tree) {
    CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType("e-puck2");
    for(CSpace::TMapPerType::iterator it = m_cRobots.begin(); it != m_cRobots.end(); ++it) {
        CEPuck2Entity* robot = any_cast<CEPuck2Entity*>(it->second);
        m_robots.push_back(robot);
    }
    TConfigurationNode& paramsNode = GetNode(t_tree, "params");
    try{
        bool is_commited;
        GetNodeAttribute(paramsNode, "is_commited", is_commited);
        m_isCommited = is_commited;

        GetNodeAttribute(paramsNode, "log_file_path", m_log_file_path);
        GetNodeAttribute(paramsNode, "repeat_interval", m_repeat_interval);
    }
    catch(...){
        cout << "error with loading params tag in CMatchingLoopFunctions class" << endl;
    }
}

void CMatchingLoopFunctions::Destroy(){
    ofstream os(m_log_file_path);
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

void CMatchingLoopFunctions::PreStep(){
    UInt32 time = GetSpace().GetSimulationClock();
    if(m_solution.second == -1 || (time % m_repeat_interval == 0 && !m_isCommited)) {
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
        if(cController1.GetType() == "non_faulty" && cController2.GetType() == "non_faulty"){
            nf_matchig.push_back(*it);
        }
        if(cController1.GetType() == "non_faulty" || cController2.GetType() == "non_faulty"){
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
    std::vector<std::pair<int,int>> matching = GetRobotPairs(m_robots);
    std::string matcing_string = "\"matching\":\"["; 
    for(unsigned i = 0; i < matching.size(); i++){
        matcing_string += "(" + to_string(matching[i].first) + "," + to_string(matching[i].second) + "),";
    }
    if(matching.size() != 0){
        matcing_string.pop_back();
    }
    matcing_string += "]\"";
    std::string log =  "{" + matcing_string + "," + cost_string + "," + tick_string + "," + nf_matching_cost_string + "," + nf_half_matching_cost_string + "}";

    m_logs.push_back(log);
}

bool CMatchingLoopFunctions::IsExperimentFinished() {
    return false;
}

REGISTER_LOOP_FUNCTIONS(CMatchingLoopFunctions, "matching_loop_functions")
