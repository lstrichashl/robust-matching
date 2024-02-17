#include "print_experiment_loop_fuctions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <list>
#include "src/controllers/virtual_forces/virtual_forces_bot.h"
#include <iostream>
#include <fstream>

CPrintExperimentFunctions::CPrintExperimentFunctions():
    m_logs(0)
    {}

void CPrintExperimentFunctions::Init(TConfigurationNode& t_tree) {
    CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType("e-puck2");
    for(CSpace::TMapPerType::iterator it = m_cRobots.begin(); it != m_cRobots.end(); ++it) {
        CEPuck2Entity* robot = any_cast<CEPuck2Entity*>(it->second);
        m_robots.push_back(robot);
    }
    TConfigurationNode& paramsNode = GetNode(t_tree, "params");
    try{
        GetNodeAttribute(paramsNode, "log_file_path", m_log_file_path);
    }
    catch(...){
        std::cout << "error with loading params tag in CPrintExperimentFunctions class" << std::endl;
    }
}


void CPrintExperimentFunctions::Destroy(){
    std::ofstream os(m_log_file_path);
    std::string robot_types = "[";
    for (unsigned i=0; i<m_robots.size(); i++){
        CVirtualForces& cController1 = dynamic_cast<CVirtualForces&>(m_robots[i]->GetControllableEntity().GetController());
        robot_types += "{\"robot_id\":\""+std::to_string(i)+"\",\"type\":\""+cController1.GetType()+"\"},";
    }
    robot_types.pop_back();
    robot_types += "]";

    MatchingResult result = GetBestMatching(m_robots);
    std::vector<std::pair<int,int>> matching = GetRobotPairs(m_robots);
    write_to_log(matching);

    std::string all_log = "[";
    for(unsigned i = 0; i < m_logs.size(); i++){
        all_log += m_logs[i] + ",";
    }
    all_log.pop_back();
    all_log += "]";

    std::string filecontent = "{\"robot_types\":"+robot_types+",\"logs\":"+all_log+"}";
    os << filecontent << std::endl;
    os.close();
}

void CPrintExperimentFunctions::PreStep(){

    // std::vector<std::pair<int,int>> matching = GetRobotPairs(m_robots);
    // std::vector<std::pair<int, CVector2>> positions = GetPositions(m_robots);
    // std::string matcing_string = "\"matching\":\"["; 
    // for(unsigned i = 0; i < matching.size(); i++){
    //     matcing_string += "(" + to_string(matching[i].first) + "," + to_string(matching[i].second) + "),";
    // }
    // std::cout << matcing_string << std::endl;
}

void CPrintExperimentFunctions::write_to_log(std::vector<std::pair<int,int>> matching){
    std::string cost_string = "\"cost\":\""+to_string(0) + "\"";
    std::string tick_string = "\"tick\":\""+to_string(GetSpace().GetSimulationClock())+"\"";
    // double nf_matching_cost = GetMatchingCost(Get_nf_matching(result), result._cost);
    // double nf_half_matching_cost = GetMatchingCost(Get_nf_half_matching(result), result._cost);
    std::string nf_matching_cost_string = "\"nf_matching_cost\":\""+to_string(0)+"\"";
    std::string nf_half_matching_cost_string = "\"nf_half_matching_cost\":\""+to_string(0)+"\"";
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


REGISTER_LOOP_FUNCTIONS(CPrintExperimentFunctions, "print_experiment_loop_fuctions")
