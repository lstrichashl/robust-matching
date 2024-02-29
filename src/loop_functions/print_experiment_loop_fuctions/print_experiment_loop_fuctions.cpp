#include "print_experiment_loop_fuctions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <list>
#include "src/controllers/abstract_controllers/base_controller.h"
#include <iostream>
#include <fstream>

CPrintExperimentFunctions::CPrintExperimentFunctions(){}

void CPrintExperimentFunctions::Init(TConfigurationNode& t_tree) {
    CBasicLoopFunctions::Init(t_tree);
}


void CPrintExperimentFunctions::Destroy(){
    std::ofstream os(m_log_file_path);
    std::string robot_types = "[";
    for (unsigned i=0; i<m_robots.size(); i++){
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(m_robots[i]->GetControllableEntity().GetController());
        robot_types += "{\"robot_id\":\""+std::to_string(i)+"\",\"type\":\""+cController1.GetType()+"\"},";
    }
    robot_types.pop_back();
    robot_types += "]";

    Clusters pairs = GetRobotPairs(GetNFRobots());
    write_to_log(pairs);

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
    CBasicLoopFunctions::PreStep();
    // std::vector<std::pair<int,int>> matching = GetRobotPairs(m_robots);
    // std::vector<std::pair<int, CVector2>> positions = GetPositions(m_robots);
    // std::string matcing_string = "\"matching\":\"["; 
    // for(unsigned i = 0; i < matching.size(); i++){
    //     matcing_string += "(" + to_string(matching[i].first) + "," + to_string(matching[i].second) + "),";
    // }
    // if(matching.size() != 0){
    //     matcing_string.pop_back();
    // }
    // matcing_string += "]\"";
    // std::cout << matcing_string << std::endl;
}

void CPrintExperimentFunctions::write_to_log(Clusters pairs){
    std::string tick_string = "\"tick\":"+to_string(GetSpace().GetSimulationClock());
    // double nf_matching_cost = GetMatchingCost(Get_nf_matching(result), result._cost);
    // double nf_half_matching_cost = GetMatchingCost(Get_nf_half_matching(result), result._cost);
    std::string matcing_string = "\"pairs\":" + pairs.ToString();
    std::string log =  "{" + matcing_string + "," + tick_string + "}";

    m_logs.push_back(log);
}


REGISTER_LOOP_FUNCTIONS(CPrintExperimentFunctions, "print_experiment_loop_fuctions")
