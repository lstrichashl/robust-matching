#include "basic_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <list>
#include <iostream>
#include <fstream>

CBasicLoopFunctions::CBasicLoopFunctions():
    m_logs(0)
    {}

void CBasicLoopFunctions::Init(TConfigurationNode& t_tree) {
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


void CBasicLoopFunctions::write_all_logs(vector<string> logs, string params_string){
    ofstream os(m_log_file_path);
    std::string robot_types = "[";
    for (unsigned i=0; i<m_robots.size(); i++){
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(m_robots[i]->GetControllableEntity().GetController());
        robot_types += "{\"robot_id\":\""+to_string(i)+"\",\"type\":\""+cController1.GetType()+"\"},";
    }
    robot_types.pop_back();
    robot_types += "]";

    std::string all_log = "[";
    for(unsigned i = 0; i < logs.size(); i++){
        all_log += logs[i] + ",";
    }
    all_log.pop_back();
    all_log += "]";

    std::string filecontent = "{\"params\":"+params_string+",\"robot_types\":"+robot_types+",\"logs\":"+all_log+"}";
    os << filecontent << endl;
    os.close();
}

bool CBasicLoopFunctions::IsExperimentFinished() {
    // if(m_robots_positions.size() != 0){
    //     std::vector<std::pair<int, CVector2>> new_robots_positions = GetPositions(m_robots);
    //     for(unsigned i = 0; i < new_robots_positions.size(); i++){
    //         std::cout << m_robots_positions[i].second.GetX() << std::endl;
    //         if(new_robots_positions[i].second != m_robots_positions[i].second){
    //             return false;
    //         }
    //     }
    //     return true;
    // }
    return false;
}


// REGISTER_LOOP_FUNCTIONS(CPrintExperimentFunctions, "print_experiment_loop_fuctions")
