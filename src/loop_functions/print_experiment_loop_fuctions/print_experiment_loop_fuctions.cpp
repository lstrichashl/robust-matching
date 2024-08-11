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
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[i])->GetController());
        robot_types += "{\"robot_id\":\""+std::to_string(i)+"\",\"type\":\""+cController1.GetType()+"\"},";
    }
    robot_types.pop_back();
    robot_types += "]";

    add_log();

    std::string all_log = "[";
    for(unsigned i = 0; i < m_logs.size(); i++){
        all_log += m_logs[i] + ",";
    }
    all_log.pop_back();
    all_log += "]";

    std::string init_positions = "[";
    for(unsigned i=0; i<m_init_positions.size();i++){
        init_positions += "["+to_string(m_init_positions[i].GetX())+","+to_string(m_init_positions[i].GetY())+"],";
    }
    init_positions.pop_back();
    init_positions += "]";

    std::string filecontent = "{\"robot_types\":"+robot_types+",\"init_positions\":"+init_positions+",\"logs\":"+all_log+"}";
    os << filecontent << std::endl;
    os.close();
}


REGISTER_LOOP_FUNCTIONS(CPrintExperimentFunctions, "print_experiment_loop_fuctions")
