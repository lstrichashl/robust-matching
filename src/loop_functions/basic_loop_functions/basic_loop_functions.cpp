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
    Reset();
}

void CBasicLoopFunctions::Reset(){
    m_last_positions = GetPositions(m_robots);
    number_of_rounds_no_change_postions = 0;
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


vector<CVector2> CBasicLoopFunctions::GetPositions(vector<CEPuck2Entity*> robots){
    vector<CVector2> positions;
    int number_of_robots = robots.size();
    for (unsigned i=0; i<number_of_robots; i++){
        CVector2 position(robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                robots[i]->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        positions.push_back(position);
    }
    return positions;
}

vector<CEPuck2Entity*> CBasicLoopFunctions::GetNFRobots(){
    vector<CEPuck2Entity*> nf_robots;
    for(unsigned i = 0; i < m_robots.size(); i++){
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(m_robots[i]->GetControllableEntity().GetController());
        if(cController1.GetType() == "non_faulty"){
            nf_robots.push_back(m_robots[i]);
        }
    }
    return nf_robots;
}



Clusters CBasicLoopFunctions::GetRobotPairs(vector<CEPuck2Entity*> robots) {
    vector<CVector2> positions = GetPositions(robots);
    vector<vector<int>> possible_robots_in_aggregarion_radios;
    Clusters robot_pairs;
    for (unsigned i = 0; i < positions.size(); i++){
        vector<int> robots_in_radios;
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
            double distance = (positions[i] - positions[j]).Length();
            if(distance <= cController2.PAIRING_THRESHOLD) {
                robots_in_radios.push_back(j);
            }
        }
        possible_robots_in_aggregarion_radios.push_back(robots_in_radios);
    }
    for(int i = 0; i < possible_robots_in_aggregarion_radios.size(); i++){
        if(possible_robots_in_aggregarion_radios[i].size() == 1){
            int other_robot_index = possible_robots_in_aggregarion_radios[i][0];
            if(possible_robots_in_aggregarion_radios[other_robot_index][0] == i){
                if(i < other_robot_index){ // make sure we add the pair only once (for example instead of [(7,8),(8,7)] will be added [(7,8)])
                    vector<int> pairs = {i,other_robot_index};
                    robot_pairs.AddCluster(pairs);
                }
            }
        }
    }
    return robot_pairs;
}

void CBasicLoopFunctions::PreStep(){
    int time = GetSpace().GetSimulationClock();
    if(time % 5 == 0){
        m_last_positions = m_new_positions;
        m_new_positions = GetPositions(GetNFRobots());
    }
}

bool CBasicLoopFunctions::IsExperimentFinished() {
    bool all_robots_are_paired = true;
    vector<CEPuck2Entity*> nf_robots = GetNFRobots();
    for(unsigned i = 0; i < nf_robots.size(); i++){
        BaseConrtoller& cController = dynamic_cast<BaseConrtoller&>(nf_robots[i]->GetControllableEntity().GetController());
        if(cController.GetEState() == STATE_ALONE){
            all_robots_are_paired = false;
        }
    }
    if(all_robots_are_paired){
        return true;
    }
    if(m_last_positions.size() != 0 && m_new_positions.size() != 0){
        for(unsigned i = 0; i < m_new_positions.size(); i++){
            if(m_new_positions[i] != m_last_positions[i]){
                return false;
            }
        }
        return true;
    }
    return false;
}