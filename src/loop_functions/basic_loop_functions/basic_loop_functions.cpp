#include "basic_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <list>
#include <iostream>
#include <fstream>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/simulator/entity/positional_entity.h>

static const UInt32      MAX_PLACE_TRIALS = 20;

CBasicLoopFunctions::CBasicLoopFunctions():
    m_logs(0),
    m_range(0)
    {}

void CBasicLoopFunctions::Init(TConfigurationNode& t_tree) {
    CVector2 cCenter(0,0);
    TConfigurationNodeIterator itDistr;
      for(itDistr = itDistr.begin(&t_tree);
          itDistr != itDistr.end();
          ++itDistr) {
            // if(itDistr->Value() == "distribute_max_range"){
            //     PlaceCluster(*itDistr);
            // }
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
    CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType("e-puck2");
    for(CSpace::TMapPerType::iterator it = m_cRobots.begin(); it != m_cRobots.end(); ++it) {
        CEPuck2Entity* robot = any_cast<CEPuck2Entity*>(it->second);
        m_robots.push_back(robot);
    }
    m_last_positions = GetPositions(m_robots);
    m_new_positions = m_last_positions;
    add_log();
}

void CBasicLoopFunctions::write_all_logs(vector<string> logs, string params_string){
    ofstream os(m_log_file_path);
    std::string robot_types = "[";
    for (unsigned i=0; i<m_robots.size(); i++){
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[i])->GetController());
        robot_types += "{\"robot_id\":\""+to_string(i)+"\",\"type\":\""+cController1.GetType()+"\"},";
    }
    robot_types.pop_back();
    robot_types += "]";

    std::string init_positions = "[";
    for(unsigned i=0; i<m_init_positions.size();i++){
        init_positions += "["+to_string(m_init_positions[i].GetX())+","+to_string(m_init_positions[i].GetY())+"],";
    }
    init_positions.pop_back();
    init_positions += "]";

    std::string all_log = "[";
    for(unsigned i = 0; i < logs.size(); i++){
        all_log += logs[i] + ",";
    }
    all_log.pop_back();
    all_log += "]";

    std::string filecontent = "{\"params\":"+params_string+",\"robot_types\":"+robot_types+",\"init_positions\":"+init_positions+",\"logs\":"+all_log+"}";
    os << filecontent << endl;
    os.close();
}

CVector2 GetPosition(CEntity* robot) {
    CEmbodiedEntity* embodiedEntity = GetEmbodiedEntity3(robot);
    CVector2 position(embodiedEntity->GetOriginAnchor().Position.GetX(),
            embodiedEntity->GetOriginAnchor().Position.GetY());
    return position;
}


vector<CVector2> CBasicLoopFunctions::GetPositions(vector<CEntity*> robots){
    vector<CVector2> positions;
    int number_of_robots = robots.size();
    for (unsigned i=0; i<number_of_robots; i++){
        CVector2 position = GetPosition(robots[i]);
        positions.push_back(position);
    }
    return positions;
}

vector<CEntity*> CBasicLoopFunctions::GetNFRobots(){
    // if(m_nf_robots.size() == 0){
    m_nf_robots = vector<CEntity*>();
        for(unsigned i = 0; i < m_robots.size(); i++){
            BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[i])->GetController());
            if(cController1.GetType() == "non_faulty"){
                m_nf_robots.push_back(m_robots[i]);
            }
        }
    // }
    return m_nf_robots;
}



Clusters CBasicLoopFunctions::GetRobotPairs(vector<CEntity*> robots) {
    vector<CVector2> positions = GetPositions(robots);
    vector<vector<int>> possible_robots_in_aggregarion_radios;
    Clusters robot_pairs;
    for (unsigned i = 0; i < robots.size(); i++){
        vector<int> robots_in_radios;
        for (unsigned j = 0; j < positions.size(); j++){
            if(i == j) {
                continue;
            }
            double distance = (positions[i] - positions[j]).Length();
            if(distance <= 0.07) {
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
    for(CEntity* robot : m_robots){
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robot)->GetController());
        cController1.m_time = time;
        CEmbodiedEntity* pcEmbodiedEntity = GetEmbodiedEntity3(robot);
        CVector2 position(pcEmbodiedEntity->GetOriginAnchor().Position.GetX(),
            pcEmbodiedEntity->GetOriginAnchor().Position.GetY());
        cController1.m_position = position;
        cController1.m_orientation = pcEmbodiedEntity->GetOriginAnchor().Orientation;
    }

    if(time == 1){
        m_last_positions = m_new_positions;
        m_new_positions = GetPositions(GetNFRobots());
        m_init_positions = m_new_positions;
    }
    if(time % 10 == 0){
        m_last_positions = m_new_positions;
        m_new_positions = GetPositions(GetNFRobots());
        add_log();
    }
    for(unsigned i = 0; i < m_robots.size(); i++){
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[i])->GetController());
        if(cController1.m_crash_time < time){
            cController1.m_is_crash = true;
        }
    }
}

void CBasicLoopFunctions::PostStep(){
    // int time = GetSpace().GetSimulationClock();
    // if(time % 50 == 0){
    //     remove_robots();
    // }
}

bool CBasicLoopFunctions::IsExperimentFinished() {
    if(GetSpace().GetSimulationClock() < 20){
        return false;
    }
    bool all_robots_are_paired = true;
    vector<CEntity*> nf_robots = GetNFRobots();
    int alone_robots_away = 0, alone_robots = 0;
    for(unsigned i = 0; i < nf_robots.size(); i++){
        BaseConrtoller& cController = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(nf_robots[i])->GetController());
        if(cController.GetEState() == STATE_ALONE){
            all_robots_are_paired = false;
            alone_robots++;
            if(cController.m_position.Length() > 10){ // TODO: make away robots const a parameter, or calcualate it based on the arena.
                alone_robots_away++;
            }
        }
    }
    if(all_robots_are_paired || alone_robots == 1){
        cout << "all robots are paired" << endl;
        return true;
    }
    if(alone_robots_away == alone_robots) {
        cout << "alone robots are away" << endl;
        return true;
    }
    return false;
}

void CBasicLoopFunctions::PostExperiment(){
    Clusters all_pairs = GetRobotPairs(m_robots);
    Clusters nf_pairs = GetRobotPairs(GetNFRobots());
    std::cout << "all pairs: (" << all_pairs.size() << ") " << all_pairs.ToString() << std::endl;
    std::cout << "nf pairs: (" << nf_pairs.size() << ") " << nf_pairs.ToString() << std::endl;
}

void CBasicLoopFunctions::add_log(){
    std::string tick_string = "\"tick\":\""+to_string(GetSpace().GetSimulationClock())+"\"";
    Clusters pairs = GetRobotPairs(m_robots);
    std::string matcing_string = "\"pairs\":" + pairs.ToString();
    double nf_distance_travel = 0;
    for(int i = 0; i < m_last_positions.size(); i++){
        nf_distance_travel += (m_last_positions[i] - m_new_positions[i]).Length();
    }
    string distance_travel_string = "\"distance_travel:\":"+to_string(nf_distance_travel);
    std::string log =  "{" + matcing_string + "," + tick_string + "," + distance_travel_string + "}";
    m_logs.push_back(log);
}



CPositionalEntity* GetPositionalEntity(CEntity* pc_entity) {
    /* Is the entity positional itself? */
    auto* pcPositionalTest = dynamic_cast<CPositionalEntity*>(pc_entity);
    if(pcPositionalTest != nullptr) {
        return pcPositionalTest;
    }
    /* Is the entity composable with a positional component? */
    auto* pcComposableTest = dynamic_cast<CComposableEntity*>(pc_entity);
    if(pcComposableTest != nullptr) {
        if(pcComposableTest->HasComponent("position")) {
        return &(pcComposableTest->GetComponent<CPositionalEntity>("position"));
        }
    }
    /* No positional entity found */
    return nullptr;
}

CVector3 GetRandomPosition(CRandom::CRNG* pcRNG, Real robot_range, const vector<CVector2>& all_positions, const CRange<Real>& arenaRange){
    CVector3 pos;
    do{
        // Real radius = pcRNG->Uniform(CRange<Real>(0.4, 0.5));
        // CRadians angle = pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
        // int robot_index = pcRNG->Uniform(CRange<UInt32>(0, all_positions.size()));
        // pos.Set(radius * Cos(angle) + all_positions[robot_index].GetX(),
        //             radius * Sin(angle) + all_positions[robot_index].GetY(),
        //             0.0f);
        pos.Set(pcRNG->Uniform(arenaRange), pcRNG->Uniform(arenaRange), 0);
    } while(!(pos.GetX() < arenaRange.GetMax() && pos.GetX() > arenaRange.GetMin() && pos.GetY() < arenaRange.GetMax() && pos.GetY() > arenaRange.GetMin()));
    return pos;
}

bool CBasicLoopFunctions::_is_connected_graph(vector<CVector2> positions, Real robot_range){
    Graph g(positions, robot_range);
	map<int, int> vertexMap;
    vector<vector<int> > components = g.findConnectedComponents(vertexMap);
    return components.size() == 1;
    // for(unsigned i = 0; i < positions.size(); i++){
    //     bool is_connected = false;
    //     for(unsigned j = 0; j < positions.size(); j++){
    //         if(i != j && (positions[i] - positions[j]).Length() < robot_range){
    //             is_connected = true;
    //             break;
    //         }
    //     }
    //     if(!is_connected){
    //         return false;
    //     }
    // }
    // return true;
}

void CBasicLoopFunctions::RemoveAll(vector<CEntity*> entites){
    for(unsigned i = 0; i < entites.size(); i++){
        RemoveEntity(*entites[i]);
    }
}

void CBasicLoopFunctions::PlaceCluster(vector<DistributeConfig> configs,Real robot_range, CRange<Real> arena_range, int base_id,vector<CEntity*> seed_entites){
    vector<CEntity*> spawn_entites, entites;
    UInt32 unMaxTrials = 100;
    /* Create a RNG (it is automatically disposed of by ARGoS) */
    CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
    CVector2 c_center(0,0);
    vector<CVector2> all_positions = GetPositions(seed_entites);
    all_positions.push_back(c_center);
    while(all_positions.size() == 1 || !_is_connected_graph(all_positions, robot_range)){
        RemoveAll(spawn_entites);
        all_positions = {c_center};
        spawn_entites = {};
        for(int c = 0; c < configs.size(); c++){
            UInt32 unQuantity = configs[c].unQuantity;
            TConfigurationNode tEntityTree = configs[c].robot_config;
            for(size_t i = 0; i < unQuantity; ++i) {
                string entity_id = ToString(base_id+i);
                SetNodeAttribute(tEntityTree, "id", entity_id);
                UInt32 unTrials = 0;
                bool bDone = false;
                bool bRetry = false;
                do {
                    ++unTrials;
                    CEntity* p = CFactory<CEntity>::New(tEntityTree.Value());
                    if(!NodeExists(tEntityTree, "body")) {
                        TConfigurationNode tBodyNode("body");
                        AddChildNode(tEntityTree, tBodyNode);
                    }
                    TConfigurationNode& tBodyNode = GetNode(tEntityTree, "body");
                    CVector3 pos = GetRandomPosition(pcRNG, robot_range, all_positions, arena_range);
                    CVector3 orientation(pcRNG->Uniform(CRange<Real>(0, 360)), 0,0);
                    SetNodeAttribute(tBodyNode, "position", pos);
                    SetNodeAttribute(tBodyNode, "orientation", orientation);
                    p->Init(tEntityTree);
                    AddEntity(*p);
                    CEmbodiedEntity* pcEmbodiedEntity = GetEmbodiedEntity3(p);
                    if(pcEmbodiedEntity->IsCollidingWithSomething()) {
                        bRetry = true;
                        RemoveEntity(*p);
                        ++unTrials;
                        if(unTrials > unMaxTrials) {
                            /* Yes, bomb out */
                            THROW_ARGOSEXCEPTION("Exceeded max trials when trying to distribute objects of type " <<
                                                tEntityTree.Value() << " with base id \"" <<
                                                entity_id << "\". I managed to place only " << i << " objects.");
                        }
                    }
                    else {
                        bDone = true;
                        CVector2 vector2_pos;
                        all_positions.push_back(pos.ProjectOntoXY(vector2_pos));
                        spawn_entites.push_back(p);
                    }
                } while(!bDone);
                if(!bDone) {
                    THROW_ARGOSEXCEPTION("Can't place robot" << entity_id);
                }
            }
            base_id += unQuantity;
        }
        arena_range = CRange<Real>(arena_range.GetMin()+0.1, arena_range.GetMax()-0.1);
    }
}

void CBasicLoopFunctions::PlaceCluster(TConfigurationNode& tDestributionTree) {
    try {
        Real robot_range;
        GetNodeAttribute(tDestributionTree, "range", robot_range);
        Real arena_size;
        GetNodeAttribute(tDestributionTree, "arena_size", arena_size);
        CRange<Real> arena_range(-arena_size/2, arena_size/2);

        TConfigurationNodeIterator itDistr;
        vector<DistributeConfig> configs;
        for(itDistr = itDistr.begin(&tDestributionTree);
          itDistr != itDistr.end();
          ++itDistr) {
            if(itDistr->Value() == "robot"){
                UInt32 unQuantity;
                GetNodeAttribute(*itDistr, "quantity", unQuantity);
                configs.push_back(DistributeConfig(GetNode(*itDistr, "e-puck2"), unQuantity));
            }
          }
        UInt32 base_id = 0;
        PlaceCluster(configs, robot_range, arena_range, base_id);        
    }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("While placing robots in a cluster", ex);
   }
}

