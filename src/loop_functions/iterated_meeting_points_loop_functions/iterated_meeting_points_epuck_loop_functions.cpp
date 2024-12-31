#include "iterated_meeting_points_epuck_loop_functions.h"


CIteratedMeetingPointsEpuck::CIteratedMeetingPointsEpuck():
    m_matching(0),
    m_costs(0),
    CPrintExperimentFunctions()
    {
        default_robot_type = "e-puck2";
    }

Graph* GenerateGraph2(vector<CEntity*> robots, double range, Graph oldGraph, vector<int> oldMatching){
    std::vector<CVector2> positions;
    std::vector<std::pair<int, CQuaternion>> orientations;
    int number_of_robots = robots.size();
    for (unsigned i=0; i<number_of_robots; i++){
        CVector2 position(GetEmbodiedEntity3(robots[i])->GetOriginAnchor().Position.GetX(),
                GetEmbodiedEntity3(robots[i])->GetOriginAnchor().Position.GetY());
        positions.push_back(position);
        orientations.push_back(std::make_pair(i, GetEmbodiedEntity3(robots[i])->GetOriginAnchor().Orientation));
    }
    Graph* G = new Graph(number_of_robots);
    for(unsigned i=0; i<number_of_robots; i++){
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robots[i])->GetController());
        if(cController1.GetEState() == STATE_PAIRED){
            unsigned j = *cController1.m_matched_robot_indexes.rbegin();
            CVector2 distance_vector1 = (positions[i] - positions[j]);
            if(distance_vector1.Length() < range){
                G->AddEdge(i, j, distance_vector1.Length());
            }
        }
        else if(cController1.GetEState() == STATE_ALONE){
            for(unsigned j=0;j<number_of_robots;j++){
                BaseConrtoller& cController2 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robots[j])->GetController());
                if(i != j && cController2.GetEState() == STATE_ALONE && cController1.m_matched_robot_indexes.find(j) == cController1.m_matched_robot_indexes.end()){
                    CVector2 distance_vector1 = (positions[i] - positions[j]);
                    if(distance_vector1.Length() < range){
                        G->AddEdge(i, j, distance_vector1.Length());
                    }
                }
            }
        }
    }
    // AddHopToGraph(G, positions);
    return G;
}

Graph* GenerateGraph3(vector<CEntity*> robots, double range, Graph oldGraph, vector<int> oldMatching){
    std::vector<CVector2> positions;
    std::vector<std::pair<int, CQuaternion>> orientations;
    int number_of_robots = robots.size();
    for (unsigned i=0; i<number_of_robots; i++){
        CVector2 position(GetEmbodiedEntity3(robots[i])->GetOriginAnchor().Position.GetX(),
                GetEmbodiedEntity3(robots[i])->GetOriginAnchor().Position.GetY());
        positions.push_back(position);
        orientations.push_back(std::make_pair(i, GetEmbodiedEntity3(robots[i])->GetOriginAnchor().Orientation));
    }
    Graph* G = new Graph(number_of_robots);
    for(unsigned i=0; i<number_of_robots; i++){
        for(unsigned j=0;j<number_of_robots;j++){
            if(i != j) {
                CRadians cZAngle1 = GetZAngleOrientation(orientations[i].second);
                CRadians cZAngle2 = GetZAngleOrientation(orientations[j].second);
                CVector2 distance_vector1 = (positions[i] - positions[j]);

                Real angle_distance = 1 - (abs(cZAngle1.GetValue() - cZAngle2.GetValue()) / 3.141592);
                if(distance_vector1.Length() < range){
                    G->AddEdge(i, j, distance_vector1.Length());
                }
            }
        }
    }
    AddHopToGraph(G, positions);
    for(unsigned i=0; i < number_of_robots; i++){
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robots[i])->GetController());
        if(cController1.GetEState() == STATE_PAIRED){
            int matched_robot = *cController1.m_matched_robot_indexes.rbegin();
            list<int> adjlist = G->AdjList(i);
            for(const int j: adjlist){
                if(j != matched_robot){
                    G->costs[G->GetEdgeIndex(i,j)] = 10000;
                }
            }
        }
        else {
            for(const int& j :cController1.m_matched_robot_indexes){
                G->costs[G->GetEdgeIndex(i,j)] = 10000;
            }
        }
    }
    return G;
}


CRadians GetZAngleOrientation(CEntity* robot1) {
   CRadians cZAngle, cYAngle, cXAngle;
   GetEmbodiedEntity3(robot1)->GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   return cZAngle;
}

CVector2 GetMeetingPoint(CEntity* robot1, CEntity* robot2) {
    CVector2 self_position(GetEmbodiedEntity3(robot1)->GetOriginAnchor().Position.GetX(),
                            GetEmbodiedEntity3(robot1)->GetOriginAnchor().Position.GetY());
    CVector2 mate_position(GetEmbodiedEntity3(robot2)->GetOriginAnchor().Position.GetX(),
                            GetEmbodiedEntity3(robot2)->GetOriginAnchor().Position.GetY());
    CVector2 meeting_point((mate_position.GetX()+self_position.GetX())/2, (mate_position.GetY()+self_position.GetY())/2);
    return meeting_point;
}

void CIteratedMeetingPointsEpuck::UpdateMatching(){
    vector<int> robots_in_matching;
    vector<CEntity*> robots_not_in_matching;
    vector<CEntity*> signle_robots = m_robots;
    try{
        Graph* g = GenerateGraph3(signle_robots, 20000, m_robotGraph, m_matching);
        UInt32 time = GetSpace().GetSimulationClock();
        MatchingResult result = GetMatchingResult(g, signle_robots, 20000);

        m_matching = result._matching;
        m_robotGraph = result._graph;
        m_robots_in_matching = result._robots;
        vector<int> matching = result._matching;
        m_matching_max_cost = -1;
        for(vector<int>::iterator it = matching.begin(); it != matching.end(); it++){
            pair<int, int> edge = m_robotGraph.GetEdge( *it );
            double cost = m_robotGraph.GetCost(edge.first, edge.second);
            if(cost > m_matching_max_cost) {
                m_matching_max_cost = cost;
            }
        }
        for(vector<int>::iterator it = matching.begin(); it != matching.end(); it++)
        {
            pair<int, int> edge = m_robotGraph.GetEdge( *it );
            CEntity* robot1 = signle_robots[edge.first];
            CEntity* robot2 = signle_robots[edge.second];
            BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robot1)->GetController());
            BaseConrtoller& cController2 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robot2)->GetController());
            cController1.m_meeting_point = GetMeetingPoint(robot1, robot2);
            cController2.m_meeting_point = GetMeetingPoint(robot2, robot1);
            cController1.m_matched_robot_indexes.insert(edge.second);
            cController2.m_matched_robot_indexes.insert(edge.first);
            cController1.matched_robot_id = cController2.GetId();
            cController2.matched_robot_id = cController1.GetId();
            cController1.NewIteration();
            cController2.NewIteration();
        }
    }
   catch(std::exception& ex) {
    cout << "catch" << endl;
      THROW_ARGOSEXCEPTION_NESTED("error while iterated meeting points epuck loop function", ex);
   }
}

CVector2 ToMateVector2(CEntity* robot1, CVector2 meeting_point) {
    CVector2 self_position(GetEmbodiedEntity3(robot1)->GetOriginAnchor().Position.GetX(),
                            GetEmbodiedEntity3(robot1)->GetOriginAnchor().Position.GetY());
    CVector2 to_mate = meeting_point - self_position;
    CRadians cZAngle = GetZAngleOrientation(robot1);
    return to_mate.Rotate(-cZAngle);
}

void CIteratedMeetingPointsEpuck::PreStep(){
    CBasicLoopFunctions::PreStep();
    UInt32 time = GetSpace().GetSimulationClock();
    if(time == 1 || time - m_start_time_iterval > m_matching_max_cost * 750){
        m_start_time_iterval = time;
        // cout << m_start_time_iterval << "," <<  m_matching_max_cost * 750 << endl;
        UpdateMatching();
    }


    // for(unsigned i=0; i<m_robots.size(); i++){
    //     if(i == 1 || i == 3){
    //         BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[i])->GetController());
    //         string result1;
    //         for(const auto& elem : cController1.m_matched_robot_indexes){
    //             result1 += std::to_string(elem) + " ";
    //         }
    //         cout << i << ": " << result1 << endl;   
    //     }
    // }
    // // cout << "iterval length" << time-m_start_time_iterval << endl;
    // // cout << "max cost" << m_matching_max_cost << endl;
    for(vector<int>::iterator it = m_matching.begin(); it != m_matching.end(); it++)
        {
            pair<int, int> edge = m_robotGraph.GetEdge( *it );
            CEntity* robot1 = m_robots[edge.first];
            CEntity* robot2 = m_robots[edge.second];
            BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robot1)->GetController());
            BaseConrtoller& cController2 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robot2)->GetController());
            cController1.m_heading = ToMateVector2(robot1, cController1.m_meeting_point);
            cController2.m_heading = ToMateVector2(robot2, cController1.m_meeting_point);
        }
}

void CIteratedMeetingPointsEpuck::Reset(){
    CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType("e-puck2");
    for(CSpace::TMapPerType::iterator it = m_cRobots.begin(); it != m_cRobots.end(); ++it) {
        CEPuck2Entity* robot = any_cast<CEPuck2Entity*>(it->second);
        m_robots.push_back(robot);
    }
    m_last_positions = GetPositions(m_robots);
    m_new_positions = m_last_positions;
    add_log();
}

REGISTER_LOOP_FUNCTIONS(CIteratedMeetingPointsEpuck, "iterated_meeting_points_epuck_loop_functions")
