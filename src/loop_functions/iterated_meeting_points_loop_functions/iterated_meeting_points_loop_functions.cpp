#include "iterated_meeting_points_loop_functions.h"


CIteratedMeetingPoints::CIteratedMeetingPoints():
    m_matching(0),
    m_costs(0),
    CBasicLoopFunctions()
    {
        default_robot_type = "eye-bot";
    }

Graph* GenerateGraph(vector<CEntity*> robots, double range, Graph oldGraph, vector<int> oldMatching){
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
        CMeetingPoint& cController1 = dynamic_cast<CMeetingPoint&>(GetControllableEntity3(robots[i])->GetController());
        if(cController1.GetEState() == STATE_PAIRED){
            unsigned j = *cController1.m_matched_robot_ids.rbegin();
            CVector2 distance_vector1 = (positions[i] - positions[j]);
            if(distance_vector1.Length() < range){
                G->AddEdge(i, j, distance_vector1.Length());
            }
        }
        else if(cController1.GetEState() == STATE_ALONE){
            for(unsigned j=0;j<number_of_robots;j++){
                CMeetingPoint& cController2 = dynamic_cast<CMeetingPoint&>(GetControllableEntity3(robots[j])->GetController());
                if(i != j && cController2.GetEState() == STATE_ALONE && cController1.m_matched_robot_ids.find(j) == cController1.m_matched_robot_ids.end()){
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

void CIteratedMeetingPoints::UpdateMatching(){
    vector<int> robots_in_matching;
    vector<CEntity*> robots_not_in_matching;
    vector<CEntity*> signle_robots = m_robots;
    try{
        Graph* g = GenerateGraph(signle_robots, 20, m_robotGraph, m_matching);
        MatchingResult result = GetMatchingResult(g, signle_robots, 20);
        m_matching = result._matching;
        m_robotGraph = result._graph;
        m_robots_in_matching = result._robots;
        vector<int> matching = result._matching;
        cout << "matching:" << endl;
        for(vector<int>::iterator it = matching.begin(); it != matching.end(); it++){
            pair<int, int> edge = m_robotGraph.GetEdge( *it );
            cout << edge.first << "," << edge.second << endl;
        }
        for(vector<int>::iterator it = matching.begin(); it != matching.end(); it++)
        {
            pair<int, int> edge = m_robotGraph.GetEdge( *it );
            CEntity* robot1 = signle_robots[edge.first];
            CEntity* robot2 = signle_robots[edge.second];
            CMeetingPoint& cController1 = dynamic_cast<CMeetingPoint&>(GetControllableEntity3(robot1)->GetController());
            CMeetingPoint& cController2 = dynamic_cast<CMeetingPoint&>(GetControllableEntity3(robot2)->GetController());
            CVector3 target = (GetEmbodiedEntity3(robot1)->GetOriginAnchor().Position + GetEmbodiedEntity3(robot2)->GetOriginAnchor().Position)/2;
            cController1.m_target_position = target;
            cController2.m_target_position = target;

            cController1.m_matched_robot_ids.insert(edge.second);
            cController2.m_matched_robot_ids.insert(edge.first);
        }
    }
   catch(std::exception& ex) {
    cout << "catch" << endl;
      THROW_ARGOSEXCEPTION_NESTED("error while iterated meeting points loop function", ex);
   }
}

void CIteratedMeetingPoints::PreStep(){
    CBasicLoopFunctions::PreStep();
    UInt32 time = GetSpace().GetSimulationClock();
    if(time == 1){
        UpdateMatching();
    }
    else if(time % 200 == 0){
        UpdateMatching();
    }
}

void CIteratedMeetingPoints::Reset(){
    CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType("eye-bot");
    for(CSpace::TMapPerType::iterator it = m_cRobots.begin(); it != m_cRobots.end(); ++it) {
        CEyeBotEntity* robot = any_cast<CEyeBotEntity*>(it->second);
        m_robots.push_back(robot);
    }
    m_last_positions = GetPositions(m_robots);
    m_new_positions = m_last_positions;
    add_log();
}

REGISTER_LOOP_FUNCTIONS(CIteratedMeetingPoints, "iterated_meeting_points_loop_functions")
