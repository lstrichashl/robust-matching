#include "iterated_meeting_points.h"


void CIteratedMeetingPoints::PreStep(){
    CBasicLoopFunctions::PreStep();
    vector<int> robots_in_matching;
    vector<CEntity*> robots_not_in_matching;
    try{
        UInt32 time = GetSpace().GetSimulationClock();
        if((m_matching.size() == 0 || (time % m_repeat_interval == 0 && !m_isCommited))) {
            MatchingResult result = GetMatchingResult(m_robots, m_range);
            m_matching = result._matching;
            m_robotGraph = result._graph;
            m_robots_in_matching = result._robots;
        }
        vector<int> matching = m_matching;
        vector<int> nf_matchig;
        vector<int> nf_half_matchig;
        for(unsigned i = 0; i < m_robots.size(); i++) {
            BaseConrtoller& cController = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[i])->GetController());
            cController.m_heading = CVector2::ZERO;
        }
        for(vector<int>::iterator it = matching.begin(); it != matching.end(); it++)
        {
            pair<int, int> edge = m_robotGraph.GetEdge( *it );
            CEntity* robot1 = m_robots[edge.first];
            CEntity* robot2 = m_robots[edge.second];
            BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robot1)->GetController());
            BaseConrtoller& cController2 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robot2)->GetController());
            cController1.m_heading = ToMateVector(robot1, robot2);
            cController2.m_heading = ToMateVector(robot2, robot1);
            robots_in_matching.push_back(edge.first);
            robots_in_matching.push_back(edge.second);
        }
        for(int i = 0; i < m_robots.size(); i++){
            if(find(robots_in_matching.begin(), robots_in_matching.end(), i) == robots_in_matching.end()){
                BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[i])->GetController());
                cController1.m_heading = cController1.RandomWalk();
            }
        }
    }
   catch(std::exception& ex) {
    cout << "catch" << endl;
      THROW_ARGOSEXCEPTION_NESTED("error while iterated meeting points loop function", ex);
   }
}

REGISTER_LOOP_FUNCTIONS(CIteratedMeetingPoints, "iterated_meeting_points_loop_functions")
