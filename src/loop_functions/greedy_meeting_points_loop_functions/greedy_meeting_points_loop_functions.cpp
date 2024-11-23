#include "greedy_meeting_points_loop_functions.h"

CGreedyMeetingPointsLoopFunctions::CGreedyMeetingPointsLoopFunctions(){

}


void CGreedyMeetingPointsLoopFunctions::Init(TConfigurationNode& t_tree) {
    CBasicLoopFunctions::Init(t_tree);
    // CGreedyMeetingPointsEpuck::Reset();
}

void CGreedyMeetingPointsLoopFunctions::Reset(){
    CBasicLoopFunctions::Reset();
    for(int i = 0; i < m_robots.size(); i++){
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[i])->GetController());
        m_robot_id_to_index[cController1.GetId()] = i;
    }
}

CVector2 GetMeetingPoint(CEntity* robot1, CEntity* robot2) {
    CVector2 self_position(GetEmbodiedEntity3(robot1)->GetOriginAnchor().Position.GetX(),
                            GetEmbodiedEntity3(robot1)->GetOriginAnchor().Position.GetY());
    CVector2 mate_position(GetEmbodiedEntity3(robot2)->GetOriginAnchor().Position.GetX(),
                            GetEmbodiedEntity3(robot2)->GetOriginAnchor().Position.GetY());
    CVector2 meeting_point((mate_position.GetX()+self_position.GetX())/2, (mate_position.GetY()+self_position.GetY())/2);
    return meeting_point;
}
CRadians GetZAngleOrientation(CEntity* robot1) {
   CRadians cZAngle, cYAngle, cXAngle;
   GetEmbodiedEntity3(robot1)->GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   return cZAngle;
}

void CGreedyMeetingPointsLoopFunctions::PostStep(){
    for(unsigned i = 0; i < m_robots.size(); i++){
        BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[i])->GetController());
        if(cController1.matched_robot_id != "-1"){
            if(cController1.m_meeting_point == CVector2::ZERO){
                int j = m_robot_id_to_index[cController1.matched_robot_id];
                BaseConrtoller& cController2 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(m_robots[j])->GetController());
                cController1.m_meeting_point = GetMeetingPoint(m_robots[i], m_robots[j]);
                cController2.m_meeting_point = GetMeetingPoint(m_robots[j], m_robots[i]);
                // cController1.m_matched_robot_indexes.insert(edge.second);
                // cController2.m_matched_robot_indexes.insert(edge.first);
                // cController1.matched_robot_id = cController2.GetId();
                // cController2.matched_robot_id = cController1.GetId();
            }

            CVector2 self_position(GetEmbodiedEntity3(m_robots[i])->GetOriginAnchor().Position.GetX(),
                        GetEmbodiedEntity3(m_robots[i])->GetOriginAnchor().Position.GetY());
            CVector2 to_mate = cController1.m_meeting_point - self_position;
            CRadians cZAngle = GetZAngleOrientation(m_robots[i]);
            cController1.m_heading =  to_mate.Rotate(-cZAngle) * 10;
        }
    }
}

REGISTER_LOOP_FUNCTIONS(CGreedyMeetingPointsLoopFunctions, "greedy_meeting_points_loop_functions")
