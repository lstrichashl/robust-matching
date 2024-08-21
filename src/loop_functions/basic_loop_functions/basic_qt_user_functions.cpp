#include "basic_qt_user_functions.h"

CBasicQTUserFunctions::CBasicQTUserFunctions():
   m_loop_functions(dynamic_cast<CBasicLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions()))
{}

void CBasicQTUserFunctions::DrawInWorld(){
    vector<CEntity*> robots = m_loop_functions.GetRobots();
    vector<CVector2> positions = m_loop_functions.GetPositions(robots);
    for(unsigned i = 0; i < positions.size(); i++) {
        CVector3 draw_text_position( positions[i].GetX()-0.05,positions[i].GetY()+0.05, 0.01);
        DrawText(draw_text_position, to_string(i), CColor::RED);
    }
    // for(unsigned rinx = 0; rinx < robots.size(); rinx++){
    //     BaseConrtoller& cController1 = dynamic_cast<BaseConrtoller&>(GetControllableEntity3(robots[rinx])->GetController());
    //     CVector3 pos(positions[rinx].GetX(), positions[rinx].GetY(), 0.1);
    //     CVector3 meetingpoint(cController1.m_meeting_point.GetX(), cController1.m_meeting_point.GetY(), 0.1);
    //     CVector3 endRay = meetingpoint + pos;
    //     // cout << cController1.m_meeting_point << endl;
    //     DrawRay(CRay3(
    //         pos,
    //         meetingpoint
    //     ));
    // }
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CBasicQTUserFunctions, "basic_qt_user_functions")
