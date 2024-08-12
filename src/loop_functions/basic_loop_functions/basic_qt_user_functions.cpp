#include "basic_qt_user_functions.h"

CBasicQTUserFunctions::CBasicQTUserFunctions():
   m_loop_functions(dynamic_cast<CBasicLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions()))
{}

void CBasicQTUserFunctions::DrawInWorld(){
    vector<CVector2> positions = m_loop_functions.GetPositions(m_loop_functions.GetRobots());
    for(unsigned i = 0; i < positions.size(); i++) {
        CVector3 draw_text_position( positions[i].GetX()-0.05,positions[i].GetY()+0.05, 0.01);
        DrawText(draw_text_position, to_string(i), CColor::RED);
    }
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CBasicQTUserFunctions, "basic_qt_user_functions")
