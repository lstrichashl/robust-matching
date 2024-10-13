#include "moving_forces_loop_function.h"


CMovingForces::CMovingForces():
    m_matching(0),
    m_costs(0),
    CBasicLoopFunctions()
    {
        default_robot_type = "e-puck2";
    }


CRadians GetZAngleOrientation(CEntity* robot1) {
   CRadians cZAngle, cYAngle, cXAngle;
   GetEmbodiedEntity3(robot1)->GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   return cZAngle;
}


void CMovingForces::PreStep(){
    CBasicLoopFunctions::PreStep();

}

REGISTER_LOOP_FUNCTIONS(CMovingForces, "moving_forces_loop_functions")
