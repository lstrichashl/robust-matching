#include "matching_qt_user_functions.h"
#include <controllers/footbot_matching/footbot_matching.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

/****************************************/
/****************************************/

CMatchingQTUserFunctions::CMatchingQTUserFunctions() {
   RegisterUserFunction<CMatchingQTUserFunctions,CFootBotEntity>(&CMatchingQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CMatchingQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CFootBotMatching& cController = dynamic_cast<CFootBotMatching&>(c_entity.GetControllableEntity().GetController());
      CVector3 mypos, matched_pos;
      mypos.Set(
         c_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
         c_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY(),
         0.0f
      );
      matched_pos.Set(
         cController.matched_bot->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
         cController.matched_bot->GetEmbodiedEntity().GetOriginAnchor().Position.GetY(),
         0.0f
      );
               
      DrawRay(CRay3(
         mypos,
         matched_pos
      ));
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CMatchingQTUserFunctions, "matching_qt_user_functions")

