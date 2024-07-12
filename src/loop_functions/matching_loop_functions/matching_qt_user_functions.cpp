#include "matching_qt_user_functions.h"

/****************************************/
/****************************************/

CMatchingQTUserFunctions::CMatchingQTUserFunctions() :
   m_matchingLoopFunctions(dynamic_cast<CMatchingLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
}

/****************************************/
/****************************************/

void CMatchingQTUserFunctions::DrawInWorld() {
	vector<int> matching = m_matchingLoopFunctions.GetMatching();
   Graph G = m_matchingLoopFunctions.GetGraph();
   for(int i = 0; i < matching.size(); i++)
	{
		pair<int, int> edge = G.GetEdge(matching[i]);
      CEPuck2Entity* cFootBot1 = m_matchingLoopFunctions.GetRobots().at(edge.first);
      CEPuck2Entity* cFootBot2 = m_matchingLoopFunctions.GetRobots().at(edge.second);

      DrawRay(CRay3(
         cFootBot1->GetEmbodiedEntity().GetOriginAnchor().Position,
         cFootBot2->GetEmbodiedEntity().GetOriginAnchor().Position
      ));
   }

   vector<CVector2> positions = m_matchingLoopFunctions.GetPositions(m_matchingLoopFunctions.GetRobots());
   for(unsigned i = 0; i < positions.size(); i++) {
      CVector3 draw_text_position( positions[i].GetX()-0.05,positions[i].GetY()+0.05, 0.01);
      DrawText(draw_text_position, to_string(i));
   }
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CMatchingQTUserFunctions, "matching_qt_user_functions")

