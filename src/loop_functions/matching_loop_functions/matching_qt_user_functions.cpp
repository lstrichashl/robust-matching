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
      CEntity* cFootBot1 = m_matchingLoopFunctions.GetRobots().at(edge.first);
      CEntity* cFootBot2 = m_matchingLoopFunctions.GetRobots().at(edge.second);

      DrawRay(CRay3(
         GetEmbodiedEntity3(cFootBot1)->GetOriginAnchor().Position,
         GetEmbodiedEntity3(cFootBot1)->GetOriginAnchor().Position
      ));
   }
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CMatchingQTUserFunctions, "matching_qt_user_functions")

