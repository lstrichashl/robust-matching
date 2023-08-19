#include "matching_qt_user_functions.h"
#include "matching_loop_functions.h"

/****************************************/
/****************************************/

CMatchingQTUserFunctions::CMatchingQTUserFunctions() :
   m_matchingLoopFunctions(dynamic_cast<CMatchingLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
}

/****************************************/
/****************************************/

void CMatchingQTUserFunctions::DrawInWorld() {
	list<int> matching = m_matchingLoopFunctions.GetMatching().first;
	double matching_cost = m_matchingLoopFunctions.GetMatching().second;
   Graph G = m_matchingLoopFunctions.GetGraph();
   for(list<int>::iterator it = matching.begin(); it != matching.end(); it++)
	{
		pair<int, int> edge = G.GetEdge( *it );
      CEPuck2Entity* cFootBot1 = m_matchingLoopFunctions.GetRobots().at(edge.first);
      CEPuck2Entity* cFootBot2 = m_matchingLoopFunctions.GetRobots().at(edge.second);

      DrawRay(CRay3(
         cFootBot1->GetEmbodiedEntity().GetOriginAnchor().Position,
         cFootBot2->GetEmbodiedEntity().GetOriginAnchor().Position
      ));
   }
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CMatchingQTUserFunctions, "matching_qt_user_functions")

