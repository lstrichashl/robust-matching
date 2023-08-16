#include "matching_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <controllers/robust_matching/robust_matching.h>
#include <list>
#include "Graph.h"
#include "Matching.h"

CMatchingLoopFunctions::CMatchingLoopFunctions() {}

void CMatchingLoopFunctions::PreStep(){
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("e-puck2");
    std::vector<std::pair<int, CVector2>> positions;
    std::vector<std::pair<int, Real>> orientations;
    std::vector<CEPuck2Entity*> footbots;
    int index = 0;
    int len = 0;
    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
        CEPuck2Entity* cFootBot = any_cast<CEPuck2Entity*>(it->second);
        footbots.push_back(cFootBot);
        CVector2 cPos;
        cPos.Set(cFootBot->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                cFootBot->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        positions.push_back(std::make_pair(index, cPos));
        CRadians cZAngle, cYAngle, cXAngle;

        cFootBot->GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
        CDegrees cAngle = ToDegrees(cZAngle);
        orientations.push_back(std::make_pair(index, cAngle.GetValue()));
        index++;
        len++;
    }
    std::vector<std::pair<int, CVector2>>::iterator it1;
    std::vector<std::pair<int, CVector2>>::iterator it2;
    Graph G(len);
	vector<double> cost(len*len);
    for(it1 = positions.begin(); it1 != positions.end(); ++it1){
        for(it2 = positions.begin(); it2 != positions.end(); ++it2){
            std::pair<int, CVector2> pos1 = *it1;
            std::pair<int, CVector2> pos2 = *it2;
            Real distance = (pos1.second - pos2.second).Length();
            G.AddEdge(pos1.first, pos2.first);
            cost[G.GetEdgeIndex(pos1.first, pos2.first)] = distance;
        }
    }
    Matching M(G);
    pair< list<int>, double > solution = M.SolveMinimumCostPerfectMatching(cost);

	list<int> matching = solution.first;
	double obj = solution.second;

	cout << "Optimal matching cost: " << obj << endl;
	for(list<int>::iterator it = matching.begin(); it != matching.end(); it++)
	{
		pair<int, int> e = G.GetEdge( *it );
        CVector2 first_pos = positions.at(e.first).second;
        CVector2 second_pos = positions.at(e.second).second;
        CVector2 diff = first_pos - second_pos;
        diff = diff.Normalize();

        CEPuck2Entity* cFootBot1 = footbots.at(e.first);
        CEPuck2Entity* cFootBot2 = footbots.at(e.second);

        CRobustMatching& cController1 = dynamic_cast<CRobustMatching&>(cFootBot1->GetControllableEntity().GetController());
        CRobustMatching& cController2 = dynamic_cast<CRobustMatching&>(cFootBot2->GetControllableEntity().GetController());

        cController1.self_position.Set(first_pos.GetX(), first_pos.GetY(), 0);
        cController2.self_position.Set(second_pos.GetX(), second_pos.GetY(), 0);
        cController1.m_cTargetPos = cController2.self_position;
        cController2.m_cTargetPos = cController1.self_position;

        cController1.mate = cFootBot2;
        cController2.mate = cFootBot1;

        
        cController1.orientation = orientations.at(e.first).second;
        cController2.orientation = orientations.at(e.second).second;
	}
}

REGISTER_LOOP_FUNCTIONS(CMatchingLoopFunctions, "matching_loop_functions")
