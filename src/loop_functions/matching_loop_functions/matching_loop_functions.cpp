#include "matching_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "src/controllers/robust_matching/robust_matching.h"
#include <list>
#include "Graph.h"
#include "Matching.h"

CMatchingLoopFunctions::CMatchingLoopFunctions() {}

void CMatchingLoopFunctions::PreStep(){
    CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType("e-puck2");
    std::vector<std::pair<int, CVector2>> positions;
    std::vector<CEPuck2Entity*> footbots;
    int index = 0;
    int number_of_robots = 0;
    for(CSpace::TMapPerType::iterator it = m_cRobots.begin(); it != m_cRobots.end(); ++it) {
        CEPuck2Entity* robot = any_cast<CEPuck2Entity*>(it->second);
        footbots.push_back(robot);
        CVector2 position;
        position.Set(robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                robot->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        positions.push_back(std::make_pair(index, position));
        index++;
        number_of_robots++;
    }
    std::vector<std::pair<int, CVector2>>::iterator it1;
    std::vector<std::pair<int, CVector2>>::iterator it2;
    Graph G(number_of_robots);
	vector<double> cost(number_of_robots*number_of_robots);
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
	double matching_cost = solution.second;

	cout << "Optimal matching cost: " << matching_cost << endl;
	for(list<int>::iterator it = matching.begin(); it != matching.end(); it++)
	{
		pair<int, int> edge = G.GetEdge( *it );
        CEPuck2Entity* cFootBot1 = footbots.at(edge.first);
        CEPuck2Entity* cFootBot2 = footbots.at(edge.second);
        CRobustMatching& cController1 = dynamic_cast<CRobustMatching&>(cFootBot1->GetControllableEntity().GetController());
        CRobustMatching& cController2 = dynamic_cast<CRobustMatching&>(cFootBot2->GetControllableEntity().GetController());
        cController1.mate = cFootBot2;
        cController2.mate = cFootBot1;
	}
}

REGISTER_LOOP_FUNCTIONS(CMatchingLoopFunctions, "matching_loop_functions")
